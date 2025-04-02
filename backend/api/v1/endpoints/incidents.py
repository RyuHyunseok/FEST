from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import json
import redis
import uuid
from datetime import datetime

from db.database import get_db
from models.incident import Incident
from schemas.incident import Incident as IncidentSchema
from schemas.incident import IncidentCreate, IncidentUpdate
from infra.mqtt.mqtt_client import mqtt_client
from sqlalchemy import func
from geoalchemy2.functions import ST_MakePoint
import os


router = APIRouter()

# Redis 클라이언트
redis_client = redis.Redis(
    host=os.getenv('REDIS_HOST', 'localhost'), 
    port=int(os.getenv('REDIS_PORT', 6379)), 
    db=int(os.getenv('REDIS_DB', 0)), 
    decode_responses=True
)

@router.get("/", response_model=List[IncidentSchema])
def get_incidents(db: Session = Depends(get_db)):
    """
    모든 화재 사고 목록 조회
    """
    incidents = db.query(Incident).all()
    
    # 결과를 변환하여 반환
    result = []
    for incident in incidents:
        # PostGIS 위치 데이터를 GeoJSON으로 변환
        location_geojson = db.scalar(func.ST_AsGeoJSON(incident.location))
        location_dict = json.loads(location_geojson)
        
        # 화재 정보를 딕셔너리로 변환
        incident_dict = {
            "incident_id": incident.incident_id,
            "status": incident.status,
            "detected_at": incident.detected_at,
            "extinguished_at": incident.extinguished_at,
            "location": {
                "x": location_dict["coordinates"][0],
                "y": location_dict["coordinates"][1]
            }
        }
        result.append(incident_dict)
    
    return result
@router.get("/active")
def get_active_incidents(db: Session = Depends(get_db)):
    """
    현재 활성화된 화재 목록 조회
    """
    # Redis에서 모든 활성화된 화재 키 가져오기
    incident_keys = redis_client.keys("incident:*")
    
    active_incidents = []
    for key in incident_keys:
        incident_data = redis_client.get(key)
        if incident_data:
            active_incidents.append(json.loads(incident_data))
    
    return active_incidents

@router.get("/{incident_id}", response_model=IncidentSchema)
def get_incident(incident_id: str, db: Session = Depends(get_db)):
    """
    특정 화재 사고 상세 정보 조회
    """
    incident = db.query(Incident).filter(Incident.incident_id == incident_id).first()
    if incident is None:
        raise HTTPException(status_code=404, detail="Incident not found")
    
    # PostGIS 위치 데이터를 GeoJSON으로 변환
    location_geojson = db.scalar(func.ST_AsGeoJSON(incident.location))
    location_dict = json.loads(location_geojson)
    
    # 화재 정보를 딕셔너리로 변환
    incident_dict = {
        "incident_id": incident.incident_id,
        "status": incident.status,
        "detected_at": incident.detected_at,
        "extinguished_at": incident.extinguished_at,
        "location": {
            "x": location_dict["coordinates"][0],
            "y": location_dict["coordinates"][1]
        }
    }
    
    return incident_dict



@router.put("/{incident_id}", response_model=IncidentSchema)
def update_incident(incident_id: str, incident: IncidentUpdate, db: Session = Depends(get_db)):
    """
    화재 상태 업데이트
    """
    db_incident = db.query(Incident).filter(Incident.incident_id == incident_id).first()
    if db_incident is None:
        raise HTTPException(status_code=404, detail="Incident not found")
    
    # 변경 사항 업데이트
    if incident.status is not None:
        db_incident.status = incident.status
    if incident.extinguished_at is not None:
        db_incident.extinguished_at = incident.extinguished_at
    
    db.commit()
    db.refresh(db_incident)
    
    # Redis 데이터도 업데이트
    redis_key = f"incident:{incident_id}"
    redis_data = redis_client.get(redis_key)
    
    if redis_data:
        incident_data = json.loads(redis_data)
        
        if incident.status is not None:
            incident_data["status"] = incident.status
        if incident.extinguished_at is not None:
            incident_data["extinguished_at"] = incident.extinguished_at.isoformat()
        
        redis_client.set(redis_key, json.dumps(incident_data))
        
        # 상태 변경을 MQTT로 발행
        mqtt_client.publish(f"incidents/{incident_id}/status", json.dumps(incident_data))
    
    return db_incident
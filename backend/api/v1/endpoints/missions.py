from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import uuid
from datetime import datetime

from db.database import get_db
from models.mission import FirefightingMission
from models.position import RobotPosition
from schemas.mission import Mission as MissionSchema
from schemas.mission import MissionCreate, MissionUpdate, MissionPath, RobotPathPoint

from sqlalchemy import func

router = APIRouter()

@router.get("/", response_model=List[MissionSchema])
def get_missions(db: Session = Depends(get_db)):
    """
    모든 화재 진압 미션 목록 조회
    """
    missions = db.query(FirefightingMission).all()
    return missions

@router.get("/{mission_id}", response_model=MissionSchema)
def get_mission(mission_id: str, db: Session = Depends(get_db)):
    """
    특정 화재 진압 미션 상세 정보 조회
    """
    mission = db.query(FirefightingMission).filter(FirefightingMission.mission_id == mission_id).first()
    if mission is None:
        raise HTTPException(status_code=404, detail="Mission not found")
    return mission

@router.get("/{mission_id}/path", response_model=MissionPath)
def get_mission_path(mission_id: str, db: Session = Depends(get_db)):
    """
    특정 미션에 대한 로봇 이동 경로 조회
    """
    # 미션 조회
    mission = db.query(FirefightingMission).filter(FirefightingMission.mission_id == mission_id).first()
    if mission is None:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    # 미션 시간 범위 내의 로봇 위치 이력 조회
    query = db.query(RobotPosition).filter(
        RobotPosition.robot_id == mission.robot_id,
        RobotPosition.recorded_at >= mission.assigned_at
    )
    
    # 완료된 미션이면 완료 시간까지만 조회
    if mission.completed_at:
        query = query.filter(RobotPosition.recorded_at <= mission.completed_at)
    
    # 시간순 정렬
    positions = query.order_by(RobotPosition.recorded_at).all()
    
    # 경로 포인트 변환
    path_points = []
    for pos in positions:
        # PostGIS Point 데이터에서 x, y 좌표 추출
        try:
            # Postgis 좌표 추출을 위해 ST_X, ST_Y 함수 사용
            x = db.scalar(func.ST_X(pos.position))
            y = db.scalar(func.ST_Y(pos.position))
            
            path_points.append({
                "x": float(x),
                "y": float(y),
                "orientation": pos.orientation,
                "recorded_at": pos.recorded_at
            })
        except Exception as e:
            print(f"Error extracting coordinates: {e}")
    
    # ORM 객체를 딕셔너리로 변환 - 중요!
    mission_dict = {
        "mission_id": mission.mission_id,
        "robot_id": mission.robot_id,
        "incident_id": mission.incident_id,
        "status": mission.status,
        "assigned_at": mission.assigned_at,
        "arrived_at": mission.arrived_at,
        "completed_at": mission.completed_at
    }
    
    # MissionPath 응답 생성
    return {
        "mission": mission_dict,
        "path_points": path_points
    }


@router.post("/", response_model=MissionSchema)
def create_mission(mission: MissionCreate, db: Session = Depends(get_db)):
    """
    새 화재 진압 미션 생성
    """
    # 미션 ID 생성 (제공되지 않은 경우)
    mission_id = mission.mission_id or f"mission_{uuid.uuid4().hex[:8]}"
    
    # 미션 객체 생성
    db_mission = FirefightingMission(
        mission_id=mission_id,
        robot_id=mission.robot_id,
        incident_id=mission.incident_id,
        status=mission.status
    )
    
    db.add(db_mission)
    db.commit()
    db.refresh(db_mission)
    return db_mission

@router.put("/{mission_id}", response_model=MissionSchema)
def update_mission(mission_id: str, mission_update: MissionUpdate, db: Session = Depends(get_db)):
    """
    화재 진압 미션 상태 업데이트
    """
    db_mission = db.query(FirefightingMission).filter(FirefightingMission.mission_id == mission_id).first()
    if db_mission is None:
        raise HTTPException(status_code=404, detail="Mission not found")
    
    # 변경 사항 업데이트
    if mission_update.status is not None:
        db_mission.status = mission_update.status
    if mission_update.arrived_at is not None:
        db_mission.arrived_at = mission_update.arrived_at
    if mission_update.completed_at is not None:
        db_mission.completed_at = mission_update.completed_at
    
    db.commit()
    db.refresh(db_mission)
    return db_mission
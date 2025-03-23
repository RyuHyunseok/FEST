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
        # ST_AsText 또는 ST_X, ST_Y 함수를 사용하여 추출해야 하지만
        # 여기서는 간단히 처리하기 위해 임시 데이터를 사용
        
        # 실제 구현에서는 아래와 같이 PostGIS 함수를 사용해야 함
        # x = db.scalar(func.ST_X(pos.position))
        # y = db.scalar(func.ST_Y(pos.position))
        
        # 임시 방법: position 객체에서 직접 접근 (실제 PostGIS에서는 다른 방식으로 접근해야 함)
        x, y = 0.0, 0.0
        try:
            # PostgreSQL의 공간 데이터 형식에 따라 접근 방식이 다를 수 있음
            wkb_element = pos.position.desc
            x = float(str(wkb_element).split('(')[1].split(' ')[0])
            y = float(str(wkb_element).split(' ')[1].split(')')[0])
        except (AttributeError, IndexError) as e:
            # 좌표 추출에 실패한 경우 로그 기록
            print(f"Error extracting coordinates: {e}")
        
        path_points.append(RobotPathPoint(
            x=x,
            y=y,
            orientation=pos.orientation,
            recorded_at=pos.recorded_at
        ))
    
    return MissionPath(mission=mission, path_points=path_points)

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
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from typing import List
import json
import redis

from db.database import get_db
from models.robot import Robot
from models.position import RobotPosition
from schemas.robot import Robot as RobotSchema
from schemas.robot import RobotCreate, RobotPosition as RobotPositionSchema, RobotCommand
from infra.mqtt.mqtt_client import mqtt_client
import os

router = APIRouter()

# Redis 클라이언트
redis_client = redis.Redis(
    host=os.getenv('REDIS_HOST', 'localhost'), 
    port=int(os.getenv('REDIS_PORT', 6379)), 
    db=int(os.getenv('REDIS_DB', 0)), 
    decode_responses=True
)

@router.get("/", response_model=List[RobotSchema])
def get_robots(db: Session = Depends(get_db)):
    """
    모든 로봇 목록 조회
    """
    robots = db.query(Robot).all()
    return robots

@router.get("/{robot_id}", response_model=RobotSchema)
def get_robot(robot_id: str, db: Session = Depends(get_db)):
    """
    특정 로봇 정보 조회
    """
    robot = db.query(Robot).filter(Robot.robot_id == robot_id).first()
    if robot is None:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot

@router.get("/{robot_id}/status")
def get_robot_status(robot_id: str):
    """
    로봇 현재 상태 조회 (Redis에서 가져옴)
    """
    # Redis에서 최신 상태 가져오기
    status_key = f"robot:{robot_id}:status"
    latest_status = redis_client.get(status_key)
    if latest_status:
        return json.loads(latest_status)
    else:
        return {"message": "No status data available"}

@router.get("/{robot_id}/position")
def get_robot_position(robot_id: str):
    """
    로봇 현재 위치 조회 (Redis에서 가져옴)
    """
    # Redis에서 최신 위치 가져오기
    position_key = f"robot:{robot_id}:position"
    latest_position = redis_client.get(position_key)
    if latest_position:
        return json.loads(latest_position)
    else:
        return {"message": "No position data available"}

@router.post("/{robot_id}/command")
def send_robot_command(robot_id: str, command: RobotCommand):
    """
    로봇에 명령 전송 (MQTT로 발행)
    """
    # MQTT 토픽 설정
    topic = f"robots/{robot_id}/command"
    
    # 명령 메시지
    message = {
        "type": command.type,
        "target": command.target
    }
    
    # MQTT로 명령 발행
    mqtt_client.publish(topic, json.dumps(message))
    
    return {"message": f"Command sent to robot {robot_id}", "command": message}

@router.post("/", response_model=RobotSchema)
def create_robot(robot: RobotCreate, db: Session = Depends(get_db)):
    """
    새 로봇 등록
    """
    db_robot = db.query(Robot).filter(Robot.robot_id == robot.robot_id).first()
    if db_robot:
        raise HTTPException(status_code=400, detail="Robot ID already registered")
    
    db_robot = Robot(
        robot_id=robot.robot_id,
        name=robot.name
    )
    db.add(db_robot)
    db.commit()
    db.refresh(db_robot)
    return db_robot
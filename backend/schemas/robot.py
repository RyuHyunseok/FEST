from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

# 공통 속성 정의
class RobotBase(BaseModel):
    name: str

# 로봇 생성 시 사용하는 스키마
class RobotCreate(RobotBase):
    robot_id: str

# DB에서 읽어올 때 사용하는 스키마
class Robot(RobotBase):
    robot_id: str
    
    class Config:
        orm_mode = True

# 로봇 위치 스키마 (MQTT로 받는 데이터)
class RobotPosition(BaseModel):
    x: float
    y: float
    orientation: float

# 로봇 상태 스키마 (MQTT로 받는 데이터)
class RobotStatus(BaseModel):
    battery: float
    status: str  # idle, moving, fighting_fire

# 로봇 명령 스키마
class RobotCommand(BaseModel):
    type: str  # move_to, extinguish
    target: dict  # {"x": 25.5, "y": 30.2}
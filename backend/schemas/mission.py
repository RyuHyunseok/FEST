from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime

# 공통 속성 정의
class MissionBase(BaseModel):
    robot_id: str
    incident_id: str
    status: Optional[str] = "assigned"  # assigned, in_progress, completed, failed

# 미션 생성 스키마
class MissionCreate(MissionBase):
    mission_id: Optional[str] = None  # 생성 시 자동 생성될 수 있음

# 미션 업데이트 스키마
class MissionUpdate(BaseModel):
    status: Optional[str] = None
    arrived_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None

# 미션 조회 스키마 (DB에서 가져온 데이터)
class Mission(MissionBase):
    mission_id: str
    assigned_at: datetime
    arrived_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    
    class Config:
        orm_mode = True

# 로봇 경로 정보 스키마
class RobotPathPoint(BaseModel):
    x: float
    y: float
    orientation: float
    recorded_at: datetime

# 미션 경로 응답 스키마
class MissionPath(BaseModel):
    mission: Mission
    path_points: list[RobotPathPoint]
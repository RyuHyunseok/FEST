from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime

# 위치 정보 스키마
class Location(BaseModel):
    x: float
    y: float

# 공통 속성 정의
class IncidentBase(BaseModel):
    location: Location
    # severity: Optional[str] = "medium"  # low, medium, high
    status: Optional[str] = "active"  # active, assigned, extinguished

# 화재 사고 생성 시 사용하는 스키마
class IncidentCreate(IncidentBase):
    incident_id: Optional[str] = None  # 생성 시 자동 생성될 수 있음

# DB에서 읽어올 때 사용하는 스키마
class Incident(IncidentBase):
    incident_id: str
    detected_at: datetime
    extinguished_at: Optional[datetime] = None
    
    class Config:
        orm_mode = True

# 화재 상태 업데이트 스키마
class IncidentUpdate(BaseModel):
    status: Optional[str] = None
    extinguished_at: Optional[datetime] = None
    
    class Config:
        orm_mode = True
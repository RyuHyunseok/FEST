from sqlalchemy import Column, String, DateTime, ForeignKey
from sqlalchemy.sql import func
from db.database import Base

class FirefightingMission(Base):
    __tablename__ = "firefighting_missions"
    
    mission_id = Column(String(50), primary_key=True, index=True)
    robot_id = Column(String(50), ForeignKey("robots.robot_id"))
    incident_id = Column(String(50), ForeignKey("incidents.incident_id"))
    
    # 미션 상태
    status = Column(String(20), default="assigned")  # assigned, in_progress, completed, failed
    
    # 시간 정보
    assigned_at = Column(DateTime, default=func.now())  # 임무 할당 시간
    arrived_at = Column(DateTime, nullable=True)        # 현장 도착 시간
    completed_at = Column(DateTime, nullable=True)      # 임무 완료 시간
    
    def __repr__(self):
        return f"<FirefightingMission(mission_id='{self.mission_id}', robot_id='{self.robot_id}', incident_id='{self.incident_id}', status='{self.status}')>"
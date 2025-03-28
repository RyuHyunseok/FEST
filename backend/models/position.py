from sqlalchemy import Column, Integer, String, Float, DateTime, ForeignKey
from sqlalchemy.sql import func
from geoalchemy2 import Geometry
from db.database import Base

class RobotPosition(Base):
    __tablename__ = "robot_positions"
    
    id = Column(Integer, primary_key=True, index=True)
    robot_id = Column(String(50), ForeignKey("robots.robot_id"))
    position = Column(Geometry("POINT", srid=4326))  # PostGIS 공간 데이터 타입
    
    # 방향 정보만 저장 (x, y는 position에 포함)
    orientation = Column(Float)  # 각도 (도)
    
    recorded_at = Column(DateTime, default=func.now())
    
    def __repr__(self):
        return f"<RobotPosition(robot_id='{self.robot_id}', orientation={self.orientation})>"
from sqlalchemy import Column, String, Integer, Float, DateTime, Boolean
from sqlalchemy.sql import func
from db.database import Base

class Robot(Base):
    __tablename__ = "robots"
    
    robot_id = Column(String(50), primary_key=True, index=True)
    name = Column(String(100))
    
    # 상태 및 배터리는 Redis에 저장하고 DB에는 저장하지 않음
    # 필요한 경우 나중에 이력 모델을 따로 만듦
    
    def __repr__(self):
        return f"<Robot(robot_id='{self.robot_id}', name='{self.name}')>"
from sqlalchemy import Column, String, DateTime
from sqlalchemy.sql import func
from geoalchemy2 import Geometry
from db.database import Base

class Incident(Base):
    __tablename__ = "incidents"
    
    incident_id = Column(String(50), primary_key=True, index=True)
    location = Column(Geometry("POINT", srid=4326))  # PostGIS 공간 데이터 타입
    # severity = Column(String(20), default="medium")  # low, medium, high
    status = Column(String(20), default="active")  # active, assigned, extinguished
    detected_at = Column(DateTime, default=func.now())
    extinguished_at = Column(DateTime, nullable=True)
    
    def __repr__(self):
        return f"<Incident(incident_id='{self.incident_id}', status='{self.status}')>"
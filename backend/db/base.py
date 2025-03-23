# 모든 모델을 여기서 임포트하여 Alembic 등에서 인식하게 함
from db.database import Base

# 모델들 임포트
from models.robot import Robot
from models.incident import Incident
from models.position import RobotPosition
from models.mission import FirefightingMission

# 이 파일은 모든 모델을 참조하기 위해 사용됨
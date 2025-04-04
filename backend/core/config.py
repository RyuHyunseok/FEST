# backend/core/config.py
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경 변수에서 설정 로드
SECRET_KEY = os.getenv("SECRET_KEY")
SYSTEM_PASSWORD = os.getenv("SYSTEM_PASSWORD")
ALGORITHM = os.getenv("ALGORITHM")
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24  # 24시간 유효


# 환경 설정 가져오기
ENVIRONMENT = os.getenv("ENVIRONMENT")
IS_DEVELOPMENT = ENVIRONMENT.lower() == "development"

# 데이터베이스 설정
DB_HOST = "localhost" if IS_DEVELOPMENT else os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_NAME = os.getenv("DB_NAME")
DB_USER = os.getenv("DB_USER")
DB_PASSWORD = os.getenv("DB_PASSWORD")

# Redis 설정
REDIS_HOST = "localhost" if IS_DEVELOPMENT else os.getenv("REDIS_HOST")
REDIS_PORT = int(os.getenv("REDIS_PORT"))
REDIS_DB = int(os.getenv("REDIS_DB"))

# MQTT 설정
MQTT_HOST = "localhost" if IS_DEVELOPMENT else os.getenv("MQTT_HOST")
MQTT_PORT = int(os.getenv("MQTT_PORT"))

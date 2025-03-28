# backend/core/config.py
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경 변수에서 설정 로드
SECRET_KEY = os.getenv("SECRET_KEY", "admin")
SYSTEM_PASSWORD = os.getenv("SYSTEM_PASSWORD", "admin")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24  # 24시간 유효
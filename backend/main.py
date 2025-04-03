from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from api.v1.services.mqtt_service import start_mqtt_listener, stop_mqtt_listener
from infra.websocket.websocket import setup_websocket, websocket_lifespan
from api.v1.routers.routers import api_router  # routers.py 파일에서 직접 import
import signal
import sys
import os

# 종료 시그널 핸들러
def signal_handler(sig, frame):
    print("종료 신호를 받았습니다. 서버를 종료합니다...")
    stop_mqtt_listener()
    sys.exit(0)

# 종료 신호 등록
signal.signal(signal.SIGINT, signal_handler) # Ctrl + c 와 같은 인터럽트 신호
signal.signal(signal.SIGTERM, signal_handler) # 시스템 종료

# 애플리케이션 lifespan 이벤트 핸들러
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 서버 시작 시 실행
    
    print("MQTT Listener 시작")
    start_mqtt_listener()
    
    async with websocket_lifespan(app):
        yield  # 서버 실행 중
    
    # 서버 종료 시 실행
    print("MQTT Listener 종료")
    stop_mqtt_listener()

# FastAPI 앱 생성 + Lifespan 적용
app = FastAPI(
    title="화재 진압 로봇 관제 API",
    description="ROS2 기반 화재 진압 로봇 시스템의 중앙 관제 API",
    version="0.1.0",
    lifespan=lifespan
)

# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "https://j12d106.p.ssafy.io",
                   ],  # Vite 기본 개발 서버 포트
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API 라우터 등록
app.include_router(api_router, prefix="/api/v1")    

@app.get("/")
async def root():
    return {"message": "FastAPI + MQTT + Redis + WebSocket Running!"}

# WebSocket 설정
setup_websocket(app)
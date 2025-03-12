from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from services.mqtt_service import start_mqtt_listener
from websocket import setup_websocket, websocket_lifespan

# 애플리케이션 lifespan 이벤트 핸들러
@asynccontextmanager
async def lifespan(app: FastAPI):
    # 서버 시작 시 실행
    print("Starting MQTT Listener...")
    start_mqtt_listener()
    
    async with websocket_lifespan(app):
        yield  # 서버 실행 중
    
    # 서버 종료 시 실행
    print("Shutting down MQTT Listener...")

# FastAPI 앱 생성 + Lifespan 적용
app = FastAPI(lifespan=lifespan)

# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],  # Vite 기본 개발 서버 포트
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "FastAPI + MQTT + Redis + WebSocket Running!"}

# WebSocket 설정
setup_websocket(app)
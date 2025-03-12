import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import redis
import asyncio
from contextlib import asynccontextmanager

# Redis 클라이언트 설정
redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

# 연결된 WebSocket 클라이언트 관리
class ConnectionManager:
    def __init__(self):
        self.active_connections = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"WebSocket client connected. Total connections: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        print(f"WebSocket client disconnected. Remaining connections: {len(self.active_connections)}")

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                print(f"Error sending to client: {e}")

manager = ConnectionManager()

# 위치 데이터 브로드캐스트 태스크
broadcast_task = None

async def broadcast_position():
    while True:
        try:
            if manager.active_connections:  # 연결된 클라이언트가 있을 때만 실행
                latest_position = redis_client.get("latest_position")
                if latest_position:
                    print(f"Broadcasting position to {len(manager.active_connections)} clients")
                    await manager.broadcast(latest_position)
            await asyncio.sleep(0.5)  # 0.5초마다 업데이트
        except Exception as e:
            print(f"Error in broadcast task: {e}")
            await asyncio.sleep(1)  # 오류 발생시 1초 대기

# FastAPI 앱에 WebSocket 라우터를 설정하는 함수
def setup_websocket(app: FastAPI):
    # WebSocket 엔드포인트 등록
    @app.websocket("/ws/robot")
    async def websocket_endpoint(websocket: WebSocket):
        await manager.connect(websocket)
        
        # 최신 위치 데이터를 즉시 전송
        latest_position = redis_client.get("latest_position")
        if latest_position:
            await websocket.send_text(latest_position)
        
        try:
            # 클라이언트에서 오는 메시지 처리 (필요한 경우)
            while True:
                data = await websocket.receive_text()
                print(f"Received message from client: {data}")
                
        except WebSocketDisconnect:
            manager.disconnect(websocket)
        except Exception as e:
            print(f"Error in WebSocket connection: {e}")
            manager.disconnect(websocket)

# Lifespan 컨텍스트 매니저 생성
@asynccontextmanager
async def websocket_lifespan(app: FastAPI):
    # 서버 시작 시 실행
    global broadcast_task
    broadcast_task = asyncio.create_task(broadcast_position())
    print("WebSocket broadcast task started")
    
    yield  # 서버 실행 중
    
    # 서버 종료 시 실행
    if broadcast_task:
        broadcast_task.cancel()
        print("WebSocket broadcast task cancelled")
import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query, HTTPException, status
import redis
import asyncio
from contextlib import asynccontextmanager

from jose import JWTError, jwt
from core.config import SECRET_KEY, ALGORITHM

import os

# Redis 클라이언트 설정
redis_client = redis.Redis(
    host=os.getenv('REDIS_HOST', 'localhost'), 
    port=int(os.getenv('REDIS_PORT', 6379)), 
    db=int(os.getenv('REDIS_DB', 0)), 
    decode_responses=True
)

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


# WebSocket 토큰 검증 함수
async def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username = payload.get("sub")
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
        return username
    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token"
        )


async def broadcast_robots_data():
    """로봇 데이터(위치, 상태)를 WebSocket으로 브로드캐스트"""
    while True:
        try:
            if manager.active_connections:  # 연결된 클라이언트가 있을 때만 실행
                # Redis에서 로봇 관련 모든 키 가져오기
                robot_position_keys = redis_client.keys("robot:*:position")
                robot_status_keys = redis_client.keys("robot:*:status")
                
                # 로봇 데이터 수집
                robots_data = {}
                
                # 위치 데이터 처리
                for key in robot_position_keys:
                    # 키에서 로봇 ID 추출 (형식: robot:{robot_id}:position)
                    parts = key.split(':')
                    if len(parts) >= 3:
                        robot_id = parts[1]
                        
                        # 해당 로봇 데이터 초기화 (없는 경우)
                        if robot_id not in robots_data:
                            robots_data[robot_id] = {}
                        
                        # 위치 데이터 추가
                        position_data = redis_client.get(key)
                        if position_data:
                            robots_data[robot_id]["position"] = json.loads(position_data)
                
                # 상태 데이터 처리
                for key in robot_status_keys:
                    # 키에서 로봇 ID 추출 (형식: robot:{robot_id}:status)
                    parts = key.split(':')
                    if len(parts) >= 3:
                        robot_id = parts[1]
                        
                        # 해당 로봇 데이터 초기화 (없는 경우)
                        if robot_id not in robots_data:
                            robots_data[robot_id] = {}
                        
                        # 상태 데이터 추가
                        status_data = redis_client.get(key)
                        if status_data:
                            robots_data[robot_id]["status"] = json.loads(status_data)

                mission_status_keys = redis_client.keys("robot:*:mission_status")
                for key in mission_status_keys:
                    parts = key.split(':')
                    if len(parts) >= 3:
                        robot_id = parts[1]
                        if robot_id not in robots_data:
                            robots_data[robot_id] = {}
                        
                        # 미션 상태 추가
                        mission_status = redis_client.get(key)
                        if mission_status:
                            # 상태가 아직 없으면 초기화
                            if "status" not in robots_data[robot_id]:
                                robots_data[robot_id]["status"] = {}
                            
                            # mission_status 필드 추가
                            robots_data[robot_id]["mission_status"] = mission_status

                
                # 화재 데이터 처리
                incident_keys = redis_client.keys("incident:*")
                incidents_data = {}
                
                for key in incident_keys:
                    # 키에서 화재 ID 추출 (형식: incident:{incident_id})
                    parts = key.split(':')
                    if len(parts) >= 2:
                        incident_id = parts[1]
                        
                        # 화재 데이터 추가
                        incident_data = redis_client.get(key)
                        if incident_data:
                            incidents_data[incident_id] = json.loads(incident_data)

                # Redis에서 침입자 관련 키 가져오기
                prowler_keys = redis_client.keys("prowler:*")
                prowlers_data = {}

                # 침입자 데이터 처리
                for key in prowler_keys:
                    # 키에서 침입자 ID 추출 (형식: prowler:{prowler_id})
                    parts = key.split(':')
                    if len(parts) >= 2:
                        prowler_id = parts[1]
                        
                        # 침입자 데이터 추가
                        prowler_data = redis_client.get(key)
                        if prowler_data:
                            prowlers_data[prowler_id] = json.loads(prowler_data)

                # 로봇 데이터와 화재 데이터를 별도의 메시지로 전송
                if robots_data:
                    robots_message = {
                        "robots": robots_data,
                        "prowlers": prowlers_data  # 항상 prowlers 필드 포함
                    }

                    # JSON 으로 직렬화하여 전송
                    message_json = json.dumps(robots_message)

                    for connection in manager.active_connections:
                        try:
                            # 경로를 확인하여 적절한 클라이언트에게만 전송
                            if hasattr(connection, 'url') and '/ws/robots' in str(connection.url):
                                await connection.send_text(message_json)
                        except Exception as e:
                            print(f"Error sending to robots client: {e}")
                
                if incidents_data:
                    incidents_message = json.dumps({"incidents": incidents_data})
                    for connection in manager.active_connections:
                        try:
                            # 경로를 확인하여 적절한 클라이언트에게만 전송
                            if hasattr(connection, 'url') and '/ws/incidents' in str(connection.url):
                                await connection.send_text(incidents_message)
                        except Exception as e:
                            print(f"Error sending to incidents client: {e}")

            await asyncio.sleep(0.05)  # 0.05초마다 업데이트
        
        except Exception as e:
            print(f"Error in broadcast task: {e}")
            await asyncio.sleep(1)  # 오류 발생시 1초 대기

# FastAPI 앱에 WebSocket 라우터를 설정하는 함수
def setup_websocket(app: FastAPI):
    # 로봇 데이터를 위한 WebSocket 엔드포인트
    @app.websocket("/ws/robots")
    async def robots_websocket_endpoint(
        websocket: WebSocket,
        token: str = Query(...)
    ):
        try:
            # 토큰 검증
            await verify_token(token)

            # 토큰이 유효한 경우 연결 수락락    
            await manager.connect(websocket)
        
            try:
                # 클라이언트에서 오는 메시지 처리 (필요한 경우)
                while True:
                    data = await websocket.receive_text()
                    print(f"Received message from client (robots): {data}")
                    
            except WebSocketDisconnect:
                manager.disconnect(websocket)
            except Exception as e:
                print(f"Error in WebSocket connection: {e}")
                manager.disconnect(websocket)

        except HTTPException as e:
            # 토큰 검증 실패 시 연결 거부
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION, reason=e.detail)

    # 화재 데이터를 위한 WebSocket 엔드포인트(인증 추가)
    @app.websocket("/ws/incidents")
    async def incidents_websocket_endpoint(
        websocket: WebSocket,
        token: str = Query(...) # 쿼리 파라미터로 토큰 받기
    ):
        
        try:
            # 토큰 검증
            await verify_token(token)

            # 토큰이 유효한 경우 연결 수락
            await manager.connect(websocket)
        
            try:
                # 클라이언트에서 오는 메시지 처리 (필요한 경우)
                while True:
                    data = await websocket.receive_text()
                    print(f"Received message from client (incidents): {data}")
                    
            except WebSocketDisconnect:
                manager.disconnect(websocket)
            except Exception as e:
                print(f"Error in WebSocket connection: {e}")
                manager.disconnect(websocket)

        except HTTPException as e:
            # 토큰 검증 실패 시 연결 거부
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION, reason=e.detail)

# Lifespan 컨텍스트 매니저 생성
@asynccontextmanager
async def websocket_lifespan(app: FastAPI):
    # 서버 시작 시 실행
    global broadcast_task
    broadcast_task = asyncio.create_task(broadcast_robots_data())
    print("WebSocket broadcast task started")
    
    yield  # 서버 실행 중
    
    # 서버 종료 시 실행
    if broadcast_task:
        broadcast_task.cancel()
        print("WebSocket broadcast task cancelled")
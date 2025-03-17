from fastapi import APIRouter
from api.v1.endpoints import robots, incidents

api_router = APIRouter()

# 로봇 관련 API 라우터
api_router.include_router(
    robots.router,
    prefix="/robots",
    tags=["robots"]
)

# 화재 사고 관련 API 라우터
api_router.include_router(
    incidents.router,
    prefix="/incidents",
    tags=["incidents"]
)
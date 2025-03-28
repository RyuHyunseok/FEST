from fastapi import APIRouter, Depends
from api.v1.endpoints import robots, incidents, auth, missions

from core.security import get_current_user


api_router = APIRouter()

# 인증 관련 API 라우터 (인증 없이 접근 가능)
api_router.include_router(
    auth.router,
    prefix="/auth",
    tags=["auth"]
)

# 보호된 API 라우터들
# 모든 엔드포인트에 인증 의존성 추가

secured_router = APIRouter(dependencies=[Depends(get_current_user)])


# 로봇 관련 API 라우터
secured_router.include_router(
    robots.router,
    prefix="/robots",
    tags=["robots"]
)

# 화재 사고 관련 API 라우터
secured_router.include_router(
    incidents.router,
    prefix="/incidents",
    tags=["incidents"]
)

# 화재 진압 미션 관련 API 라우터 추가
secured_router.include_router(
    missions.router,
    prefix="/missions",
    tags=["missions"]
)

# 보호된 라우터를 메인 라우터에 포함
api_router.include_router(secured_router)
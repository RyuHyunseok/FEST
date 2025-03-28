# backend/core/security.py
from datetime import datetime, timedelta
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import JWTError, jwt
from passlib.context import CryptContext
from pydantic import BaseModel
from core.config import SECRET_KEY, SYSTEM_PASSWORD, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES

# 비밀번호 해싱 도구
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# 토큰 모델
class Token(BaseModel):
    access_token: str
    token_type: str

# 비밀번호 인증 함수
def verify_password(plain_password, system_password=SYSTEM_PASSWORD):
    # 간단한 인증을 위해 직접 비교
    return plain_password == system_password

# JWT 토큰 생성
def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

# 토큰 검증 의존성
security = HTTPBearer()

def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """
    인증된 사용자 확인 함수
    
    이 함수는 요청의 Authorization 헤더에서 JWT 토큰을 검증하고,
    토큰이 유효한 경우 사용자 식별자를 반환합니다.
    
    Args:
        credentials: HTTP 요청의 Authorization 헤더에서 추출된 Bearer 토큰
                    (HTTPBearer 의존성에 의해 자동으로 제공됨)
    
    Returns:
        유효한 토큰의 경우 사용자 식별자(username) 반환
        
    Raises:
        HTTPException: 토큰이 유효하지 않거나 만료된 경우 401 Unauthorized 오류 발생
    """
    try:
        # JWT 토큰 복호화 및 검증
        # SECRET_KEY로 서명 확인, ALGORITHM으로 암호화 알고리즘 지정
        payload = jwt.decode(credentials.credentials, SECRET_KEY, algorithms=[ALGORITHM])
        
        # 토큰의 'sub' 필드에서 사용자 식별자 추출
        # 여기서는 단순 비밀번호 인증만 사용하므로 실제 사용자 계정은 없음
        # 'sub' 필드는 JWT 표준을 따르기 위해 포함되며, 로그인 시 'admin'으로 설정됨
        username = payload.get("sub")
        
        # 'sub' 필드가 없는 경우 토큰이 올바르게 생성되지 않았다고 판단
        if username is None:
            raise HTTPException(
                status_code=401, 
                detail="Invalid authentication credentials"
            )
            
        # 유효한 토큰인 경우 사용자 식별자 반환
        # 이 값은 라우트 핸들러에서 현재 인증된 사용자를 식별하는 데 사용될 수 있음
        return username
        
    except JWTError:
        # 토큰 복호화 또는 검증 실패 시 (형식 오류, 만료, 잘못된 서명 등)
        # 401 Unauthorized 응답 반환
        raise HTTPException(
            status_code=401, 
            detail="Invalid authentication credentials"
        )
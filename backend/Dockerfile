# backend/Dockerfile
FROM python:3.11-slim

# 작업 디렉토리 설정
WORKDIR /app

# 시스템 의존성 설치 (선택사항, PostGIS 등을 위해)
RUN apt-get update && apt-get install -y \
    build-essential \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Python 환경 설정
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# 필요한 Python 패키지 설치를 위한 requirements.txt 파일 복사
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 애플리케이션 소스 코드 복사
COPY . .

# 애플리케이션 포트 노출
EXPOSE 8000

# 컨테이너 실행 시 실행될 명령어
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
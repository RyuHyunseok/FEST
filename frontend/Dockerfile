# 빌드 단계
FROM node:22-alpine as build

WORKDIR /app

# 의존성 파일 복사 및 설치
COPY package*.json ./
RUN npm install

# 전체 소스 코드 복사
COPY . .

# 프로덕션 빌드 수행
RUN npm run build

# 실행 단계
FROM nginx:alpine

# NGINX 설정 파일 복사
COPY --from=build /app/dist /usr/share/nginx/html
COPY nginx/conf.d/default.conf /etc/nginx/conf.d/default.conf

# 포트 노출
EXPOSE 80

# NGINX 실행
CMD ["nginx", "-g", "daemon off;"]
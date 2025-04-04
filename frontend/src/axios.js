import axios from 'axios'
import router from './router'

// API 기본 URL 설정 (백엔드 서버 주소에 맞게 변경)
<<<<<<< HEAD
axios.defaults.baseURL = 'http://localhost:8000'
=======
// axios.defaults.baseURL = 'http://localhost:8000'
// axios.defaults.baseURL = 'https://j12d106.p.ssafy.io/api/v1'

const IS_PRODUCTION = import.meta.env.VITE_ENVIRONMENT === 'production';
const AXIOS_URL = IS_PRODUCTION
? 'https://j12d106.p.ssafy.io/api/v1' 
: 'http://localhost:8000';

axios.defaults.baseURL = AXIOS_URL;
>>>>>>> origin/develop

// 요청 인터셉터 - 토큰 포함
axios.interceptors.request.use(
  config => {
    const token = localStorage.getItem('authToken')
    
    if (token) {
      config.headers.Authorization = `Bearer ${token}`
    }
    
    return config
  },
  error => {
    return Promise.reject(error)
  }
)

// 응답 인터셉터 - 인증 에러 처리
axios.interceptors.response.use(
  response => response,
  error => {
    // 401 에러 처리 (토큰 만료 또는 인증 실패)
    if (error.response && error.response.status === 401) {
      console.log('인증 실패, 로그인 페이지로 이동합니다.')
      localStorage.removeItem('authToken')
      router.push('/login')
    }
    
    return Promise.reject(error)
  }
)

export default axios
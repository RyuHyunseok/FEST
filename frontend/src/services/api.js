import axios from 'axios';

// API 기본 URL 설정
// const API_URL = 'http://localhost:8000/api/v1';
const API_URL = 'http://j12d106.p.ssafy.io/api/v1';

// 토큰 가져오기
const getToken = () => localStorage.getItem('authToken');

// Axios 인스턴스 생성
const apiClient = axios.create({
  baseURL: API_URL
});

// 요청 인터셉터 - 토큰 포함
apiClient.interceptors.request.use(
  config => {
    const token = getToken();
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  error => Promise.reject(error)
);

// 응답 인터셉터 - 인증 에러 처리
apiClient.interceptors.response.use(
  response => response,
  error => {
    if (error.response && error.response.status === 401) {
      // 인증 실패 시 로그인 페이지로 이동
      console.log('API 인증 실패, 로그인 페이지로 이동합니다');
      localStorage.removeItem('authToken');
      window.location.href = '/login';
    }
    return Promise.reject(error);
  }
);

// Auth API
export const authService = {
  login(password) {
    const formData = new FormData();
    formData.append('username', 'admin'); // OAuth2 스펙상 필요
    formData.append('password', password);
    
    return apiClient.post('/auth/token', formData);
  }
};

// Robots API
export const robotService = {
  getAllRobots() {
    return apiClient.get('/robots');
  },
  
  getRobotById(robotId) {
    return apiClient.get(`/robots/${robotId}`);
  },
  
  getRobotStatus(robotId) {
    return apiClient.get(`/robots/${robotId}/status`);
  },
  
  getRobotPosition(robotId) {
    return apiClient.get(`/robots/${robotId}/position`);
  },
  
  sendCommand(robotId, command) {
    return apiClient.post(`/robots/${robotId}/command`, command);
  }
};

// Incidents API
export const incidentService = {
    getAllIncidents() {
      return apiClient.get('/incidents');
    },
    
    getActiveIncidents() {
      return apiClient.get('/incidents/active');
    },
    
    getIncidentById(incidentId) {
      return apiClient.get(`/incidents/${incidentId}`);
    },
    
    updateIncident(incidentId, data) {
      return apiClient.put(`/incidents/${incidentId}`, data);
    }
  };

  // Missions API
export const missionService = {
    getAllMissions() {
      return apiClient.get('/missions');
    },
    
    getMissionById(missionId) {
      return apiClient.get(`/missions/${missionId}`);
    },
    
    getMissionPath(missionId) {
      return apiClient.get(`/missions/${missionId}/path`);
    },
    
    createMission(data) {
      return apiClient.post('/missions', data);
    },
    
    updateMission(missionId, data) {
      return apiClient.put(`/missions/${missionId}`, data);
    }
  };
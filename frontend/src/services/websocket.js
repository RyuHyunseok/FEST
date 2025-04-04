// WebSocket 서비스

// WebSocket 서버 URL
<<<<<<< HEAD
const WS_BASE_URL = 'ws://localhost:8000';
=======
// const WS_BASE_URL = 'ws://localhost:8000';
// const WS_BASE_URL = 'wss://j12d106.p.ssafy.io';

const IS_PRODUCTION = import.meta.env.VITE_ENVIRONMENT === 'production';
const WS_BASE_URL = IS_PRODUCTION 
  ? 'wss://j12d106.p.ssafy.io' 
  : 'ws://localhost:8000';
>>>>>>> origin/develop

class WebSocketService {
  constructor() {
    this.robotsSocket = null;
    this.incidentsSocket = null;
    this.connected = {
      robots: false,
      incidents: false
    };
    this.callbacks = {
      robots: [],
      incidents: [],
      connection: []
    };
    this.isLoggedOut = false;
  }

  // 로봇 WebSocket 연결
  connectRobots(token) {
    if (this.isLoggedOut || !token) return;
    
    if (this.robotsSocket) {
      this.robotsSocket.close();
    }
    
    this.robotsSocket = new WebSocket(`${WS_BASE_URL}/ws/robots?token=${token}`);
    
    this.robotsSocket.onopen = () => {
      this.connected.robots = true;
      console.log('Robots WebSocket connected');
      this._notifyConnectionStatus();
    };
    
    this.robotsSocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        this._notifyRobotsCallbacks(data);
      } catch (error) {
        console.error('Error parsing robot message:', error);
      }
    };
    
    this.robotsSocket.onclose = () => {
      this.connected.robots = false;
      console.log('Robots WebSocket disconnected');
      this._notifyConnectionStatus();
      
      // 로그아웃 상태가 아닌 경우에만 재연결 시도
      if (!this.isLoggedOut && token) {
        console.log('Attempting to reconnect robots WebSocket...');
        setTimeout(() => this.connectRobots(token), 3000);
      }
    };
    
    this.robotsSocket.onerror = (error) => {
      console.error('Robots WebSocket error:', error);
      this.connected.robots = false;
      this._notifyConnectionStatus();
    };
  }

  // 화재 WebSocket 연결
  connectIncidents(token) {
    if (this.isLoggedOut || !token) return;
    
    if (this.incidentsSocket) {
      this.incidentsSocket.close();
    }
    
    this.incidentsSocket = new WebSocket(`${WS_BASE_URL}/ws/incidents?token=${token}`);
    
    this.incidentsSocket.onopen = () => {
      this.connected.incidents = true;
      console.log('Incidents WebSocket connected');
      this._notifyConnectionStatus();
    };
    
    this.incidentsSocket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        this._notifyIncidentsCallbacks(data);
      } catch (error) {
        console.error('Error parsing incident message:', error);
      }
    };
    
    this.incidentsSocket.onclose = () => {
      this.connected.incidents = false;
      console.log('Incidents WebSocket disconnected');
      this._notifyConnectionStatus();
      
      // 로그아웃 상태가 아닌 경우에만 재연결 시도
      if (!this.isLoggedOut && token) {
        console.log('Attempting to reconnect incidents WebSocket...');
        setTimeout(() => this.connectIncidents(token), 3000);
      }
    };
    
    this.incidentsSocket.onerror = (error) => {
      console.error('Incidents WebSocket error:', error);
      this.connected.incidents = false;
      this._notifyConnectionStatus();
    };
  }

  // 모든 WebSocket 연결
  connect() {
    const token = localStorage.getItem('authToken');
    console.log('WebSocket 연결 시작, 토큰 존재:', !!token);
    
    if (!token) return;
    
    this.isLoggedOut = false;
    
    // 이미 연결되었거나 연결 중인 경우 중복 연결 방지
    if (this.robotsSocket && (this.robotsSocket.readyState === 0 || this.robotsSocket.readyState === 1)) {
      console.log('로봇 WebSocket이 이미 연결됨 또는 연결 중');
    } else {
      console.log('로봇 WebSocket 새 연결 시작');
      this.connectRobots(token);
    }
    
    if (this.incidentsSocket && (this.incidentsSocket.readyState === 0 || this.incidentsSocket.readyState === 1)) {
      console.log('화재 WebSocket이 이미 연결됨 또는 연결 중');
    } else {
      console.log('화재 WebSocket 새 연결 시작');
      this.connectIncidents(token);
    }
  }

  // 모든 WebSocket 연결 종료
  disconnect() {
    console.log('WebSocket 연결 종료 시작');
    this.isLoggedOut = true;
    
    if (this.robotsSocket) {
      console.log('로봇 WebSocket 연결 종료 중...');
      this.robotsSocket.close();
      this.robotsSocket = null;
    }
    
    if (this.incidentsSocket) {
      console.log('화재 WebSocket 연결 종료 중...');
      this.incidentsSocket.close();
      this.incidentsSocket = null;
    }
    
    this.connected.robots = false;
    this.connected.incidents = false;
    this._notifyConnectionStatus();
    console.log('모든 WebSocket 연결 종료 완료');
  }

  // 연결 상태 확인
  isConnected() {
    return {
      robots: this.connected.robots,
      incidents: this.connected.incidents
    };
  }

  // 로봇 데이터 콜백 등록
  onRobotsData(callback) {
    this.callbacks.robots.push(callback);
    return () => {
      this.callbacks.robots = this.callbacks.robots.filter(cb => cb !== callback);
    };
  }

  // 화재 데이터 콜백 등록
  onIncidentsData(callback) {
    this.callbacks.incidents.push(callback);
    return () => {
      this.callbacks.incidents = this.callbacks.incidents.filter(cb => cb !== callback);
    };
  }

  // 연결 상태 변경 콜백 등록
  onConnectionChange(callback) {
    this.callbacks.connection.push(callback);
    callback(this.connected);
    return () => {
      this.callbacks.connection = this.callbacks.connection.filter(cb => cb !== callback);
    };
  }

  // 내부 메서드: 로봇 데이터 콜백 호출
  _notifyRobotsCallbacks(data) {
    this.callbacks.robots.forEach(callback => callback(data));
  }

  // 내부 메서드: 화재 데이터 콜백 호출
  _notifyIncidentsCallbacks(data) {
    this.callbacks.incidents.forEach(callback => callback(data));
  }

  // 내부 메서드: 연결 상태 변경 콜백 호출
  _notifyConnectionStatus() {
    this.callbacks.connection.forEach(callback => callback(this.connected));
  }
}

// 싱글톤 인스턴스 생성하여 내보내기
const wsService = new WebSocketService();
export default wsService;
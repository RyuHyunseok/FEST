<template>
  <v-container fluid>
    <v-card class="mx-auto my-4" max-width="800">
      <v-card-title class="d-flex justify-space-between align-center">
        <span>로봇 모니터링</span>
        <v-btn color="error" @click="logout">로그아웃</v-btn>
      </v-card-title>
      
      <v-card-text>
        <!-- WebSocket 연결 상태 -->
        <v-alert v-if="!robotsConnected" type="error" density="compact" class="mb-2">
          <v-icon start>mdi-wifi-off</v-icon>
          로봇 데이터 WebSocket 연결이 끊어졌습니다. 재연결 중...
        </v-alert>
        <v-alert v-else type="success" density="compact" class="mb-2">
          <v-icon start>mdi-wifi</v-icon>
          로봇 실시간 데이터 수신 중
        </v-alert>
        
        <v-alert v-if="!incidentsConnected" type="error" density="compact" class="mb-2">
          <v-icon start>mdi-wifi-off</v-icon>
          화재 데이터 WebSocket 연결이 끊어졌습니다. 재연결 중...
        </v-alert>
        <v-alert v-else type="success" density="compact" class="mb-2">
          <v-icon start>mdi-wifi</v-icon>
          화재 실시간 데이터 수신 중
        </v-alert>
        
        <!-- 화재 알림 -->
        <v-alert v-if="activeIncidents.length > 0" type="warning" density="compact" class="mb-2">
          <v-icon start>mdi-fire-alert</v-icon>
          현재 {{ activeIncidents.length }}건의 화재가 진행 중입니다!
        </v-alert>
        
        <v-divider class="my-4"></v-divider>
        
        <!-- 화재 목록 -->
        <v-expansion-panels v-if="activeIncidents.length > 0">
          <v-expansion-panel v-for="incident in activeIncidents" :key="incident.incident_id">
            <v-expansion-panel-title>
              <v-icon color="error" class="mr-2">mdi-fire</v-icon>
              화재 ID: {{ incident.incident_id }} (상태: {{ incident.status }})
            </v-expansion-panel-title>
            <v-expansion-panel-text>
              <p>위치: X {{ incident.location.x }}, Y {{ incident.location.y }}</p>
              <p>발생 시간: {{ formatTime(incident.detected_at) }}</p>
              <p v-if="incident.extinguished_at">진압 시간: {{ formatTime(incident.extinguished_at) }}</p>
            </v-expansion-panel-text>
          </v-expansion-panel>
        </v-expansion-panels>
        
        <p v-else class="text-center my-4">현재 활성화된 화재가 없습니다.</p>
      </v-card-text>
    </v-card>
  </v-container>
</template>

<script>
export default {
  name: 'RobotMonitoring',
  data() {
    return {
      robotsWs: null,
      incidentsWs: null,
      robotsConnected: false,
      incidentsConnected: false,
      isLoggedOut: false, // 로그아웃 상태 추적을 위한 플래그
      robots: {}, // 로봇 데이터
      incidents: {}, // 화재 데이터
      activeIncidents: [] // 활성 화재 목록
    };
  },
  computed: {
    token() {
      return localStorage.getItem('authToken');
    }
  },
  mounted() {
    // 토큰이 없으면 로그인 페이지로 리다이렉트
    if (!this.token) {
      this.$router.push('/login');
      return;
    }
    
    this.isLoggedOut = false; // 컴포넌트 마운트 시 로그아웃 상태 초기화
    this.setupRobotsWebSocket();
    this.setupIncidentsWebSocket();
  },
  beforeDestroy() {
    this.isLoggedOut = true; // 컴포넌트 소멸 시 로그아웃 상태로 설정
    this.closeWebSockets();
  },
  methods: {
    setupRobotsWebSocket() {
      // 로그아웃 상태면 WebSocket 연결 시도하지 않음
      if (this.isLoggedOut || !this.token) {
        return;
      }
      
      // 기존 WebSocket 정리
      if (this.robotsWs) {
        this.robotsWs.close();
      }
      
      // 로봇 데이터용 WebSocket
      this.robotsWs = new WebSocket(`ws://localhost:8000/ws/robots?token=${this.token}`);
      
      this.robotsWs.onopen = () => {
        this.robotsConnected = true;
        console.log('로봇 데이터 WebSocket 연결됨');
      };
      
      this.robotsWs.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.robots = data.robots || {};
          console.log('로봇 데이터 수신:', data);
        } catch (error) {
          console.error('로봇 메시지 파싱 오류:', error);
        }
      };
      
      this.robotsWs.onclose = () => {
        this.robotsConnected = false;
        console.log('로봇 데이터 WebSocket 연결 끊김');
        
        // 로그아웃 상태가 아닌 경우에만 재연결 시도
        if (!this.isLoggedOut && this.token) {
          console.log('로봇 데이터 WebSocket 재연결 시도 중...');
          setTimeout(this.setupRobotsWebSocket, 3000);
        }
      };
      
      this.robotsWs.onerror = (error) => {
        console.error('로봇 데이터 WebSocket 오류:', error);
        this.robotsConnected = false;
      };
    },
    
    setupIncidentsWebSocket() {
      // 로그아웃 상태면 WebSocket 연결 시도하지 않음
      if (this.isLoggedOut || !this.token) {
        return;
      }
      
      // 기존 WebSocket 정리
      if (this.incidentsWs) {
        this.incidentsWs.close();
      }
      
      // 화재 데이터용 WebSocket
      this.incidentsWs = new WebSocket(`ws://localhost:8000/ws/incidents?token=${this.token}`);
      
      this.incidentsWs.onopen = () => {
        this.incidentsConnected = true;
        console.log('화재 데이터 WebSocket 연결됨');
      };
      
      this.incidentsWs.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.incidents = data.incidents || {};
          
          // 화재 알림 처리
          this.processIncidents();
          
          console.log('화재 데이터 수신:', data);
        } catch (error) {
          console.error('화재 메시지 파싱 오류:', error);
        }
      };
      
      this.incidentsWs.onclose = () => {
        this.incidentsConnected = false;
        console.log('화재 데이터 WebSocket 연결 끊김');
        
        // 로그아웃 상태가 아닌 경우에만 재연결 시도
        if (!this.isLoggedOut && this.token) {
          console.log('화재 데이터 WebSocket 재연결 시도 중...');
          setTimeout(this.setupIncidentsWebSocket, 3000);
        }
      };
      
      this.incidentsWs.onerror = (error) => {
        console.error('화재 데이터 WebSocket 오류:', error);
        this.incidentsConnected = false;
      };
    },
    
    closeWebSockets() {
      // 로봇 WebSocket 종료
      if (this.robotsWs) {
        if (this.robotsWs.readyState === WebSocket.OPEN || 
            this.robotsWs.readyState === WebSocket.CONNECTING) {
          this.robotsWs.close();
        }
        this.robotsWs = null;
      }
      
      // 화재 WebSocket 종료
      if (this.incidentsWs) {
        if (this.incidentsWs.readyState === WebSocket.OPEN || 
            this.incidentsWs.readyState === WebSocket.CONNECTING) {
          this.incidentsWs.close();
        }
        this.incidentsWs = null;
      }
    },
    
    processIncidents() {
      // 활성 화재만 필터링
      const activeList = [];
      
      for (const [id, incident] of Object.entries(this.incidents)) {
        if (incident.status === 'active') {
          // 새 화재 발생 알림 (기존에 없던 화재)
          const wasAlreadyActive = this.activeIncidents.some(inc => inc.incident_id === id);
          if (!wasAlreadyActive) {
            this.showFireAlert(incident);
          }
          
          // 목록에 추가
          activeList.push({
            incident_id: id,
            ...incident
          });
        }
      }
      
      // 활성 화재 목록 업데이트
      this.activeIncidents = activeList;
    },
    
    showFireAlert(incident) {
      // 콘솔에 화재 알림 출력
      console.warn('🔥 화재 발생 알림 🔥');
      console.warn(`화재 ID: ${incident.incident_id}`);
      console.warn(`위치: X ${incident.location.x}, Y ${incident.location.y}`);
      console.warn(`발생 시간: ${new Date(incident.detected_at).toLocaleString()}`);
      
      // 실제 프로덕션에서는 여기에 소리, 알림 등을 추가할 수 있음
    },
    
    formatTime(timestamp) {
      // 타임스탬프가 문자열이면 그대로 반환
      if (typeof timestamp === 'string') {
        return timestamp;
      }
      
      // 타임스탬프가 밀리초 단위인 경우
      if (timestamp > 1000000000000) {
        timestamp = timestamp / 1000;
      }
      
      // 타임스탬프를 날짜로 변환
      try {
        return new Date(timestamp).toLocaleString();
      } catch (e) {
        return '알 수 없는 시간';
      }
    },
    
    logout() {
      // 로그아웃 상태로 설정
      this.isLoggedOut = true;
      
      // WebSocket 연결 종료
      this.closeWebSockets();
      
      // 토큰 삭제
      localStorage.removeItem('authToken');
      
      // 로그인 페이지로 리다이렉트
      this.$router.push('/login');
    }
  }
};
</script>
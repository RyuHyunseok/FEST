<template>
  <v-container fluid>
    <v-card class="mx-auto my-4" max-width="800">
      <v-card-title class="d-flex justify-space-between align-center">
        <span>로봇 모니터링</span>
        <v-btn color="error" @click="logout">로그아웃</v-btn>
      </v-card-title>
      
      <v-card-text>
        <!-- WebSocket 연결 상태 -->
        <v-alert v-if="!connected" type="error" density="compact" class="mb-4">
          <v-icon start>mdi-wifi-off</v-icon>
          WebSocket 연결이 끊어졌습니다. 재연결 중...
        </v-alert>
        <v-alert v-else type="success" density="compact" class="mb-4">
          <v-icon start>mdi-wifi</v-icon>
          실시간 데이터 수신 중
        </v-alert>
        
        <v-divider class="my-4"></v-divider>
        
        <!-- 간단한 메시지 표시 -->
        <p class="text-center text-body-1 my-8">
          WebSocket 연결 테스트 페이지입니다.<br>
          연결 상태: <strong>{{ connected ? '연결됨' : '연결 끊김' }}</strong>
        </p>
      </v-card-text>
    </v-card>
  </v-container>
</template>

<script>
export default {
  name: 'RobotMonitoring',
  data() {
    return {
      ws: null,
      connected: false,
      isLoggedOut: false // 로그아웃 상태 추적을 위한 플래그 추가
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
    this.setupWebSocket();
  },
  beforeDestroy() {
    this.isLoggedOut = true; // 컴포넌트 소멸 시 로그아웃 상태로 설정
    this.closeWebSocket();
  },
  methods: {
    setupWebSocket() {
      // 로그아웃 상태면 WebSocket 연결 시도하지 않음
      if (this.isLoggedOut || !this.token) {
        return;
      }
      
      // 기존 WebSocket 정리
      this.closeWebSocket();
      
      // 로봇 데이터용 WebSocket
      this.ws = new WebSocket(`ws://localhost:8000/ws/robots?token=${this.token}`);
      
      this.ws.onopen = () => {
        this.connected = true;
        console.log('WebSocket 연결됨');
      };
      
      this.ws.onmessage = (event) => {
        try {
          console.log('데이터 수신:', event.data);
          // 필요한 경우 여기서 데이터 처리
        } catch (error) {
          console.error('메시지 파싱 오류:', error);
        }
      };
      
      this.ws.onclose = () => {
        this.connected = false;
        console.log('WebSocket 연결 끊김');
        
        // 로그아웃 상태가 아닌 경우에만 재연결 시도
        if (!this.isLoggedOut && this.token) {
          console.log('재연결 시도 중...');
          setTimeout(this.setupWebSocket, 3000);
        }
      };
      
      this.ws.onerror = (error) => {
        console.error('WebSocket 오류:', error);
        this.connected = false;
      };
    },
    closeWebSocket() {
      if (this.ws) {
        // WebSocket 연결 상태에 따른 처리
        if (this.ws.readyState === WebSocket.OPEN || 
            this.ws.readyState === WebSocket.CONNECTING) {
          this.ws.close();
        }
        this.ws = null;
      }
    },
    logout() {
      // 로그아웃 상태로 설정
      this.isLoggedOut = true;
      
      // WebSocket 연결 종료
      this.closeWebSocket();
      
      // 토큰 삭제
      localStorage.removeItem('authToken');
      
      // 로그인 페이지로 리다이렉트
      this.$router.push('/login');
    }
  }
};
</script>
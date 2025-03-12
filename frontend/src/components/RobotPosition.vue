<template>
  <v-container>
    <v-card class="mx-auto my-4" max-width="600">
      <v-card-title class="text-h5">
        로봇 위치 모니터링
      </v-card-title>

      <v-card-text>
        <div v-if="connected" class="text-subtitle-1 green--text">
          <v-icon color="green">mdi-wifi</v-icon> 연결됨: 실시간 위치 업데이트 수신 중
        </div>
        <div v-else class="text-subtitle-1 red--text">
          <v-icon color="red">mdi-wifi-off</v-icon> 연결 끊김: 서버에 연결 중...
        </div>

        <v-divider class="my-4"></v-divider>
        
        <div class="text-body-2 mt-2">
          unity 맵
        </div>
      </v-card-text>
    </v-card>
  </v-container>
</template>

<script>
import { ref, reactive, onMounted, onUnmounted } from 'vue';

export default {
  name: 'RobotPosition',

  setup() {
    const ws = ref(null);
    const connected = ref(false);
    const position = reactive({
      x: 0,
      y: 0,
      orientation: 0
    });

    // WebSocket 연결 설정
    const setupWebSocket = () => {
      console.log('표준 WebSocket 연결 시도 중...');
      ws.value = new WebSocket('ws://localhost:8000/ws/robot');
      
      ws.value.onopen = () => {
        connected.value = true;
        console.log('WebSocket 연결됨');
      };
      
      ws.value.onclose = () => {
        connected.value = false;
        console.log('WebSocket 연결 끊김, 재연결 시도 중...');
        // 재연결 시도
        setTimeout(setupWebSocket, 3000);
      };
      
      ws.value.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          console.log('위치 업데이트 수신:', data);
          position.x = data.x;
          position.y = data.y;
          position.orientation = data.orientation;
        } catch (error) {
          console.error('메시지 파싱 오류:', error);
        }
      };
      
      ws.value.onerror = (error) => {
        console.error('WebSocket 오류:', error);
      };
    };

    onMounted(() => {
      setupWebSocket();
    });

    onUnmounted(() => {
      if (ws.value && ws.value.readyState === WebSocket.OPEN) {
        ws.value.close();
      }
    });

    return {
      connected,
      position
    };
  }
};
</script>
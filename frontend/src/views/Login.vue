<template>
    <div class="login-page">
      <!-- 배경 애니메이션 - z-index 낮게 설정 -->
      <div class="login-background">
        <div class="particles">
          <div v-for="i in 15" :key="i" class="particle"></div>
        </div>
      </div>
      
      <!-- 로그인 카드 - 배경보다 상위 레이어 -->
      <v-container class="fill-height login-container" fluid>
        <v-row justify="center" align="center">
          <v-col cols="12" sm="8" md="6" lg="4">
            <v-card class="login-card mx-auto" elevation="8">
              <div class="login-header">
                <v-icon color="white" size="48" class="mb-2">mdi-robot-industrial</v-icon>
                <h1 class="text-h4 mb-1 text-white">FEST 로봇</h1>
                <h2 class="text-subtitle-1 text-white">화재 진압 로봇 모니터링 시스템</h2>
              </div>
              
              <v-card-text class="login-form">
                <v-form @submit.prevent="login" ref="form">
                  <div class="text-center mb-6">
                    <span class="text-h6">관리자 로그인</span>
                  </div>
                  
                  <v-alert
                    v-if="errorMessage"
                    type="error"
                    variant="tonal"
                    closable
                    class="mb-4"
                    @click:close="errorMessage = ''"
                  >
                    {{ errorMessage }}
                  </v-alert>
                  
                  <v-text-field
                    v-model="password"
                    :append-icon="showPassword ? 'mdi-eye' : 'mdi-eye-off'"
                    :type="showPassword ? 'text' : 'password'"
                    label="비밀번호"
                    placeholder="관리자 비밀번호를 입력하세요"
                    variant="outlined"
                    :error-messages="passwordError"
                    @click:append="showPassword = !showPassword"
                    @blur="validatePassword"
                    @input="clearError"
                  ></v-text-field>
                  
                  <v-checkbox
                    v-model="rememberMe"
                    label="로그인 상태 유지"
                    color="primary"
                    hide-details
                  ></v-checkbox>
                  
                  <v-btn
                    type="submit"
                    block
                    color="error"
                    size="large"
                    class="mt-6"
                    :loading="isLoading"
                  >
                    로그인
                  </v-btn>
                </v-form>
              </v-card-text>
              
              <v-btn
                  color="info"
                  variant="text"
                  block
                  @click="debugNavigate"
                >
                  디버그: 대시보드로 이동
                </v-btn>

              <v-card-text class="text-center text-caption text-medium-emphasis">
                &copy; {{ new Date().getFullYear() }} FEST 프로젝트
              </v-card-text>
            </v-card>
          </v-col>
        </v-row>
      </v-container>
    </div>
  </template>
  
  <script>
  import { authService } from '../services/api';
  
  export default {
    name: 'Login',
    data() {
      return {
        password: '',
        showPassword: false,
        rememberMe: false,
        isLoading: false,
        errorMessage: '',
        passwordError: ''
      };
    },
    created() {
      // 이미 로그인되어 있으면 대시보드로 리다이렉트
      if (localStorage.getItem('authToken')) {
        this.$router.push('/dashboard');
      }
    },
    methods: {
      validatePassword() {
        if (!this.password) {
          this.passwordError = '비밀번호를 입력하세요';
          return false;
        }
        return true;
      },
      
      clearError() {
        this.passwordError = '';
        this.errorMessage = '';
      },
      
      async login() {
      if (!this.validatePassword()) return;
      
      this.isLoading = true;
      
      try {
        const response = await authService.login(this.password);
        
        // 토큰 저장
        const token = response.data.access_token;
        console.log('받은 토큰 형식 확인:', {
          token: token,
          length: token ? token.length : 0
        });
        
        localStorage.setItem('authToken', token);
        
        // 로그인 성공 시 메인 페이지로 이동
        this.$router.push('/dashboard');
      } catch (error) {
        console.error('로그인 실패:', error);
        
        if (error.response && error.response.status === 401) {
          this.errorMessage = '잘못된 비밀번호입니다. 다시 시도해주세요.';
        } else {
          this.errorMessage = '로그인 중 오류가 발생했습니다. 잠시 후 다시 시도해주세요.';
        }
      } finally {
        this.isLoading = false;
      }
    },
    
    debugNavigate() {
      // 디버깅용: 라우터 상태 출력
      console.log('현재 라우터 객체:', this.$router);
      console.log('현재 라우트:', this.$route);
      
      // 스토리지에 토큰 저장
      localStorage.setItem('authToken', 'debug-token');
      
      // 프로그래밍 방식으로 대시보드로 이동
      this.$router.push('/dashboard');
    }

    }
  };
  </script>
  
  <style scoped>
  .login-page {
    min-height: 100vh;
    background: linear-gradient(135deg, #ff5252, #b71c1c);
    position: relative;
    overflow: hidden;
  }
  
  .login-container {
    position: relative;
    z-index: 2; /* 컨테이너를 배경보다 위에 배치 */
  }
  
  .login-card {
    border-radius: 12px;
    overflow: hidden;
    max-width: 450px;
    position: relative;
    z-index: 2; /* 카드를 배경보다 위에 배치 */
  }
  
  .login-header {
    background-color: #d32f2f;
    padding: 30px 20px;
    text-align: center;
  }
  
  .login-form {
    padding: 30px;
  }
  
  /* 배경 입자 애니메이션 */
  .login-background {
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    z-index: 1; /* 배경은 가장 아래 레이어에 배치 */
    pointer-events: none; /* 배경이 입력을 방해하지 않도록 설정 */
  }
  
  .particles {
    width: 100%;
    height: 100%;
  }
  
  .particle {
    position: absolute;
    border-radius: 50%;
    background: rgba(255, 255, 255, 0.3);
    animation: float 15s infinite;
  }
  
  .particle:nth-child(1) { width: 80px; height: 80px; left: 10%; top: 20%; animation-delay: 0s; }
  .particle:nth-child(2) { width: 40px; height: 40px; left: 20%; top: 80%; animation-delay: 1s; }
  .particle:nth-child(3) { width: 60px; height: 60px; left: 30%; top: 40%; animation-delay: 2s; }
  .particle:nth-child(4) { width: 50px; height: 50px; left: 40%; top: 60%; animation-delay: 3s; }
  .particle:nth-child(5) { width: 70px; height: 70px; left: 50%; top: 10%; animation-delay: 4s; }
  .particle:nth-child(6) { width: 30px; height: 30px; left: 60%; top: 70%; animation-delay: 5s; }
  .particle:nth-child(7) { width: 65px; height: 65px; left: 70%; top: 30%; animation-delay: 6s; }
  .particle:nth-child(8) { width: 45px; height: 45px; left: 80%; top: 50%; animation-delay: 7s; }
  .particle:nth-child(9) { width: 55px; height: 55px; left: 90%; top: 25%; animation-delay: 8s; }
  .particle:nth-child(10) { width: 25px; height: 25px; left: 15%; top: 90%; animation-delay: 9s; }
  .particle:nth-child(11) { width: 35px; height: 35px; left: 35%; top: 85%; animation-delay: 10s; }
  .particle:nth-child(12) { width: 75px; height: 75px; left: 55%; top: 95%; animation-delay: 11s; }
  .particle:nth-child(13) { width: 85px; height: 85px; left: 75%; top: 75%; animation-delay: 12s; }
  .particle:nth-child(14) { width: 35px; height: 35px; left: 85%; top: 15%; animation-delay: 13s; }
  .particle:nth-child(15) { width: 45px; height: 45px; left: 95%; top: 45%; animation-delay: 14s; }
  
  @keyframes float {
    0%, 100% {
      transform: translateY(0);
    }
    50% {
      transform: translateY(-50px);
    }
  }
  </style>
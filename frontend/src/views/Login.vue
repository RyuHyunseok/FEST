<template>
  <v-container class="fill-height d-flex align-center justify-center" fluid>
    <v-row>
      <v-col cols="12" sm="10" md="6" lg="4" class="mx-auto">
        <v-card class="elevation-12" min-width="400px">
          <v-toolbar color="primary" dark flat>
            <v-toolbar-title>FEST 로봇 관제 시스템</v-toolbar-title>
          </v-toolbar>
          <v-card-text class="pa-6">
            <v-form @submit.prevent="login">
              <v-text-field
                v-model="password"
                :append-icon="showPassword ? 'mdi-eye' : 'mdi-eye-off'"
                :type="showPassword ? 'text' : 'password'"
                label="비밀번호"
                name="password"
                hint="관리자 비밀번호를 입력하세요"
                @click:append="showPassword = !showPassword"
                @keyup.enter="login"
                variant="outlined"
                class="mt-4"
                size="large"
              ></v-text-field>
            </v-form>
          </v-card-text>
          <v-card-actions class="pa-6 pt-0">
            <v-spacer></v-spacer>
            <v-btn color="primary" size="large" @click="login" :loading="loading" min-width="120">로그인</v-btn>
          </v-card-actions>
          <v-snackbar v-model="snackbar" :color="snackbarColor" timeout="3000">
            {{ snackbarText }}
          </v-snackbar>
        </v-card>
      </v-col>
    </v-row>
  </v-container>
</template>

<script>
import axios from 'axios';

export default {
  name: 'LoginPage',
  data() {
    return {
      password: '',
      showPassword: false,
      loading: false,
      snackbar: false,
      snackbarText: '',
      snackbarColor: 'error'
    };
  },
  methods: {
    async login() {
      if (!this.password) {
        this.showError('비밀번호를 입력하세요');
        return;
      }

      this.loading = true;

      try {
        // FormData 형식으로 전송 (OAuth2 스펙)
        const formData = new FormData();
        formData.append('username', 'admin'); // username은 무시되지만 OAuth2 스펙상 필요함
        formData.append('password', this.password);

        const response = await axios.post('/api/v1/auth/token', formData);
        
        // 토큰 저장
        localStorage.setItem('authToken', response.data.access_token);
        
        // 성공 메시지 표시
        this.snackbarColor = 'success';
        this.snackbarText = '로그인 성공!';
        this.snackbar = true;
        
        // 로봇 위치 페이지로 이동
        setTimeout(() => {
          this.$router.push('/robots');
        }, 1000);
      } catch (error) {
        console.error('로그인 실패:', error);
        this.showError('로그인 실패: 비밀번호를 확인하세요');
      } finally {
        this.loading = false;
      }
    },
    showError(message) {
      this.snackbarColor = 'error';
      this.snackbarText = message;
      this.snackbar = true;
    }
  }
};
</script>
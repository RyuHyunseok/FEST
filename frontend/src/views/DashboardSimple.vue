<template>
    <div class="dashboard-simple">
      <h1>대시보드</h1>
      <p>기본 대시보드 내용입니다</p>
      
      <v-card class="mx-auto my-4" max-width="500">
        <v-card-title>상태 요약</v-card-title>
        <v-card-text>
          <p>로봇 상태: 활성화</p>
          <p>화재 상태: 없음</p>
        </v-card-text>
      </v-card>
      
      <v-btn color="primary" @click="testFunction">테스트 버튼</v-btn>
    </div>
  </template>
  
  <script>
  export default {
    name: 'DashboardSimple',
    data() {
      return {
        message: '테스트 메시지'
      }
    },
    methods: {
      testFunction() {
        alert('버튼이 작동합니다!');
      }
    }
  };
  </script>
  
  <style scoped>
  .dashboard-simple {
    padding: 20px;
  }
  </style>
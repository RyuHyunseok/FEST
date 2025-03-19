<template>
  <v-app>
    <v-app-bar app color="primary" dark>
      <v-app-bar-title>FEST 로봇 모니터링 시스템</v-app-bar-title>
    </v-app-bar>

    <v-main>
      <router-view></router-view>
    </v-main>

    <v-footer app color="primary" dark>
      <span>&copy; {{ new Date().getFullYear() }} - FEST 프로젝트</span>
    </v-footer>
  </v-app>
</template>

<script>
export default {
  name: 'App'
};
</script>
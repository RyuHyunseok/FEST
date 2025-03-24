<template>
    <v-app-bar
      :color="color"
      :elevation="elevation"
      :flat="flat"
      class="topbar"
    >
      <template v-slot:prepend>
        <v-app-bar-nav-icon
          v-if="showNavIcon"
          @click="$emit('toggle-drawer')"
        ></v-app-bar-nav-icon>
        <v-btn
          v-if="showBackButton"
          icon="mdi-arrow-left"
          @click="goBack"
        ></v-btn>
      </template>
  
      <v-app-bar-title>
        <div class="d-flex align-center">
          <v-icon color="error" class="mr-2">mdi-robot-industrial</v-icon>
          <span class="font-weight-bold">{{ title }}</span>
          <span class="subtitle-text ml-2">{{ subtitle }}</span>
        </div>
      </v-app-bar-title>
  
      <v-spacer></v-spacer>
  
      <div v-if="connectionStatus" class="connection-status d-flex align-center mr-3">
        <div class="status-indicator mr-2">
          <v-icon
            :color="allConnected ? 'success' : 'error'"
            :icon="allConnected ? 'mdi-wifi' : 'mdi-wifi-off'"
            size="small"
          ></v-icon>
        </div>
        <span class="text-caption">{{ connectionStatusText }}</span>
      </div>
  
      <v-btn
        v-if="refreshAction"
        icon="mdi-refresh"
        :loading="isRefreshing"
        @click="handleRefresh"
      ></v-btn>
  
      <slot name="actions"></slot>
    </v-app-bar>
  </template>
  
  <script>
  export default {
    name: 'Topbar',
    props: {
      title: {
        type: String,
        default: 'FEST 로봇 모니터링 시스템'
      },
      subtitle: {
        type: String,
        default: ''
      },
      color: {
        type: String,
        default: 'background'
      },
      elevation: {
        type: [Number, String],
        default: 1
      },
      flat: {
        type: Boolean,
        default: false
      },
      showNavIcon: {
        type: Boolean,
        default: true
      },
      showBackButton: {
        type: Boolean,
        default: false
      },
      refreshAction: {
        type: Function,
        default: null
      },
      connectionStatus: {
        type: Object,
        default: null
      }
    },
    data() {
      return {
        isRefreshing: false
      };
    },
    computed: {
      allConnected() {
        if (!this.connectionStatus) return true;
        return this.connectionStatus.robots && this.connectionStatus.incidents;
      },
      connectionStatusText() {
        if (!this.connectionStatus) return '';
        
        if (this.allConnected) {
          return '서버 연결됨';
        } else if (!this.connectionStatus.robots && !this.connectionStatus.incidents) {
          return '서버 연결 끊김';
        } else {
          const disconnected = [];
          if (!this.connectionStatus.robots) disconnected.push('로봇');
          if (!this.connectionStatus.incidents) disconnected.push('화재');
          
          return `${disconnected.join('/')} 연결 끊김`;
        }
      }
    },
    methods: {
      goBack() {
        this.$router.go(-1);
      },
      async handleRefresh() {
        if (!this.refreshAction || this.isRefreshing) return;
        
        this.isRefreshing = true;
        try {
          await this.refreshAction();
        } catch (error) {
          console.error('Refresh failed:', error);
        } finally {
          this.isRefreshing = false;
        }
      }
    }
  };
  </script>
  
  <style scoped>
  .topbar {
    border-bottom: 1px solid rgba(0, 0, 0, 0.12);
  }
  
  .subtitle-text {
    font-size: 0.875rem;
    opacity: 0.7;
  }
  
  .connection-status {
    font-size: 0.75rem;
    padding: 4px 8px;
    border-radius: 12px;
    background-color: rgba(0, 0, 0, 0.05);
  }
  
  .status-indicator {
    display: flex;
    align-items: center;
    justify-content: center;
  }
  
  .dark .connection-status {
    background-color: rgba(255, 255, 255, 0.05);
  }
  </style>
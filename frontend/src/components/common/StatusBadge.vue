<template>
    <div :class="['status-badge', statusClass, { pulse: blink }]" :style="customStyle">
      <v-icon v-if="icon" size="small" :icon="icon" class="mr-1" />
      {{ text }}
    </div>
  </template>
  
  <script>
  export default {
    name: 'StatusBadge',
    props: {
      status: {
        type: String,
        required: true
      },
      type: {
        type: String,
        default: 'default',
        validator: (val) => ['default', 'robot', 'incident', 'mission'].includes(val)
      },
      blink: {
        type: Boolean,
        default: false
      },
      customColor: {
        type: String,
        default: ''
      }
    },
    computed: {
      statusConfig() {
        // 상태별 설정 매핑
        const configs = {
          default: {
            active: { color: 'success', text: '활성', icon: 'mdi-check-circle' },
            idle: { color: 'info', text: '대기 중', icon: 'mdi-clock-outline' },
            inactive: { color: 'grey', text: '비활성', icon: 'mdi-cancel' },
            warning: { color: 'warning', text: '경고', icon: 'mdi-alert' },
            error: { color: 'error', text: '오류', icon: 'mdi-alert-circle' }
          },
          robot: {
            idle: { color: 'info', text: '대기 중', icon: 'mdi-robot' },
            moving: { color: 'success', text: '이동 중', icon: 'mdi-robot-mower' },
            fighting_fire: { color: 'error', text: '화재 진압 중', icon: 'mdi-fire' },
            on_mission: { color: 'warning', text: '임무 수행 중', icon: 'mdi-robot-industrial'}
          },
          incident: {
            active: { color: 'error', text: '화재 발생', icon: 'mdi-fire' },
            assigned: { color: 'warning', text: '로봇 배정됨', icon: 'mdi-fire-truck' },
            extinguished: { color: 'success', text: '진압 완료', icon: 'mdi-check-circle' }
          },
          mission: {
            assigned: { color: 'warning', text: '할당됨', icon: 'mdi-clipboard-outline' },
            in_progress: { color: 'info', text: '진행 중', icon: 'mdi-progress-clock' },
            completed: { color: 'success', text: '완료됨', icon: 'mdi-check-circle' },
            failed: { color: 'error', text: '실패', icon: 'mdi-close-circle' }
          }
        };
        
        // 타입에 맞는 설정 객체 가져오기
        const typeConfigs = configs[this.type] || configs.default;
        
        // 상태에 맞는 설정 반환 (없으면 기본값)
        return typeConfigs[this.status] || { 
          color: 'grey', 
          text: this.status, 
          icon: null 
        };
      },
      
      statusClass() {
        return `status-${this.statusConfig.color}`;
      },
      
      text() {
        return this.statusConfig.text;
      },
      
      icon() {
        return this.statusConfig.icon;
      },
      
      customStyle() {
        return this.customColor ? { backgroundColor: this.customColor } : {};
      }
    }
  };
  </script>
  
  <style scoped>
  .status-badge {
    display: inline-flex;
    align-items: center;
    padding: 4px 8px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: 500;
    color: white;
  }
  
  .status-success {
    background-color: #4CAF50;
  }
  
  .status-error {
    background-color: #FF5252;
  }
  
  .status-warning {
    background-color: #FFC107;
    color: #333;
  }
  
  .status-info {
    background-color: #2196F3;
  }
  
  .status-grey {
    background-color: #9E9E9E;
  }
  
  .pulse {
    animation: pulse-animation 1.5s infinite;
  }
  
  @keyframes pulse-animation {
    0% {
      opacity: 1;
    }
    50% {
      opacity: 0.7;
    }
    100% {
      opacity: 1;
    }
  }
  </style>
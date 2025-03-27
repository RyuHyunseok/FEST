<template>
    <v-navigation-drawer
      v-model="drawer"
      :rail="rail"
      permanent
      @click="rail = false"
      :location="location"
      :color="color"
      class="sidebar"
    >
      <v-list-item
        prepend-avatar="https://randomuser.me/api/portraits/lego/1.jpg"
        :title="userName"
        subtitle="관리자"
        @click="rail = !rail"
        class="sidebar-header"
      >
        <template v-slot:append>
          <v-btn
            variant="text"
            icon="mdi-chevron-left"
            @click.stop="rail = !rail"
          ></v-btn>
        </template>
      </v-list-item>
  
      <v-divider></v-divider>
  
      <v-list density="compact" nav>
        <v-list-item
          v-for="item in menuItems"
          :key="item.title"
          :prepend-icon="item.icon"
          :title="item.title"
          :to="item.to"
          :active="isActive(item.to)"
          :value="item.title"
          class="sidebar-item"
        >
          <template v-if="item.badge" v-slot:append>
            <v-badge
              :color="item.badgeColor || 'error'"
              :content="item.badge"
              inline
            ></v-badge>
          </template>
        </v-list-item>
      </v-list>
  
      <template v-slot:append>
        <div class="pa-2">
          <v-btn
            variant="outlined"
            color="error"
            block
            prepend-icon="mdi-logout"
            @click="logout"
          >
            로그아웃
          </v-btn>
        </div>
      </template>
    </v-navigation-drawer>
  </template>
  
  <script>
  export default {
    name: 'Sidebar',
    props: {
      modelValue: {
        type: Boolean,
        default: true
      },
      location: {
        type: String,
        default: 'left'
      },
      color: {
        type: String,
        default: 'background'
      }
    },
    data() {
      return {
        drawer: this.modelValue,
        rail: false,
        userName: '관리자',
        activeRoute: null,
        menuItems: [
          {
            title: '대시보드',
            icon: 'mdi-view-dashboard',
            to: '/dashboard'
          },
          // {
          //   title: '로봇 모니터링',
          //   icon: 'mdi-robot',
          //   to: '/robots',
          //   badge: '1'
          // },
          {
            title: '화재 이력',
            icon: 'mdi-fire',
            to: '/incidents'
          },
          // {
          //   title: '미션 관리',
          //   icon: 'mdi-clipboard-list',
          //   to: '/missions'
          // },
          // {
          //   title: '설정',
          //   icon: 'mdi-cog',
          //   to: '/settings'
          // }
        ]
      };
    },
    watch: {
      modelValue(val) {
        this.drawer = val;
      },
      drawer(val) {
        this.$emit('update:modelValue', val);
      },
      $route(route) {
        this.activeRoute = route.path;
      }
    },
    mounted() {
      this.activeRoute = this.$route.path;
    },
    methods: {
      isActive(path) {
        return this.activeRoute && this.activeRoute.startsWith(path);
      },
      logout() {
        localStorage.removeItem('authToken');
        this.$router.push('/login');
      }
    }
  };
  </script>
  
  <style scoped>
  .sidebar-header {
    min-height: 64px;
  }
  
  .sidebar {
    border-radius: 0;
  }
  
  .sidebar-item {
    margin-bottom: 4px;
  }
  
  .v-navigation-drawer.v-theme--dark {
    background-color: #1E1E1E;
  }
  </style>
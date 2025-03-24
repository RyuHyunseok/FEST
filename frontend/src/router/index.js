import { createRouter, createWebHistory } from 'vue-router'

// 화면 컴포넌트 import
import Login from '../views/Login.vue'
import Dashboard from '../views/Dashboard.vue'
// 심플 대시보드는 남겨두지만 사용하지 않음
import DashboardSimple from '../views/DashboardSimple.vue'
const routes = [
  {
    path: '/',
    redirect: '/login'
  },
  {
    path: '/login',
    name: 'Login',
    component: Login
  },
  {
    path: '/dashboard',
    name: 'Dashboard',
    component: Dashboard
  },
  // 404 페이지
  {
    path: '/:pathMatch(.*)*',
    redirect: '/dashboard'
  }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

// 네비게이션 가드
router.beforeEach((to, from, next) => {
  const token = localStorage.getItem('authToken')
  
  // 디버깅용 로그 추가
  console.log('[Router] 네비게이션 가드 실행:', { path: to.path, token: !!token })
  
  // 로그인이 필요한 페이지에 접근하는 경우
  if (to.path !== '/login' && !token) {
    console.log('[Router] 인증 필요: 로그인 페이지로 리다이렉트')
    next('/login')
  } 
  // 이미 로그인한 상태에서 로그인 페이지로 가려는 경우
  else if (to.path === '/login' && token) {
    console.log('[Router] 이미 인증됨: 대시보드로 리다이렉트')
    next('/dashboard')
  }
  // 그 외의 경우 정상 진행
  else {
    console.log('[Router] 정상 진행:', to.path)
    next()
  }
})

export default router
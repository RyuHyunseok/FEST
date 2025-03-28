import { createApp } from 'vue'
import { createVuetify } from 'vuetify'
import * as components from 'vuetify/components'
import * as directives from 'vuetify/directives'
import { aliases, mdi } from 'vuetify/iconsets/mdi'

// 스타일
import 'vuetify/styles'
import '@mdi/font/css/materialdesignicons.css'
import './assets/styles/main.css'

// 앱 컴포넌트 및 라우터
import App from './App.vue'
import router from './router'

// Vuetify 인스턴스 생성
const vuetify = createVuetify({
  components,
  directives,
  icons: {
    defaultSet: 'mdi',
    aliases,
    sets: {
      mdi
    }
  },
  theme: {
    defaultTheme: 'light',
    themes: {
      light: {
        colors: {
          primary: '#2196F3',
          secondary: '#424242',
          accent: '#FF4081',
          error: '#FF5252',
          info: '#2196F3',
          success: '#4CAF50',
          warning: '#FB8C00',
          background: '#FFFFFF'
        }
      },
      dark: {
        colors: {
          primary: '#2196F3',
          secondary: '#757575',
          accent: '#FF4081',
          error: '#FF5252',
          info: '#2196F3',
          success: '#4CAF50',
          warning: '#FB8C00',
          background: '#121212'
        }
      }
    }
  }
})

// 앱 생성 및 마운트
createApp(App)
  .use(router)
  .use(vuetify)
  .mount('#app')
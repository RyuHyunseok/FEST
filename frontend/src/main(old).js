import { createApp } from 'vue'
import './style.css'
import App from './App.vue'
import router from './router'
import './axios'  // axios 설정 임포트

// Vuetify
import 'vuetify/styles'
import { createVuetify } from 'vuetify'
import * as components from 'vuetify/components'
import * as directives from 'vuetify/directives'
import { aliases, mdi } from 'vuetify/iconsets/mdi'

const vuetify = createVuetify({
    components,
    directives,
    icons: {
        defaultSet: 'mdi',
        aliases,
        sets: {
            mdi,
        }
    }
})

createApp(App)
    .use(router)
    .use(vuetify)
    .mount('#app')
<template>
    <v-dialog v-model="isVisible" max-width="400" @click:outside="close">
      <v-card>
        <v-card-title :class="`text-h5 bg-${color} text-white`">
          <v-icon class="mr-2">{{ icon }}</v-icon>
          {{ title }}
        </v-card-title>
        
        <v-card-text class="pa-4 text-center">
          <v-icon :size="64" :color="color" class="my-4">{{ contentIcon }}</v-icon>
          <div class="text-h6 mb-2">{{ heading }}</div>
          <div class="text-body-1">{{ message }}</div>
        </v-card-text>
        
        <v-card-actions>
          <v-spacer></v-spacer>
          <v-btn
            :color="color"
            variant="tonal"
            @click="close"
          >
            {{ buttonText }}
          </v-btn>
        </v-card-actions>
      </v-card>
    </v-dialog>
  </template>
  
  <script>
  export default {
    name: 'AlertDialog',
    props: {
      modelValue: {
        type: Boolean,
        default: false
      },
      title: {
        type: String,
        default: '알림'
      },
      icon: {
        type: String,
        default: 'mdi-bell'
      },
      contentIcon: {
        type: String,
        default: 'mdi-information'
      },
      color: {
        type: String,
        default: 'primary'
      },
      heading: {
        type: String,
        default: '알림이 있습니다'
      },
      message: {
        type: String,
        default: '시스템에서 새로운 알림이 발생했습니다.'
      },
      buttonText: {
        type: String,
        default: '확인'
      }
    },
    computed: {
      isVisible: {
        get() {
          return this.modelValue;
        },
        set(value) {
          this.$emit('update:modelValue', value);
        }
      }
    },
    methods: {
      close() {
        this.isVisible = false;
        this.$emit('confirm');
      }
    }
  }
  </script>
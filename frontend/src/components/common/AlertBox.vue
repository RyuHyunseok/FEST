<template>
    <v-alert
      :type="type"
      :variant="variant"
      :icon="icon"
      :title="title"
      :text="text"
      :border="border"
      :density="density"
      :closable="closable"
      class="alert-box"
      :class="{ 'alert-animation': animate }"
    >
      <slot></slot>
    </v-alert>
  </template>
  
  <script>
  export default {
    name: 'AlertBox',
    props: {
      type: {
        type: String,
        default: 'info',
        validator: (val) => ['success', 'info', 'warning', 'error'].includes(val)
      },
      variant: {
        type: String,
        default: 'tonal'
      },
      icon: {
        type: String,
        default: null
      },
      title: {
        type: String,
        default: ''
      },
      text: {
        type: String,
        default: ''
      },
      border: {
        type: String,
        default: null
      },
      density: {
        type: String,
        default: 'comfortable'
      },
      closable: {
        type: Boolean,
        default: false
      },
      animate: {
        type: Boolean,
        default: false
      }
    }
  };
  </script>
  
  <style scoped>
  .alert-box {
    margin-bottom: 16px;
  }
  
  .alert-animation {
    animation: alert-fade-in 0.5s ease-out;
  }
  
  @keyframes alert-fade-in {
    from {
      opacity: 0;
      transform: translateY(-10px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
  </style>
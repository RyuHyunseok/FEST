<template>
  <div class="map-container" ref="mapContainer" :class="{ 'fullscreen': isFullscreen }">
    <!-- 컨트롤 버튼 -->
    <div class="map-controls">
      <v-btn
        icon="mdi-fullscreen"
        size="small"
        variant="tonal"
        @click="toggleFullscreen"
        class="control-btn"
      ></v-btn>
      <v-btn
        icon="mdi-refresh"
        size="small"
        variant="tonal"
        @click="resetCamera"
        class="control-btn"
      ></v-btn>
    </div>
    
    <!-- 로딩 인디케이터 -->
    <div v-if="!isModelLoaded" class="loading-overlay">
      <v-progress-circular indeterminate color="primary"></v-progress-circular>
      <div class="mt-2">맵 로딩 중...</div>
    </div>
  </div>
</template>

<script>
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export default {
  name: 'PathMapViewer',
  props: {
    pathPoints: {
      type: Array,
      default: () => []
    },
    incident: {
      type: Object,
      default: null
    }
  },
  data() {
    return {
      isInitialized: false,
      isModelLoaded: false,
      isFullscreen: false,
      initialCameraPosition: null,
      renderingError: false
    };
  },
  watch: {
    pathPoints: {
      handler() {
        if (this.isInitialized && this.isModelLoaded) {
          this.updatePathPoints();
        }
      },
      deep: true
    },
    incident: {
      handler() {
        if (this.isInitialized && this.isModelLoaded) {
          this.updateIncidentMarker();
        }
      },
      deep: true
    }
  },
  beforeCreate() {
    this._three = null;
  },
  mounted() {
    console.log('PathMapViewer 마운트됨');
    this.initThree();
    
    // 이벤트 리스너 등록
    window.addEventListener('resize', this.onWindowResize);
    document.addEventListener('fullscreenchange', this.onFullscreenChange);
  },
  beforeUnmount() {
    // 이벤트 리스너 정리
    window.removeEventListener('resize', this.onWindowResize);
    document.removeEventListener('fullscreenchange', this.onFullscreenChange);
    
    this.cleanup();
  },
  methods: {
    initThree() {
      try {
        console.log('Three.js 초기화 시작');
        
        // Three.js 객체 저장할 객체 생성
        this._three = {
          pathMarkers: [],
          pathLine: null,
          incidentMarker: null,
          animationFrameId: null,
          lastRenderTime: null
        };
        
        // Scene 생성
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0xf5f5f5);
        
        // 컨테이너 크기 확인
        const width = this.$refs.mapContainer.clientWidth;
        const height = this.$refs.mapContainer.clientHeight;
        console.log('컨테이너 크기:', width, height);
        
        // Camera 생성
        const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        this.initialCameraPosition = new THREE.Vector3(5, 5, 5);
        camera.position.copy(this.initialCameraPosition);
        
        // Renderer 생성
        const renderer = new THREE.WebGLRenderer({ 
          antialias: true, 
          alpha: true,
          powerPreference: 'high-performance'
        });
        renderer.setSize(width, height);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.$refs.mapContainer.appendChild(renderer.domElement);
        
        // 조명 추가
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 10, 7.5);
        directionalLight.castShadow = true;
        scene.add(directionalLight);
        
        // 컨트롤 추가
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.rotateSpeed = 0.8;
        controls.panSpeed = 0.5;
        controls.zoomSpeed = 1.0;
        controls.minDistance = 1;
        controls.maxDistance = 100;
        
        // 참조 저장
        this._three.scene = scene;
        this._three.camera = camera;
        this._three.renderer = renderer;
        this._three.controls = controls;
        
        // // 그리드 헬퍼 추가
        // const gridHelper = new THREE.GridHelper(20, 20, 0x888888, 0xcccccc);
        // scene.add(gridHelper);
        
        // GLB 모델 로드
        this.loadModel();
        
        // 애니메이션 시작
        this.animate();
        
        this.isInitialized = true;
        console.log('Three.js 초기화 완료');
      } catch (error) {
        console.error('Three.js 초기화 오류:', error);
        this.renderingError = true;
      }
    },
    
    loadModel() {
      try {
        console.log('GLB 모델 로드 시작');
        
        const { scene } = this._three;
        if (!scene) {
          console.error('Scene이 초기화되지 않았습니다');
          return;
        }
        
        const loader = new GLTFLoader();
        
        loader.load(
          '/WarehouseIndustrial.glb',
          (gltf) => {
            console.log('GLB 모델 로드 성공');
            
            // 모델 씬에 추가
            const model = gltf.scene;
            scene.add(model);
            
            // 모델 참조 저장
            this._three.model = model;
            
            // 카메라 위치 조정
            this.focusCameraOnModel(model);
            
            this.isModelLoaded = true;
            
            // 모델 로드 후 경로와 화재 마커 업데이트
            this.updatePathPoints();
            this.updateIncidentMarker();
          },
          (xhr) => {
            const progress = Math.floor((xhr.loaded / xhr.total) * 100);
            console.log(`GLB 로딩 진행도: ${progress}%`);
          },
          (error) => {
            console.error('GLB 모델 로드 오류:', error);
            this.renderingError = true;
          }
        );
      } catch (error) {
        console.error('loadModel 메소드 오류:', error);
        this.renderingError = true;
      }
    },
    
    updatePathPoints() {
      try {
        const { scene } = this._three;
        if (!scene) return;
        
        // 기존 경로 제거
        this.clearPathObjects();
        
        if (!this.pathPoints || this.pathPoints.length === 0) {
          console.log('경로 포인트가 없음');
          return;
        }
        
        console.log(`${this.pathPoints.length}개의 경로 포인트 표시`);
        
        // 경로 포인트를 프록시에서 분리
        const points = JSON.parse(JSON.stringify(this.pathPoints));
        
        // 마커 생성에 사용할 지오메트리와 재질
        const markerGeometry = new THREE.SphereGeometry(1, 16, 16);
        const markerMaterial = new THREE.MeshBasicMaterial({ color: 0x2196f3 });
        
        // 각 포인트마다 마커 생성
        this._three.pathMarkers = [];
        points.forEach((point, index) => {
          if (point.x !== undefined && point.y !== undefined) {
            // Unity 좌표계(x,y)를 Three.js 좌표계(z,x)로 변환
            // Three.js에서 y는 높이
            const marker = new THREE.Mesh(markerGeometry, markerMaterial.clone());
            marker.position.set(point.y, 0.3, point.x);
            scene.add(marker);
            this._three.pathMarkers.push(marker);
            
            // 첫 번째와 마지막 포인트 로깅
            if (index === 0 || index === points.length - 1) {
              console.log(`${index === 0 ? '첫' : '마지막'} 포인트: Unity(${point.x}, ${point.y}) -> Three.js(${point.y}, 0.3, ${point.x})`);
            }
          }
        });
        
        // 경로 선 생성
        if (points.length >= 2) {
          const linePoints = points
            .filter(p => p.x !== undefined && p.y !== undefined)
            .map(p => new THREE.Vector3(p.y, 0.3, p.x));
          
          if (linePoints.length >= 2) {
            const lineGeometry = new THREE.BufferGeometry().setFromPoints(linePoints);
            const lineMaterial = new THREE.LineBasicMaterial({ 
              color: 0x4286f4, 
              linewidth: 3,
              linecap: 'round'
            });
            this._three.pathLine = new THREE.Line(lineGeometry, lineMaterial);
            scene.add(this._three.pathLine);
          }
        }
      } catch (error) {
        console.error('경로 포인트 업데이트 오류:', error);
      }
    },
    
    updateIncidentMarker() {
      try {
        const { scene } = this._three;
        if (!scene) return;
        
        // 기존 화재 마커 제거
        if (this._three.incidentMarker) {
          scene.remove(this._three.incidentMarker);
          this._three.incidentMarker = null;
        }
        
        if (!this.incident || !this.incident.location) {
          console.log('화재 위치 정보 없음');
          return;
        }
        
        // 화재 위치 데이터를 프록시에서 분리
        const location = JSON.parse(JSON.stringify(this.incident.location));
        console.log('원본 화재 위치:', location);
        
        // Unity 좌표계(x,y)를 Three.js 좌표계(z,x)로 변환
        const threeJSPosition = {
<<<<<<< HEAD
          x: location.y, // Unity y -> Three.js x
          y: 0.3,       // 바닥에서 약간 위에 표시
          z: location.x  // Unity x -> Three.js z
=======
          x: -location.x, // Unity y -> Three.js x
          y: 0.3,       // 바닥에서 약간 위에 표시
          z: location.y  // Unity x -> Three.js z
>>>>>>> origin/develop
        };
        
        console.log('변환된 화재 위치:', threeJSPosition);
        
        // 화재 마커 생성 (빨간색 원통)
        const geometry = new THREE.CylinderGeometry(2, 2, 0.5, 32);
        const material = new THREE.MeshBasicMaterial({ 
          color: 0xff5252,
          transparent: true,
          opacity: 0.8
        });
        
        this._three.incidentMarker = new THREE.Mesh(geometry, material);
        this._three.incidentMarker.position.set(threeJSPosition.x, threeJSPosition.y, threeJSPosition.z);
        scene.add(this._three.incidentMarker);
        
        // 화재 마커에 애니메이션 효과 추가
        this._three.incidentMarker.userData.pulse = 0;
      } catch (error) {
        console.error('화재 마커 업데이트 오류:', error);
      }
    },
    
    clearPathObjects() {
      const { scene } = this._three;
      if (!scene) return;
      
      // 경로 마커 제거
      if (this._three.pathMarkers && this._three.pathMarkers.length) {
        this._three.pathMarkers.forEach(marker => {
          if (marker) {
            scene.remove(marker);
          }
        });
        this._three.pathMarkers = [];
      }
      
      // 경로 선 제거
      if (this._three.pathLine) {
        scene.remove(this._three.pathLine);
        this._three.pathLine = null;
      }
    },
    
    focusCameraOnModel(model) {
      try {
        const { camera, controls } = this._three;
        if (!camera || !controls || !model) return;
        
        // 바운딩 박스 계산
        const box = new THREE.Box3().setFromObject(model);
        const center = box.getCenter(new THREE.Vector3());
        const size = box.getSize(new THREE.Vector3());
        
        // 모델 크기에 맞게 카메라 위치 조정
        const maxDim = Math.max(size.x, size.y, size.z);
        const fov = camera.fov * (Math.PI / 180);
        let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
        cameraZ *= 1.5; // 여유 공간 확보
        
        // 새로운 카메라 위치 설정
        const newPosition = new THREE.Vector3(
          center.x, 
          center.y + cameraZ/3, 
          center.z + cameraZ
        );
        
        // 카메라 위치 저장 및 설정
        this.initialCameraPosition = newPosition.clone();
        camera.position.copy(newPosition);
        controls.target.set(center.x, center.y, center.z);
        controls.update();
        
        console.log('카메라 위치 조정 완료');
      } catch (error) {
        console.error('카메라 위치 조정 오류:', error);
      }
    },
    
    animate() {
      if (!this._three || !this._three.renderer) return;
      
      try {
        // 현재 시간 기록
        const now = Date.now();
        if (!this._three.lastRenderTime) {
          this._three.lastRenderTime = now;
        }
        
        // 애니메이션 프레임 요청
        const animationFrame = requestAnimationFrame(this.animate);
        this._three.animationFrameId = animationFrame;
        
        // 16ms(약 60fps)마다 렌더링
        if (now - this._three.lastRenderTime >= 16) {
          // 컨트롤 업데이트
          if (this._three.controls) {
            this._three.controls.update();
          }
          
          // 화재 마커 애니메이션 (있는 경우)
          if (this._three.incidentMarker) {
            const marker = this._three.incidentMarker;
            marker.userData.pulse = (marker.userData.pulse || 0) + 0.05;
            const scale = 1 + 0.2 * Math.sin(marker.userData.pulse);
            marker.scale.set(scale, 1, scale);
          }
          
          // 렌더링
          this._three.renderer.render(this._three.scene, this._three.camera);
          this._three.lastRenderTime = now;
        }
      } catch (error) {
        console.error('애니메이션 오류:', error);
        
        // 심각한 오류 발생 시 애니메이션 중지
        if (this._three.animationFrameId) {
          cancelAnimationFrame(this._three.animationFrameId);
          this._three.animationFrameId = null;
        }
        
        this.renderingError = true;
      }
    },
    
    onWindowResize() {
      if (!this._three || !this._three.camera || !this._three.renderer || !this.$refs.mapContainer) {
        return;
      }
      
      console.log('창 크기 변경 감지, 리사이징 처리');
      
      // 컨테이너 크기 측정
      const width = this.$refs.mapContainer.clientWidth;
      const height = this.$refs.mapContainer.clientHeight;
      
      // 카메라 종횡비 업데이트
      this._three.camera.aspect = width / height;
      this._three.camera.updateProjectionMatrix();
      
      // 렌더러 크기 조정
      this._three.renderer.setSize(width, height);
      
      // 즉시 렌더링하여 변경사항 적용
      if (this._three.scene && this._three.camera) {
        this._three.renderer.render(this._three.scene, this._three.camera);
      }
    },
    
    onFullscreenChange() {
      this.isFullscreen = !!document.fullscreenElement;
      setTimeout(this.onWindowResize, 100);
    },
    
    toggleFullscreen() {
      if (!document.fullscreenElement) {
        this.$refs.mapContainer.requestFullscreen().catch(err => {
          console.error('전체화면 전환 실패:', err);
        });
      } else {
        document.exitFullscreen();
      }
    },
    
    resetCamera() {
      if (!this._three || !this._three.camera || !this._three.controls) return;
      
      if (this.initialCameraPosition) {
        this._three.camera.position.copy(this.initialCameraPosition);
        
        if (this._three.model) {
          // 모델 중심으로 컨트롤 타겟 설정
          const box = new THREE.Box3().setFromObject(this._three.model);
          const center = box.getCenter(new THREE.Vector3());
          this._three.controls.target.copy(center);
        } else {
          this._three.controls.target.set(0, 0, 0);
        }
        
        this._three.controls.update();
        
        // 즉시 렌더링
        if (this._three.renderer && this._three.scene) {
          this._three.renderer.render(this._three.scene, this._three.camera);
        }
      } else {
        // 초기 위치 정보가 없으면 모델 중심으로 재설정
        this.focusCameraOnModel(this._three.model);
      }
    },
    
    cleanup() {
      // 애니메이션 프레임 취소
      if (this._three && this._three.animationFrameId) {
        cancelAnimationFrame(this._three.animationFrameId);
        this._three.animationFrameId = null;
      }
      
      if (!this._three) return;
      
      try {
        console.log('Three.js 리소스 정리 시작');
        
        // 경로 객체 정리
        this.clearPathObjects();
        
        // 화재 마커 정리
        if (this._three.incidentMarker) {
          this._three.scene.remove(this._three.incidentMarker);
          this._three.incidentMarker = null;
        }
        
        // 모델 정리
        if (this._three.model) {
          this._three.scene.remove(this._three.model);
          this._three.model = null;
        }
        
        // 렌더러 정리
        if (this._three.renderer) {
          if (this._three.renderer.domElement && this._three.renderer.domElement.parentNode) {
            this._three.renderer.domElement.parentNode.removeChild(this._three.renderer.domElement);
          }
          this._three.renderer.dispose();
          this._three.renderer = null;
        }
        
        // 씬 리소스 정리
        if (this._three.scene) {
          this.disposeScene(this._three.scene);
          this._three.scene = null;
        }
        
       // 참조 제거
       this._three = null;
        this.isInitialized = false;
        this.isModelLoaded = false;
        
        console.log('Three.js 리소스 정리 완료');
      } catch (error) {
        console.error('정리 작업 오류:', error);
      }
    },
    
    disposeScene(scene) {
      // 씬의 모든 자식 객체 반복
      if (scene.children) {
        while(scene.children.length > 0) {
          const child = scene.children[0];
          
          // 재귀적으로 자식의 자식 정리
          if (child.children && child.children.length > 0) {
            this.disposeScene(child);
          }
          
          // 지오메트리 정리
          if (child.geometry) {
            child.geometry.dispose();
          }
          
          // 재질 정리
          if (child.material) {
            if (Array.isArray(child.material)) {
              child.material.forEach(material => material.dispose());
            } else {
              child.material.dispose();
            }
          }
          
          scene.remove(child);
        }
      }
    }
  }
}
</script>

<style scoped>
.map-container {
  width: 100%;
  height: 400px;
  background-color: #f0f0f0;
  overflow: hidden;
  position: relative;
  transition: all 0.3s ease;
}

.map-container.fullscreen {
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  z-index: 9999;
}

.map-controls {
  position: absolute;
  top: 10px;
  right: 10px;
  z-index: 10;
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.control-btn {
  background-color: rgba(255, 255, 255, 0.7) !important;
  border: 1px solid rgba(0, 0, 0, 0.1);
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
}

.loading-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background-color: rgba(240, 240, 240, 0.7);
  z-index: 5;
}
</style>
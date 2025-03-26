<template>
  <div class="map-container" ref="mapContainer"></div>
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
      isModelLoaded: false
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
  },
  beforeUnmount() {
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
          incidentMarker: null
        };
        
        // Scene 생성
        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0xf0f0f0);
        
        // 컨테이너 크기 확인
        const width = this.$refs.mapContainer.clientWidth;
        const height = this.$refs.mapContainer.clientHeight;
        console.log('컨테이너 크기:', width, height);
        
        // Camera 생성
        const camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        camera.position.set(5, 5, 5);
        
        // Renderer 생성
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(width, height);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.$refs.mapContainer.appendChild(renderer.domElement);
        
        // 조명 추가
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
        directionalLight.position.set(5, 10, 7.5);
        scene.add(directionalLight);
        
        // 컨트롤 추가
        const controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        
        // 참조 저장
        this._three.scene = scene;
        this._three.camera = camera;
        this._three.renderer = renderer;
        this._three.controls = controls;
        
        // GLB 모델 로드
        this.loadModel();
        
        // 애니메이션 시작
        this.animate();
        
        this.isInitialized = true;
        console.log('Three.js 초기화 완료');
      } catch (error) {
        console.error('Three.js 초기화 오류:', error);
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
          }
        );
      } catch (error) {
        console.error('loadModel 메소드 오류:', error);
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
        const markerGeometry = new THREE.SphereGeometry(1, 8, 8);
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
            const lineMaterial = new THREE.LineBasicMaterial({ color: 0xff4081, linewidth: 2 });
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
          x: location.y, // Unity y -> Three.js x
          y: 0.3,       // 바닥에서 약간 위에 표시
          z: location.x  // Unity x -> Three.js z
        };
        
        console.log('변환된 화재 위치:', threeJSPosition);
        
        // 화재 마커 생성 (빨간색 원통)
        const geometry = new THREE.CylinderGeometry(3, 3, 3, 16);
        const material = new THREE.MeshBasicMaterial({ color: 0xff5252 });
        this._three.incidentMarker = new THREE.Mesh(geometry, material);
        this._three.incidentMarker.position.set(threeJSPosition.x, threeJSPosition.y, threeJSPosition.z);
        scene.add(this._three.incidentMarker);
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
        
        camera.position.set(center.x, center.y + cameraZ/3, center.z + cameraZ);
        controls.target.set(center.x, center.y, center.z);
        controls.update();
        
        console.log('카메라 위치 조정 완료');
      } catch (error) {
        console.error('카메라 위치 조정 오류:', error);
      }
    },
    
    animate() {
      if (!this._three || !this._three.renderer) return;
      
      const animationFrame = requestAnimationFrame(this.animate);
      this._three.animationFrameId = animationFrame;
      
      // 컨트롤 업데이트
      if (this._three.controls) {
        this._three.controls.update();
      }
      
      try {
        this._three.renderer.render(this._three.scene, this._three.camera);
      } catch (error) {
        console.error('렌더링 오류:', error);
        cancelAnimationFrame(animationFrame);
      }
    },
    
    cleanup() {
      if (!this._three) return;
      
      try {
        console.log('Three.js 리소스 정리 시작');
        
        // 애니메이션 정지
        if (this._three.animationFrameId) {
          cancelAnimationFrame(this._three.animationFrameId);
        }
        
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
        }
        
        // 참조 제거
        this._three = null;
        this.isInitialized = false;
        
        console.log('Three.js 리소스 정리 완료');
      } catch (error) {
        console.error('정리 작업 오류:', error);
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
}
</style>
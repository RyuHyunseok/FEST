<template>
    <v-card class="robot-map-card">
      <v-card-title class="d-flex justify-space-between align-center">
        <div class="d-flex align-center">
          <v-icon color="error" class="mr-2">mdi-map-marker-radius</v-icon>
          <span>로봇 위치 모니터링</span>
        </div>
        <div>
          <v-btn
            icon="mdi-fullscreen"
            size="small"
            variant="text"
            @click="toggleFullscreen"
            class="mr-1"
          ></v-btn>
          <v-btn
            icon="mdi-refresh"
            size="small"
            variant="text"
            :loading="isLoading"
            @click="resetCamera"
          ></v-btn>
        </div>
      </v-card-title>
  
      <v-divider></v-divider>
  
      <div class="position-relative">
        <!-- 맵 내부에 로딩 인디케이터 -->
        <div v-if="isLoading" class="map-loading-overlay">
          <v-progress-circular indeterminate color="error"></v-progress-circular>
        </div>
  
        <!-- 로봇 및 화재 상태 오버레이 -->
        <div class="map-info-overlay">
          <div v-if="Object.keys(robots).length > 0" class="robot-counter">
            <v-icon color="error" size="small">mdi-robot</v-icon>
            <span>{{ Object.keys(robots).length }} 로봇 활성화</span>
          </div>
          <div v-if="Object.keys(incidents).length > 0" class="incident-counter">
            <v-icon color="error" size="small">mdi-fire</v-icon>
            <span>{{ Object.keys(incidents).length }} 화재 발생</span>
          </div>
        </div>
  
        <!-- Three.js 맵 컨테이너 -->
        <div ref="mapContainer" class="map-container" :class="{ fullscreen: isFullscreen }"></div>
  
        <!-- 선택된 로봇 정보 표시 -->
        <div v-if="selectedRobot" class="robot-detail-overlay">
          <v-card flat class="robot-detail-card">
            <v-card-title class="py-1">
              <span class="text-subtitle-2">{{ selectedRobot.name || `로봇 ${selectedRobotId}` }}</span>
            </v-card-title>
            <v-card-text class="py-1">
              <div class="d-flex align-center">
                <status-badge :status="selectedRobot.status?.status || 'idle'" type="robot" class="mr-2" />
                <span class="text-caption">배터리: {{ selectedRobot.status?.battery || 0 }}%</span>
              </div>
              <div class="text-caption mt-1">
                위치: ({{ formatCoord(selectedRobot.position?.x) }}, {{ formatCoord(selectedRobot.position?.y) }})
              </div>
            </v-card-text>
            <v-card-actions class="py-1">
              <v-btn
                variant="text"
                size="small"
                color="primary"
                @click="showRobotDetails(selectedRobotId)"
              >
                상세 정보
              </v-btn>
            </v-card-actions>
          </v-card>
        </div>
      </div>
    </v-card>
  </template>
  
  <script>
  import * as THREE from 'three';
  import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
  import StatusBadge from '../common/StatusBadge.vue';
  
  export default {
    name: 'RobotMap',
    components: {
      StatusBadge
    },
    props: {
      robots: {
        type: Object,
        required: true,
        default: () => ({})
      },
      incidents: {
        type: Object,
        required: true,
        default: () => ({})
      }
    },
    data() {
      return {
        scene: null,
        camera: null,
        renderer: null,
        controls: null,
        robotMeshes: {},
        incidentMeshes: {},
        isLoading: true,
        isFullscreen: false,
        selectedRobotId: null,
        animationFrameId: null,
        gridSize: 20
      };
    },
    computed: {
      selectedRobot() {
        if (!this.selectedRobotId || !this.robots[this.selectedRobotId]) {
          return null;
        }
        return this.robots[this.selectedRobotId];
      }
    },
    watch: {
      robots: {
        deep: true,
        handler() {
          this.updateRobots();
        }
      },
      incidents: {
        deep: true,
        handler() {
          this.updateIncidents();
        }
      }
    },
    mounted() {
      this.initThreeJs();
      window.addEventListener('resize', this.onWindowResize);
      document.addEventListener('fullscreenchange', this.onFullscreenChange);
    },
    beforeUnmount() {
      this.cleanupThreeJs();
      window.removeEventListener('resize', this.onWindowResize);
      document.removeEventListener('fullscreenchange', this.onFullscreenChange);
    },
    methods: {
      initThreeJs() {
        // Scene 생성
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xf5f5f5);
  
        // 카메라 설정
        const width = this.$refs.mapContainer.clientWidth;
        const height = this.$refs.mapContainer.clientHeight;
        this.camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
        this.camera.position.set(15, 15, 15);
        this.camera.lookAt(0, 0, 0);
  
        // 렌더러 설정
      try {
        // 첫 번째 방법으로 렌더러 생성 시도
        this.renderer = new THREE.WebGLRenderer({ 
          antialias: true,
          powerPreference: 'high-performance',
          failIfMajorPerformanceCaveat: false
        });
      } catch (e) {
        console.warn('[RobotMap] 기본 WebGL 렌더러 생성 실패, 대체 방법 시도:', e);
        try {
          // 대체 방법으로 시도
          this.renderer = new THREE.WebGLRenderer({ 
            antialias: false,
            precision: 'lowp'
          });
        } catch (e2) {
          console.error('[RobotMap] WebGL 렌더러 생성 최종 실패:', e2);
          this.isLoading = false;
          return;
        }
      }

      this.renderer.setSize(width, height);
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); // 성능 향상을 위해 최대 픽셀 비율 제한
      this.renderer.shadowMap.enabled = false; // 그림자 비활성화 (성능 향상)
      this.$refs.mapContainer.appendChild(this.renderer.domElement);
        
      
      // 컨트롤 설정
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        this.controls.minDistance = 5;
        this.controls.maxDistance = 50;
        this.controls.maxPolarAngle = Math.PI / 2;
  
        // 그리드 헬퍼 추가
        const gridHelper = new THREE.GridHelper(this.gridSize, this.gridSize, 0x888888, 0xcccccc);
        this.scene.add(gridHelper);
  
        // 좌표축 추가
        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);
  
        // 조명 설정
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);
  
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 20, 10);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);
  
        // 바닥 추가
        const groundGeometry = new THREE.PlaneGeometry(this.gridSize, this.gridSize);
        const groundMaterial = new THREE.MeshLambertMaterial({ 
          color: 0xcccccc, 
          side: THREE.DoubleSide,
          transparent: true,
          opacity: 0.3
        });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = Math.PI / 2;
        ground.position.y = -0.01;
        ground.receiveShadow = true;
        this.scene.add(ground);
  
        // 화재 진압 구역 표시
        this.addFirefightingZones();
  
        // 초기 로봇 및 화재 데이터 반영
        this.updateRobots();
        this.updateIncidents();
  
        // 애니메이션 시작
        this.animate();
        
        // 로딩 완료
        this.isLoading = false;
      },
  
      animate() {
      try {
        // 렌더링 필수 객체 확인
        if (!this.renderer || !this.scene || !this.camera) {
          console.log('[RobotMap] 렌더링 필수 객체가 없어 애니메이션 스킵');
          cancelAnimationFrame(this.animationFrameId);
          return;
        }

        this.animationFrameId = requestAnimationFrame(this.animate);

        // controls 업데이트
        if (this.controls) {
          this.controls.update();
        }

        // THREE.js 내부 오류 방지를 위한 조치
        try {
          // 렌더링 전에 각 객체의 matrixWorld 업데이트
          this.scene.updateMatrixWorld();
          
          // 렌더러에서 캐시 제거 - 프록시 관련 문제 방지에 도움
          if (this.renderer.info && this.renderer.info.memory) {
            const memoryInfo = this.renderer.info.memory;
            if (memoryInfo.geometries > 1000 || memoryInfo.textures > 100) {
              console.log('[RobotMap] 메모리 정리:', memoryInfo);
              this.renderer.dispose();
            }
          }
          
          // 렌더링
          this.renderer.render(this.scene, this.camera);
        } catch (renderError) {
          console.warn('[RobotMap] 렌더링 오류 발생, 재시도:', renderError);
          
          // 앱 전체가 중단되는 것을 방지하기 위해 다음 프레임에서 다시 시도
          cancelAnimationFrame(this.animationFrameId);
          setTimeout(() => {
            this.animationFrameId = requestAnimationFrame(this.animate);
          }, 1000);
        }
      } catch (error) {
        console.error('[RobotMap] 애니메이션 오류:', error);
        
        // 심각한 오류 발생 시 애니메이션 중지
        cancelAnimationFrame(this.animationFrameId);
      }
    },
  
      addFirefightingZones() {
        // 화재 진압 구역 시각화 (원형 구역으로 표시)
        const zones = [
          { x: 5, z: 5, radius: 3, color: 0xffcccc },
          { x: -5, z: -5, radius: 4, color: 0xccffcc },
          { x: -5, z: 5, radius: 2.5, color: 0xccccff }
        ];
  
        zones.forEach(zone => {
          const geometry = new THREE.CircleGeometry(zone.radius, 32);
          const material = new THREE.MeshBasicMaterial({ 
            color: zone.color,
            transparent: true,
            opacity: 0.3,
            side: THREE.DoubleSide
          });
          const circle = new THREE.Mesh(geometry, material);
          circle.rotation.x = Math.PI / 2;
          circle.position.set(zone.x, 0.02, zone.z);
          this.scene.add(circle);
  
          // 구역 외곽선 추가
          const edgesGeometry = new THREE.EdgesGeometry(geometry);
          const edgesMaterial = new THREE.LineBasicMaterial({ 
            color: new THREE.Color(zone.color).multiplyScalar(0.7)
          });
          const edges = new THREE.LineSegments(edgesGeometry, edgesMaterial);
          edges.rotation.x = Math.PI / 2;
          edges.position.set(zone.x, 0.03, zone.z);
          this.scene.add(edges);
        });
      },
  
      createRobotMesh(robotId) {
        // 로봇 몸체 - 원통형
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 0.2, 16);
        const bodyMaterial = new THREE.MeshLambertMaterial({ color: 0xff4444 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0.1;
        body.castShadow = true;
  
        // 로봇 머리 - 반구
        const headGeometry = new THREE.SphereGeometry(0.15, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
        const headMaterial = new THREE.MeshLambertMaterial({ color: 0xdddddd });
        const head = new THREE.Mesh(headGeometry, headMaterial);
        head.position.y = 0.25;
        head.castShadow = true;
  
        // 로봇 눈
        const eyeGeometry = new THREE.SphereGeometry(0.03, 8, 8);
        const eyeMaterial = new THREE.MeshBasicMaterial({ color: 0x333333 });
        
        const leftEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
        leftEye.position.set(0.07, 0.28, 0.12);
        
        const rightEye = new THREE.Mesh(eyeGeometry, eyeMaterial);
        rightEye.position.set(-0.07, 0.28, 0.12);
  
        // 방향 표시 화살표
        const arrowGeometry = new THREE.ConeGeometry(0.08, 0.2, 8);
        const arrowMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(0, 0.1, 0.4);
        arrow.rotation.x = Math.PI / 2;
  
        // 로봇 그룹으로 묶기
        const robotGroup = new THREE.Group();
        robotGroup.add(body);
        robotGroup.add(head);
        robotGroup.add(leftEye);
        robotGroup.add(rightEye);
        robotGroup.add(arrow);
        
        // 선택 이벤트를 위한 설정
        robotGroup.userData = { id: robotId, type: 'robot' };
        
        // 씬에 추가
        this.scene.add(robotGroup);
        
        // 로봇 메시 맵에 저장
        this.robotMeshes[robotId] = robotGroup;
        
        return robotGroup;
      },
  
      createIncidentMesh(incidentId, isActive = true) {
        // 화재 효과 그룹
        const fireGroup = new THREE.Group();
        
        // 기본 원형 표시
        const baseGeometry = new THREE.CircleGeometry(0.5, 16);
        const baseMaterial = new THREE.MeshBasicMaterial({
          color: 0xff3300,
          transparent: true,
          opacity: 0.7,
          side: THREE.DoubleSide
        });
        const baseCircle = new THREE.Mesh(baseGeometry, baseMaterial);
        baseCircle.rotation.x = Math.PI / 2;
        baseCircle.position.y = 0.05;
        fireGroup.add(baseCircle);
        
        // 활성 화재 효과 (꺼지면 표시 안함)
        if (isActive) {
          // 불꽃 효과 (원뿔)
          const flameGeometry = new THREE.ConeGeometry(0.3, 1, 16);
          const flameMaterial = new THREE.MeshBasicMaterial({
            color: 0xff5500,
            transparent: true,
            opacity: 0.8
          });
          const flame = new THREE.Mesh(flameGeometry, flameMaterial);
          flame.position.y = 0.5;
          fireGroup.add(flame);
          
          // 작은 불꽃 (더 작은 원뿔)
          const smallFlameGeometry = new THREE.ConeGeometry(0.15, 0.6, 16);
          const smallFlameMaterial = new THREE.MeshBasicMaterial({
            color: 0xffcc00,
            transparent: true,
            opacity: 0.9
          });
          const smallFlame = new THREE.Mesh(smallFlameGeometry, smallFlameMaterial);
          smallFlame.position.y = 0.7;
          fireGroup.add(smallFlame);
          
          // 연기 효과 (구)
          const smokeGeometry = new THREE.SphereGeometry(0.2, 8, 8);
          const smokeMaterial = new THREE.MeshBasicMaterial({
            color: 0x777777,
            transparent: true,
            opacity: 0.4
          });
          const smoke = new THREE.Mesh(smokeGeometry, smokeMaterial);
          smoke.position.y = 1.2;
          fireGroup.add(smoke);
          
          // 불꽃 애니메이션 설정
          fireGroup.userData.animationParams = {
            time: Math.random() * 10,
            flamePulse: 0,
            smokeRise: 0
          };
        } else {
          // 꺼진 화재는 X 표시
          const lineMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00 });
          
          // X의 첫 번째 선
          const line1Geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(-0.4, 0.1, -0.4),
            new THREE.Vector3(0.4, 0.1, 0.4)
          ]);
          const line1 = new THREE.Line(line1Geometry, lineMaterial);
          fireGroup.add(line1);
          
          // X의 두 번째 선
          const line2Geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0.4, 0.1, -0.4),
            new THREE.Vector3(-0.4, 0.1, 0.4)
          ]);
          const line2 = new THREE.Line(line2Geometry, lineMaterial);
          fireGroup.add(line2);
        }
        
        // userData에 정보 저장
        fireGroup.userData = { id: incidentId, type: 'incident', isActive };
        
        // 씬에 추가
        this.scene.add(fireGroup);
        
        // 맵에 저장
        this.incidentMeshes[incidentId] = fireGroup;
        
        return fireGroup;
      },
  
      updateRobots() {
        // 현재 로봇 ID 목록
        const currentRobotIds = Object.keys(this.robots);
        
        // 로봇 메시 맵에 있는 모든 ID 배열
        const existingRobotIds = Object.keys(this.robotMeshes);
        
        // 새 로봇 추가
        currentRobotIds.forEach(robotId => {
          const robotData = this.robots[robotId];
          
          // 위치 데이터가 없으면 건너뜀
          if (!robotData.position) return;
          
          // 새 로봇이면 메시 생성
          if (!existingRobotIds.includes(robotId)) {
            this.createRobotMesh(robotId);
          }
          
          // 로봇 위치 및 방향 업데이트
          if (this.robotMeshes[robotId]) {
            const mesh = this.robotMeshes[robotId];
            const { x, y, orientation } = robotData.position;
            
            // 위치 설정
            mesh.position.set(x, 0, y);
            
            // 방향 설정 (라디안으로 변환)
            const rotationY = (orientation * Math.PI) / 180;
            mesh.rotation.y = rotationY;
            
            // 상태에 따른 색상 변경
            if (robotData.status) {
              const bodyMaterial = mesh.children[0].material;
              
              switch (robotData.status.status) {
                case 'idle':
                  bodyMaterial.color.set(0x4444ff);
                  break;
                case 'moving':
                  bodyMaterial.color.set(0x44ff44);
                  break;
                case 'fighting_fire':
                  bodyMaterial.color.set(0xff4444);
                  break;
                default:
                  bodyMaterial.color.set(0xff4444);
              }
            }
          }
        });
        
        // 제거된 로봇 처리
        existingRobotIds.forEach(robotId => {
          if (!currentRobotIds.includes(robotId)) {
            // 씬에서 제거
            this.scene.remove(this.robotMeshes[robotId]);
            // 객체에서 제거
            delete this.robotMeshes[robotId];
          }
        });
      },
  
      updateIncidents() {
        // 현재 화재 ID 목록
        const currentIncidentIds = Object.keys(this.incidents);
        
        // 기존 화재 ID 목록
        const existingIncidentIds = Object.keys(this.incidentMeshes);
        
        // 새 화재 추가 및 기존 화재 업데이트
        currentIncidentIds.forEach(incidentId => {
          const incidentData = this.incidents[incidentId];
          
          // 위치 데이터가 없으면 건너뜀
          if (!incidentData.location) return;
          
          const isActive = incidentData.status === 'active';
          
          // 새 화재이거나 상태가 변경된 경우
          if (!existingIncidentIds.includes(incidentId) || 
              this.incidentMeshes[incidentId].userData.isActive !== isActive) {
            
            // 기존 화재가 있으면 제거
            if (existingIncidentIds.includes(incidentId)) {
              this.scene.remove(this.incidentMeshes[incidentId]);
            }
            
            // 새 화재 메시 생성
            this.createIncidentMesh(incidentId, isActive);
          }
          
          // 위치 업데이트
          if (this.incidentMeshes[incidentId]) {
            const mesh = this.incidentMeshes[incidentId];
            const { x, y } = incidentData.location;
            
            // 위치 설정
            mesh.position.set(x, 0, y);
          }
        });
        
        // 제거된 화재 처리
        existingIncidentIds.forEach(incidentId => {
          if (!currentIncidentIds.includes(incidentId)) {
            // 씬에서 제거
            this.scene.remove(this.incidentMeshes[incidentId]);
            // 객체에서 제거
            delete this.incidentMeshes[incidentId];
          }
        });
      },
  
      onWindowResize() {
        if (!this.camera || !this.renderer || !this.$refs.mapContainer) return;
        
        const width = this.$refs.mapContainer.clientWidth;
        const height = this.$refs.mapContainer.clientHeight;
        
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        
        this.renderer.setSize(width, height);
      },
  
      toggleFullscreen() {
        if (!document.fullscreenElement) {
          this.$refs.mapContainer.requestFullscreen();
        } else {
          document.exitFullscreen();
        }
      },
  
      onFullscreenChange() {
        this.isFullscreen = !!document.fullscreenElement;
        // 화면 크기 변경 후 리사이징 처리
        setTimeout(this.onWindowResize, 100);
      },
  
      resetCamera() {
        if (!this.camera || !this.controls) return;
        
        this.isLoading = true;
        
        // 카메라 초기 위치로 재설정
        this.camera.position.set(15, 15, 15);
        this.camera.lookAt(0, 0, 0);
        this.controls.target.set(0, 0, 0);
        this.controls.update();
        
        this.isLoading = false;
      },
  
      showRobotDetails(robotId) {
        this.$emit('select-robot', robotId);
      },
  
      formatCoord(value) {
        if (value === undefined || value === null) return 'N/A';
        return value.toFixed(2);
      },
  
      cleanupThreeJs() {
      if (this.animationFrameId) {
        cancelAnimationFrame(this.animationFrameId);
      }
      
      // 렌더러 정리
      if (this.renderer) {
        try {
          // renderer.domElement이 DOM에 연결되어 있는지 확인
          if (this.renderer.domElement && this.renderer.domElement.parentNode) {
            this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
          }
          
          // 렌더러 리소스 정리
          this.renderer.dispose();
          this.renderer.forceContextLoss();
          this.renderer = null;
        } catch (e) {
          console.warn('[RobotMap] 렌더러 정리 중 오류:', e);
        }
      }
      
      // 씬과 관련 객체 정리
      if (this.scene) {
        try {
          this.disposeSceneObjects(this.scene);
          this.scene = null;
        } catch (e) {
          console.warn('[RobotMap] 씬 정리 중 오류:', e);
        }
      }
      
      // 참조 제거
      this.camera = null;
      this.controls = null;
      this.robotMeshes = {};
      this.incidentMeshes = {};
    },
    
    disposeSceneObjects(obj) {
      // 모든 오브젝트의 리소스 정리
      if (obj.children) {
        for (let i = obj.children.length - 1; i >= 0; i--) {
          this.disposeSceneObjects(obj.children[i]);
          obj.remove(obj.children[i]);
        }
      }
      
      // 기하학 객체 정리
      if (obj.geometry) {
        obj.geometry.dispose();
      }
      
      // 재질 객체 정리
      if (obj.material) {
        if (Array.isArray(obj.material)) {
          obj.material.forEach(material => {
            if (material.map) material.map.dispose();
            if (material.lightMap) material.lightMap.dispose();
            if (material.bumpMap) material.bumpMap.dispose();
            if (material.normalMap) material.normalMap.dispose();
            if (material.specularMap) material.specularMap.dispose();
            if (material.envMap) material.envMap.dispose();
            material.dispose();
          });
        } else {
          if (obj.material.map) obj.material.map.dispose();
          if (obj.material.lightMap) obj.material.lightMap.dispose();
          if (obj.material.bumpMap) obj.material.bumpMap.dispose();
          if (obj.material.normalMap) obj.material.normalMap.dispose();
          if (obj.material.specularMap) obj.material.specularMap.dispose();
          if (obj.material.envMap) obj.material.envMap.dispose();
          obj.material.dispose();
        }
      }
    },
    }
  };
  </script>
  
  <style scoped>
  .robot-map-card {
    height: 100%;
    display: flex;
    flex-direction: column;
    border-radius: 8px;
    overflow: hidden;
  }
  
  .map-container {
    position: relative;
    width: 100%;
    height: 400px;
    overflow: hidden;
    background-color: #f0f0f0;
    transition: all 0.3s ease;
  }
  
  .map-container.fullscreen {
    height: 100vh;
    width: 100vw;
  }
  
  .map-loading-overlay {
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    bottom: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    background-color: rgba(0, 0, 0, 0.2);
    z-index: 2;
  }
  
  .map-info-overlay {
    position: absolute;
    top: 8px;
    left: 8px;
    z-index: 1;
    background-color: rgba(255, 255, 255, 0.8);
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 12px;
    box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
  }
  
  .robot-counter, .incident-counter {
    display: flex;
    align-items: center;
    margin: 2px 0;
  }
  
  .robot-counter span, .incident-counter span {
    margin-left: 4px;
  }
  
  .robot-detail-overlay {
    position: absolute;
    bottom: 16px;
    right: 16px;
    z-index: 1;
  }
  
  .robot-detail-card {
    background-color: rgba(255, 255, 255, 0.9);
    width: 200px;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.15);
  }
  </style>
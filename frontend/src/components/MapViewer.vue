<template>
  <div class="map-container" ref="mapContainer"></div>
</template>

<script>
import * as THREE from 'three';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export default {
  name: 'MapViewer',
  props: {
    robots: {
      type: Object,
      default: () => ({})
    },
    incidents: {
    type: Object,
    default: () => ({})
    },
    prowlers: {
      type: Object,
      default: () => ({})
    }
  },
  data() {
    return {
      robotMeshes: {}, // 로봇 ID를 키로 가지는 메시 객체
      incidentMeshes: {}, // 화재 ID를 키로 가지는 메시 객체 추가
      prowlerMeshes: {},
      forceRender: false
    };
  },
  watch: {
    robots: {
      deep: true,
      handler(newRobots) {
        this.updateRobotPositions(newRobots);
      }
    },
    incidents: {
    deep: true,
    immediate: true,
    handler(newIncidents, oldIncidents) {
      // console.log('MapViewer: incidents 변경 감지', newIncidents);
      this.updateIncidents(newIncidents);
      }
    },
    prowlers: {
      deep: true,
      handler(newProwlers, oldProwlers) {
    console.log('침입자 데이터 변경 감지:', 
      '새로운 침입자 수:', Object.keys(newProwlers).length, 
      '이전 침입자 수:', oldProwlers ? Object.keys(oldProwlers).length : 0);
    this.updateProwlers(newProwlers);
  }
    }
  },
  mounted() {
    this.initThree();
    this.loadModel();
    this.animate();

    // 창 크기 조정 시 반응형으로 대응
    window.addEventListener('resize', this.onWindowResize);
  },
  beforeUnmount() {
    window.removeEventListener('resize', this.onWindowResize);
    if (this.renderer) {
      this.renderer.dispose();
    }
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
    }
  },
  methods: {
    initThree() {
      // 씬 생성
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0xf0f0f0);

      // 카메라 설정
      this.camera = new THREE.PerspectiveCamera(
        75,
        this.$refs.mapContainer.clientWidth / this.$refs.mapContainer.clientHeight,
        0.1,
        1000
      );
      this.camera.position.set(5, 5, 5);

      // 렌더러 설정
      this.renderer = new THREE.WebGLRenderer({ antialias: true });
      this.renderer.setSize(
        this.$refs.mapContainer.clientWidth,
        this.$refs.mapContainer.clientHeight
      );
      this.renderer.shadowMap.enabled = true;
      this.$refs.mapContainer.appendChild(this.renderer.domElement);

      // 조명 설정
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
      this.scene.add(ambientLight);

      const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
      directionalLight.position.set(5, 10, 7.5);
      directionalLight.castShadow = true;
      this.scene.add(directionalLight);

      // 컨트롤 추가 (마우스로 화면 회전, 확대/축소)
      this.controls = new OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.25;
    },
    loadModel() {
      const loader = new GLTFLoader();
      
      // GLB 파일 로드
      loader.load(
        '/WarehouseIndustrial.glb',
        (gltf) => {
          this.model = gltf.scene;
          this.scene.add(this.model);
          
          // 모델 중앙에 카메라 위치
          const box = new THREE.Box3().setFromObject(this.model);
          const center = box.getCenter(new THREE.Vector3());
          const size = box.getSize(new THREE.Vector3());
          
          // 모델 크기에 맞게 카메라 위치 조정
          const maxDim = Math.max(size.x, size.y, size.z);
          const fov = this.camera.fov * (Math.PI / 180);
          let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
          cameraZ *= 1.5; // 여유 공간 확보
          
          this.camera.position.set(center.x, center.y + cameraZ / 3, center.z + cameraZ);
          this.controls.target.set(center.x, center.y, center.z);
          this.controls.update();
          
          // 초기 로봇 위치 설정 (로드가 완료된 후에만 실행)
          this.updateRobotPositions(this.robots);
        },
        (xhr) => {
          console.log((xhr.loaded / xhr.total * 100) + '% loaded');
        },
        (error) => {
          console.error('An error happened during GLB loading', error);
        }
      );
    },
    createRobotMesh() {
      // 로봇을 표현할 기하학적 형태 생성 - 크기 증가
      const geometry = new THREE.CylinderGeometry(1.5, 1.5, 1, 16); // 크기와 세그먼트 수 증가
      const material = new THREE.MeshLambertMaterial({ color: 0xff0000 }); // 빨간색
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      
      // 메시 위에 화살표 추가하여 방향 표시
      const arrowGeometry = new THREE.ConeGeometry(1, 1, 10);
      const arrowMaterial = new THREE.MeshLambertMaterial({ color: 0xff0000 }); // 노란색 화살표
      const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
      arrow.position.set(0, 0, 3); // 메시 앞쪽에 위치
      arrow.rotation.x = Math.PI / 2; // 앞쪽을 가리키도록 회전
      
      // 로봇 메시 그룹 생성
      const group = new THREE.Group();
      group.add(mesh);
      group.add(arrow);
      
      return group;
    },
    updateRobotPositions(robots) {
      if (!this.scene) return;
      
      // 각 로봇에 대해 처리
      Object.entries(robots).forEach(([robotId, data]) => {
        // 로봇 위치 데이터가 있는지 확인
        if (data.position) {
          const { x, y, orientation } = data.position;
          
          // 로봇 메시가 없으면 새로 생성
          if (!this.robotMeshes[robotId]) {
            const robotMesh = this.createRobotMesh();
            this.scene.add(robotMesh);
            this.robotMeshes[robotId] = robotMesh;
          }
          
          // Unity 좌표계를 Three.js 좌표계로 변환
          // Unity는 왼손 좌표계, Three.js는 오른손 좌표계
          // 예: Unity의 (x, y) -> Three.js의 (x, 0.3, -y) 매핑 또는 적절히 조정
          const robotMesh = this.robotMeshes[robotId];
          
          // 옵션 1: 그대로 매핑 (x, y) -> (x, 높이, y)
          // robotMesh.position.set(x, 0.3, y);
          
          // 옵션 2: x축 반전 (Unity 좌표계가 반대인 경우)
          // robotMesh.position.set(-x, 0.3, y);
          
          // 옵션 3: z축 반전 (Unity 좌표계가 반대인 경우)
          // robotMesh.position.set(x, 0.3, -y);
          
          // 옵션 4: x, z축 모두 반전
          // robotMesh.position.set(-x, 0.3, -y);
          
          // 옵션 5: x, z축 교체 (Unity의 x,y를 Three.js의 z,x로)
          robotMesh.position.set(y, 0.3, x);
          
          // 방향 설정 (라디안으로 변환)
          // 방향 각도도 좌표계에 맞게 조정해야 할 수 있음
          const rotationY = (orientation * Math.PI) / 180;
          robotMesh.rotation.y = rotationY;
          
          // 콘솔에 디버깅 정보 출력
          // console.log(`Robot ${robotId} position: Unity(${x}, ${y}) -> Three.js(${robotMesh.position.x}, ${robotMesh.position.y}, ${robotMesh.position.z})`);
        }
      });
    },
    animate() {  
      this.animationFrameId = requestAnimationFrame(this.animate);
      
      if (this.controls) {
        this.controls.update();
      }
      
  // 화재 애니메이션 효과 (불꽃이 흔들리는 효과)
  if (this.incidentMeshes) {
    Object.values(this.incidentMeshes).forEach(item => {
      if (item.mesh) {
        // 불꽃 흔들림 효과
        const time = Date.now() * 0.001; // 시간 기반 애니메이션
        const flame = item.mesh.children[1]; // 큰 불꽃
        const smallFlame = item.mesh.children[2]; // 작은 불꽃
        
        if (flame && smallFlame) {
          flame.position.x = Math.sin(time * 2) * 0.2;
          flame.position.z = Math.cos(time * 3) * 0.2;
          flame.scale.y = 0.9 + Math.sin(time * 5) * 0.1;
          
          smallFlame.position.x = Math.sin(time * 3) * 0.3;
          smallFlame.position.z = Math.cos(time * 2) * 0.3;
          smallFlame.scale.y = 0.8 + Math.sin(time * 6) * 0.2;
        }
      }
    });
  }
// 강제 렌더링 플래그 확인
if (this.forceRender) {
    this.forceRender = false;
    this.renderer.render(this.scene, this.camera);
    console.log('강제 렌더링 수행됨');
  } else {
    this.renderer.render(this.scene, this.camera);
  }

},


    onWindowResize() {
      if (!this.camera || !this.renderer) return;
      
      this.camera.aspect = this.$refs.mapContainer.clientWidth / this.$refs.mapContainer.clientHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(
        this.$refs.mapContainer.clientWidth,
        this.$refs.mapContainer.clientHeight
      );
    },

    // 화재 관련 메서드

    createIncidentMesh(incidentId, location) {
  // 화재 효과 그룹
  const fireGroup = new THREE.Group();
  
  // 기본 원형 표시 (바닥에 표시)
  const baseGeometry = new THREE.CircleGeometry(3, 32);
  const baseMaterial = new THREE.MeshBasicMaterial({
    color: 0xff3300,
    transparent: true,
    opacity: 0.7,
    side: THREE.DoubleSide
  });
  const baseCircle = new THREE.Mesh(baseGeometry, baseMaterial);
  baseCircle.rotation.x = Math.PI / 2; // 바닥에 평행하게
  baseCircle.position.y = 0.1; // 바닥 위에 약간 띄우기
  fireGroup.add(baseCircle);
  
  // 불꽃 효과 (원뿔)
  const flameGeometry = new THREE.ConeGeometry(2, 5, 32);
  const flameMaterial = new THREE.MeshBasicMaterial({
    color: 0xff5500,
    transparent: true,
    opacity: 0.8
  });
  const flame = new THREE.Mesh(flameGeometry, flameMaterial);
  flame.position.y = 2.5; // 중심 높이
  fireGroup.add(flame);
  
  // 작은 불꽃 (더 작은 원뿔)
  const smallFlameGeometry = new THREE.ConeGeometry(1, 3, 32);
  const smallFlameMaterial = new THREE.MeshBasicMaterial({
    color: 0xffcc00,
    transparent: true,
    opacity: 0.9
  });
  const smallFlame = new THREE.Mesh(smallFlameGeometry, smallFlameMaterial);
  smallFlame.position.y = 4; // 중심 높이 위
  fireGroup.add(smallFlame);
  
  // 좌표 설정 (Unity 좌표계를 Three.js 좌표계로 변환)
  fireGroup.position.set(location.y, 0, location.x);
  
  // 씬에 추가
  this.scene.add(fireGroup);
  
  // 맵에 저장
  this.incidentMeshes[incidentId] = {
    mesh: fireGroup
  };
  
  return fireGroup;
},

updateIncidents(incidents) {
  if (!this.scene) return;
  
  // console.log('updateIncidents 호출됨:', incidents);
  
  // 현재 화재 ID 목록
  const currentIncidentIds = Object.keys(incidents);
  // console.log('현재 화재 ID들:', currentIncidentIds);
  // console.log('현재 맵에 있는 화재 ID들:', Object.keys(this.incidentMeshes));
  
  // 먼저 extinguished 상태의 화재 제거
  currentIncidentIds.forEach(incidentId => {
    const incidentData = incidents[incidentId];
    // console.log(`화재 ID ${incidentId} 상태:`, incidentData.status);

    // extinguished 상태면 화재 제거
    if (incidentData.status === 'extinguished') {
      if (this.incidentMeshes[incidentId]) {
        // console.log(`화재 ID ${incidentId} 제거 시도`);
        try {
          // 화재 메시 참조 저장
          const meshToRemove = this.incidentMeshes[incidentId].mesh;
          
          // 씬에서 제거
          if (meshToRemove) {
            this.scene.remove(meshToRemove);
            // console.log(`화재 ID ${incidentId} 씬에서 제거 성공`);
            
            // 메모리 해제
            if (meshToRemove.geometry) meshToRemove.geometry.dispose();
            if (meshToRemove.material) {
              if (Array.isArray(meshToRemove.material)) {
                meshToRemove.material.forEach(m => m.dispose());
              } else {
                meshToRemove.material.dispose();
              }
            }
            
            // 자식 객체 제거
            while (meshToRemove.children.length > 0) {
              const child = meshToRemove.children[0];
              meshToRemove.remove(child);
              if (child.geometry) child.geometry.dispose();
              if (child.material) {
                if (Array.isArray(child.material)) {
                  child.material.forEach(m => m.dispose());
                } else {
                  child.material.dispose();
                }
              }
            }
          }
          
          // 객체에서 제거
          delete this.incidentMeshes[incidentId];
          // console.log(`화재 ID ${incidentId} 완전히 제거됨`);
        } catch (error) {
          // console.error(`화재 ID ${incidentId} 제거 중 오류:`, error);
        }
      }
    }
  });
  
  // 남은 화재 처리 (active 상태)
  currentIncidentIds.forEach(incidentId => {
    const incidentData = incidents[incidentId];
    
    // extinguished 상태는 이미 위에서 처리했으므로 건너뜀
    if (incidentData.status === 'extinguished') return;
    
    // 위치 데이터가 없으면 건너뜀
    if (!incidentData.location) return;
    
    // 화재가 맵에 없으면 새로 생성
    if (!this.incidentMeshes[incidentId]) {
      // console.log(`새 화재 ID ${incidentId} 생성`);
      this.createIncidentMesh(incidentId, incidentData.location);
    } else {
      // 위치만 업데이트
      // console.log(`화재 ID ${incidentId} 위치 업데이트`);
      const incidentMesh = this.incidentMeshes[incidentId].mesh;
      incidentMesh.position.set(incidentData.location.y, 0, incidentData.location.x);
    }
  });
  
  // console.log('업데이트 후 맵에 남은 화재 ID들:', Object.keys(this.incidentMeshes));
},

createProwlerMesh(prowlerId, location) {
  // X 표시 생성을 위한 그룹
  const prowlerGroup = new THREE.Group();
  
  // 선 굵기
  const thickness = 1;
  const length = 10;
  
  // X 표시의 첫 번째 선 (직육면체로 대체)
  const box1Geometry = new THREE.BoxGeometry(thickness, thickness, length);
  const material = new THREE.MeshBasicMaterial({ color: 0xFF10F0 });
  const line1 = new THREE.Mesh(box1Geometry, material);
  line1.rotation.y = Math.PI / 4; // 대각선으로 회전
  
  // X 표시의 두 번째 선 (직육면체로 대체)
  const box2Geometry = new THREE.BoxGeometry(thickness, thickness, length);
  const line2 = new THREE.Mesh(box2Geometry, material);
  line2.rotation.y = -Math.PI / 4; // 반대 방향 대각선으로 회전
  
  // 선들을 그룹에 추가
  prowlerGroup.add(line1);
  prowlerGroup.add(line2);
  
  // 위치 설정 (Unity 좌표계를 Three.js 좌표계로 변환)
  prowlerGroup.position.set(location.y, 0.5, location.x);
  
  // 씬에 추가
  this.scene.add(prowlerGroup);
  
  // 맵에 저장
  this.prowlerMeshes[prowlerId] = {
    mesh: prowlerGroup
  };
  
  return prowlerGroup;
},
  
  updateProwlers(prowlers) {
    if (!this.scene) return;

    console.log('updateProwlers 호출됨:', prowlers);
    console.log('현재 맵에 표시된 침입자:', Object.keys(this.prowlerMeshes));
    
    // 현재 침입자 ID 목록
    const currentProwlerIds = Object.keys(prowlers);
    
    Object.keys(this.prowlerMeshes).forEach(prowlerId => {
    if (!currentProwlerIds.includes(prowlerId)) {
      console.log(`침입자 ID ${prowlerId} 제거 시도`);
      
      try {
        // 씬에서 메시 제거
        const meshToRemove = this.prowlerMeshes[prowlerId].mesh;
        if (meshToRemove) {
          this.scene.remove(meshToRemove);
          
          // 자식 객체 제거 및 메모리 해제
          while(meshToRemove.children.length > 0) {
            const child = meshToRemove.children[0];
            meshToRemove.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
              if (Array.isArray(child.material)) {
                child.material.forEach(m => m.dispose());
              } else {
                child.material.dispose();
              }
            }
          }
          
          // 메모리 해제
          if (meshToRemove.geometry) meshToRemove.geometry.dispose();
          if (meshToRemove.material) {
            if (Array.isArray(meshToRemove.material)) {
              meshToRemove.material.forEach(m => m.dispose());
            } else {
              meshToRemove.material.dispose();
            }
          }
        }
        
        // 객체에서 제거
        delete this.prowlerMeshes[prowlerId];
        console.log(`침입자 ID ${prowlerId} 제거 완료`);
        
        // 강제 렌더링 트리거 (이 부분이 중요)
        this.forceRender = true;
      } catch (error) {
        console.error(`침입자 ID ${prowlerId} 제거 중 오류:`, error);
      }
    }
  });
    
    // 현재 침입자 추가 또는 업데이트
    currentProwlerIds.forEach(prowlerId => {
      const prowlerData = prowlers[prowlerId];
      
      // 위치 데이터가 없으면 건너뜀
      if (!prowlerData.location) return;
      
      // 침입자가 맵에 없으면 새로 생성
      if (!this.prowlerMeshes[prowlerId]) {
        this.createProwlerMesh(prowlerId, prowlerData.location);
      } else {
        // 위치만 업데이트
        const prowlerMesh = this.prowlerMeshes[prowlerId].mesh;
        prowlerMesh.position.set(prowlerData.location.y, 0.5, prowlerData.location.x);
      }
    });
  }

  }
};
</script>

<style scoped>
.map-container {
  width: 100%;
  height: 500px;
  overflow: hidden;
}
</style>
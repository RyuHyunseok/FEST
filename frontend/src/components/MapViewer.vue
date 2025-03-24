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
    }
  },
  data() {
    return {
      robotMeshes: {} // 로봇 ID를 키로 가지는 메시 객체
    };
  },
  watch: {
    robots: {
      deep: true,
      handler(newRobots) {
        this.updateRobotPositions(newRobots);
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
          console.log(`Robot ${robotId} position: Unity(${x}, ${y}) -> Three.js(${robotMesh.position.x}, ${robotMesh.position.y}, ${robotMesh.position.z})`);
        }
      });
    },
    animate() {
      this.animationFrameId = requestAnimationFrame(this.animate);
      
      if (this.controls) {
        this.controls.update();
      }
      
      this.renderer.render(this.scene, this.camera);
    },
    onWindowResize() {
      if (!this.camera || !this.renderer) return;
      
      this.camera.aspect = this.$refs.mapContainer.clientWidth / this.$refs.mapContainer.clientHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(
        this.$refs.mapContainer.clientWidth,
        this.$refs.mapContainer.clientHeight
      );
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
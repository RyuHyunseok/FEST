<template>
    <div class="dashboard">
      <!-- 레이아웃 구성 -->
      <Sidebar v-model="drawer" />
      
      <div class="dashboard-content">
        <Topbar 
          :connection-status="connectionStatus" 
          :refresh-action="refreshData"
          @toggle-drawer="drawer = !drawer"
        />
        
        <!-- 메인 콘텐츠 영역 -->
        <v-container fluid class="pa-4">

                    <!-- 화재 모니터링 섹션 (버튼만 배치) -->
                    <v-row class="mt-4">
            <v-col cols="12">
              <v-card>
                <v-card-title class="d-flex justify-space-between align-center">
                  <div class="d-flex align-center">
                    <v-icon color="error" class="mr-2">mdi-fire-alert</v-icon>
                    <span>화재 모니터링</span>
                  </div>
                  <v-btn
                    color="error"
                    variant="outlined"
                    prepend-icon="mdi-fire"
                    @click="$router.push('/incidents')"
                  >
                    화재 이력 보기
                  </v-btn>
                </v-card-title>
                
                <v-divider></v-divider>
                
                <v-card-text v-if="activeIncidentCount === 0" class="text-center py-4">
                  <v-icon icon="mdi-check-circle" size="large" color="success" class="mb-2"></v-icon>
                  <div class="text-body-1">현재 활성화된 화재가 없습니다</div>
                  <div class="text-caption text-medium-emphasis">시스템이 상황을 모니터링 중입니다</div>
                </v-card-text>
                
                <v-card-text v-else class="text-center py-4">
                  <v-icon icon="mdi-fire-alert" size="large" color="error" class="mb-2"></v-icon>
                  <div class="text-body-1">{{ activeIncidentCount }}건의 화재가 진행 중입니다!</div>
                  <v-btn
                    color="error"
                    class="mt-2"
                    @click="$router.push('/incidents')"
                  >
                    화재 상세 정보 보기
                  </v-btn>
                </v-card-text>
              </v-card>
            </v-col>
          </v-row>


          <!-- 연결 상태 알림 -->
          <!-- <v-row v-if="!isAllConnected">
            <v-col cols="12">
              <AlertBox
                type="warning"
                icon="mdi-wifi-off"
                title="서버 연결 상태"
                :animate="true"
              >
                <div>일부 데이터 연결이 끊어졌습니다. 실시간 데이터를 받지 못할 수 있습니다.</div>
                <v-btn 
                  color="warning" 
                  variant="text" 
                  size="small" 
                  class="mt-2"
                  @click="reconnectWebsockets"
                >
                  재연결 시도
                </v-btn>
              </AlertBox>
            </v-col>
          </v-row> -->
          
          <!-- 상태 요약 카드 -->
          <!-- <v-row>
            <v-col cols="12" sm="6" md="3">
              <v-card class="dashboard-card">
                <v-card-text class="d-flex align-center">
                  <v-avatar color="primary" class="mr-4">
                    <v-icon color="white">mdi-robot</v-icon>
                  </v-avatar>
                  <div>
                    <div class="text-h4">{{ robotCount }}</div>
                    <div class="text-caption">활성 로봇</div>
                  </div>
                </v-card-text>
              </v-card>
            </v-col>
            
            <v-col cols="12" sm="6" md="3">
              <v-card class="dashboard-card">
                <v-card-text class="d-flex align-center">
                  <v-avatar color="error" class="mr-4">
                    <v-icon color="white">mdi-fire</v-icon>
                  </v-avatar>
                  <div>
                    <div class="text-h4">{{ activeIncidentCount }}</div>
                    <div class="text-caption">활성 화재</div>
                  </div>
                </v-card-text>
              </v-card>
            </v-col>
            
            <v-col cols="12" sm="6" md="3">
              <v-card class="dashboard-card">
                <v-card-text class="d-flex align-center">
                  <v-avatar color="success" class="mr-4">
                    <v-icon color="white">mdi-check-circle</v-icon>
                  </v-avatar>
                  <div>
                    <div class="text-h4">{{ extinguishedCount }}</div>
                    <div class="text-caption">진압 완료</div>
                  </div>
                </v-card-text>
              </v-card>
            </v-col>
            
            <v-col cols="12" sm="6" md="3">
              <v-card class="dashboard-card">
                <v-card-text class="d-flex align-center">
                  <v-avatar color="info" class="mr-4">
                    <v-icon color="white">mdi-clipboard-clock</v-icon>
                  </v-avatar>
                  <div>
                    <div class="text-h4">{{ activeMissionCount }}</div>
                    <div class="text-caption">진행 중인 미션</div>
                  </div>
                </v-card-text>
              </v-card>
            </v-col>
          </v-row> -->
          
          <!-- 메인 콘텐츠 -->
          <v-row>
                      <!-- 로봇 맵 -->
          <v-col cols="12" md="8">
            <v-card class="mx-auto my-4">
              <v-card-title class="d-flex justify-space-between align-center">
                <div class="d-flex align-center">
                  <v-icon color="error" class="mr-2">mdi-map-marker-radius</v-icon>
                  <span>로봇 위치 모니터링</span>
                </div>
              </v-card-title>
              
              <v-divider></v-divider>
              
              <v-card-text>
                <MapViewer :robots="robots" :incidents="incidents" :prowlers="prowlers"/>
              </v-card-text>
            </v-card>
          </v-col>
            
            <!-- 사이드 패널 - 로봇 목록 -->
            <v-col cols="12" md="4">
              <v-card class="robot-list-card">
                <v-card-title class="d-flex justify-space-between align-center">
                  <div class="d-flex align-center">
                    <v-icon color="primary" class="mr-2">mdi-robot</v-icon>
                    <span>로봇 목록</span>
                  </div>
                  <v-btn
                    icon="mdi-refresh"
                    size="small"
                    variant="text"
                    :loading="isLoadingRobots"
                    @click="fetchRobots"
                  ></v-btn>
                </v-card-title>
                
                <v-divider></v-divider>
                
                <v-list>
                  <div v-if="isLoadingRobots" class="d-flex justify-center pa-4">
                    <v-progress-circular indeterminate color="primary"></v-progress-circular>
                  </div>
                  
                  <div v-else-if="robotList.length === 0" class="text-center pa-4">
                    <v-icon icon="mdi-robot-off" size="large" color="grey" class="mb-2"></v-icon>
                    <div class="text-body-1">등록된 로봇이 없습니다</div>
                  </div>
                  
                  <template v-else>
                    <v-list-item
                      v-for="robot in robotList"
                      :key="robot.robot_id"
                      :ripple="true"
                      :active="selectedRobotId === robot.robot_id"
                      @click="selectRobot(robot.robot_id)"
                    >
                      <template v-slot:prepend>
                        <v-avatar
                          :color="getRobotStatusColor(robot)"
                          class="mr-2"
                        >
                          <v-icon color="white">mdi-robot</v-icon>
                        </v-avatar>
                      </template>
                      
                      <v-list-item-title>{{ robot.name || `로봇 ${robot.robot_id}` }}</v-list-item-title>
                      
                      <v-list-item-subtitle class="d-flex justify-space-between mt-1">
                        <status-badge
                          :status="getRobotStatus(robot)"
                          type="robot"
                        ></status-badge>
                        <span v-if="getRobotBattery(robot) !== null" class="text-caption">
                          배터리: {{ getRobotBattery(robot) }}%
                        </span>
                      </v-list-item-subtitle>
                    </v-list-item>
                  </template>
                </v-list>
              </v-card>
              
              <!-- 선택된 로봇 상세 정보 -->
              <v-card v-if="selectedRobot" class="mt-4">
                <v-card-title class="d-flex justify-space-between align-center">
                  <div class="d-flex align-center">
                    <v-icon color="primary" class="mr-2">mdi-robot</v-icon>
                    <span>{{ selectedRobot.name || `로봇 ${selectedRobotId}` }}</span>
                  </div>
                  <v-btn
                    icon="mdi-close"
                    size="small"
                    variant="text"
                    @click="selectedRobotId = null"
                  ></v-btn>
                </v-card-title>
                
                <v-divider></v-divider>
                
                <v-card-text>
                  <!-- 로봇 상태 정보 -->
                <div class="mb-4">
                  <div class="d-flex align-center mb-2">
                    <status-badge
                      :status="getRobotStatus(selectedRobot)"
                      type="robot"
                      class="mr-2"
                    ></status-badge>
                  </div>
                  
                  <!-- 배터리 게이지 -->
                  <div v-if="getRobotBattery(selectedRobot) !== null" class="mb-3">
                    <div class="d-flex justify-space-between align-center mb-1">
                      <span class="text-caption">
                        <v-icon size="small" color="primary">mdi-battery</v-icon>
                        배터리
                      </span>
                      <span class="text-caption font-weight-medium">
                        {{ getRobotBattery(selectedRobot) }}%
                      </span>
                    </div>
                    <v-progress-linear
                      :model-value="getRobotBattery(selectedRobot)"
                      :color="getBatteryColor(getRobotBattery(selectedRobot))"
                      height="10"
                      rounded
                      striped
                    ></v-progress-linear>
                  </div>
                  
                  <!-- 물 게이지 -->
                  <div v-if="getRobotWater(selectedRobot) !== null" class="mb-2">
                    <div class="d-flex justify-space-between align-center mb-1">
                      <span class="text-caption">
                        <v-icon size="small" color="info">mdi-water</v-icon>
                        물 잔량
                      </span>
                      <span class="text-caption font-weight-medium">
                        {{ getRobotWater(selectedRobot) }}%
                      </span>
                    </div>
                    <v-progress-linear
                      :model-value="getRobotWater(selectedRobot)"
                      color="info"
                      height="10"
                      rounded
                      striped
                    ></v-progress-linear>
                  </div>
                </div>
                  
                  <!-- 위치 정보 -->
                  <v-sheet
                    v-if="selectedRobot.position"
                    class="pa-3 mb-4 rounded"
                    color="grey-lighten-4"
                  >
                    <div class="text-caption text-medium-emphasis mb-1">현재 위치</div>
                    <div class="d-flex justify-space-between">
                      <div>
                        <div>X: {{ formatCoord(selectedRobot.position.x) }}</div>
                        <div>Y: {{ formatCoord(selectedRobot.position.y) }}</div>
                      </div>
                      <div>
                        <div>방향: {{ formatCoord(selectedRobot.position.orientation) }}°</div>
                      </div>
                    </div>
                  </v-sheet>
                  
                  <!-- 명령 버튼 -->
                  <!-- <div class="d-flex justify-space-between">
                    <v-btn
                      color="primary"
                      variant="tonal"
                      prepend-icon="mdi-crosshairs-gps"
                      @click="showMoveCommandDialog = true"
                    >
                      이동 명령
                    </v-btn>
                    
                    <v-btn
                      color="error"
                      variant="tonal"
                      prepend-icon="mdi-fire-extinguisher"
                      @click="showExtinguishDialog = true"
                    >
                      진화 명령
                    </v-btn>
                  </div> -->
                  
                  <!-- 경로 보기 버튼 -->
                  <!-- <v-btn
                    color="info"
                    variant="text"
                    block
                    class="mt-4"
                    prepend-icon="mdi-map-marker-path"
                  >
                    로봇 경로 보기
                  </v-btn> -->
                </v-card-text>
              </v-card>
            </v-col>
          </v-row>

        </v-container>
      </div>
      
      <!-- 다이얼로그: 이동 명령 -->
      <v-dialog v-model="showMoveCommandDialog" max-width="500">
        <v-card>
          <v-card-title>
            <v-icon class="mr-2">mdi-crosshairs-gps</v-icon>
            로봇 이동 명령
          </v-card-title>
          
          <v-card-text>
            <v-form ref="moveForm">
              <v-row>
                <v-col cols="6">
                  <v-text-field
                    v-model="moveCommand.x"
                    label="X 좌표"
                    type="number"
                    step="0.1"
                    hide-details
                    variant="outlined"
                    density="compact"
                  ></v-text-field>
                </v-col>
                
                <v-col cols="6">
                  <v-text-field
                    v-model="moveCommand.y"
                    label="Y 좌표"
                    type="number"
                    step="0.1"
                    hide-details
                    variant="outlined"
                    density="compact"
                  ></v-text-field>
                </v-col>
              </v-row>
            </v-form>
          </v-card-text>
          
          <v-card-actions>
            <v-spacer></v-spacer>
            <v-btn
              color="grey"
              variant="text"
              @click="showMoveCommandDialog = false"
            >
              취소
            </v-btn>
            <v-btn
              color="primary"
              @click="sendMoveCommand"
              :loading="isSendingCommand"
            >
              명령 전송
            </v-btn>
          </v-card-actions>
        </v-card>
      </v-dialog>
      
      <!-- 다이얼로그: 진화 명령 -->
      <v-dialog v-model="showExtinguishDialog" max-width="500">
        <v-card>
          <v-card-title>
            <v-icon class="mr-2" color="error">mdi-fire-extinguisher</v-icon>
            화재 진화 명령
          </v-card-title>
          
          <v-card-text>
            <div class="text-body-1 mb-4">
              현재 선택된 로봇에게 화재 진화 명령을 내립니다.
              로봇이 현재 위치에서 화재 진화를 수행합니다.
            </div>
            
            <v-alert
              type="warning"
              variant="tonal"
              border="start"
              density="compact"
            >
              로봇이 화재 발생 위치에 있는지 확인하세요!
            </v-alert>
          </v-card-text>
          
          <v-card-actions>
            <v-spacer></v-spacer>
            <v-btn
              color="grey"
              variant="text"
              @click="showExtinguishDialog = false"
            >
              취소
            </v-btn>
            <v-btn
              color="error"
              @click="sendExtinguishCommand"
              :loading="isSendingCommand"
            >
              진화 명령 전송
            </v-btn>
          </v-card-actions>
        </v-card>
      </v-dialog>
    </div>


    <!-- 화재 경고 다이얼로그 -->
  <alert-dialog
    v-model="showFireAlert"
    title="화재 경고!"
    icon="mdi-fire-alert"
    content-icon="mdi-fire"
    color="error"
    heading="화재가 발생했습니다!"
    message="시스템에서 새로운 화재가 감지되었습니다."
    button-text="확인"
    @confirm="onFireAlertConfirm"
  />
  
  <!-- 침입자 경고 다이얼로그 -->
  <alert-dialog
    v-model="showProwlerAlert"
    title="침입자 경고!"
    icon="mdi-alert-circle"
    content-icon="mdi-account-alert"
    color="warning"
    heading="침입자가 감지되었습니다!"
    message="시스템에서 새로운 침입자가 감지되었습니다."
    button-text="확인"
    @confirm="onProwlerAlertConfirm"
  />


  </template>
  
  <script>
import Sidebar from '../components/layout/Sidebar.vue';
import Topbar from '../components/layout/Topbar.vue';
import MapViewer from '../components/MapViewer.vue';
import AlertBox from '../components/common/AlertBox.vue';
import StatusBadge from '../components/common/StatusBadge.vue';
  
  import { robotService, incidentService } from '../services/api';
  import wsService from '../services/websocket';
  import AlertDialog from '../components/common/AlertDialog.vue';

  export default {
    name: 'Dashboard',
    components: {
      Sidebar,
      Topbar,
      MapViewer,
      AlertBox,
      StatusBadge,
      AlertDialog
    },
    data() {
      return {
        drawer: false,
        isLoading: false,
        isLoadingRobots: false,
        isSendingCommand: false,
        connectionStatus: {
          robots: false,
          incidents: false
        },
        robots: {},
        incidents: {},
        robotList: [],
        prowlers: {},
        selectedRobotId: null,
        showMoveCommandDialog: false,
        showExtinguishDialog: false,
        moveCommand: {
          x: 0,
          y: 0
        },
        // 구독 해제 함수
        unsubscribeCallbacks: [],
        showFireAlert: false,
        showProwlerAlert: false,
        currentFireId: null,
        currentProwlerId: null,

        // 이미 알림을 표시한 ID 추적
        notifiedFireIds: new Set(),
        notifiedProwlerIds: new Set()
      };
    },
    computed: {
      isAllConnected() {
        return this.connectionStatus.robots && this.connectionStatus.incidents;
      },
      robotCount() {
        return Object.keys(this.robots).length;
      },
      activeIncidentCount() {
        return Object.values(this.incidents).filter(incident => 
          incident.status === 'active'
        ).length;
      },
      extinguishedCount() {
        return Object.values(this.incidents).filter(incident => 
          incident.status === 'extinguished'
        ).length;
      },
      activeMissionCount() {
        // 실제 미션 데이터가 필요할 경우 추가 구현 필요
        // 지금은 간단한 더미 값 반환
        return 1;
      },
      selectedRobot() {
        if (!this.selectedRobotId || !this.robots[this.selectedRobotId]) {
          return null;
        }
        return this.robots[this.selectedRobotId];
      }
    },
    created() {
    console.log('[Dashboard] 컴포넌트 생성됨');
    
    // 인증 토큰 확인 (디버깅용)
    const token = localStorage.getItem('authToken');
    console.log('[Dashboard] 인증 토큰 존재:', !!token);
    
    // 접속 시 초기 데이터 로드
    this.fetchInitialData();
    
    // WebSocket 연결 및 구독
    this.setupWebSockets();
  },

  beforeUnmount() {
  console.log('[Dashboard] 컴포넌트 언마운트 전 정리 작업');
  // 모든 구독 해제
  this.unsubscribeAll();
  
  // Dashboard에서 벗어날 때 웹소켓 연결 종료
  wsService.disconnect();
},

    methods: {
    
        initBasicData() {
      console.log('[Dashboard] 기본 데이터 초기화');
      // 초기 데이터만 설정
      this.robots = {};
      this.incidents = {};
      
      // 테스트용 더미 데이터 추가
      this.robotList = [
        { robot_id: 'robot_1', name: '로봇 1' },
        { robot_id: 'robot_2', name: '로봇 2' }
      ];
    },


      async fetchInitialData() {
        this.isLoading = true;
        try {
          await Promise.all([
            this.fetchRobots(),
            this.fetchActiveIncidents()
          ]);
        } catch (error) {
          console.error('초기 데이터 로드 중 오류:', error);
        } finally {
          this.isLoading = false;
        }
      },
      
      async fetchRobots() {
        this.isLoadingRobots = true;
        try {
          const response = await robotService.getAllRobots();
          // robotList는 DB에서 가져온 로봇 목록
          this.robotList = response.data;
        } catch (error) {
          console.error('로봇 데이터 로드 중 오류:', error);
        } finally {
          this.isLoadingRobots = false;
        }
      },
      
      async fetchActiveIncidents() {
        try {
          const response = await incidentService.getActiveIncidents();
          // 활성 화재 목록을 Map 형태로 변환
          const incidentsMap = {};
          response.data.forEach(incident => {
            incidentsMap[incident.incident_id] = incident;
          });
          // 기존 WebSocket 데이터와 병합
          this.incidents = { ...this.incidents, ...incidentsMap };
        } catch (error) {
          console.error('활성 화재 데이터 로드 중 오류:', error);
        }
      },
      
      setupWebSockets() {

        console.log('[Dashboard] 웹소켓 설정 시작');

        // 연결 상태 변경 콜백 등록
        const unsubscribeConnection = wsService.onConnectionChange(status => {
          this.connectionStatus = status;
        });
        this.unsubscribeCallbacks.push(unsubscribeConnection);
        
        // 로봇 데이터 콜백 등록
        const unsubscribeRobots = wsService.onRobotsData(data => {
          if (data.robots) {
            this.robots = data.robots;
          }
          if (data.prowlers) {
            // 새로운 침입자 감지 로직
      const newProwlers = { ...data.prowlers };
      const oldProwlers = { ...this.prowlers };
      
      // 이전에 없던 새 침입자 찾기
      Object.keys(newProwlers).forEach(id => {
        if (!oldProwlers[id] && !this.notifiedProwlerIds.has(id)) {
          // 새 침입자 발생 알림 표시
          this.currentProwlerId = id;
          this.showProwlerAlert = true;
          this.notifiedProwlerIds.add(id); // 알림 표시 기록
          
        }
      });
      
      // 침입자 데이터 업데이트
      this.prowlers = newProwlers;
          }
        });
        this.unsubscribeCallbacks.push(unsubscribeRobots);
        
        // 화재 데이터 콜백 등록
        const unsubscribeIncidents = wsService.onIncidentsData(data => {
          if (data.incidents) {
            // 새로운 화재 감지 로직
      const newIncidents = { ...data.incidents };
      const oldIncidents = { ...this.incidents };
      
      // 활성 상태이고 이전에 없던 새 화재 찾기
      Object.keys(newIncidents).forEach(id => {
        if (newIncidents[id].status === 'active' && 
            (!oldIncidents[id] || oldIncidents[id].status !== 'active') && 
            !this.notifiedFireIds.has(id)) {
          // 새 화재 발생 알림 표시
          this.currentFireId = id;
          this.showFireAlert = true;
          this.notifiedFireIds.add(id); // 알림 표시 기록
          
        }
      });
      
      // 화재 데이터 업데이트
      this.incidents = newIncidents;
          }
        });
        this.unsubscribeCallbacks.push(unsubscribeIncidents);
        
        // WebSocket 연결 시작
        wsService.connect();
      },
      
      unsubscribeAll() {
        // 모든 구독 해제
        this.unsubscribeCallbacks.forEach(unsubscribe => unsubscribe());
        this.unsubscribeCallbacks = [];
      },
      
      reconnectWebsockets() {
        wsService.disconnect();
        setTimeout(() => {
          wsService.connect();
        }, 1000);
      },
      
      refreshData() {
        return this.fetchInitialData();
      },
      
      selectRobot(robotId) {
        this.selectedRobotId = robotId;
      },
      
      getRobotStatus(robot) {
      // robots 객체에서 해당 로봇 ID의 실시간 정보를 가져옴
      const realTimeRobot = this.robots[robot.robot_id];
      
      // 실시간 데이터가 있으면 그것을 사용, 없으면 입력된 robot 객체 사용
      if (realTimeRobot) {
        return realTimeRobot.mission_status || realTimeRobot.status?.status || 'idle';
      }
      
      // 기존 로직 유지
      return robot.mission_status || robot.status?.status || 'idle';
      },
      
      getRobotBattery(robot) {
        const battery = robot?.status?.battery;
        return battery !== undefined ? Math.round(battery) : null;
      },

      getRobotWater(robot) {
        const water = robot?.status?.water;
        return water !== undefined ? Math.round(water) : null;
      },
      
      getRobotStatusColor(robot) {
        const status = this.getRobotStatus(robot);
        switch (status) {
          case 'idle': return 'primary';
          case 'moving': return 'success';
          case 'fighting_fire': return 'error';
          default: return 'grey';
        }
      },

      getBatteryColor(batteryLevel) {
  if (batteryLevel === null || batteryLevel === undefined) return 'grey';
  if (batteryLevel > 70) return 'success';
  if (batteryLevel > 30) return 'warning';
  return 'error';
},
      
      formatCoord(value) {
        if (value === undefined || value === null) return 'N/A';
        return parseFloat(value).toFixed(2);
      },
      
      async sendMoveCommand() {
        if (!this.selectedRobotId) return;
        
        this.isSendingCommand = true;
        
        try {
          const command = {
            type: 'move_to',
            target: {
              x: parseFloat(this.moveCommand.x),
              y: parseFloat(this.moveCommand.y)
            }
          };
          
          await robotService.sendCommand(this.selectedRobotId, command);
          
          // 성공 시 다이얼로그 닫기
          this.showMoveCommandDialog = false;
          
          // Toast 메시지나 알림 표시 코드 추가 가능
          
        } catch (error) {
          console.error('이동 명령 전송 중 오류:', error);
          // 오류 메시지 표시 코드 추가 가능
        } finally {
          this.isSendingCommand = false;
        }
      },
      
      async sendExtinguishCommand() {
        if (!this.selectedRobotId) return;
        
        this.isSendingCommand = true;
        
        try {
          const command = {
            type: 'extinguish',
            target: {}
          };
          
          await robotService.sendCommand(this.selectedRobotId, command);
          
          // 성공 시 다이얼로그 닫기
          this.showExtinguishDialog = false;
          
          // Toast 메시지나 알림 표시 코드 추가 가능
          
        } catch (error) {
          console.error('진화 명령 전송 중 오류:', error);
          // 오류 메시지 표시 코드 추가 가능
        } finally {
          this.isSendingCommand = false;
        }
      }
    }
  };
  </script>
  
  <style scoped>
  .dashboard {
    display: flex;
    height: 100vh;
    overflow: hidden;
  }
  
  .dashboard-content {
    flex: 1;
    display: flex;
    flex-direction: column;
    overflow: hidden;
    padding-top: 64px; /* Topbar 높이만큼 padding 추가 */
  }
  
  .dashboard-card {
    border-radius: 8px;
    transition: transform 0.2s;
  }
  
  .dashboard-card:hover {
    transform: translateY(-2px);
  }
  
  .robot-list-card {
    height: 300px;
    overflow: auto;
  }
  
  .v-container {
    overflow-y: auto;
    flex: 1;
  }
  </style>
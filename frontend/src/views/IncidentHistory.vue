<template>
    <div class="incident-history">
      <Sidebar v-model="drawer" />
      <div class="incident-history-content">
        <Topbar 
          title="화재 이력 조회"
          subtitle="과거 화재 기록 및 진압 미션 분석"
          :refresh-action="fetchIncidents"
          @toggle-drawer="drawer = !drawer"
        />
        
        <v-container fluid class="pa-4 mt-12">
          <!-- 로딩 표시 -->
          <v-row v-if="isLoading">
            <v-col cols="12" class="text-center">
              <v-progress-circular indeterminate color="error"></v-progress-circular>
              <div class="mt-2">데이터를 불러오는 중...</div>
            </v-col>
          </v-row>
          
          <!-- 메인 콘텐츠 -->
          <v-row v-else>
            <!-- 화재 목록 -->
            <v-col cols="12" md="4">
              <v-card>
                <v-card-title class="d-flex align-center">
                  <v-icon color="error" class="mr-2">mdi-fire</v-icon>
                  화재 이력
                </v-card-title>
                
                <v-divider></v-divider>
                
                <v-list select-strategy="single-select">
                  <div v-if="incidents.length === 0" class="text-center pa-4">
                    <v-icon icon="mdi-fire-off" size="large" color="grey" class="mb-2"></v-icon>
                    <div class="text-body-1">화재 기록이 없습니다</div>
                  </div>
                  
                  <v-list-item
                    v-for="incident in incidents"
                    :key="incident.incident_id"
                    :active="selectedIncidentId === incident.incident_id"
                    @click="selectIncident(incident.incident_id)"
                  >
                    <template v-slot:prepend>
                      <v-avatar
                        :color="getIncidentStatusColor(incident.status)"
                        class="mr-2"
                      >
                        <v-icon color="white">mdi-fire</v-icon>
                      </v-avatar>
                    </template>
                    
                    <v-list-item-title>화재 #{{ incident.incident_id }}</v-list-item-title>
                    
                    <v-list-item-subtitle class="d-flex align-center mt-1">
                      <status-badge
                        :status="incident.status"
                        type="incident"
                        class="mr-2"
                      ></status-badge>
                      <span class="text-caption">
                        {{ formatDateTime(incident.detected_at) }}
                      </span>
                    </v-list-item-subtitle>
                  </v-list-item>
                </v-list>
              </v-card>
            </v-col>
            
            <!-- 선택된 화재 상세 / 미션 정보 -->
            <v-col cols="12" md="8">
              <div v-if="!selectedIncident" class="text-center mt-10">
                <v-icon icon="mdi-playlist-check" size="large" color="grey" class="mb-2"></v-icon>
                <div class="text-h6">화재를 선택하여 상세 정보를 확인하세요</div>
              </div>
              
              <template v-else>
                <!-- 화재 정보 카드 -->
                <v-card class="mb-4">
                  <v-card-title class="d-flex justify-space-between align-center">
                    <div class="d-flex align-center">
                      <v-icon color="error" class="mr-2">mdi-fire-alert</v-icon>
                      화재 정보
                    </div>
                    <status-badge
                      :status="selectedIncident.status"
                      type="incident"
                    ></status-badge>
                  </v-card-title>
                  
                  <v-divider></v-divider>
                  
                  <v-card-text>
                    <v-row>
                      <v-col cols="12" md="6">
                        <div class="text-subtitle-2 mb-1">화재 ID</div>
                        <div class="mb-3">{{ selectedIncident.incident_id }}</div>
                        
                        <div class="text-subtitle-2 mb-1">위치</div>
                        <div class="mb-3">
                          X: {{ formatCoord(selectedIncident.location?.x) }}, 
                          Y: {{ formatCoord(selectedIncident.location?.y) }}
                        </div>
                      </v-col>
                      
                      <v-col cols="12" md="6">
                        <div class="text-subtitle-2 mb-1">감지 시각</div>
                        <div class="mb-3">{{ formatDateTime(selectedIncident.detected_at) }}</div>
                        
                        <div class="text-subtitle-2 mb-1">진압 시각</div>
                        <div v-if="selectedIncident.extinguished_at" class="mb-3">
                          {{ formatDateTime(selectedIncident.extinguished_at) }}
                        </div>
                        <div v-else class="text-medium-emphasis mb-3">
                          진압되지 않음
                        </div>
                      </v-col>
                    </v-row>
                    
                    <div v-if="selectedIncident.extinguished_at" class="text-subtitle-2 mb-1">
                      진압 소요 시간
                    </div>
                    <div v-if="selectedIncident.extinguished_at" class="mb-3">
                      {{ calculateDuration(selectedIncident.detected_at, selectedIncident.extinguished_at) }}
                    </div>
                  </v-card-text>
                </v-card>
                
                <!-- 미션 정보 카드 -->
                <v-card v-if="selectedMission" class="mb-4">
                  <v-card-title class="d-flex justify-space-between align-center">
                    <div class="d-flex align-center">
                      <v-icon color="primary" class="mr-2">mdi-clipboard-check</v-icon>
                      진압 미션 정보
                    </div>
                    <status-badge
                      :status="selectedMission.status"
                      type="mission"
                    ></status-badge>
                  </v-card-title>
                  
                  <v-divider></v-divider>
                  
                  <v-card-text>
                    <v-row>
                      <v-col cols="12" md="6">
                        <div class="text-subtitle-2 mb-1">미션 ID</div>
                        <div class="mb-3">{{ selectedMission.mission_id }}</div>
                        
                        <div class="text-subtitle-2 mb-1">담당 로봇</div>
                        <div class="mb-3">{{ selectedMission.robot_id }}</div>
                      </v-col>
                      
                      <v-col cols="12" md="6">
                        <div class="text-subtitle-2 mb-1">할당 시각</div>
                        <div class="mb-3">{{ formatDateTime(selectedMission.assigned_at) }}</div>
                        
                        <div class="text-subtitle-2 mb-1">현장 도착 시각</div>
                        <div v-if="selectedMission.arrived_at" class="mb-3">
                          {{ formatDateTime(selectedMission.arrived_at) }}
                        </div>
                        <div v-else class="text-medium-emphasis mb-3">
                          도착하지 않음
                        </div>
                        
                        <div class="text-subtitle-2 mb-1">완료 시각</div>
                        <div v-if="selectedMission.completed_at" class="mb-3">
                          {{ formatDateTime(selectedMission.completed_at) }}
                        </div>
                        <div v-else class="text-medium-emphasis mb-3">
                          진행 중
                        </div>
                      </v-col>
                    </v-row>
                    
                    <div v-if="selectedMission.arrived_at" class="text-subtitle-2 mb-1">
                      현장 도착 소요 시간
                    </div>
                    <div v-if="selectedMission.arrived_at" class="mb-3">
                      {{ calculateDuration(selectedMission.assigned_at, selectedMission.arrived_at) }}
                    </div>
                    
                    <div v-if="selectedMission.completed_at" class="text-subtitle-2 mb-1">
                      진압 작업 소요 시간
                    </div>
                    <div v-if="selectedMission.completed_at && selectedMission.arrived_at" class="mb-3">
                      {{ calculateDuration(selectedMission.arrived_at, selectedMission.completed_at) }}
                    </div>
                    
                    <div v-if="selectedMission.completed_at" class="text-subtitle-2 mb-1">
                      총 미션 소요 시간
                    </div>
                    <div v-if="selectedMission.completed_at" class="mb-3">
                      {{ calculateDuration(selectedMission.assigned_at, selectedMission.completed_at) }}
                    </div>
                  </v-card-text>
                </v-card>
                
                <!-- 경로 시각화 카드 (로봇 경로 Map) -->
                <v-card>
                  <v-card-title class="d-flex justify-space-between align-center">
                    <div class="d-flex align-center">
                      <v-icon color="primary" class="mr-2">mdi-map-marker-path</v-icon>
                      로봇 이동 경로
                    </div>
                    <div v-if="isLoadingPath">
                      <v-progress-circular indeterminate size="24"></v-progress-circular>
                    </div>
                  </v-card-title>
                  
                  <v-divider></v-divider>
                  
                  <!-- 이 코드로 카드 내용 부분을 교체 -->
<v-card-text>
  <div style="border: 1px solid red; padding: 10px; margin-bottom: 10px;">
    <!-- 디버깅 정보 -->
    <div class="text-body-2 mb-2">
      <strong>디버깅 정보:</strong><br>
      경로 포인트: {{ pathPoints.length }}개<br>
      로딩 상태: {{ isLoadingPath ? '로딩 중' : '로딩 완료' }}<br>
      화재 위치: {{ selectedIncident && selectedIncident.location ? `X: ${selectedIncident.location.x}, Y: ${selectedIncident.location.y}` : '없음' }}
    </div>
  </div>

  <div v-if="isLoadingPath" class="text-center py-4">
    <v-progress-circular indeterminate color="primary"></v-progress-circular>
    <div class="mt-2">경로 데이터 로딩 중...</div>
  </div>
  
  <!-- 경로 시각화 컴포넌트 - 항상 표시 -->
  <div class="path-map-container" style="height: 400px; border: 2px solid blue;">
    <path-map-viewer
      :path-points="pathPoints"
      :incident="selectedIncident"
    ></path-map-viewer>
  </div>
</v-card-text>
                </v-card>
              </template>
            </v-col>
          </v-row>
        </v-container>
      </div>
    </div>
  </template>

<script>
import Sidebar from '../components/layout/Sidebar.vue';
import Topbar from '../components/layout/Topbar.vue';
import StatusBadge from '../components/common/StatusBadge.vue';
import { incidentService, missionService } from '../services/api';
import PathMapViewer from '../components/PathMapViewer.vue';

export default {
name: 'IncidentHistory',
components: {
  Sidebar,
  Topbar,
  StatusBadge,
  PathMapViewer,
},
data() {
  return {
    drawer: false,
    isLoading: false,
    isLoadingPath: false,
    incidents: [],
    selectedIncidentId: null,
    selectedIncident: null,
    selectedMission: null,
    pathPoints: []
  };
},
computed: {
  // 선택된 화재의 진압 시간 계산
  extinguishDuration() {
    if (!this.selectedIncident || !this.selectedIncident.extinguished_at) {
      return null;
    }
    
    const startTime = new Date(this.selectedIncident.detected_at).getTime();
    const endTime = new Date(this.selectedIncident.extinguished_at).getTime();
    
    return this.formatDuration(endTime - startTime);
  }
},
created() {
  this.fetchIncidents();
},
methods: {
  async fetchIncidents() {
    this.isLoading = true;
    try {
      const response = await incidentService.getAllIncidents();
      this.incidents = response.data;
    } catch (error) {
      console.error('화재 이력 조회 중 오류:', error);
    } finally {
      this.isLoading = false;
    }
  },
  
  async selectIncident(incidentId) {
    this.selectedIncidentId = incidentId;
    
    // 선택한 화재 정보 가져오기
    try {
      const response = await incidentService.getIncidentById(incidentId);
      this.selectedIncident = response.data;
      
      // 관련 미션 찾기
      this.fetchRelatedMission(incidentId);
    } catch (error) {
      console.error('화재 상세 정보 조회 중 오류:', error);
    }
  },
  
  async fetchRelatedMission(incidentId) {
    // 미션 목록에서 해당 화재 ID와 연결된 미션 찾기
    try {
      const response = await missionService.getAllMissions();
      // 이 화재 ID와 연결된 미션 찾기
      const relatedMission = response.data.find(mission => 
        mission.incident_id === incidentId
      );
      
      this.selectedMission = relatedMission;
      
      // 미션이 존재하면 경로 데이터 가져오기
      if (relatedMission) {
        this.fetchMissionPath(relatedMission.mission_id);
      } else {
        this.pathPoints = [];
      }
    } catch (error) {
      console.error('미션 조회 중 오류:', error);
      this.selectedMission = null;
      this.pathPoints = [];
    }
  },
  
  async fetchMissionPath(missionId) {
      this.isLoadingPath = true;
      this.pathPoints = [];
      
      try {
        console.log('미션 경로 조회 시작:', missionId);
        const response = await missionService.getMissionPath(missionId);
        console.log('미션 경로 응답 받음:', response.data);
        
        if (response.data && Array.isArray(response.data.path_points)) {
          const points = response.data.path_points;
          console.log(`${points.length}개의 경로 포인트 데이터 수신`);
          
          // 경로 데이터의 유효성 검증 및 변환
          this.pathPoints = points.map((point, index) => {
            // 좌표 값이 있는지 확인하고 숫자로 변환
            const x = point.x !== undefined ? Number(point.x) : null;
            const y = point.y !== undefined ? Number(point.y) : null;
            
            if (x === null || y === null || isNaN(x) || isNaN(y)) {
              console.warn(`경로 포인트 #${index}에 유효하지 않은 좌표가 있습니다:`, point);
            }
            
            return {
              x: x,
              y: y,
              orientation: point.orientation !== undefined ? Number(point.orientation) : 0,
              recorded_at: point.recorded_at
            };
          }).filter(point => point.x !== null && point.y !== null && !isNaN(point.x) && !isNaN(point.y));
          
          console.log(`${this.pathPoints.length}개의 유효한 경로 포인트 처리 완료`);
        } else {
          console.warn('경로 데이터가 없거나 형식이 잘못되었습니다.');
        }
      } catch (error) {
        console.error('미션 경로 조회 중 오류:', error);
      } finally {
        this.isLoadingPath = false;
      }
    },
  
  renderPathMap() {
    // 이 기능은 이제 RobotPathViewer 컴포넌트에서 처리됩니다.
    console.log('경로 시각화는 RobotPathViewer 컴포넌트에서 처리됩니다.');
  },
  
  formatDateTime(dateStr) {
    if (!dateStr) return '없음';
    
    const date = new Date(dateStr);
    return date.toLocaleString('ko-KR', {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit'
    });
  },
  
  formatCoord(value) {
    if (value === undefined || value === null) return 'N/A';
    return parseFloat(value).toFixed(2);
  },
  
  getIncidentStatusColor(status) {
    switch (status) {
      case 'active': return 'error';
      case 'assigned': return 'warning';
      case 'extinguished': return 'success';
      default: return 'grey';
    }
  },
  
  calculateDuration(startTime, endTime) {
    if (!startTime || !endTime) return '계산 불가';
    
    const start = new Date(startTime).getTime();
    const end = new Date(endTime).getTime();
    const durationMs = end - start;
    
    return this.formatDuration(durationMs);
  },
  
  formatDuration(durationMs) {
    // 밀리초를 시, 분, 초로 변환
    const seconds = Math.floor(durationMs / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);
    
    const remainingMinutes = minutes % 60;
    const remainingSeconds = seconds % 60;
    
    // 시간 형식으로 출력
    let result = '';
    if (hours > 0) {
      result += `${hours}시간 `;
    }
    if (remainingMinutes > 0 || hours > 0) {
      result += `${remainingMinutes}분 `;
    }
    result += `${remainingSeconds}초`;
    
    return result;
  }
}}
</script>

<style scoped>
.path-map-container {
  width: 100%;
  height: 400px; /* 명시적인 높이 지정 */
  margin-bottom: 16px;
  border: 1px solid #eee;
  border-radius: 4px;
  overflow: hidden;
}
</style>
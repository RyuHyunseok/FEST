"""
MQTT 토픽 발행 테스트 노드

이 노드는 MQTT를 통해 로봇의 위치와 상태를 시뮬레이션하여 발행합니다.
주요 기능:
- 로봇의 부드러운 이동 시뮬레이션
- 배터리 상태 및 로봇 상태 발행
- 화재 발생 시뮬레이션 및 화재 진압 작업 시뮬레이션
- 미션 상태 업데이트
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import random
import math
import json
import uuid

class SmoothPositionPublisher(Node):
    def __init__(self):
        super().__init__('smooth_position_publisher')

        # 로봇 ID 설정 (실제 프로젝트에서는 파라미터로 받을 수 있음)
        self.robot_id = "fest_1"

        # MQTT 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect('localhost', 1883, 60)  # MQTT 브로커 연결 (MQTT 브로커의 주소, MQTT 기본 포트, Keep-Alive 시간(60초 마다 브로커에게 ping 메시지 보냄))
        self.position_topic = f"robots/{self.robot_id}/position"  # 로봇 위치 토픽
        self.status_topic = f"robots/{self.robot_id}/status"  # 로봇 상태 토픽

        # 로봇의 초기 위치 및 이동 속도 설정
        self.x = random.uniform(0, 5)
        self.y = random.uniform(0, 5)
        self.orientation = random.uniform(0, 360)  # 초기 방향 (도)
        self.speed = 0.1  # 이동 속도 (한 번에 0.1m 이동)
        
        # 로봇 상태 설정
        self.battery = 100
        self.status = "idle"  # idle, moving, fighting_fire

        # 일회성 타이머 저장을 위한 딕셔너리
        self.one_shot_timers = {}

        # 타이머 설정
        # self.position_timer = self.create_timer(0.5, self.publish_position)  # 0.5초마다 위치 발행
        self.status_timer = self.create_timer(5.0, self.publish_status)  # 5초마다 상태 발행
        
        # 테스트용 화재 발생 시뮬레이션을 위한 일회성 타이머 (10초 후 한 번만 발생)
        self.fire_timer = self.create_timer(10.0, self.fire_once)

<<<<<<< HEAD
=======
        # 테스트용 침입자 발생 시뮬레이션(20초 후 한 번만 실행)
        self.prowler_timer = self.create_timer(20.0, self.prowler_once)

>>>>>>> origin/develop
    def fire_once(self):
            # 타이머 취소 (한 번만 실행하기 위해)
            self.fire_timer.cancel()
            # 화재 시뮬레이션 실행
            self.simulate_fire()

<<<<<<< HEAD
=======
    def prowler_once(self):

        self.prowler_timer.cancel()
        self.simulate_prowler() 
>>>>>>> origin/develop

    def publish_status(self):
        # MQTT 메시지 생성
        status_data = {
            "battery": round(self.battery, 1),
            "status": self.status
        }

        # MQTT로 상태 메시지 발행
        self.mqtt_client.publish(self.status_topic, json.dumps(status_data)) # json.dumps : dict 객체를 문자열로 변환(통신할 때는 객체가 아니라 json 형태의 문자열로 통신함)
        
        # client.publish(topic, payload) 
        # paypload : 메시지의 "내용", 브로커로 전송될 실제 데이터

        # 직렬화
        """
        직렬화(Serialization)
        : 네트워크를 통해 전송할 수 있도록 변환하는 과정
        
        예를 들어, 딕셔너리(dict) -> JSON 문자열로 변환시키는 것(직렬화)
        반대로 역직렬화(Deserialization)는 JSON 데이터를 다시 Python 객체로 변환하는 과정을 말함.

        """

        self.get_logger().info(f'Publishing status to MQTT: {status_data}')
    
<<<<<<< HEAD
=======
    def simulate_prowler(self):
        prowler_x = random.uniform(0, 10)
        prowler_y = random.uniform(0, 10)

        prowler_id = f"prowler_{uuid.uuid4().hex[:8]}"

        prowler_data = {
            "prowler_id": prowler_id,
            "location" : {"x": round(prowler_x, 2), "y": round(prowler_y)},
            "detected_at": int(self.get_clock().now().nanoseconds / 1000000)
        }

        self.mqtt_client.publish("prowler/new", json.dumps(prowler_data))
        self.get_logger().info(f'침입자 시뮬레이션 정보: {prowler_data}')

>>>>>>> origin/develop
    def simulate_fire(self):
        """테스트용 화재 발생 시뮬레이션"""
        # 랜덤하게 화재 위치 생성
        fire_x = random.uniform(0, 10)
        fire_y = random.uniform(0, 10)
        
        # 화재 ID 생성
        fire_id = f"fire_{uuid.uuid4().hex[:8]}"
        
        # 화재 정보 메시지 생성
        fire_data = {
            "incident_id": fire_id,
            "location": {"x": round(fire_x, 2), "y": round(fire_y, 2)},
            # "severity": random.choice(["low", "medium", "high"]),
            "status": "active",
            "detected_at": int(self.get_clock().now().nanoseconds / 1000000)  # 밀리초로 변환
        }
        
        # MQTT로 화재 메시지 발행
        self.mqtt_client.publish("incidents/new", json.dumps(fire_data))
        self.get_logger().info(f'Simulating new fire: {fire_data}')
        
        # 로봇 상태를 이동 중으로 변경
        self.status = "moving"
        
        # 5초 후에 화재 진압 작업 시작 (일회성 타이머 대신 일반 타이머 + 콜백에서 취소)
        timer_key = f"start_firefighting_{fire_id}"
        self.one_shot_timers[timer_key] = self.create_timer(5.0, lambda: self.start_firefighting_callback(fire_id, timer_key))
    
    def start_firefighting_callback(self, fire_id, timer_key, mission_id=None):
        """화재 진압 작업 시작 + 타이머 취소"""
        # 일회성 타이머 취소
        if timer_key in self.one_shot_timers:
            self.one_shot_timers[timer_key].cancel()
            del self.one_shot_timers[timer_key]
        
        # 화재 진압 작업 시작
        self.status = "fighting_fire"
        self.get_logger().info(f'Starting to fight fire: {fire_id}')
        
        # 미션 상태 업데이트 (현장 도착)
        if mission_id:
            mission_update = {
                "status": "in_progress",
                "arrived_at": int(self.get_clock().now().nanoseconds / 1000000)  # 밀리초로 변환
            }
            self.mqtt_client.publish(f"mission/{mission_id}/update", json.dumps(mission_update))
            self.get_logger().info(f'Mission {mission_id} status updated: arrived at scene')


        # 20초 후에 화재 진압 완료 (일회성 타이머 대신 일반 타이머 + 콜백에서 취소)
        finish_timer_key = f"finish_firefighting_{fire_id}"
        self.one_shot_timers[finish_timer_key] = self.create_timer(20, lambda: self.finish_firefighting_callback(fire_id, finish_timer_key))
    
    def finish_firefighting_callback(self, fire_id, timer_key, mission_id=None):
        """화재 진압 완료 + 타이머 취소"""
        # 일회성 타이머 취소
        if timer_key in self.one_shot_timers:
            self.one_shot_timers[timer_key].cancel()
            del self.one_shot_timers[timer_key]
        
        # 화재 진압 완료
        self.status = "idle"
        self.get_logger().info(f'Finished fighting fire: {fire_id}')
        
        # 현재 시간 (밀리초)
        current_time = int(self.get_clock().now().nanoseconds / 1000000)
        
        # 화재 상태 업데이트 메시지 생성
        update_data = {
            "incident_id": fire_id,
            "status": "extinguished",
            "extinguished_at": current_time
        }
        
        # MQTT로 화재 상태 업데이트 메시지 발행
        self.mqtt_client.publish(f"incidents/{fire_id}/status", json.dumps(update_data))
        
        # 미션 상태 업데이트 (완료)
        if mission_id:
            mission_update = {
                "status": "completed",
                "completed_at": current_time
            }
            self.mqtt_client.publish(f"missions/{mission_id}/update", json.dumps(mission_update))
            self.get_logger().info(f'Mission {mission_id} status updated: completed')

def main(args=None):
    rclpy.init(args=args)
    node = SmoothPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

## ROS2명령어(OpenCV_YOLO)
#### build
```
colcon build --symlink-install
```

#### endpoint
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

#### 실행 (빌드 후)
```
ros2 run perception_prowler_py opencv_yolo
```

#### OpenCV 창닫기
- q

#### Cuda 버전
- CUDA 12.8
- Pytorch 2.4.1 (CUDA11.8이 가장 안정적이지만 상관은 없음)

```
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

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
ros2 run perception_py opencv_yolo
```

#### OpenCV 창닫기
- q

#### Cuda 버전
```
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```
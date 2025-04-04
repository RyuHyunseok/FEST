# Common Utils

ROS2 프로젝트에서 사용되는 공통 유틸리티 패키지입니다.

## 기능

### 파일 경로 생성 (File Path Generation)

`create_file_path` 함수를 통해 ROS2 패키지 내의 파일 절대 경로를 생성할 수 있습니다.

```cpp
std::string create_file_path(const std::string& package_name, const std::string& relative_path);
```

#### 매개변수
- `package_name`: ROS2 패키지 이름
- `relative_path`: 패키지 내 파일의 상대 경로

#### 반환값
- 파일의 절대 경로 (문자열)

#### 사용 예시
```cpp
#include "common_utils/file_path.hpp"

std::string path = common_utils::create_file_path("my_package", "config/params.yaml");
```

## 의존성 패키지
- ament_cmake
- ament_index_cpp

## 빌드 환경
- C++14 이상
- CMake 3.5 이상

## 설치
이 패키지는 ROS2 워크스페이스에서 colcon build를 통해 빌드할 수 있습니다:

```bash
cd ~/ros2_ws
colcon build --packages-select common_utils
```
```
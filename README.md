# go2_control_cpp

Unitree **GO2** 사족보행 로봇을 ROS2(Foxy)로 제어하기 위한 C++ 실험용 패키지입니다. Unitree의 고수준 Sport API(`unitree_api`, `unitree_go`)를 통해 로봇에게 이동/회전 명령을 보내고, IMU·위치 상태를 구독해 **웨이포인트 기록 → 웨이포인트 자율 주행**을 구현하는 프로토타입들을 모아 둔 R&D 저장소입니다.

> ⚠️ 정식 제품이 아닌 실험/튜닝용 코드 모음입니다. 같은 기능의 여러 버전(mk0, mk1, mk2…)이 병존하며, 일부는 CMake 빌드 대상에서 제외되어 있습니다.

## 주요 기능

- **웨이포인트 기록(Recorder)**: 로봇을 조이스틱 등으로 수동 운전하면서 이동 거리/회전 각도 기준으로 위치(x, y, yaw)를 CSV로 자동 기록
- **웨이포인트 주행(Navigator)**: 기록된 CSV를 읽어 각 지점으로 회전 후 직진(또는 회전+직진 동시 제어)하며 순차 이동
- **단순 데모(Walker)**: 전진 → 정지 → 90도 회전 → 정지를 반복하는 사각형 주행 데모

모든 제어 로직은 로봇의 `/sportmodestate` 토픽(IMU yaw, 위치, 속도)을 구독해 **P/PD 제어**로 선속도(`x`)와 각속도(`z`) 명령을 계산하고, Unitree Sport API(`/api/sport/request`)로 발행하는 구조입니다.

## 기술 스택

| 항목 | 내용 |
|---|---|
| 언어 | C++17 |
| 프레임워크 | ROS2 **Foxy**, `ament_cmake` |
| 빌드 시스템 | colcon |
| 로봇 SDK | Unitree `unitree_api`, `unitree_go` (Sport API 기반 고수준 제어) |
| 주요 ROS 의존성 | `rclcpp`, `geometry_msgs`, `std_srvs`, (package.xml 상 `nav_msgs`, `tf2`, `tf2_ros`, `sensor_msgs`도 선언) |

## 패키지 구조

```
go2_control_cpp/
├── CMakeLists.txt
├── package.xml
└── src/
    ├── simple_go2_walker.cpp            # 전진-정지-회전-정지 반복 데모
    ├── manual_waypointRecorder_mk0.cpp  # 웨이포인트 기록 노드 (0.5m / 60°)
    ├── manual_waypointRecorder_mk1.cpp  # 웨이포인트 기록 노드 (1.5m / 45°, 파라미터만 다름)
    ├── waypoint_navigator_mk0.cpp       # CSV 기반 P제어 주행 (원본)
    ├── waypoint_navigator_mk1.cpp       # mk0 + 속도/딜레이 튜닝
    ├── waypoint_navigator_mk2.cpp       # 회전+직진 동시 제어, PD 제어, 정체 감지·회피
    ├── waypoinr_navigator_mk3.cpp       # 부드러운 곡선 주행 + 후진 회피 (파일명 오타, 빌드 미포함)
    ├── waypoint_navigator_mk4.cpp       # mk3 개선(속도 2.0, sendStop 지연 제거, 빌드 미포함)
    ├── waypoint_navigator_mk5.cpp       # 향상된 PD 게인 + anti-windup + 후진 회피
    └── waypoint_navigator_mk6.cpp       # mk0 계열로 회귀, 지연(delay) 제거 버전
```

### 노드별 요약

| 노드 (실행 파일) | 소스 | 설명 |
|---|---|---|
| `simple_go2_walker` | `simple_go2_walker.cpp` | 1초 전진 → 0.5초 정지 → 90° 제자리 회전(P 제어) → 0.5초 정지를 반복하는 사각형 주행 데모 |
| `manual_waypointRecorder_mk0` / `mk1` | `manual_waypointRecorder_mk*.cpp` | `/sportmodestate` 를 구독해 이동 거리(`min_distance`) 또는 회전각(`min_rotation_deg`) 조건을 만족할 때마다 웨이포인트를 기록. 서비스(`/start_recording`, `/stop_recording`, `/save_waypoints`)로 제어, 결과를 타임스탬프 CSV + 지정 파일명(`output_file`)으로 저장 |
| `waypoint_navigator_mk0` | `waypoint_navigator_mk0.cpp` | CSV(x, y)를 읽어 "목표 방향으로 회전 → 직진" 단계를 반복하며 각 웨이포인트에 순차 도달 (P 제어) |
| `waypoint_navigator_mk1` | `waypoint_navigator_mk1.cpp` | mk0과 동일 로직, 이동 속도 범위(1.0~2.0 m/s)와 웨이포인트 전환 딜레이만 변경 |
| `waypoint_navigator_mk2` | `waypoint_navigator_mk2.cpp` | 회전과 직진을 동시에 수행하는 PD 제어로 변경, 속도/헤딩 변화가 없을 때 "정체(stuck)"로 판단해 회피 동작 수행 |
| `waypoint_navigator_mk5` | `waypoint_navigator_mk5.cpp` | mk2 계열의 개선판. 더 높은 속도·수렴 게인, anti-windup, 정체 시 후진 회피 |
| `waypoint_navigator_mk6` | `waypoint_navigator_mk6.cpp` | mk0/mk1 계열(단순 P 제어)로 회귀하되 불필요한 지연(sleep)을 제거한 버전 |

> `waypoinr_navigator_mk3.cpp`(파일명 오타)와 `waypoint_navigator_mk4.cpp`는 소스는 존재하지만 `CMakeLists.txt`에 빌드 대상으로 등록되어 있지 않아 **현재 빌드되지 않습니다.**

## 동작 원리

1. **상태 입력**: `unitree_go/msg/SportModeState`의 `imu_state.rpy[2]`(yaw, rad)와 `position[0,1]`(x, y, m)을 구독
2. **제어 계산**: 목표 각도/위치와 현재 값의 오차에 P(또는 PD) 게인을 곱해 각속도(`vyaw`)·선속도(`vx`)를 산출하고 최소/최대 속도로 클램핑
3. **명령 발행**: `unitree_api::msg::Request`에 Sport API ID(이동 `1008`, 정지 `1003`)와 JSON 파라미터(`{"x":..,"y":..,"z":..}`)를 담아 `/api/sport/request`로 발행
4. (Navigator 계열) 각 웨이포인트 도착 판정(거리 허용오차) 후 다음 웨이포인트로 전환, 전체 완료 시 정지 상태 유지

## 빌드 방법

ROS2 Foxy, colcon, Unitree `unitree_api`/`unitree_go` 메시지 패키지가 설치된 워크스페이스에서:

```bash
cd ~/ros2_ws        # 이 패키지가 src/ 아래 위치한 colcon 워크스페이스
colcon build --packages-select go2_control_cpp
source install/setup.bash
```

## 사용 방법

### 1) 웨이포인트 기록

```bash
ros2 run go2_control_cpp manual_waypointRecorder_mk1 \
  --ros-args -p min_distance:=1.0 -p min_rotation_deg:=45.0 -p output_file:=waypoints.csv

ros2 service call /start_recording std_srvs/srv/Trigger
# 로봇을 조이스틱 등으로 주행
ros2 service call /stop_recording std_srvs/srv/Trigger
ros2 service call /save_waypoints std_srvs/srv/Trigger
```

### 2) 기록된 웨이포인트로 자율 주행

```bash
ros2 run go2_control_cpp waypoint_navigator_mk6 \
  --ros-args -p csv_file:=waypoints.csv
```

## 참고 / 알려진 이슈

- `src/build`, `src/install`, `src/log` 등 colcon 빌드 산출물이 `.gitignore` 설정에도 불구하고 저장소에 커밋되어 있습니다.
- 각 버전의 게인(Kp, Kd) 및 속도 제한값은 실제 로봇에서의 튜닝 실험값이므로, 다른 환경(지면 마찰, 로봇 개체차 등)에서는 재조정이 필요할 수 있습니다.
- `package.xml`의 라이선스/설명 필드는 아직 작성되지 않은 상태(`TODO`)입니다.

## 라이선스

명시된 라이선스가 없습니다(`package.xml`에 `TODO: License declaration`).

# my-car-astar

ROS 2 Humble / TurtleBot3 A* 주행 실습.

## 현재 구현

- 지도와 TF 위치를 이용한 A* 경로 계획 및 경로 주행
- `/emergency_stop` (`std_srvs/srv/SetBool`) 서비스로 수동 비상정지
- `/scan` 라이다 전방 ±20°의 0.4m 이내 장애물 감지 시 자동 비상정지
- 전방 0.5m 이상 확보 및 최신 라이다 데이터 수신 시에만 해제 허용
- 정지 즉시 기존 경로 삭제, 정지 중 0 속도 명령 반복 및 새 목표 거부
- 해제 후 자동 재출발하지 않으며 RViz에서 새 목표를 지정해야 주행
- 객체 감지와 웹캠은 현재 구현 범위에서 제외

## 빌드 및 통합 실행

저장소를 ROS2 워크스페이스의 `src/astar_navigation`에 둡니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select astar_navigation
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch astar_navigation astar_simulation.launch.py
```

가제보, 지도 서버, AMCL, A*용 RViz, 주행 노드가 함께 실행됩니다.
기존 시뮬레이션과 주행 노드를 종료한 뒤 실행하세요. Ctrl+C로 함께 종료합니다.
RViz에서 `2D Pose Estimate`로 초기 위치와 방향을 정한 후,
`2D Goal Pose`로 빈 공간의 목적지를 지정합니다.

기본 지도는 패키지에 포함되어 있으며 다음처럼 교체할 수 있습니다.

```bash
ros2 launch astar_navigation astar_simulation.launch.py map:=/absolute/path/map.yaml
```

통합 실행의 기본 ROS 도메인은 0입니다. 비상정지 터미널에도
`export ROS_DOMAIN_ID=0`을 적용해야 합니다.
다른 도메인은 `ros_domain_id:=98`로 지정하고 명령 터미널도 동일하게 맞춥니다.
노드는 `/map`, `/goal_pose`, TF `map -> base_footprint`를 사용하고
`/astar_path`와 `/cmd_vel`을 발행합니다.

## 비상정지

위 환경을 source한 다른 터미널에서 `export ROS_DOMAIN_ID=0`을 적용 후 실행합니다.

```bash
# 정지 유지
ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: true}'

# 해제: 이후 RViz에서 새 목표 지정
ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: false}'
```

이 기능은 해당 주행 노드가 실행 중일 때 해당 노드의 출력을 제어합니다.
다른 teleop/주행 노드가 `/cmd_vel`을 동시에 발행하지 않도록 합니다.
하드웨어 비상정지 장치는 아닙니다. 노드를 재시작하면 정지 상태는 초기화됩니다.

## 라이다 자동 정지

통합 런치에서 자동 활성화됩니다. 객체 종류를 분류하지 않고 거리를 감지합니다.
라이다 프레임의 +X가 로봇 전방이라는 전제이며, 전방 ±20°만 검사합니다.
거리 기준은 로봇 외곽이 아닌 라이다 센서 기준입니다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `stop_distance` | 0.4 | 자동 정지 거리(m), 이하에서 정지 |
| `release_distance` | 0.5 | 해제에 필요한 최소 거리(m) |
| `front_half_angle_deg` | 20.0 | 검사할 전방 좌우 각도(도) |
| `scan_timeout` | 1.0 | 마지막 수신 이후 허용 시간(실시간 초) |

파라미터는 주행 노드 시작 시 읽습니다. 정지 거리는 해제 거리보다 작아야 합니다.
유효한 전방 데이터가 없으면 새 목표를 거부하며,
주행 중 데이터가 끊기거나 유효하지 않으면 경로를 지우고 정지합니다.
정지 상태는 장애물이 사라져도 유지됩니다. 해제 후 새 목표가 필요합니다.
양의 무한대는 측정 범위 안에 반사체가 없는 것으로 처리하고,
NaN, 음의 무한대, 0, 유한 측정 범위 밖 값은 제외합니다.
전방 측정값이 모두 무효라면 해제를 거부합니다.

### 가제보 수동 시연

1. 통합 런치를 켜고 초기 위치와 빈 공간의 목표를 지정합니다.
2. 가제보에서 상자를 로봇 전방 센서 높이의 0.4m 이내로 배치합니다.
3. 주행 터미널의 `Laser emergency stop` 로그와 로봇 정지를 확인합니다.
4. 장애물을 남겨둔 채 해제 서비스를 호출하면 `success=False`가 나와야 합니다.
5. 장애물을 전방 0.5m 밖으로 옮깁니다. 자동 재출발하지 않아야 합니다.
6. 해제 서비스가 `success=True`인지 확인한 뒤 새 목표를 지정합니다.

옆/뒤 장애물은 이 전방 감지 기능의 대상이 아닙니다.
가제보에서 상자를 배치해 진행하는 시각적 시연은 별도 확인이 필요합니다.

## 확인 결과

- ROS2 서비스 통신: 정지/해제 응답 확인
- 정지 중 0 속도 반복 발행, 목표 거부, 경로 삭제 확인
- 해제 후 기존 경로로 자동 재출발하지 않음 확인
- colcon 패키지 빌드 성공
- 라이다 ROS2 통신 테스트 5개 통과: 자동 정지·경로 삭제·0 속도 발행,
  장애물 잔존/수신 중단 시 해제 거부, 후방 장애물 제외, 수동 정지 및 해제
- 해제해도 기존 경로로 자동 재출발하지 않음 확인
- 각도 범위·NaN·무한대·잘못된 스캔 입력 검사
- 위 테스트는 격리된 도메인의 합성 LaserScan으로 진행했으며,
  가제보 상자 배치에 의한 자동 정지 시연은 아직 확인 전

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
cd ~/turtlebot3_ws/src/astar_navigation
ROS_DOMAIN_ID=87 python3 -m unittest discover -s test -p test_lidar_safety.py -v
```

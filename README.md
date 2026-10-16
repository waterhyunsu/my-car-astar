# my-car-astar

ROS 2 Humble / TurtleBot3 A* 주행 실습.

## 현재 구현

- 지도와 TF 위치를 이용한 A* 경로 계획 및 경로 주행
- `/emergency_stop` (`std_srvs/srv/SetBool`) 서비스로 수동 비상정지
- 정지 즉시 기존 경로 삭제, 정지 중 0 속도 명령 반복 및 새 목표 거부
- 해제 후 자동 재출발하지 않으며 RViz에서 새 목표를 지정해야 주행
- 객체 감지와 웹캠은 현재 구현 범위에서 제외

## 빌드 및 주행 노드 실행

저장소를 ROS2 워크스페이스의 `src/astar_navigation`에 둡니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select astar_navigation
source install/setup.bash
ros2 run astar_navigation astar_node
```

별도로 지도 서버, 위치 추정 및 로봇/시뮬레이터를 실행해야 합니다.
노드는 `/map`, `/goal_pose`, TF `map -> base_footprint`를 사용하고
`/astar_path`와 `/cmd_vel`을 발행합니다.

## 비상정지

위 환경을 source한 다른 터미널에서 실행합니다.

```bash
# 정지 유지
ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: true}'

# 해제: 이후 RViz에서 새 목표 지정
ros2 service call /emergency_stop std_srvs/srv/SetBool '{data: false}'
```

이 기능은 해당 주행 노드가 실행 중일 때 해당 노드의 출력을 제어합니다.
다른 teleop/주행 노드가 `/cmd_vel`을 동시에 발행하지 않도록 합니다.
하드웨어 비상정지 장치는 아닙니다. 노드를 재시작하면 정지 상태는 초기화됩니다.

## 확인 결과

- ROS2 서비스 통신: 정지/해제 응답 확인
- 정지 중 0 속도 반복 발행, 목표 거부, 경로 삭제 확인
- 해제 후 기존 경로로 자동 재출발하지 않음 확인
- colcon 패키지 빌드 성공
- 가제보 실제 주행 중 정지 검증은 아직 진행 전

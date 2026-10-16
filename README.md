# my-car-astar

ROS 2 Humble / TurtleBot3 A* 주행 실습.

## 현재 구현

- 지도와 TF 위치를 이용한 A* 경로 계획 및 경로 주행
- `/emergency_stop` (`std_srvs/srv/SetBool`) 서비스로 수동 비상정지
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

## 확인 결과

- ROS2 서비스 통신: 정지/해제 응답 확인
- 정지 중 0 속도 반복 발행, 목표 거부, 경로 삭제 확인
- 해제 후 기존 경로로 자동 재출발하지 않음 확인
- colcon 패키지 빌드 성공
- 가제보 실제 주행 중 정지 검증은 아직 진행 전

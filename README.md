# naive-pp ROS 2 인터페이스 명세

1. naive-pp는 외부 에이전트가 보낸 지시문에서 장소명을 알아낸다. <br>
2. 장소명은 로봇의 occupancy map에 사람이 붙인 장소 정보와 rule base로 맞춘다. <br>
3. 토폴로지 그래프를 통해 거쳐갈 방과 출입구를 정하고, 점유 지도 위에 A*로 경로를 만들어 G1 하위제어를 수행한다. <br>

- VLN/VLM 모델, 카메라 영상, 시뮬레이터 gt값은 쓰지 않는다. 
- 도착 후 확인 동작도 마찬가지로 하지 않는다.
- 기준 환경 : Ubuntu GPU의 Isaac Sim 5.1(Fast DDS)

<br>

## 1. 지시 1회의 흐름

1. 외부 에이전트가 /vln/instruction으로 지시문을 보낸다.
2. naive_pp가 지시문에서 목표 장소를 찾고 /vln/feedback으로 accepted를 보낸다.
3. localizer가 위치를 찾은 상태(/localization/status의 state가 localized)이면, naive_pp가 목표 좌표, 토폴로지 경로, A* 경로를 만들어 /plan으로 내고 planned를 보낸다.
4. path_follower가 /plan을 따라 /cmd_vel을 내고, g1_sim(실로봇에서는 보행 API)이 G1을 걷게 한다.
5. 걷는 동안 naive_pp는 5초마다 progress를 보낸다. 지도에 없는 장애물이 LiDAR에 보여 앞 경로가 막히면 다시 계획한다(replanning, replanned).
6. path_follower가 경로 끝 도착(/nav/status의 reached)을 알리고 로봇이 목표에서 0.5 m 안이면, naive_pp가 done을 보낸다.

<br>

## 2. 노드 구성

### 2.1 로봇 내부 실행 노드

| 노드 | 실행 | 역할 |
|---|---|---|
| g1_sim | g1_factory.isaac.isaac_sim_node (Isaac Sim, 별도 컨테이너) <br> 또는 g1_factory.ros2.sim_node (MuJoCo, launch의 sim:=true). <br> 실로봇에서는 센서 드라이버와 보행 API 브리지 | 센서 발행, /cmd_vel을 G1 보행 정책으로 전달 |
| map_file | g1_factory.ros2.map_file_node | 지도 묶음의 점유 지도와 장소 그래프를 시작할 때 1회 발행 |
| localizer | g1_factory.ros2.localizer_node | LiDAR 스캔을 지도 묶음의 점유 지도에 맞춰 TF map → odom 발행 |
| naive_pp | naive_pp.ros.pp_node | 지시 해석, 목표 좌표, 토폴로지 경로, A*, 재계획, 피드백 |
| path_follower | g1_factory.ros2.follower_node | /plan을 순수 추종으로 따라 /cmd_vel 발행 |


### 2.2 에이전트 쪽 노드

| 노드 | 실행 | 역할 |
|---|---|---|
| external_agent | 외부 에이전트, 또는 scripts/send_instruction.py | 지시 발행, 피드백 구독 |
| naive_pp_client | naive_pp.ros.client_node | 시험용 에이전트. 위치를 찾은 뒤 지시를 1회 보내고, 실행을 기록하고 끝남 (launch의 instruction:=...) |


<br>

## 3. 로봇 내부 토픽

### 3.1 센서 (g1-sim)

| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 |
|---|---|---|
| /livox/lidar | sensor_msgs/msg/PointCloud2 | g1_sim → localizer (지도 정합), naive_pp (지도에 없는 장애물) |
| /odom | nav_msgs/msg/Odometry | g1_sim → localizer, naive_pp, path_follower, naive_pp_client |
| /tf | tf2_msgs/msg/TFMessage | g1_sim (odom → base_link) → localizer, naive_pp, path_follower |
| /tf_static | tf2_msgs/msg/TFMessage | g1_sim (base_link → livox_frame, camera_link) → localizer, naive_pp, path_follower |
| /clock | rosgraph_msgs/msg/Clock | g1_sim → use_sim_time 노드 전부 |
| /livox/imu | sensor_msgs/msg/Imu | g1_sim → (없음) |
| /joint_states | sensor_msgs/msg/JointState | g1_sim → (없음) |
| /camera/camera/color/image_raw | sensor_msgs/msg/Image rgb8 | g1_sim → (없음) |
| /camera/camera/aligned_depth_to_color/image_raw | sensor_msgs/msg/Image 16UC1 (mm) | g1_sim → (없음) |
| /camera/camera/color/camera_info, /camera/camera/aligned_depth_to_color/camera_info | sensor_msgs/msg/CameraInfo | g1_sim → (없음) |
| /map | nav_msgs/msg/OccupancyGrid | g1_sim → (없음). 시뮬레이터 장면에서 만든 정답 지도라 naive-pp는 쓰지 않음. frame_id는 map이지만 시뮬레이터 세계 좌표라 /slam_map과 겹쳐 보면 어긋남 |
| /semantic_map | std_msgs/msg/String JSON | isaac_sim_node만 발행 → (없음). 시뮬레이터 세계 좌표의 구역, 물체 정답 |
| /sim/status | std_msgs/msg/String JSON | g1_sim → (없음). 시뮬레이터 전용 |

### 3.2 지도 / 위치

| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 |
|---|---|---|
| /slam_map | nav_msgs/msg/OccupancyGrid | map_file → (RViz, 에이전트가 볼 수 있음) |
| /slam/semantic_map | std_msgs/msg/String JSON | map_file → (RViz, 에이전트가 볼 수 있음). 방/통로, 출입구, 기준점 |
| /initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | 사람(RViz 2D Pose Estimate) → localizer |
| /tf | tf2_msgs/msg/TFMessage | localizer (map → odom) → naive_pp, path_follower |
| /localization/pose | geometry_msgs/msg/PoseWithCovarianceStamped | localizer → naive_pp_client |
| /localization/status | std_msgs/msg/String JSON | localizer → naive_pp, naive_pp_client |

### 3.3 계획 / 제어

| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 |
|---|---|---|
| /plan | nav_msgs/msg/Path | naive_pp → path_follower, naive_pp_client |
| /nav/status | std_msgs/msg/String JSON | path_follower → naive_pp |
| /cmd_vel | geometry_msgs/msg/Twist | path_follower → g1_sim |
| /naive_pp/goal | geometry_msgs/msg/PoseStamped | naive_pp → (RViz) |
| /naive_pp/route | nav_msgs/msg/Path | naive_pp → (RViz). 로봇 → 지나갈 출입구들 → 목표 |

### 3.4 좌표계

| 변환 | 발행 노드 | 내용 |
|---|---|---|
| map → odom | localizer | 지도 묶음의 좌표계(지도를 만들 때 로봇이 출발한 자세가 원점)와 odom 사이. 첫 위치를 찾기 전에는 없음 |
| odom → base_link | g1_sim | 오도메트리. 실로봇에서는 보행 오도메트리 또는 FAST-LIO2 |
| base_link → livox_frame, camera_link → camera_color_optical_frame | g1_sim (/tf_static) | 센서 장착 위치 |


<br>

## 4. 에이전트 통신 토픽

| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 |
|---|---|---|
| /vln/instruction | std_msgs/msg/String 자연어 또는 JSON | external_agent → naive_pp |
| /vln/feedback | std_msgs/msg/String JSON | naive_pp → external_agent |


### 4.1 지시 형식

| 형식 | 예 |
|---|---|
| 자연어 (영어, 한국어) | Go to the operations control room and check whether the status indicator lights on the server rack are on. |
| JSON + 자연어 | {"task_id": "t1", "instruction": "펌프실로 가"} |
| 장소 지정 (id, 라벨, 별칭) | {"task_id": "t1", "place": "office_control"} |
| 좌표 지정 (map 좌표) | {"task_id": "t1", "goal": {"x": 58.6, "y": -8.7, "yaw": -1.57}} |
| 경유지 (마지막이 목적지) | {"task_id": "t1", "subgoals": ["bridge", "office_lounge"]} |
| VLN 에이전트 형식 | {"task_id": "t1", "steps": [...], "target": "...", "question": "..."}. instruction이 없으면 steps의 goal, target, question을 이어 붙인 글을 해석 |
| 취소 | {"cancel": true}, stop, 취소 등. 멈추고 그 자리 유지 |


### 4.2 피드백 이벤트

모든 이벤트에 task_id, state, sim_time_s가 들어간다. 끝 상태는 done, failed, canceled

| state | 시점 | 필드 |
|---|---|---|
| accepted | 지시 해석 직후 | instruction, target {kind, id, label}, match (exact, partial, keyword, id, pose), matched, waypoints, also_mentioned |
| waiting | 위치를 아직 못 찾았을 때 5초마다, 위치를 잃었을 때 | reason, localization |
| planned | 첫 경로 | target, goal [x, y, yaw], route, route_text, doors, search (route, map), length_m, poses, expansions, planning_s, start, start_place |
| replanning | 다시 계획하기 직전 | reason |
| replanned | 새 경로 | planned와 같음 |
| plan_failed | 경로를 못 만들었을 때. 2초 뒤 다시 시도, 12번 넘게 실패하면 failed | reason, attempt |
| progress | 따라가는 동안 5초마다 | remaining_m, place, pose, travelled_m, lidar_obstacle_cells |
| done | 도착 | pose, place, in_target_place, goal_distance_m, elapsed_s, travelled_m, replans, path_length_m |
| failed | 실패 (경로 없음, 시간 초과 1800 s, 해석 실패) | done의 필드 + reason |
| canceled | 취소, 새 지시 | done의 필드 + reason |

실제 측정 실행의 이벤트(planned의 일부 필드는 생략).

```text
{"task_id": "323fd69d", "state": "accepted", "sim_time_s": 3.8, "instruction": "Go to the operations control room and check whether the status indicator lights on the server rack are on.", "target": {"kind": "place", "id": "office_control", "label": "Operations control"}, "match": "exact", "matched": "operations control room", "waypoints": [], "also_mentioned": []}
{"task_id": "323fd69d", "state": "planned", "sim_time_s": 3.8, "target": {"kind": "place", "id": "office_control", "label": "Operations control"}, "goal": [58.45, -6.75, -1.286], "route_text": "Material storage -> Warehouse main aisle -> Warehouse-office bridge -> Office corridor -> Operations control", "doors": [[2.95, 0.7], [35.7, -1.65], [39.6, -1.65], [57.55, -3.6]], "search": "route", "length_m": 62.12, "poses": 624, "expansions": 32886, "planning_s": 0.219, "start": [-0.22, 2.06, -0.05], "start_place": "Material storage"}
{"task_id": "323fd69d", "state": "progress", "sim_time_s": 4.0, "remaining_m": 62.2, "place": "Material storage", "pose": [-0.22, 2.07, -0.04], "travelled_m": 0.0, "lidar_obstacle_cells": 0}
{"task_id": "323fd69d", "state": "done", "sim_time_s": 166.61, "pose": [58.405, -6.608, -1.239], "place": "Operations control", "in_target_place": true, "goal_distance_m": 0.149, "elapsed_s": 162.8, "travelled_m": 62.1, "replans": 0, "path_length_m": 62.12}
```

<br>

## 5. 데이터 크기

대역폭은 평균 크기와 주기를 곱한 값(1 KB = 1000 B).

### 5.1 로봇 센서 (g1_sim)

| 토픽 | 데이터 | 메시지 크기 | 주기 | 대역폭 | naive-pp 사용 |
|---|---|---|---|---|---|
| /camera/camera/color/image_raw | 640×480 rgb8 | 921,672 B (고정) | 30 Hz | 27.65 MB/s | X |
| /camera/camera/aligned_depth_to_color/image_raw | 640×480 16UC1 | 614,472 B (고정) | 30 Hz | 18.43 MB/s | X |
| 카메라 camera_info 2개 | K, D, R, P | 384 B 각 | 30 Hz | 11.5 KB/s 각 | X |
| /livox/lidar | 20,000 광선 중 반사점, 점당 26 B | Isaac Sim 470.0 KB (456.2–479.7 KB). 이번 MuJoCo 505.9 KB (476.4–520.2 KB) | 10 Hz | 4.70 MB/s (MuJoCo 5.06 MB/s) | O |
| /livox/imu | 각속도, 가속도 | 324 B | 200 Hz | 64.8 KB/s | X |
| /odom | 자세, 속도, 공분산 | 724 B | 50 Hz | 36.2 KB/s | O |
| /joint_states | 12관절 위치, 속도, 토크 | 644 B | 50 Hz | 32.2 KB/s | X |
| /tf | g1_sim odom → base_link 50 Hz와 localizer map → odom 20 Hz | 92–100 B | 70 Hz | 6.8 KB/s | O |
| /tf_static | 센서 장착 변환 3개 | 316 B | 1회 | - | O |
| /clock | 시뮬레이션 시간 | 12 B | 100 Hz | 1.2 KB/s | O |
| /map | 시뮬레이터 정답 지도 0.1 m, 780×440 셀 | 343,304 B | 1회 | - | X |
| /sim/status | JSON | MuJoCo 496–512 B, Isaac Sim 436–452 B | 2 Hz | 1.0 KB/s | X |


### 5.2 naive-pp 관련 토픽 (이번 실행)

| 토픽 | 데이터 | 메시지 크기 | 주기, 횟수 | 대역폭 |
|---|---|---|---|---|
| /slam_map | campus_isaac 점유 지도 0.1 m, 796×482 셀 | 383,776 B | 1회 | - |
| /slam/semantic_map | JSON: 방 9개, 장소 13개, 연결 12개, 기준점 4개 | 7,280 B | 1회 | - |
| /localization/pose | 자세와 공분산 | 364 B | 10 Hz | 3.6 KB/s |
| /localization/status | JSON | 152–276 B | 2 Hz | 0.5 KB/s |
| /plan | 0.1 m 간격 PoseStamped, 자세당 72 B | 44,956 B (624개, 62.1 m) | 계획마다 1개 (이번 1회) | - |
| /naive_pp/goal | 목표 자세 | 76 B | 계획마다 (이번 1회) | - |
| /naive_pp/route | 로봇, 출입구 4개, 목표 | 460 B | 계획마다 (이번 1회) | - |
| /nav/status | JSON | 236–240 B | 1 Hz | 0.2 KB/s |
| /cmd_vel | 전진, 옆, 회전 속도 | 52 B | 20 Hz | 1.0 KB/s |
| /vln/instruction | 이번 지시문 (자연어 106자) | 116 B | 1회 | - |
| /vln/feedback | JSON. accepted 379 B, planned 668 B, progress 200–214 B, done 269 B | 200–668 B | 작업 1개에 34회 (accepted, planned, progress 31, done) | 0.05 KB/s |

- 로봇 내부에서 도는 토픽 주기는 Isaac Sim 기준 약 51 MB/s입니다. 그중 90 %가 카메라 영상이고, naive-pp에서는 카메라 관련 토픽은 구독하지 않는다.
- naive-pp 쪽 노드가 구독하는 큰 토픽은 /livox/lidar 하나(4.7–5.1 MB/s) 이고 localizer와 naive_pp가 각각 받는다.
- 로봇 밖으로 오가는 양(/vln/instruction, /vln/feedback)은 작업 1개에 약 8 KB / progress가 5초마다 나가므로 걷는 시간에 비례한다.

<br>

## 6. Publisher 옵션

| 토픽 | 발행 노드 | 주기, 발행 시점 | QoS |
|---|---|---|---|
| /livox/lidar | g1_sim | 10 Hz | BestEffort Volatile KeepLast 5 |
| /odom | g1_sim | 50 Hz | Reliable Volatile KeepLast 10 |
| /tf (odom → base_link) | g1_sim | 50 Hz | Reliable Volatile KeepLast 100 |
| /tf_static | g1_sim | 시작할 때 1회 | Reliable TransientLocal KeepLast 1 |
| /clock | g1_sim | 100 Hz | BestEffort Volatile KeepLast 1 |
| /livox/imu | g1_sim | 200 Hz | BestEffort Volatile KeepLast 50 |
| /joint_states | g1_sim | 50 Hz | Reliable Volatile KeepLast 10 |
| 카메라 영상과 camera_info 4개 | g1_sim | 30 Hz (Isaac Sim --camera-rate 기본값. naive-pp launch의 MuJoCo는 2 Hz) | BestEffort Volatile KeepLast 5 |
| /map | g1_sim | 시작할 때 1회 | Reliable TransientLocal KeepLast 1 |
| /sim/status | g1_sim | 2 Hz | Reliable Volatile KeepLast 10 |
| /slam_map, /slam/semantic_map | map_file | 시작할 때 1회 | Reliable TransientLocal KeepLast 1 |
| /tf (map → odom) | localizer | 20 Hz, 첫 위치를 찾은 뒤부터 | Reliable Volatile KeepLast 100 |
| /localization/pose | localizer | 10 Hz, 첫 위치를 찾은 뒤부터 | Reliable Volatile KeepLast 10 |
| /localization/status | localizer | 스캔 정합마다 (2 Hz) | Reliable Volatile KeepLast 10 |
| /plan | naive_pp | planned, replanned마다 1개. 멈출 때는 현재 자세 1점 | Reliable Volatile KeepLast 1 |
| /vln/feedback | naive_pp | 이벤트마다, progress는 5초마다 | Reliable Volatile KeepLast 50 |
| /naive_pp/goal, /naive_pp/route | naive_pp | planned, replanned마다 | Reliable TransientLocal KeepLast 1 |
| /cmd_vel | path_follower | 20 Hz. 경로가 있는 동안(제자리 유지 포함), 끝난 뒤 0 명령 몇 번 후 멈춤 | Reliable Volatile KeepLast 10 |
| /nav/status | path_follower | following 1 Hz, reached와 failed는 1회 | Reliable Volatile KeepLast 50 |
| /vln/instruction | external_agent | 지시마다 | Reliable Volatile KeepLast 10 |

- 센서는 BestEffort와 얕은 큐를 쓴다. 늦게 도착한 스캔은 쓸모가 없으므로 다시 보내지 않는다.
- 명령, 상태, 이벤트는 Reliable. 지시나 도착 이벤트가 하나라도 빠지면 작업이 멈춘다.
- 지도와 정적 TF, 목표 표시는 TransientLocal이라 늦게 시작한 노드도 마지막 값을 받는다.
- /plan의 header.stamp는 작업마다 하나인 plan id. 재계획한 경로도 같은 stamp로 보내므로, path_follower는 이를 새 작업이 아닌 경로 교체로 처리한다.

<br>

## 7. Subscriber 옵션

| 구독 노드 | 토픽 | QoS | 처리 |
|---|---|---|---|
| naive_pp | /vln/instruction | Reliable KeepLast 10 | 지시 해석 → accepted. 진행 중 작업이 있으면 canceled로 끝내고 새 작업 시작 |
| | /odom + /tf map → odom | Reliable KeepLast 10 | /odom에 최신 map → odom을 합성한 map 좌표 자세. 계획과 진행 판단은 이 자세만 씀 |
| | /localization/status | Reliable KeepLast 10 | localized가 될 때까지 계획을 미룸(waiting). lost가 되면 제자리에 멈추고 다시 찾기를 기다림 |
| | /livox/lidar | BestEffort KeepLast 5 | 따라가는 동안만 처리. 스캔 stamp의 TF로 map 좌표로 옮기고, 거리 0.3–8 m, 높이 0.15–1.7 m 점 중 지도 장애물에서 0.25 m 넘게 떨어진 점을 15초 동안 장애물로 둠. 앞 8 m 경로가 막히면 다시 계획(2초 간격) |
| | /nav/status | Reliable KeepLast 50 | 현재 plan id의 reached: 목표에서 0.5 m 안이면 done, 아니면 다시 계획. failed: 다시 계획 |
| localizer | /livox/lidar | BestEffort KeepLast 5 | 최근 스캔을 odom 좌표로 옮겨 2 Hz로 점유 지도와 정합 |
| | /odom | Reliable KeepLast 10 | 정합 사이의 움직임 |
| | /initialpose | Reliable KeepLast 10 | 이 자세(1.5 m, 20° 안)에서 위치 찾기를 다시 시작 |
| path_follower | /plan | Reliable KeepLast 1 | 경로 교체. 같은 plan id면 재계획으로 처리, 1점 경로는 제자리 유지 |
| | /odom + /tf map → odom | Reliable KeepLast 10 | map 좌표 자세. 0.5 s 넘게 끊기거나 map → odom이 없으면 멈춤 |
| g1_sim | /cmd_vel | Reliable KeepLast 10 | 0.5 s 넘게 명령이 없으면 0. 보행 정책은 50 Hz로 최신 명령을 씀 |
| external_agent | /vln/feedback | Reliable KeepLast 50 | done, failed, canceled를 받으면 작업 끝 |
| use_sim_time 노드 전부 | /clock | BestEffort KeepLast 1 | rclpy TimeSource 기본 |

naive_pp, localizer, path_follower는 /tf와 /tf_static을 tf2_ros 기본 QoS로 구독한다.

<br>

## 8. 노드별 발행과 구독

### 8.1 로봇 노드

| 노드 | 발행 | 구독 |
|---|---|---|
| g1_sim | /clock, /livox/lidar, /livox/imu, /odom, /joint_states, /tf, /tf_static, 카메라 4개, /map, /sim/status (Isaac Sim은 /semantic_map도) | /cmd_vel |
| map_file | /slam_map, /slam/semantic_map | (없음) |
| localizer | /tf (map → odom), /localization/pose, /localization/status | /livox/lidar, /odom, /initialpose, /tf, /tf_static |
| naive_pp | /plan, /vln/feedback, /naive_pp/goal, /naive_pp/route | /vln/instruction, /odom, /localization/status, /livox/lidar, /nav/status, /tf, /tf_static |
| path_follower | /cmd_vel, /nav/status | /plan, /odom, /tf, /tf_static |

### 8.2 에이전트 쪽 노드

| 노드 | 발행 | 구독 |
|---|---|---|
| external_agent | /vln/instruction | /vln/feedback |
| naive_pp_client | /vln/instruction | /vln/feedback, /localization/status, /localization/pose, /odom, /plan |

### 8.3 로봇 내부와 외부

g1_sim(실로봇에서는 센서 드라이버와 보행 제어), map_file, localizer, naive_pp, path_follower는 모두 로봇 안에서 돈다. <br> LiDAR와 영상 같은 대용량 토픽도 로봇 안에서만 돈다. 로봇 밖과 주고받는 것은 /vln/instruction(에이전트 → 로봇)과 /vln/feedback(로봇 → 에이전트) 두 토픽뿐이고 VLM 서버나 HTTP 호출은 없다.

<br>

## 9. 실행


```bash
# 터미널 1: Isaac Sim 컨테이너 (map -> odom은 localizer가 내므로 --no-map-tf)
bash scripts/run_isaac_sim.sh --spawn -12 3.8 0 --no-map-tf
# 터미널 2: naive-pp 컨테이너 (Fast DDS가 기본값, G1_DDS는 지정하지 않음)
bash scripts/g1_docker.sh ros2 launch naive-pp/launch/naive_pp.launch.py package:=/ws/naive-pp/maps/campus_isaac initial_pose:=-0.2,2.2,0
# 터미널 3: 외부 에이전트 대신 지시 1회
bash scripts/g1_docker.sh python3 scripts/send_instruction.py "Go to the operations control room."
```

- 토픽명은 launch의 remaps:=로 바꿀 수 있다.
- VLN 스택(launch/g1_hvln.launch.py)과 동시에 띄우지 않게 주의. 둘 다 /vln/instruction을 구독함

<br>

## 10. 외부 에이전트 Publisher / Subscriber 코드

에이전트는 아래 두 토픽만 있으면 됩니다(scripts/send_instruction.py에서 발췌).

```python
COMMAND = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
EVENTS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
self.pub = self.create_publisher(String, "/vln/instruction", COMMAND)
self.create_subscription(String, "/vln/feedback", self.on_feedback, EVENTS)

def tick(self):  # 구독자가 연결된 뒤 1회 발행
    if not self.sent and self.pub.get_subscription_count() > 0:
        self.pub.publish(String(data=self.payload))
        self.sent = True

def on_feedback(self, msg):
    event = json.loads(msg.data)
    if event.get("state") in ("done", "failed", "canceled"):
        self.done = event["state"]
```



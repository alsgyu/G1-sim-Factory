| 토픽 | 메시지 타입 | 발행 노드 → 구독 노드 | 데이터 | 메시지 크기(평균, 최소–최대) | 주기(sim) | 대역폭(평균×주기) |
|---|---|---|---|---|---:|---:|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | g1_sim → vln_agent, vln_navigator | 640×480 rgb8 (921,600 B 픽셀) | 921.67 KB (921.67 KB–921.67 KB) | 30 Hz (측정 30.0) | 27.65 MB/s |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | g1_sim → vln_agent, vln_navigator | 640×480 16UC1 mm (614,400 B 픽셀) | 614.47 KB (614.47 KB–614.47 KB) | 30 Hz (측정 30.0) | 18.43 MB/s |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | g1_sim → vln_navigator | 20,000 ray/scan 중 반사점만, 26 B/점 (x,y,z,intensity,tag,line,timestamp) | 470.02 KB (456.15 KB–479.66 KB) | 10 Hz (측정 10.0) | 4.70 MB/s |
| `/livox/imu` | `sensor_msgs/msg/Imu` | g1_sim → (없음) | 각속도+가속도, orientation 없음 | 324 B (324 B–324 B) | 200 Hz (측정 200.0) | 64.80 KB/s |
| `/odom` | `nav_msgs/msg/Odometry` | g1_sim → external_agent, path_follower, vln_agent, vln_navigator | pose+twist(+공분산 72 double) | 724 B (724 B–724 B) | 50 Hz (측정 50.0) | 36.20 KB/s |
| `/joint_states` | `sensor_msgs/msg/JointState` | g1_sim → (없음) | 다리 12관절 position/velocity/effort | 644 B (644 B–644 B) | 50 Hz (측정 50.0) | 32.20 KB/s |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | g1_sim → vln_agent, vln_navigator | K/D/R/P, plumb_bob | 384 B (384 B–384 B) | 30 Hz (측정 30.0) | 11.52 KB/s |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | g1_sim → (없음) | color와 동일 내부 파라미터 | 384 B (384 B–384 B) | 30 Hz (측정 30.0) | 11.52 KB/s |
| `/tf` | `tf2_msgs/msg/TFMessage` | g1_sim → vln_agent, vln_navigator | odom→base_link 1개 | 100 B (100 B–100 B) | 50 Hz (측정 50.0) | 5.00 KB/s |
| `/clock` | `rosgraph_msgs/msg/Clock` | g1_sim → external_agent, path_follower, vln_agent, vln_navigator | sim time | 12 B (12 B–12 B) | 100 Hz (측정 100.0) | 1.20 KB/s |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | path_follower → g1_sim | vx, vy, yaw rate | 52 B (52 B–52 B) | 20 Hz (측정 19.3) | 1.04 KB/s |
| `/sim/status` | `std_msgs/msg/String` | g1_sim → external_agent | JSON (구역·충돌·넘어짐·RTF·처리시간) | 436 B (428 B–444 B) | 2 Hz (측정 2.0) | 873 B/s |
| `/map` | `nav_msgs/msg/OccupancyGrid` | g1_sim → external_agent, vln_navigator | 0.1 m 격자 780×340 = 265,200 셀 int8 | 265.30 KB (265.30 KB–265.30 KB) | 이벤트 (1회/에피소드) | - |
| `/vln/decision_image` | `sensor_msgs/msg/CompressedImage` | vln_navigator → external_agent | JPEG q80 640×480, VLM 입력 영상 + 번호 웨이포인트 | 25.69 KB (20.34 KB–29.42 KB) | 이벤트 (5회/에피소드) | - |
| `/vln/inspection_image` | `sensor_msgs/msg/CompressedImage` | vln_agent → external_agent | JPEG q85 640×480 | 22.94 KB (22.94 KB–22.94 KB) | 이벤트 (1회/에피소드) | - |
| `/plan` | `nav_msgs/msg/Path` | vln_navigator → external_agent, path_follower | 0.1 m 간격 PoseStamped 목록 (경로 길이에 비례) | 2.04 KB (100 B–5.64 KB) | 이벤트 (9회/에피소드) | - |
| `/vln/feedback` | `std_msgs/msg/String` | vln_agent → external_agent | JSON 진행/결과 이벤트 | 441 B (88 B–2.55 KB) | 이벤트 (13회/에피소드) | - |
| `/vln/nav_request` | `std_msgs/msg/String` | vln_agent → vln_navigator | JSON 단계 계획·대상·명판 글자 / 취소 | 416 B (416 B–416 B) | 이벤트 (1회/에피소드) | - |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | g1_sim → vln_agent, vln_navigator | map→odom, base_link→camera_link→optical, base_link→livox_frame | 396 B (396 B–396 B) | 이벤트 (1회/에피소드) | - |
| `/vln/nav_status` | `std_msgs/msg/String` | vln_navigator → vln_agent | JSON VLN 결정·단계 완료·대상 위치·도착 | 365 B (88 B–740 B) | 이벤트 (10회/에피소드) | - |
| `/nav/status` | `std_msgs/msg/String` | path_follower → external_agent, vln_agent, vln_navigator | JSON (planned/following/reached/failed) | 224 B (168 B–240 B) | 이벤트 (90회/에피소드) | - |
| `/vln/instruction` | `std_msgs/msg/String` | external_agent → vln_agent | UTF-8 지시문 또는 JSON | 96 B (96 B–96 B) | 이벤트 (1회/에피소드) | - |
| `/vln/subgoals` | `geometry_msgs/msg/PoseArray` | - → (없음) | 하위목표 자세 목록 (oracle 기준선 전용) | - | 이벤트 (0회/에피소드) | - |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | - → (없음) | map 좌표 목표 자세 (oracle 기준선 전용) | - | 이벤트 (0회/에피소드) | - |

| 토픽 | 발행 QoS (노드) | 구독 QoS (노드) |
|---|---|---|
| `/camera/camera/color/image_raw` | BestEffort/Volatile/UNKNOWN (g1_sim) | BestEffort/Volatile/UNKNOWN (vln_agent)<br>BestEffort/Volatile/UNKNOWN (vln_navigator) |
| `/camera/camera/aligned_depth_to_color/image_raw` | BestEffort/Volatile/UNKNOWN (g1_sim) | BestEffort/Volatile/UNKNOWN (vln_agent)<br>BestEffort/Volatile/UNKNOWN (vln_navigator) |
| `/livox/lidar` | BestEffort/Volatile/UNKNOWN (g1_sim) | BestEffort/Volatile/UNKNOWN (vln_navigator) |
| `/livox/imu` | BestEffort/Volatile/UNKNOWN (g1_sim) | - |
| `/odom` | Reliable/Volatile/UNKNOWN (g1_sim) | Reliable/Volatile/UNKNOWN (vln_agent)<br>Reliable/Volatile/UNKNOWN (vln_navigator)<br>Reliable/Volatile/UNKNOWN (path_follower)<br>Reliable/Volatile/UNKNOWN (external_agent) |
| `/joint_states` | Reliable/Volatile/UNKNOWN (g1_sim) | - |
| `/camera/camera/color/camera_info` | BestEffort/Volatile/UNKNOWN (g1_sim) | BestEffort/Volatile/UNKNOWN (vln_agent)<br>BestEffort/Volatile/UNKNOWN (vln_navigator) |
| `/camera/camera/aligned_depth_to_color/camera_info` | BestEffort/Volatile/UNKNOWN (g1_sim) | - |
| `/tf` | Reliable/Volatile/UNKNOWN (g1_sim) | Reliable/Volatile/UNKNOWN (vln_agent)<br>Reliable/Volatile/UNKNOWN (vln_navigator) |
| `/clock` | BestEffort/Volatile/UNKNOWN (g1_sim) | BestEffort/Volatile/UNKNOWN (vln_agent)<br>BestEffort/Volatile/UNKNOWN (vln_navigator)<br>BestEffort/Volatile/UNKNOWN (path_follower)<br>BestEffort/Volatile/UNKNOWN (external_agent) |
| `/cmd_vel` | Reliable/Volatile/UNKNOWN (path_follower) | Reliable/Volatile/UNKNOWN (g1_sim) |
| `/sim/status` | Reliable/Volatile/UNKNOWN (g1_sim) | Reliable/Volatile/UNKNOWN (external_agent) |
| `/map` | Reliable/TransientLocal/UNKNOWN (g1_sim) | Reliable/TransientLocal/UNKNOWN (vln_navigator)<br>Reliable/TransientLocal/UNKNOWN (external_agent) |
| `/vln/decision_image` | Reliable/Volatile/UNKNOWN (vln_navigator) | Reliable/Volatile/UNKNOWN (external_agent) |
| `/vln/inspection_image` | Reliable/Volatile/UNKNOWN (vln_agent) | Reliable/Volatile/UNKNOWN (external_agent) |
| `/plan` | Reliable/Volatile/UNKNOWN (vln_navigator) | Reliable/Volatile/UNKNOWN (path_follower)<br>Reliable/Volatile/UNKNOWN (external_agent) |
| `/vln/feedback` | Reliable/Volatile/UNKNOWN (vln_agent) | Reliable/Volatile/UNKNOWN (external_agent) |
| `/vln/nav_request` | Reliable/Volatile/UNKNOWN (vln_agent) | Reliable/Volatile/UNKNOWN (vln_navigator) |
| `/tf_static` | Reliable/TransientLocal/UNKNOWN (g1_sim) | Reliable/TransientLocal/UNKNOWN (vln_agent)<br>Reliable/TransientLocal/UNKNOWN (vln_navigator) |
| `/vln/nav_status` | Reliable/Volatile/UNKNOWN (vln_navigator) | Reliable/Volatile/UNKNOWN (vln_agent) |
| `/nav/status` | Reliable/Volatile/UNKNOWN (path_follower) | Reliable/Volatile/UNKNOWN (vln_agent)<br>Reliable/Volatile/UNKNOWN (vln_navigator)<br>Reliable/Volatile/UNKNOWN (external_agent) |
| `/vln/instruction` | Reliable/Volatile/UNKNOWN (external_agent) | Reliable/Volatile/UNKNOWN (vln_agent) |
| `/vln/subgoals` | - | - |
| `/goal_pose` | - | - |

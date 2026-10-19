# 로봇이 직접 만드는 SLAM 지도

지금까지 VLN은 시뮬레이터가 장면 정보로 만든 점유 지도(`/map`)를 썼습니다. 실제 로봇에서는 로봇이 센서로 지도를 직접 만들고, 사람이 그 지도 위에 구역과 기준점을 표시합니다. 여기서는 시뮬레이션에서도 같은 순서로 지도를 만듭니다. G1이 캠퍼스를 한 바퀴 걸으면서 머리의 LiDAR와 IMU만으로 지도를 만들고, 그 지도를 파일 묶음으로 저장해 외부 에이전트와 공유하고, VLN도 그 지도로 실행합니다.

## 어떤 SLAM을 쓰나

G1 머리에는 Livox MID-360(360도 LiDAR, 내장 IMU)이 있습니다. 이 센서에는 FAST-LIO2 같은 LiDAR-관성 SLAM이 가장 잘 맞습니다.

- 거리를 직접 재기 때문에 축척이 정확하고(cm 단위), 조명이나 벽면 질감의 영향을 받지 않습니다.
- CPU만으로 실시간으로 돌아 G1의 온보드 컴퓨터(Jetson Orin)에서도 쓸 수 있습니다. G1을 다루는 연구에서도 MID-360과 FAST-LIO 조합을 상태 추정에 씁니다.
- VLN에 필요한 것은 어디를 걸어갈 수 있는지 알려 주는 점유 지도입니다. LiDAR 점군에서 바로 만들 수 있습니다.

파운데이션 SLAM 모델(MASt3R-SLAM, VGGT 계열, DROID-SLAM 등)은 카메라 영상만으로 조밀한 3D를 복원합니다. LiDAR가 없는 로봇이나, 색이 있는 조밀한 복원이 필요할 때 유용합니다. 하지만 GPU가 필요하고, 단안 카메라로는 축척이 불확실하며, 질감이 적은 벽과 반복되는 설비가 많은 공장에서는 정합이 흔들리기 쉽습니다. 그래서 기하 지도는 LiDAR-관성 SLAM으로 만들고, 파운데이션 모델은 나중에 시멘틱 층(객체와 구역)을 자동으로 채우는 보조 수단으로 검토하는 것을 권합니다.

## 흐름

1. 로봇을 시작 자세에 세웁니다. 이 자세가 지도의 원점이 됩니다.
2. 캠퍼스를 한 바퀴 걷습니다. 실제 로봇은 사람이 조종기로 몰고, 시뮬레이션에서는 `mapping_tour` 노드가 정해진 경유점(`configs/slam_mapping.yaml`)을 따라 걷게 합니다.
3. FAST-LIO2가 LiDAR와 IMU로 로봇 자세를 추정하고 각 스캔을 하나의 좌표계에 정합합니다.
4. `slam_map` 노드가 시작할 때 잰 중력으로 스캔을 대략 수평으로 맞춰 3D 복셀 지도(0.1 m, 높이 0.05 m)에 쌓습니다. 저장할 때는 지도 전체의 바닥에 평면을 맞춰 기울기를 다시 잡습니다. 로봇이 서면서 흔들리는 동안 잰 중력은 1도쯤 틀어질 수 있는데, 1도면 40 m 떨어진 바닥이 0.7 m 높아져 바닥이 장애물로 보이기 때문입니다. 그다음 점유 지도(0.1 m 칸)를 만듭니다. 바닥에서 0.15-1.6 m 높이(G1 키 1.32 m)에 점이 모인 칸은 장애물입니다. 빈 공간은 바닥 점이 보였고, 걸어온 경로에서 몸 높이로 직선이 닿는 칸입니다. 선반 아래 팔레트 틈으로 보인 바닥처럼 로봇이 들어갈 수 없는 곳은 빈 공간으로 치지 않습니다. 나머지는 미확인입니다.
5. 다 걸으면 `maps/<이름>/`에 저장합니다.
6. 사람이 이 지도 위에 구역과 기준점을 표시합니다. 시뮬레이션에서는 시뮬레이터의 주석을 지도 좌표로 옮겨 `semantic_map.json`으로 저장해, 사람이 그린 주석을 대신합니다.
7. 외부 에이전트에게는 이 폴더를 그대로 주거나, 같은 내용을 ROS 토픽으로 보냅니다.

## 지도 묶음

| 파일 | 내용 |
|---|---|
| `map.yaml`, `map.pgm` | 점유 지도. ROS map_server와 Nav2가 읽는 표준 형식(흰색 빈 공간, 검은색 장애물, 회색 미확인) |
| `map.png` | 미리보기와 걸은 경로 |
| `cloud.pcd` | 3D 점군(0.1 m 복셀), PCL과 CloudCompare 등에서 열림 |
| `trajectory.csv` | 매핑 중 SLAM이 추정한 LiDAR 자세 |
| `semantic_map.json` | 같은 좌표계의 구역, 객체, 장소 그래프, 기준점 |
| `meta.json` | 좌표계 정의, SLAM 설정, 크기와 시간 |
| `map_world.yaml`, `evaluation.json`, `comparison.png` | 시뮬레이션에서만: 시뮬레이터 좌표로 옮긴 지도와 정답 비교 |

좌표계는 매핑을 시작할 때 LiDAR 바로 아래 바닥이 원점이고, x축은 그때 로봇이 보던 방향, z축은 위쪽입니다. 외부 에이전트가 이 지도에서 위치나 경로를 말할 때도 이 좌표를 씁니다.

ROS로 공유할 때는 매핑 중 `/slam/map`(OccupancyGrid, latched)이 2초마다 갱신되고, 저장한 지도는 `map_file` 노드로 다시 발행할 수 있습니다.

```bash
bash scripts/g1_docker.sh python3 -m g1_factory.ros2.map_file_node --ros-args -p yaml:=maps/campus/map.yaml -p topic:=/slam_map
```

## 실행 (Ubuntu, Isaac Sim)

이미지에 FAST-LIO2가 추가되었으므로 처음 한 번 이미지를 다시 빌드합니다(UBUNTU_ISAAC.md 2.1).

```bash
docker build -f docker/Dockerfile.ros2 -t g1-sim-ros2:humble .
```

터미널 1에서 Isaac Sim을 매핑 시작 자세로 띄웁니다.

```bash
bash scripts/run_isaac_sim.sh --spawn -12 3.8 0
```

터미널 2에서 매핑을 실행합니다. 로봇이 경유점을 따라 걷고, 다 걸으면 지도를 저장하고 끝납니다.

```bash
bash scripts/g1_docker.sh ros2 launch launch/g1_slam_mapping.launch.py name:=campus
```

결과 그림을 만듭니다.

```bash
bash scripts/g1_docker.sh python3 scripts/plot_slam_map.py maps/campus
```

VLN을 이 지도로 실행합니다. 시뮬레이터 좌표로 옮긴 `map_world.yaml`을 씁니다.

```bash
bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py task:=inspect_conveyor map_file:=maps/campus/map_world.yaml
VLN_MAP_FILE=maps/campus/map_world.yaml bash scripts/vln_console.sh
python3 scripts/hvln_experiment.py --out outputs/exp_slam --set map_file=/ws/maps/campus/map_world.yaml
```

`map_file`을 주면 VLN(`vln_navigator`)만 이 지도를 쓰고, 평가기는 계속 정답 지도로 거리를 잽니다.

## 실제 로봇에서

- LiDAR 드라이버: `livox_ros_driver2`를 CustomMsg(xfer_format 1)로 실행합니다.
- FAST-LIO2: 저장소의 `config/mid360.yaml`(lidar_type 1, LiDAR-IMU 외부 파라미터 T = [-0.011, -0.02329, 0.04412])을 쓰고, `imu_topic`은 드라이버의 `/livox/imu` 그대로 둡니다.
- `slam_map` 노드는 `evaluate:=false`로 실행합니다(정답이 없으므로).
- 넓은 현장이나 긴 경로에서는 오차가 쌓이므로 루프 클로저가 있는 방식(FAST-LIO-SAM, GLIM 등)을 더하는 것을 검토합니다.
- 지도를 만든 뒤 VLN을 운용할 때는 로봇이 이 지도 안에서 자기 위치를 계속 추정해야 합니다(재위치 추정, map에서 odom으로의 변환). 시뮬레이션에서는 시작 자세를 알기 때문에 이 단계를 생략하고 시뮬레이터 좌표로 옮긴 지도를 씁니다.
- 시뮬레이션과 다른 점: 시뮬레이션 LiDAR는 한 스캔의 점을 한순간에 찍으므로 점별 시간 없이 읽는 일반 PointCloud2 방식(lidar_type 5)으로 읽고, 시뮬레이터 IMU를 RELIABLE로 다시 발행한 `/slam/imu`를 씁니다(`configs/fast_lio_g1_sim.yaml`).

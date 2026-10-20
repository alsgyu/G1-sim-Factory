# 로봇이 직접 만드는 지도와 사람이 다는 주석

실제 로봇에서는 로봇이 센서로 지도를 직접 만들고, 사람이 그 지도 위에 구역 이름과 기준점을 표시합니다. 시뮬레이션에서도 같은 순서를 따릅니다.

1. G1이 캠퍼스를 한 바퀴 걸으며 머리의 LiDAR와 IMU만으로 지도를 만듭니다(FAST-LIO2).
2. 사람이 주석 도구로 그 지도 위에 방과 통로를 그리고 이름을 붙입니다.
3. 방 사이의 연결과 출입구(토폴로지)는 지도와 주석에서 자동으로 만들어집니다.
4. 이 모두를 한 폴더(지도 묶음)로 저장해, VLN과 외부 에이전트가 같은 지도와 같은 이름을 씁니다.

이 과정에서 시뮬레이터 정답은 쓰지 않습니다. 정답은 결과가 얼마나 맞는지 비교할 때만 씁니다.

## 어떤 SLAM을 쓰나

G1 머리에는 Livox MID-360(360도 LiDAR, 내장 IMU)이 있습니다. 이 센서에는 FAST-LIO2 같은 LiDAR-관성 SLAM이 가장 잘 맞습니다.

- 거리를 직접 재기 때문에 축척이 정확하고(cm 단위), 조명이나 벽면 질감의 영향을 받지 않습니다.
- CPU만으로 실시간으로 돌아 G1의 온보드 컴퓨터(Jetson Orin)에서도 쓸 수 있습니다. G1을 다루는 연구에서도 MID-360과 FAST-LIO 조합을 상태 추정에 씁니다.
- VLN에 필요한 것은 어디를 걸어갈 수 있는지 알려 주는 점유 지도입니다. LiDAR 점군에서 바로 만들 수 있습니다.

파운데이션 SLAM 모델(MASt3R-SLAM, VGGT 계열, DROID-SLAM 등)은 카메라 영상만으로 조밀한 3D를 복원합니다. LiDAR가 없는 로봇이나, 색이 있는 조밀한 복원이 필요할 때 유용합니다. 하지만 GPU가 필요하고, 단안 카메라로는 축척이 불확실하며, 질감이 적은 벽과 반복되는 설비가 많은 공장에서는 정합이 흔들리기 쉽습니다. 그래서 기하 지도는 LiDAR-관성 SLAM으로 만들고, 파운데이션 모델은 나중에 시멘틱 층(객체와 구역)을 자동으로 채우는 보조 수단으로 검토하는 것을 권합니다.

## VLN에 필요한 정보와 출처

| 정보 | 출처 | 쓰는 곳 |
|---|---|---|
| 점유 지도 (어디를 걸을 수 있나) | 로봇 SLAM, 자동 | 웨이포인트 그래프, 경로, 장애물 |
| 웨이포인트 그래프 (영상 위 번호 후보) | 점유 지도에서 자동 (1.5 m 간격) | VLM이 고르는 다음 이동 |
| 방과 통로의 범위, 이름, 별칭 | 사람 주석 | 지시를 단계로 나눌 때 구역 목록, 구역 방향과 거리, 구역 이탈 거절 |
| 기준점 (교차로, 연결 통로, 입구) | 사람 주석 | 지도 맥락 |
| 장소 그래프와 출입구 | 점유 지도 + 사람이 그린 범위에서 자동 | 목표 구역까지의 경로와 다음 출입구 방향 |
| 방 입구 | 비워 두면 출입구에서 자동, 사람이 정할 수도 있음 | 구역까지 보행 거리 |
| 세부 대상 (검사대, 충전 도크, 서버 랙 등) | 로봇에게 주지 않음 | 평가에만 사용 |

점유 지도만으로는 어디를 걸을 수 있는지만 알 수 있고, "조립 라인"이 어디인지는 모릅니다. 우리 지시는 "어디에 가서 무엇을 확인해" 같은 목표형이라 구역 이름이 필요합니다. MapGPT가 원래 다루는 R2R의 "탁자를 지나 왼쪽으로" 같은 경로 설명형 지시와 다른 점입니다.

## 흐름

1. 로봇을 시작 자세에 세웁니다. 이 자세가 지도의 원점이 됩니다.
2. 캠퍼스를 한 바퀴 걷습니다. 실제 로봇은 사람이 조종기로 몰고, 시뮬레이션에서는 `mapping_tour` 노드가 정해진 경유점(`configs/slam_mapping.yaml`)을 따라 걷게 합니다.
3. FAST-LIO2가 LiDAR와 IMU로 로봇 자세를 추정하고 각 스캔을 하나의 좌표계에 정합합니다.
4. `slam_map` 노드가 3D 복셀 지도(0.1 m, 높이 0.05 m)와 점유 지도(0.1 m 칸)를 만듭니다.
   - 시작할 때 잰 중력으로 스캔을 대략 수평으로 맞추고, 저장할 때 지도 전체 바닥에 평면을 맞춰 기울기를 다시 잡습니다. 로봇이 서면서 흔들리는 동안 잰 중력은 1도쯤 틀어질 수 있고, 1도면 40 m 떨어진 바닥이 0.7 m 높아져 장애물로 보이기 때문입니다.
   - 바닥에서 0.15-1.6 m 높이(G1 키 1.32 m)에 점이 모인 칸은 장애물입니다.
   - 빈 공간은 바닥 점이 보였고, 걸어온 경로에서 몸 높이로 직선이 닿는 칸입니다. 선반 아래 팔레트 틈으로 보인 바닥처럼 로봇이 들어갈 수 없는 곳은 빈 공간으로 치지 않습니다. 나머지는 미확인입니다.
5. 다 걸으면 `maps/<이름>/`에 저장합니다.
6. 사람이 주석 도구(`tools/map_annotator.html`)로 방과 통로, 기준점을 그려 `annotations.json`으로 저장합니다.
7. `scripts/build_map_package.py`가 주석과 지도에서 장소 그래프와 출입구를 자동으로 만들어 `semantic_map.json`에 씁니다.
8. VLN은 `map_package:=maps/<이름>`으로 이 묶음 하나만 읽습니다. 외부 에이전트에게는 이 폴더를 그대로 주거나 ROS 토픽으로 보냅니다.

## 주석 도구

브라우저로 여는 HTML 한 장입니다. 인터넷 연결이나 설치가 필요 없습니다.

- **열기:** `map.yaml`과 `map.pgm`을 함께 고릅니다. 저장해 둔 `annotations.json`을 같이 고르면 이어서 고칠 수 있습니다.
- **방:** 지도에서 끌어서 사각형을 그리면 방이 생기고, 오른쪽에 이름, id, 별칭(다른 이름, 한국어 이름 등)을 적습니다.
- **통로:** 위쪽 종류를 "통로"로 바꾸고 그립니다. 십자형 통로처럼 사각형 하나로 안 되는 모양은 구역을 고르고 "사각형 추가"로 더 그립니다.
- **기준점:** "기준점"을 누르고 지도를 클릭합니다.
- **입구:** 방을 고르고 "입구"를 누른 뒤 클릭합니다. 비워 두면 자동으로 찾습니다.
- **저장:** "annotations.json 저장"으로 내려받아 지도 폴더에 넣습니다. 좌표는 지도 좌표계 그대로 저장됩니다.

저장소 폴더를 웹 서버로 띄우면 주소에 파일을 지정해 바로 열 수도 있습니다.

```bash
python3 -m http.server 8770
```

그다음 브라우저에서 `http://127.0.0.1:8770/tools/map_annotator.html?map=/maps/campus/map.yaml`을 엽니다(기존 주석을 함께 열려면 `&annotations=/maps/campus/annotations.json`).

## 토폴로지 자동 생성

`g1_factory/map_package.py`가 점유 지도와 사람이 그린 범위만으로 장소 그래프를 만듭니다.

1. 지도의 빈 칸마다 그 칸을 덮는 사각형의 장소를 붙입니다. 방이 통로보다 우선입니다.
2. 사람이 벽보다 조금 안쪽에 그린 부분을 채우려고 장소를 빈 칸을 따라 0.6 m 넓힙니다.
3. 그래도 남은 4 m² 이상의 빈 공간은 "이름 없는 구역"이 됩니다(이름을 붙이지 않은 방 등).
4. 두 장소의 빈 칸이 맞닿는 곳이 연결이고, 맞닿은 부분의 가운데가 출입구입니다. 벽과 유리는 장애물 칸이라, 벽 너머의 방은 연결되지 않습니다.
5. 방 입구를 비워 두면 그 방의 가장 넓은 출입구에서 방 안쪽으로 0.8 m 들어간 점을 씁니다.

결과는 기존 시멘틱 맵과 같은 형식(`zones`, `places`, `edges`, `anchors`)이라, VLN의 토폴로지 경로 안내가 그대로 동작합니다. `topology.png`로 눈으로 확인할 수 있습니다.

## 지도 묶음

| 파일 | 내용 |
|---|---|
| `map.yaml`, `map.pgm` | 점유 지도. ROS map_server와 Nav2가 읽는 표준 형식(흰색 빈 공간, 검은색 장애물, 회색 미확인) |
| `cloud.pcd` | 3D 점군(0.1 m 복셀, 높이 0.05 m), PCL과 CloudCompare 등에서 열림 |
| `trajectory.csv`, `map.png`, `meta.json` | 매핑 경로, 미리보기, 좌표계와 SLAM 설정 |
| `annotations.json` | 사람이 그린 방, 통로, 기준점 |
| `semantic_map.json` | 방(이름, 별칭, 범위, 입구), 장소 그래프와 출입구, 기준점. VLN과 외부 에이전트가 쓰는 것 |
| `topology.png` | 주석과 자동 토폴로지를 지도 위에 그린 확인용 그림 |
| `map_world.yaml`, `evaluation.json`, `comparison.png`, `semantic_truth.json`, `annotation_check.json` | 시뮬레이션에서만: 시뮬레이터 좌표로 옮긴 지도, 정답과의 비교 |

좌표계는 매핑을 시작할 때 LiDAR 바로 아래 바닥이 원점이고, x축은 그때 로봇이 보던 방향, z축은 위쪽입니다. 외부 에이전트가 위치나 경로를 말할 때도 이 좌표와 `semantic_map.json`의 이름을 씁니다.

ROS로 공유할 때 `map_file` 노드가 점유 지도와 시멘틱 맵을 latched 토픽으로 발행합니다(`map_package`로 VLN을 띄우면 함께 뜹니다).

```bash
bash scripts/g1_docker.sh python3 -m g1_factory.ros2.map_file_node --ros-args -p package:=maps/campus -p frame:=map
```

이 명령은 점유 지도를 `/slam_map`에, 시멘틱 맵 JSON을 `/slam/semantic_map`에 발행합니다.

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

주석 도구로 `maps/campus/annotations.json`을 만든 뒤 지도 묶음을 완성합니다. 매핑 시작 자세가 같으면 `configs/annotations_campus.json`(Mac에서 그린 예시)을 도구로 불러와 고쳐 써도 됩니다.

```bash
bash scripts/g1_docker.sh python3 scripts/build_map_package.py maps/campus
```

VLN을 지도 묶음으로 실행합니다. 시뮬레이션에서는 `package_frame:=world`(기본값)가 시작 자세로 지도 묶음을 시뮬레이터 좌표로 옮깁니다.

```bash
bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py task:=inspect_conveyor map_package:=maps/campus
VLN_MAP_PACKAGE=maps/campus bash scripts/vln_console.sh
python3 scripts/hvln_experiment.py --out outputs/exp_package --set map_package=/ws/maps/campus
```

지도 묶음을 쓰면 VLN(`vln_agent`, `vln_navigator`)의 점유 지도, 구역, 기준점, 토폴로지가 모두 지도 묶음에서 옵니다. 평가기는 계속 시뮬레이터 정답으로 결과를 잽니다. 점유 지도만 바꿔 보려면 `map_file:=maps/campus/map_world.yaml`을 씁니다.

## 실제 로봇에서

- LiDAR 드라이버: `livox_ros_driver2`를 CustomMsg(xfer_format 1)로 실행합니다.
- FAST-LIO2: 저장소의 `config/mid360.yaml`(lidar_type 1, LiDAR-IMU 외부 파라미터 T = [-0.011, -0.02329, 0.04412])을 쓰고, `imu_topic`은 드라이버의 `/livox/imu` 그대로 둡니다.
- `slam_map` 노드는 `evaluate:=false`로 실행합니다(정답이 없으므로).
- VLN은 `package_frame:=map`으로 실행합니다. 이때 로봇이 이 지도 안에서 자기 위치를 계속 추정해야 합니다(재위치 추정으로 map에서 odom으로의 변환을 발행). 시뮬레이션은 시작 자세를 알기 때문에 이 단계를 생략했습니다.
- 넓은 현장이나 긴 경로에서는 오차가 쌓이므로 루프 클로저가 있는 방식(FAST-LIO-SAM, GLIM 등)을 더하는 것을 검토합니다.
- 시뮬레이션과 다른 점: 시뮬레이션 LiDAR는 한 스캔의 점을 한순간에 찍으므로 점별 시간 없이 읽는 일반 PointCloud2 방식(lidar_type 5)으로 읽고, 시뮬레이터 IMU를 RELIABLE로 다시 발행한 `/slam/imu`를 씁니다(`configs/fast_lio_g1_sim.yaml`).

# 보고서 지표 측정 방법

최종 보고서에 올리는 지표 6개를 어떻게 정의하고 측정하는지 정리한 문서입니다. 측정은 Ubuntu 노트북의 Isaac Sim 5.1에서 하고, 계산과 보고서 작성은 스크립트가 합니다. 정답은 시뮬레이터 장면에서 나오며 평가에만 씁니다. 로봇은 정답을 받지 않습니다.

## 지표

| 번호 | 지표 | 무엇을 무엇과 비교하나 | 계산 스크립트 |
|---|---|---|---|
| 1.1 | SLAM 궤적 오차 | 매핑 중 FAST-LIO2가 추정한 LiDAR 위치 대 같은 시각의 정답 위치 | `scripts/eval_map.py` |
| 1.2 | 토폴로지 출입구 검출과 위치 오차 | 지도 묶음의 자동 장소 그래프 대 시뮬레이터 장소 그래프 | `scripts/eval_map.py` |
| 2.1 | 최종 지도에서 LiDAR 위치 추정 정확도 | localizer가 지도 묶음에서 찾은 위치 대 정답 위치 | `scripts/eval_localization.py` |
| 2.2 | 위치와 구역 범위로 현재 구역 판정 정확도 | 로봇이 정한 구역 대 정답 위치의 정답 구역 | `scripts/eval_localization.py` |
| 3.1 | VLN 중 카메라 영상으로 목표 대상을 찾은 정확도 | VLN이 보고한 대상 위치 대 정답 대상 위치 | `scripts/eval_vln.py` |
| 3.2 | 대상 앞에서 확인 질문에 답한 판정 정확도 | VLN의 yes/no 대 장면의 정답 | `scripts/eval_vln.py` |

세 결과를 `scripts/metrics_report.py`가 보고서 한 편(`REPORT.md`)으로 묶습니다.

## 지표 정의

### 1.1 SLAM 궤적 오차

- 매핑할 때(`evaluate:=true`) 지도 묶음에 SLAM 궤적(`trajectory.csv`)과 같은 행의 정답 궤적(`trajectory_truth.csv`, 시뮬레이터 좌표의 LiDAR 위치)이 저장됩니다.
- ATE, 시작 자세 정렬: 지도 좌표를 매핑 시작 자세로 시뮬레이터 좌표에 옮긴 뒤의 위치 오차입니다. 실제로 지도 좌표가 정해지는 방식이라 VLN이 겪는 오차와 같습니다.
- ATE, 최적 정렬: 궤적 전체가 가장 잘 맞도록 회전하고 이동한 뒤의 오차(축척 없는 Umeyama 정렬)입니다. 논문에서 흔히 쓰는 ATE와 같은 정의라 다른 SLAM과 비교할 수 있습니다.
- 10 m 구간 상대 오차: 정답 경로 10 m마다 그 구간의 움직임을 비교한 위치 오차(구간 길이 대비 %)와 방향 오차(도/m)입니다. 누적 표류를 나타냅니다.
- 평균, RMSE, 중앙값, 95번째 백분위, 최대를 내고, 매핑을 여러 번 하면 실행 간 평균과 표준편차를 냅니다.

### 1.2 토폴로지 출입구 검출과 위치 오차

- 출입구는 장소 그래프의 연결(두 장소가 맞닿는 곳) 하나입니다. 정답은 시뮬레이터 장소 그래프의 연결 12개입니다.
- 장소는 이름이 아니라 자리로 짝을 짓습니다. 지도 묶음의 각 장소에 속한 빈 칸이 시뮬레이터의 어느 장소(구역, 건물 안 통로, 연결 통로)에 가장 많이 놓이는지로 정합니다(절반 이상). 사람이 이름을 다르게 붙여도 결과가 같습니다.
- 짝지은 두 장소가 정답에서도 이어져 있으면 검출, 정답에 없는 연결이면 오검출, 찾지 못한 정답 연결이면 미검출입니다. 정밀도, 재현율(95% 구간), F1을 냅니다.
- 위치 오차는 검출한 출입구 점과 정답 출입구 점 사이의 거리입니다. 맞닿은 폭이 6 m 이하면 벽의 출입구(문 1.7-1.8 m, 통로 입구 3-4 m), 넓으면 벽 없이 바닥만 나뉜 경계로 나눠 냅니다. 벽 없는 경계는 자동 출입구를 맞닿은 부분의 가운데로 잡으므로, 정답 출입구 점과 정의가 달라 1-2 m 차이가 납니다.
- 같은 지도로 두 번 계산합니다. 사람 주석을 그대로 쓴 결과가 실제 로봇이 쓰는 지도의 값이고, 시뮬레이터 구역을 주석 대신 넣은 결과는 사람의 오차를 뺀 값(SLAM 지도와 알고리즘만의 오차)입니다.
- 참고로 사람이 그린 방과 정답 구역의 IoU를 함께 적습니다. 주석의 품질이며 로봇 성능은 아닙니다.

### 2.1 최종 지도에서 LiDAR 위치 추정 정확도

- `launch/g1_localization_eval.launch.py`가 로봇을 `configs/metrics.yaml`의 경로로 걷게 합니다. 경로는 캠퍼스의 방 9개와 통로를 모두 지나고(약 330 m), 시작은 매핑 시작 자세와 다른 곳(충전 구역)입니다.
- 로봇은 시뮬레이터 지도로 걷고, localizer는 지도 묶음에서 위치만 찾습니다(`publish_tf:=false`). 측정이 로봇의 움직임에 영향을 주지 않습니다.
- localizer는 사람이 지도에서 출발점을 대략 짚어 주는 것처럼, 실제 출발 자세에서 0.8 m, 0.6 m, 12도 틀린 초기 자세로 시작합니다(`initial_offset`).
- 추적 오차: 5 Hz로 localizer의 위치를 지도 묶음의 `evaluation.json` 변환으로 시뮬레이터 좌표에 옮겨 같은 시각의 정답과 비교합니다. 이 변환은 매핑 시작 자세에서 정해지므로 지도 자체의 오차가 포함됩니다. 그래서 경로 전체를 최적 정렬한 뒤의 오차도 함께 냅니다.
- 첫 위치 추정 시험: 3 m 걸을 때마다 그 자리의 스캔으로 위치를 처음부터 다시 찾게 합니다. 초기 자세는 정답에서 일부러 틀리게 줍니다. 범위 안(0.5-1.5 m, 0-20도)과 범위 밖(2.5-5 m, 0-30도) 두 가지입니다. 결과가 정답에서 0.5 m, 5도 안이면 맞음, 찾았다고 했지만 그보다 멀면 잘못 맞춤, 받아들이지 않으면 거절입니다. 범위 안에서는 맞음 비율이, 범위 밖에서는 잘못 맞춤이 없어야 한다는 것이 중요합니다.
- 실행 중 lost 횟수와 첫 위치 추정까지의 시간도 적습니다.

### 2.2 위치와 구역 범위로 현재 구역 판정 정확도

- 로봇이 VLN 중에 쓰는 규칙 그대로입니다. 위치 추정 결과가 사람이 그린 어느 방의 범위 안인지로 구역을 정하고, 어느 방에도 없으면 통로입니다(`g1_factory/roadmap.py`의 `region_at`).
- 정답은 정답 위치가 시뮬레이터의 어느 구역 안인지입니다. 카메라는 쓰지 않습니다.
- 2.1과 같은 실행에서 5 Hz로 기록합니다. 연속 표본은 서로 독립이 아니므로 95% 구간은 1 m 걸을 때마다 한 표본으로 줄여 계산하고, 전체 표본의 비율도 함께 냅니다.
- 구역별 정확도, 혼동표, 틀린 표본이 구역 경계에서 얼마나 떨어져 있는지를 냅니다.
- `--grid`를 주면 위치 오차 없이 장소마다 고르게 뽑은 정답 위치에서 같은 규칙을 적용한 정확도를 함께 냅니다. 사람이 그린 구역 범위만의 오차입니다.

### 3.1 VLN 중 카메라 영상으로 목표 대상을 찾은 정확도

- 실제 운용과 같은 조건에서 VLN을 돌립니다. 로봇이 만든 지도 묶음을 쓰고, 지도 안에서 스스로 위치를 찾고(`package_frame:=map`), Qwen3-VL-8B를 씁니다(`configs/metrics_vln.yaml`). 과제 11개를 시작 자세 3가지(seed 0-2)로 돌려 33개 에피소드입니다.
- VLN이 영상에서 대상에 상자를 그리면 깊이로 대상 위치를 구해 보고합니다. 평가기는 이 위치를 시뮬레이터 좌표로 옮겨, 정답 대상의 바닥 면적에서 1.0 m 안이면 맞음으로 셉니다.
- 주 지표는 전체 에피소드 기준입니다. 대상을 보고하지 못한 에피소드(길을 잃음, 결정 한도 등)도 틀림입니다. 보고한 에피소드 중 맞은 비율과 보고한 위치의 오차도 함께 냅니다.

### 3.2 대상 앞에서 확인 질문에 답한 판정 정확도

- 대상 앞에 서면 VLM이 확인 질문(예: "압력계가 있나")에 yes/no로 답합니다. 장면의 정답(`configs/vln_tasks.yaml`의 `expected_verdict`)과 같으면 맞음입니다.
- 주 지표는 전체 에피소드 기준이며 답하지 못한 에피소드는 틀림입니다. 답한 에피소드 중 정확도와, 대상을 맞게 찾은 에피소드 중 정확도도 함께 냅니다. 마지막 값이 순수한 판정 능력에 가깝습니다.

### 공통

- 비율에는 Wilson 95% 구간을 붙입니다. 에피소드 33개 정도에서는 구간이 넓다는 점을 보고서에 함께 적습니다.
- 과제 성공률, SPL, 충돌, 결정 수, VLM 응답 시간은 참고 값으로 함께 냅니다.

## 측정 절차 (Ubuntu, Isaac Sim)

명령은 모두 저장소 폴더에서 실행합니다. 처음 한 번 준비가 되어 있어야 합니다: 최신 코드, Docker 이미지 `g1-sim-ros2:humble`, Isaac Sim 이미지와 캐시, 펌프실이 들어간 `outputs/isaac/campus.usd`, Ollama의 `qwen3-vl:8b-instruct`(문맥 8192 이상). 설정은 [UBUNTU_ISAAC.md](UBUNTU_ISAAC.md)에 있습니다.

GPU 컨테이너가 뜨는지 먼저 확인합니다.

```bash
docker run --rm --gpus all g1-sim-ros2:humble nvidia-smi
```

측정 중에는 설정(파라미터, 모델, 지도)을 바꾸지 않습니다. 바꿔야 하면 처음부터 다시 측정하고, 바꾼 내용을 보고서 환경 항목에 적습니다.

### 1단계: 매핑 3회 (1.1)

같은 경로로 3번 매핑합니다. 매번 Isaac을 새로 띄웁니다. 첫 번째 지도(`campus`)를 이후 모든 측정에 쓰는 지도로 미리 정해 둡니다. 결과를 보고 가장 좋은 지도를 고르지 않습니다.

터미널 1:

```bash
bash scripts/run_isaac_sim.sh --spawn -12 3.8 0
```

터미널 2 (두 번째, 세 번째는 `name:=campus_r2`, `name:=campus_r3`):

```bash
bash scripts/g1_docker.sh ros2 launch launch/g1_slam_mapping.launch.py name:=campus
```

매번 `maps/<이름>/evaluation.json`을 보고 진행 여부를 판단합니다. 장애물 정밀도가 0.9보다 낮거나 위치 오차(`ate_rmse_m`)가 0.3 m보다 크면 지도에 문제가 있는 것이니 멈추고 원인을 확인합니다. 경유점 실패도 로그에서 확인합니다. 그 지도는 버리지 말고 실패로 기록합니다.

### 2단계: 주석과 토폴로지 (1.2)

`campus` 지도에 주석 도구(`tools/map_annotator.html`)로 방 9개, 통로, 기준점을 그리고 `maps/campus/annotations.json`으로 저장합니다. 매핑 시작 자세가 같으므로 `configs/annotations_campus.json`을 불러와 지도의 벽에 맞게 고쳐도 됩니다. 정답 구역 그림은 보지 않고 지도에 보이는 벽과 설비만 보고 그립니다. 주석한 사람과 걸린 시간을 적어 둡니다.

```bash
bash scripts/g1_docker.sh python3 scripts/build_map_package.py maps/campus
```

### 3단계: 지도 지표 계산 (1.1, 1.2)

```bash
bash scripts/g1_docker.sh python3 scripts/eval_map.py maps/campus --runs maps/campus maps/campus_r2 maps/campus_r3 --out outputs/metrics/map
```

### 4단계: 위치 추정과 구역 판정 (2.1, 2.2)

경로의 시작 자세(충전 구역, 북쪽)로 Isaac을 띄웁니다. 시뮬레이터가 지도 좌표 변환을 그대로 내도 됩니다(`--no-map-tf` 없이).

터미널 1:

```bash
bash scripts/run_isaac_sim.sh --spawn 12 -3.8 1.5708
```

터미널 2. 경로를 다 걸으면 저절로 끝납니다(시뮬레이션 약 15-20분):

```bash
bash scripts/g1_docker.sh ros2 launch launch/g1_localization_eval.launch.py map_package:=maps/campus output_dir:=/ws/outputs/metrics/localization
```

```bash
bash scripts/g1_docker.sh python3 scripts/eval_localization.py outputs/metrics/localization --grid
```

로그에 `localized: map <- odom = ...`가 나와야 측정이 시작됩니다. `no fix yet`만 계속 나오면 멈추고 확인합니다.

### 5단계: VLN (3.1, 3.2)

실험 실행기가 에피소드마다 Isaac을 띄우고(`--no-map-tf` 자동), 끝나면 내립니다. 33개 에피소드에 4-6시간쯤 걸립니다. 중간에 멈춰도 같은 명령으로 다시 실행하면 끝난 에피소드는 건너뜁니다.

```bash
python3 scripts/hvln_experiment.py --config configs/metrics_vln.yaml --simulator isaac --out outputs/metrics/vln
```

```bash
bash scripts/g1_docker.sh python3 scripts/eval_vln.py outputs/metrics/vln
```

시뮬레이터가 멈춘 에피소드(`sim_stalled`)는 실행기가 한 번 더 돌립니다. 그래도 실패하면 결과에 그대로 남기고, 보고서에 개수를 적습니다.

### 6단계: 보고서

```bash
bash scripts/g1_docker.sh python3 scripts/metrics_report.py --map outputs/metrics/map --loc outputs/metrics/localization --vln outputs/metrics/vln --out outputs/metrics/report --env "GPU=RTX 5080 Laptop" "Isaac Sim=5.1.0" "VLM=qwen3-vl:8b-instruct (Ollama)"
```

`outputs/metrics/report/REPORT.md`가 보고서입니다. 결과 요약 표, 측정 환경, 측정 방법 요약, 지표별 표와 그림, 한계를 담습니다. 모든 숫자는 `report.json`에 있습니다.

## 결과 파일

| 경로 | 내용 |
|---|---|
| `maps/campus*/` | 지도 묶음. `trajectory.csv`, `trajectory_truth.csv`, `evaluation.json`, `annotations.json`, `semantic_map.json`, `semantic_truth.json` |
| `outputs/metrics/map/` | `map_metrics.json`, `map_metrics.md`, `fig_trajectory.png`, `fig_doorways.png` |
| `outputs/metrics/localization/` | 원자료 `samples.csv`, `fix_trials.csv`, `status.jsonl`, `run.json`, `launch_args.json`; 결과 `localization_metrics.json`, `localization_metrics.md`, `fig_localization.png` |
| `outputs/metrics/vln/` | 에피소드별 `summary.json`, `feedback.jsonl`, `decisions/`, `inspection.jpg`, `localization.csv`; 결과 `vln_metrics.json`, `vln_metrics.md`, 실행기 표 `results.md`, `episodes.csv` |
| `outputs/metrics/report/` | `REPORT.md`, `report.json`, `figures/` |

계산 스크립트는 원자료만 읽으므로, 정의를 바꾸거나 표를 다시 만들 때 Isaac을 다시 돌릴 필요가 없습니다.

## 개발 환경에서 확인한 것 (Mac, MuJoCo)

측정 파이프라인 전체를 MuJoCo 개발 그래프와 기존 지도 `outputs/maps/campus_dev`로 짧게 돌려 확인했습니다(2026-10-30). 아래 값은 파이프라인 점검용이며 보고서 결과가 아닙니다.

- 1.1: 짧은 매핑(`configs/metrics.yaml`의 `localization_short` 경로, 47 m)으로 `trajectory_truth.csv`가 저장되고, ATE 0.100 m(시작 자세 정렬), 0.067 m(최적 정렬), 10 m 구간 0.50%가 계산됨.
- 1.2: `campus_dev`에서 정답 출입구 12개 모두 검출, 오검출 0. 정답 구역을 넣어도 위치 오차가 비슷하여(평균 0.90 m 대 0.99 m), 남는 오차는 사람 주석이 아니라 벽 없는 경계의 출입구 정의 차이임을 확인.
- 2.1, 2.2: 47 m 경로에서 표본 693개, 첫 위치 추정 시험 46회. 위치 오차 평균 0.063 m, 범위 안 시험 23회 중 22회 맞음(1회는 펌프실 통로에서 0.54 m로 기준 0.5 m를 조금 넘음), 범위 밖 23회 모두 거절. 구역 판정 97.9%(1 m마다), 틀린 표본은 모두 경계에서 0.4 m 안.
- 3: 이전 실행 5개로 집계와 보고서 생성까지 확인.

지표 계산 함수는 단위 테스트(`tests/test_metrics.py`)로 확인합니다: Wilson 구간, 최적 정렬 ATE, 구간 표류, 이름과 무관한 장소 짝짓기, 출입구 검출 계산, 구역 혼동표, 첫 위치 추정 판정.

# Warehouse + Office 검증 결과

## 계층형 VLN ROS 2 스택

실행일 2026-10-16. macOS(M1 Pro 16GB) Docker arm64 · ROS 2 Humble · CycloneDDS · MuJoCo 3.5.0 EGL(Mesa llvmpipe, CPU 렌더링) · Qwen3-VL 4B Instruct(Ollama 0.40.5, Metal).

- 단위 테스트 `python -m pytest -q`: **93 passed** (기존 79 + 계층형 VLN 14).
- 실제 ROS 2 그래프(`g1_sim`·`vln_agent`·`global_planner`·`path_follower`·`external_agent`)를 `scripts/hvln_experiment.py`로 실행. 카메라 10 Hz(측정 회차만 30 Hz).

### 최종 코드 결과 (`vlm_4b`, seed 0)

| 과제 | 계획된 장소 | 판정 (정답) | 최종 오차 | SPL | 경로추종 RMSE | 재계획 | 결과 |
|---|---|---|---:|---:|---:|---:|---|
| `inspect_conveyor` | assembly_line → assembly_conveyor | yes (yes) | 0.21 m | 0.79 | 1.3 / 4.3 cm | 0 | 성공 |
| `inspect_dock_cart_ko` | charging → charging_dock_3 | no (no) | 0.18 m | 0.88 | 1.3 / 1.9 cm | 0 | 성공 |
| `inspect_meeting_table_en` | office_meeting → meeting_table | no (no) | 0.13 m | 0.96 | 2.1 / 3.1 cm | 0 | 성공 |
| `inspect_station_paraphrase` | inspection → inspection_station_2 | yes (yes) | 1.00 m | 0.77 | 2.0 / 4.3 / 1.3 cm | 1 | 성공 |

- 4/4 성공, 판정 정확도 4/4, 충돌·넘어짐 0. VLM 호출 평균 5.1 s(벽시계).
- `inspect_station_paraphrase`: 지도에 없는 팔레트 더미를 LiDAR로 감지해 1회 재계획, 충돌 없이 우회.

### 규칙 기반 대비 (중간 버전, 같은 과제·seed 0/1)

| 과제 | `rule` | `vlm_4b` |
|---|---:|---:|
| `inspect_station_paraphrase` (별칭이 문장에 없음) | 0/2 — "검사 구역"까지만 찾고 검사대 미특정 | 2/2 |
| `inspect_dock_cart_ko` | 2/2 도착, 판정 "unknown" | 2/2 도착, 판정 "unknown" → 아래 수정 후 "no" |

### 토픽 측정 회차 (`inspect_conveyor`, 카메라 30 Hz, `topic_monitor` 켬)

- 21개 토픽 전부 발행·구독 노드·QoS를 ROS 그래프에서 확인. 크기·주기: [ROS2_INTERFACE.md](ROS2_INTERFACE.md).
- 실시간 대비 0.149배. 처리시간: RGB-D 렌더 183 ms/프레임(CPU), LiDAR 20,000 ray 51 ms(8스레드), 물리+정책 0.27 ms/스텝. → 30 Hz 실시간에는 GPU EGL 필요(`scripts/check_hvln_env.py`가 15 ms 기준으로 판정).
- VLM HTTP 실측(8회): 계획 요청 5.6 KB·2.9 s, 대상 찾기 17.5–27.0 KB·5.1 s, 확인 18.0–37.9 KB·6.2 s.

### 검증 중 발견해 수정한 문제

| 문제 | 수정 |
|---|---|
| 도착 후 "새 영상 대기"가 매번 기준 시각을 갱신해 70초 지연, 그동안 제자리 보행으로 1.8 m 밀림 | 도착 시각 = 경로추종기 보고 시각, 대기 중 재설정 안 함. 도착 후 위치 유지(±0.12 m) 추가 |
| 재접근 거리를 bbox 중앙 깊이로 계산해 컨베이어 0.4 m 앞까지 접근 | 가까운 면(하위 10% 깊이) 기준 1.4 m. 대상이 화면에서 작을 때만 재접근 |
| goal id를 float로 변환 → 벽시계(1.79e9 s)에서 ns 정밀도 손실, 도착 이벤트 매칭 실패 가능 | goal stamp 원본을 Path header로 그대로 복사 |
| 확인 이미지가 "done"보다 늦게 도착해 저장 누락 | 외부 에이전트가 이미지 도착(최대 2 s)까지 대기 |
| 충전 도크 접근 자세가 주차 칸 안 → 카트 유무를 볼 수 없음 | 칸 바깥(13, −7.4)으로 이동 |
| VLM이 "볼 대상"을 존재 여부를 묻는 물체(카트)로 지정 → 찾지 못해 엉뚱한 방향에서 확인 | 계획 프롬프트: 볼 대상 = 항상 있는 기준물. 못 찾으면 원래 방향으로 복귀 후 확인 |
| 대상이 없을 때 "확인 불가"(unknown) 응답 | 확인 프롬프트: 관찰 내용 먼저 기술, 장소가 보이면 없음 = "no" |
| 검사대 위 부품이 상판과 같은 색의 평판 → "부품 없음" 오판 | 주황색 0.44×0.32×0.24 m 부품. 저장 프레임 4장 판정 4/4 |
| sim 종료 시 `/clock` 정지 → sim-time 타이머가 멈춰 외부 에이전트 무한 대기 | 벽시계 감시 타이머(120 s) 추가 |

### 미검증

- Ubuntu GPU 스테이션의 실시간(RTF ≥ 0.95) 실행, Qwen3-VL 8B, 전체 실험 행렬 63회: [HVLN.md 5.3절](HVLN.md#53-실험-절차) 절차로 실행 필요.
- Isaac Sim·실제 G1 연결: 토픽 매핑과 `sim:=false`·`remaps:=` 경로만 준비. 실제 연결 미검증.
- 과제당 seed 1–2회. 성공률은 소규모 검증이며 통계적 성능 주장이 아님.
- `docs/images/`의 장면 이미지는 작업대·컨베이어 높이 조정 전 렌더.

## 뷰어 수정 추가 검증 · MuJoCo 3.5.0

- `1/2/3` 키와 HUD 갱신이 겹치는 교착 원인 확인. 공식 3.5.0의 `WaitForAtomicNoGil` 수정 적용. 3.3.7/3.4.0 GUI 실행 시 업그레이드 안내.
- [이전 바인딩](https://github.com/google-deepmind/mujoco/blob/3.3.7/python/mujoco/simulate.cc), [수정된 바인딩](https://github.com/google-deepmind/mujoco/blob/3.5.0/python/mujoco/simulate.cc).
- 오른쪽 위 전체 맵: 8구역·통로·장애물·경로·목표·G1 위치/방향. 10Hz 갱신, 창 크기 변경과 선반 배치 변경 반영.
- 실제 MuJoCo 모델·장면과 캡처용 viewer 대역으로 키 전환, 일시정지, 좌표·방향, 화면 배치, 크기 변경, 오버레이 잠금 범위 검증.
- `python -m pytest -q`: **79 passed** (뷰어·미니맵 17개 포함).
- 3.5.0에서 `warehouse_to_office` 재실행: 완주 91.84초, RMSE 0.217m, 목표 오차 0.296m, 충돌·넘어짐 0회.
- 실제 GLFW 창에서의 키 입력은 미검증. 컨테이너가 UNIX 소켓 생성을 차단하여 Xvfb 실행 불가. 위 테스트는 네이티브 렌더링 스레드의 교착 재현 테스트를 대신하지 않음.

## 기존 전체 경로 검증 · MuJoCo 3.3.7

실행일: 2026-09-22. Ubuntu 24.04 컨테이너, Python 3.12, MuJoCo 3.3.7, Torch 2.7.1+cpu.

## 실제 G1 보행

**4개 시나리오 모두 완주.** 초기 reset 이후 위치를 강제로 옮기지 않고 공식 보행 정책의 관절 토크로 이동.

| 시나리오 | 의미 구역 방문 | 기준 경로 | 시뮬레이션 시간 | 경로 RMSE | 최대 경로 오차 | 목표 오차 | 충돌 / 넘어짐 |
|---|---:|---:|---:|---:|---:|---:|---|
| `warehouse_tour` | 4/4 | 94.8 m | 300.24 s | 0.233 m | 0.382 m | 0.299 m | 0 / 0 |
| `office_tour` | 4/4 | 60.0 m | 197.00 s | 0.111 m | 0.203 m | 0.298 m | 0 / 0 |
| `warehouse_to_office` | 2/2 | 29.8 m | 91.84 s | 0.217 m | 0.354 m | 0.296 m | 0 / 0 |
| `full_tour` | 8/8 | 184.6 m | 597.76 s | 0.196 m | 0.382 m | 0.298 m | 0 / 0 |

- Warehouse 48×32m와 Office 24×24m가 같은 장면. 연결 통로로 실제 이동.
- `full_tour`: Warehouse 4구역 → Office 4개 방, 총 21개 waypoint 순서대로 통과.
- 목표 허용 오차 0.3m. 경로 오차: 50Hz pose에서 기준 선분까지의 최단거리.
- 충돌·넘어짐 확인: 물리 주기 500Hz. 최저 몸체 높이 약 0.764m.
- seed=0, 고정 배치에서 각 시나리오 1회. 다양한 배치·외란에 대한 성능 보장은 아님.
- 긴 직선에서 경로 오차가 증가함. 이전 작은 방의 4.8cm RMSE를 새 환경 성능으로 사용하지 않음.
- [원본 summary 모음](validation/results.json). 실행별 `zone_events.json`에 실제 구역 진입 시점 기록.

## 코드·장면

- `python -m pytest -q`: **62 passed**.
- Python 컴파일, `git diff --check` 통과.
- 실제 모델: 12 actuators, 8개 의미 구역, 8개 온도계, 약 1,369 geoms.
- 실제 MuJoCo 광선으로 Office 벽·4개 출입구·Warehouse 연결 통로 확인.
- 경로 장애물 검증, 잘못된 구역·배치 거부, 선반 이동, 합성 온도 센서 검증.
- EGL/Mesa로 실제 공장·Office·G1·ego RGB와 유한한 양수 depth 렌더링 확인.
- GUI 상태 표시·카메라 키·경로 표시 코드는 MuJoCo 공개 API를 사용. 디버그 표시는 정책 입력 이미지에 포함되지 않음.

## 미검증 범위

- 사용자 Ubuntu PC에 직접 설치하지 않음. README 명령 실행 필요.
- 이 환경에는 데스크톱 디스플레이가 없어 실제 GUI 창을 띄운 사용자 조작은 미검증. headless 물리·렌더링과 GUI 표시 제어 로직은 검증.
- NaVILA GPU 체크포인트 추론·언어 지시 성공률은 미검증. 위 수치는 고정 경로 보행 성능.
- 온도는 합성값. 컨베이어 운전·제조 공정·실제 충전·로봇의 선반 운반은 미구현.
- SLAM·동적 장애물 회피·새 경로 계획 모델·다중 G1은 별도 연구 항목.
- Ubuntu 22.04/Python 3.10 및 OSMesa는 설치 대상. 실제 검증은 Ubuntu 24.04/Python 3.12/EGL.

## 재현

```bash
source .venv/bin/activate
python -m pytest -q
for scenario in warehouse_tour office_tour warehouse_to_office full_tour; do
  python -m g1_factory.run --scenario "$scenario" --headless || break
done
MUJOCO_GL=egl python scripts/render_scene.py
```

정책·설정 해시는 `summary.json`, 구역·경로는 `scenario.json`, 이동 기록은 `trajectory.csv`에 저장.

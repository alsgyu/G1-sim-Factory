# Warehouse + Office 검증 결과

## 지도 기반 계층형 VLN

실행일 2026-10-17. macOS(M1 Pro 16GB) Docker arm64 · ROS 2 Humble · CycloneDDS · MuJoCo 3.5.0 EGL(CPU 렌더링) · Qwen3-VL 4B Instruct(Ollama 0.40.5, Metal, 문맥 8192). 카메라 10 Hz, 그림자 끔. Isaac Sim과 8B는 이 Mac에서 실행하지 않음.

- 단위 테스트 `python -m pytest -q`: **120 passed**.
- 실제 ROS 2 그래프(`g1_sim`·`vln_agent`·`vln_navigator`·`path_follower`·`external_agent`)를 `scripts/hvln_experiment.py`로 실행. 로봇은 대상 위치를 받지 않음.

### 실행 예: `inspect_station_paraphrase` ("부품 품질을 확인하는 곳에 가서 두 번째 비전 검사 장비 위에 부품이 올려져 있는지 봐줘", 조립 라인에서 출발)

| 시뮬레이션 시간 | 계층 | 내용 |
|---:|---|---|
| 4.0 s | 1 (단계 분해) | ① 품질 검사 구역으로 이동 ② 두 번째 비전 검사 장비 찾기, 명판 STATION 2 |
| 8.6–51.4 s | 2 (VLN) | 영상 위 번호 웨이포인트 중 선택 ×3 (검사 구역 방향 보행거리 이득·표지판 근거) |
| 72.4 s | 2 | 검사 구역 진입 → ①단계 자동 완료 |
| 78.8 s | 2 | `approach`: 영상 속 STATION 2 bbox → 확대 이미지로 명판 재판독 "STATION 2" 일치 → 깊이로 대상 위치 |
| 89.6 s | 3·4 | 대상 앞 도착(VLN 결정 4회, 이동 26.0 m, 최단 23.9 m, 충돌 0) |

- 영상으로 찾은 대상 위치 오차 0.18 m, 정지 위치 대상 외곽 0.55 m. 이 회차의 확인 답변은 오답("부품 없음") → 아래 표의 VQA·접근 거리 문제를 찾아 수정.

### 검증 중 발견해 수정한 문제

| 문제 (실측) | 수정 |
|---|---|
| `vln_agent`가 같은 파라미터를 두 번 선언해 시작 직후 종료 | 한 번만 선언 |
| 새 지시·취소 시 VLN 모드에는 `/goal_pose` 구독자가 없어 로봇이 이전 구간을 계속 걸음 | 내비게이터가 제자리 1점 경로(`/plan`)로 정지 |
| LiDAR가 구간을 막고 우회도 불가하면 추종기가 막힌 경로를 계속 따라감 | 정지 후 VLN이 다른 길 선택 |
| 제자리 회전 제한(3회)이 0.5 m 위치 기준이라 휴머노이드가 돌며 밀리면 초기화 → 6회 연속 회전 | 이동·접근 결정 때만 초기화 (회귀 테스트 추가) |
| 확인 VQA: "갈색 블록이 있다"고 관찰하고도 "부품 없음" | 프롬프트: 대상 위 물체 = on, 판정은 관찰과 일치. 저장 프레임 9장(예 5·아니오 4) 9/9 |
| 옆에 있는 대상(컨베이어)을 69° 시야 밖에 두고 지나침 | 대상 구역 첫 도착 시 4방향 둘러보기 → VLM이 방향 선택 |
| 둘러보기 요청(영상 4장 ≈ 4,200 토큰)이 Ollama 기본 문맥 4096 초과 → HTTP 400 → 작업 실패 | 문맥 8192 설정·`check_hvln_env.py` 점검 추가. 호출 실패 시 현재 방향으로 계속 |
| `approach` 조건이 "명판 확인" 필수 → 명판이 반대편인 컨베이어 옆을 지나침 | 번호로 지정된 대상만 명판 확인, 그 외는 2차 확인 |
| 접근 정지 위치를 상판 안쪽 깊이 기준으로 계산 → 검사대 0.44 m 앞에서 상판만 보임(4B·8B 모두 "부품 없음") | 영상으로 찾은 점이 지도 장애물 위면 지도 외곽에서 1.4 m |
| 평가: 10 m 컨베이어를 중심점 거리로 채점 → 컨베이어 앞에 선 성공을 실패로 판정 | 대상 바닥 외곽(`footprint`, 장면 형상과 테스트로 일치 확인) 기준, 컨베이어 A·B 모두 정답 |
| 단계 분해(4B): 찾기 단계 누락, 대상에서 "빈" 조건 누락, 명판을 "VISION INSPECTION 2"로 추측(실제 "STATION 2"), 지시에 없는 "서버 랙 1" | 찾기 단계 검증(재질의 후 추가), 구별 조건 유지, 명판은 번호·문자만 비교, 지어낸 번호 제거 |
| 2차 확인이 조건을 무시해 카트가 서 있는 충전 도크를 "빈 도크"로 승인 | 대상 앞 바닥까지 포함한 확대 이미지 + 조건까지 판단 → 같은 프레임에서 거절 확인 |
| MuJoCo 시뮬레이터 세그폴트 1회(전체 실행 중 1회, 원인 미확정) | LiDAR 스레드별 `MjData`(출력 동일 확인), 실험 실행기가 시뮬레이터 정지 시 1회 재시도·첫 시도 보관 |
| 실험이 갈수록 느려짐(결정 간격 10분 이상): Ollama llama-server가 프롬프트를 RAM에 캐시(1개 ≈ 340 MB, 상한 8 GB) → 14 GB, 스왑 16 GB | `LLAMA_ARG_CACHE_RAM=0` → 3 GB, 스왑 8 GB. Ubuntu 설정에도 반영 |
| Fast DDS 기본 설정에서 30 Hz RGB(0.9 MB) 수신 57% | UDP 전용 + 16 MB 소켓 버퍼 프로파일(`configs/fastdds.xml`) → 100% |

### Isaac Sim 5.1 + Qwen3-VL-8B (Ubuntu, RTX 5080 Laptop 16 GB, 드라이버 580)

- 스모크 테스트 `SMOKE PASS`: 0.3 m/s 명령에 6초 동안 1.80 m 보행, 골반 높이 0.775 m, RGB 640×480, 깊이 유효 76%, LiDAR 18,169점.
- 처음 실행에서 고친 문제: Isaac Python에 `mujoco` 없음(`sensors.py` 선택적 import), 에셋 파일 권한 0600 → uid 1234가 URDF를 못 읽음(0644로 저장).
- `inspect_conveyor` 단일 에피소드(seed 0):
  - 1회차: VLN이 조립 라인 도착 → 둘러보기로 CONVEYOR A 방향 선택 → 4 m 밖에서 `stop` → 판정은 맞았지만 성공 반경 밖. 마지막 단계의 `stop`을 거절하도록 수정.
  - 2회차: **성공**. VLN 결정 4회, 영상으로 찾은 대상이 컨베이어 외곽 안(오차 0.0 m), 정지 거리 0.86 m, 판정 정답("a few beige boxes on the conveyor" → yes), 충돌·넘어짐 0, VLM 지연 평균 3.9 s, 실시간 대비 0.67.

### Isaac Sim 경로 오프라인 검증 (Mac)

- 캠퍼스 USD 내보내기: 상자 1,236 · 평면 57 · 원기둥 54 · 타원체 12 · 충돌체 192 · 재질 187, 누락 0. Z-up, 1 m 단위 확인.
- 카메라·LiDAR 수학: 쿼터니언 왕복, 4개 100° 깊이 카메라로 MID-360 패턴 모사 시 MuJoCo 광선 투사 대비 중앙값 오차 1.1 cm.
- NVIDIA 문서와 대조해 수정: 카메라는 `World.reset()` 뒤 `initialize()`, 캐시 폴더 uid 1234 소유, Docker 간 ROS 2는 UDP Fast DDS.
- 넘어짐·시간 종료 시 마지막 상태 발행(평가기가 정지가 아닌 넘어짐으로 기록).
- 실제 실행(로봇 보행·센서·토픽)은 Ubuntu에서 `bash scripts/run_isaac_sim.sh --smoke-test 8`부터 확인 필요.

### 미검증

- Isaac Sim 5.1 + 8B 실험 행렬(현재 단일 과제 1개만 실행), 실시간 비율 0.95 이상 운용: [UBUNTU_ISAAC.md](UBUNTU_ISAAC.md) 8절.
- 프롬프트는 이 9개 과제로 개발함 → 일반화 성능은 새 지시문(보류 세트)으로 따로 측정해야 함.
- 과제당 seed 1회. 성공률은 동작 확인용 소규모 결과이며 통계적 성능 주장이 아님.

## 지도 조회 + A* 스택 (이전 주 경로 → 현재 `oracle` 기준선)

> 아래는 VLM이 의미 지도에서 **목표 장소를 골라 좌표로 조회**하고 A*로 가던 이전 버전의 결과. 목표 위치를 미리 아는 방식이라 VLN이 아니며, 지금은 `planner_mode:=oracle` 상한 기준선으로만 사용. 현재 주 경로(VLN) 결과는 위 "지도 기반 계층형 VLN" 절.

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

- (당시 기준) Ubuntu 실행·8B·Isaac 연결은 미검증 → 현재 상태는 위 VLN 절의 "미검증" 참고.
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

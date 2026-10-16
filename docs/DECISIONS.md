# 도구·모델 선정

## 시뮬레이터: MuJoCo(개발·검증) + Isaac Sim 5.1(VLN 실험)

| 기준 | MuJoCo | Isaac Sim |
|---|---|---|
| 강점 | 공식 G1 보행 정책으로 빠른 반복, CPU·Mac에서도 실행 | 시각 품질·센서·대규모 USD 환경 |
| 기본 자원 | CPU 물리·정책 추론 가능 | RTX GPU 중심, 설치·드라이버 구성 부담 |
| 공장 편집 | MJCF·Python으로 구역·선반 변경 (원본) | 같은 장면을 USD로 내보냄 (`scripts/export_campus_usd.py`) |
| 결정 | 개발·단위 테스트·Mac 검증 | **VLN 실험 대상** (Ubuntu GPU 스테이션). 아래 "Isaac Sim 5.1" 절 |

새로운 보행 정책은 학습하지 않음. 기존 정책의 입력인 몸체 좌표계 `(vx, vy, yaw_rate)`만 waypoint 추종기로 제공.
로봇 위치를 매 프레임 덮어쓰지 않음. 초기 reset 이후 모든 이동은 관절 토크·접촉 물리 결과.

## 공간·실험 구분

- 한 장면에 48×32m Warehouse 4기능 구역과 24×24m Office 4개 방.
- 연구실 이름은 독립 실험의 시나리오 프리셋. 물리 구역의 의미와 분리.
- 구역 간 이동은 중앙 통로를 지나는 고정 그래프. 새 경로 계획 연구는 `routes.py` 교체로 분리.
- 데스크톱 GUI는 MuJoCo 뷰어에 상태·경로 표시를 추가. 별도 웹 애플리케이션은 불필요.

## 보행: Unitree 공식 pretrained G1

- 저장소: [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym).
- 커밋: `276801e46c5d433564f24658bac64f254b7d2d4b`.
- 모델: `deploy/pre_train/g1/motion.pt`.
- 로봇: `resources/robots/g1_description/g1_12dof.xml`.
- 관측 47개, 동작 12개. 물리·PD 500Hz, 정책 50Hz.
- 양팔·허리 고정. 29 DOF G1 제어·매니퓰레이션 정책으로 간주하지 않음.

## VLN: 선택적 NaVILA

- 모델: `a8cheng/navila-llama3-8b-8f`.
- 선정: 이미지 이력 + 자연어에서 이동 지시를 생성하는 공개 VLN 모델.
- 기존 공개 로봇 통합을 G1용 검증 완료 모델로 간주하지 않음.
- 별도 서버·환경 제공. GPU 모델 추론 및 공장 도메인 성능은 추가 검증 필요.
- ViNT·NoMaD는 유용한 시각 내비게이션 대안이지만 자연어 조건 VLN과 구별해야 함.

## VLN: 지도 기반 계층형 VLN (주 경로)

조건: 로봇은 SLAM 지도 + 사람이 단 의미 주석(구역·고정 기준점)을 가짐. 확인할 세부 대상의 위치는 모름. 외부 에이전트가 "어디 가서 무엇을 확인" 지시를 줌. 공장 데이터로 학습 불가.

| 기준 | 지도 기반 VLM 웨이포인트 선택 (채택) | 지도 조회 + A* (`oracle`) | NaVILA (e2e 생성형) |
|---|---|---|---|
| 목표 위치 | 모름 → 영상으로 찾음 | 미리 앎 (**VLN 아님**) | 모름 |
| 고수준 계획 | VLM이 매 결정마다 영상 + 지시 + 지도 맥락으로 다음 웨이포인트 선택 | 없음 (좌표 조회) | 매 스텝 이동 명령 텍스트 생성 |
| 지도 사용 | 주행 가능 공간 → 후보, 의미 주석 → 구역 방향·보행거리 힌트 | 경로계획 | 안 함 |
| 출력 형태 | 결정형: 후보 번호·회전·bbox·정지 중 선택, 검증 후 실행 | - | 생성형: 자유 텍스트 → 파싱 |
| 이동 안전성 | 지도에서 직선으로 갈 수 있는 점만 후보, LiDAR 국소 우회 | 지도·LiDAR A* | 장애물 회피 없음 |
| 근거 기록 | 결정마다 이미지·생각·선택 | 경로 | 텍스트 |
| 역할 | **주 조건** — `docs/HVLN.md` | 상한 기준선 | e2e 비교 — `docs/VLN.md` |

- 방법: MapGPT(Chen et al., ACL 2024)의 지도 프롬프트(위상 지도 + 방문 이력을 텍스트로)를 연속 공장 지도·휴머노이드에 맞게 변경. 후보 웨이포인트를 영상 위 번호로 그려 고르게 하는 시각 프롬프트(PIVOT, VLMnav). 대상 구역 도착 시 네 방향 파노라마 관측(R2R·MapGPT의 전방위 관측에 해당, 머리 카메라 69° 시야 보완).
- 제외: VLMaps(세부 대상이 지도에 없으면 결국 좌표 조회), VLFM(미탐사 환경용, 지도가 이미 있음), ETPNav·BEVBert(R2R-CE 집 실내 학습 → 공장 재학습 필요).
- VLM: **Qwen3-VL-8B-Instruct** (실험 주 조건), 4B(경량 비교). 한국어·영어 지시, bbox(0–1000), JSON 응답을 한 모델로 처리. temperature 0.
- API는 OpenAI 호환 `/v1/chat/completions`로 고정 → Ollama·vLLM·사내 서버 모두 코드 변경 없이 교체.
- 규칙은 VLM 출력을 거절·재질의만 함(구역 이탈, 회전 반복, 명판 식별자 불일치, 조건까지 보는 2차 확인 실패, 찾기 단계 없는 계획). 행동을 대신 정하는 것은 세 번 모두 무효일 때의 결정적 대체(돌아보기 → 가장 이득 큰 후보)뿐이며 `note`로 기록.
- 센서 토픽은 실제 G1 드라이버(realsense2_camera, livox_ros_driver2) 기본 이름·형식과 맞춤 → 실로봇 전환 시 노드 수정 최소화.

## 시뮬레이터: Isaac Sim 5.1 (실험) + MuJoCo (개발·검증)

- 실험 대상은 Isaac Sim 5.1(공식 컨테이너 `nvcr.io/nvidia/isaac-sim:5.1.0`). 캠퍼스 장면은 MuJoCo 장면에서 USD로 내보내 배치·좌표·의미 주석이 두 시뮬레이터에서 같음.
- G1: `unitree_rl_gym`의 `g1_12dof.urdf`(보행 정책 학습에 쓴 모델)를 Isaac URDF 임포터로 로드. 정책 관측·PD 계산은 `policy_core.py`로 MuJoCo와 공유, 물리 200 Hz + 정책 50 Hz.
- ROS 2: Isaac Sim 내장 Humble 라이브러리 + rclpy(Python 3.11)로 같은 `SimTopics` 발행 코드 사용 → VLN 스택은 시뮬레이터를 구분하지 않음. DDS는 Isaac 기본인 Fast DDS(UDP, `configs/fastdds.xml`).
- LiDAR: MID-360 비반복 패턴을 100° 깊이 카메라 4개에서 샘플링해 모사(MuJoCo 광선 투사 대비 중앙값 오차 1.1 cm).

## 근거

- [MuJoCo Python 설치·viewer](https://mujoco.readthedocs.io/en/3.3.7/python.html).
- [Isaac Sim 하드웨어 요구](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html), [Isaac Sim 5.1 컨테이너](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html), [ROS 2 브리지·Fast DDS 설정](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html).
- [MapGPT](https://arxiv.org/abs/2401.07314), [VLMnav](https://arxiv.org/abs/2411.05755), [PIVOT](https://arxiv.org/abs/2402.07872), [VLFM](https://arxiv.org/abs/2312.03275), [VLMaps](https://arxiv.org/abs/2210.05714), [ETPNav](https://arxiv.org/abs/2304.03047), [Qwen3-VL](https://github.com/QwenLM/Qwen3-VL).
- [Unitree 공식 MuJoCo 배포](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b/deploy/deploy_mujoco).
- [NaVILA](https://github.com/AnjieCheng/NaVILA), [NaVILA-Bench](https://github.com/yang-zj1026/NaVILA-Bench).

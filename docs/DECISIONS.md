# 도구·모델 선정

## 시뮬레이터: MuJoCo

| 기준 | MuJoCo | Isaac Sim |
|---|---|---|
| 이번 목표 | 공식 G1 보행 정책으로 빠르게 경로 평가 | 시각 품질·센서·대규모 USD 환경에 강점 |
| 기본 자원 | CPU 물리·정책 추론 가능 | RTX GPU 중심, 설치·드라이버 구성 부담 |
| 공장 편집 | MJCF·Python으로 구역·선반 변경 | USD·Omniverse 생태계 |
| 결정 | 현재 기본 도구 | 시각 도메인 차이가 연구 병목이 될 때 재검토 |

새로운 보행 정책은 학습하지 않음. 기존 정책의 입력인 몸체 좌표계 `(vx, vy, yaw_rate)`만 waypoint 추종기로 제공.
로봇 위치를 매 프레임 덮어쓰지 않음. 초기 reset 이후 모든 이동은 관절 토크·접촉 물리 결과.

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

## 근거

- [MuJoCo Python 설치·viewer](https://mujoco.readthedocs.io/en/3.3.7/python.html).
- [Isaac Sim 하드웨어 요구](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
- [Unitree 공식 MuJoCo 배포](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b/deploy/deploy_mujoco).
- [NaVILA](https://github.com/AnjieCheng/NaVILA), [NaVILA-Bench](https://github.com/yang-zj1026/NaVILA-Bench).

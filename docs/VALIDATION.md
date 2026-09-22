# Warehouse + Office 검증 결과

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

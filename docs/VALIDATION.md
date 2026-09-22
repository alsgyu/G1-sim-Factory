# 검증 결과

실행일: 2026-09-22. Ubuntu 24.04 컨테이너, Python 3.12, MuJoCo 3.3.7, Torch 2.7.1+cpu.

## 실제 보행·경로 추종

공식 G1 XML·사전학습 정책으로 실행. 위치 강제 이동 없음.

| 구역 | 결과 | 시뮬레이션 시간 | 경로 RMSE | 최대 경로 오차 | 목표 오차 | 충돌/넘어짐 |
|---|---|---:|---:|---:|---:|---|
| lab_a | 4/4 waypoint 완주 | 35.20 s | 0.0484 m | 0.0875 m | 0.2955 m | 0 / 0 |
| lab_b | 4/4 waypoint 완주 | 35.20 s | 0.0484 m | 0.0875 m | 0.2955 m | 0 / 0 |
| lab_c | 4/4 waypoint 완주 | 35.20 s | 0.0484 m | 0.0875 m | 0.2955 m | 0 / 0 |
| lab_d | 4/4 waypoint 완주 | 35.20 s | 0.0484 m | 0.0875 m | 0.2955 m | 0 / 0 |

- 동일 배치를 네 위치에 놓은 실행, seed=0. 서로 다른 난이도의 4개 벤치마크가 아님.
- 목표 통과 기준 0.3m. 경로 RMSE는 50Hz pose에서 기준 선분까지의 최단거리.
- 몸체 최저 높이 약 0.7642m. 충돌·넘어짐 확인은 물리 주기 500Hz.
- 최단 경로 SPL, 언어 이해, SLAM, 장애물 재계획 성능은 측정하지 않음.
- [기계 판독 가능한 원본 summary](validation/results.json).

## 코드·렌더링

- `python -m pytest -q`: **38 passed**.
- Python 컴파일, bash 문법 검사 통과.
- 공식 에셋 32개 다운로드 및 Git blob 체크섬 검증 통과.
- 실제 G1 공장 장면: 12 actuators, 27 meshes, 8 mocap shelves, 2 cameras.
- EGL/Mesa headless에서 공장·구역·ego RGB 및 유한한 양수 depth 생성 확인.
- 선반 이동·잘못된 배치 거부·연구실별 독립 변경 확인.
- NaVILA 출력 파서·상대 이동 목표 처리 확인.

## 미검증 범위

- 사용자 Ubuntu PC에 직접 설치하거나 접속하지 않음. 해당 PC에서는 README 설치 명령 실행 필요.
- 데스크톱 GUI는 이 환경에 디스플레이가 없어 미검증. headless 물리·렌더링 검증 완료.
- `MUJOCO_GL=osmesa` 경로는 문서화했지만 이 환경은 EGL/Mesa로 검증.
- NaVILA 전체 체크포인트 다운로드·GPU 추론·G1 공장 VLN 성공률은 미검증.
- Ubuntu 22.04/Python 3.10 조합은 설치 대상이지만 이 작업의 실제 테스트 환경은 24.04/3.12.
- 센서 온도는 합성값. 동적 장애물·외란·마찰 변화·다중 G1은 별도 실험 필요.

## 재현

```bash
source .venv/bin/activate
python scripts/fetch_assets.py --verify-only
python -m pytest -q
for lab in lab_a lab_b lab_c lab_d; do
  python -m g1_factory.run --lab "$lab" --headless || break
done
```

모델 버전·정책 SHA256·설정 SHA256은 각 `summary.json`에 기록.

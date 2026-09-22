# VLN 실행과 연구 확장

| 모드 | 입력 | 실행 조건 |
|---|---|---|
| 기본 경로 추종 | 구역 방문 순서 | 기본 설치만 필요 |
| NaVILA VLN | RGB 이력 + 언어 지시 | 별도 GPU 추론 서버 필요 |

- 기본 실행은 경로 추종 평가. 언어 이해 성능을 평가하지 않음.
- 선택 모델: **NaVILA `navila-llama3-8b-8f`**.
- 영상·언어 → 전진/회전/정지 → 기존 G1 사전학습 보행 정책.
- 실제 모델 호출 어댑터 제공. **GPU 모델 추론·현재 Warehouse/Office의 VLN 성공률은 미검증.**

## 1. GPU 서버 설치

- Ubuntu 22.04 x86_64 · Python 3.10 · NVIDIA GPU.
- 공식 벤치마크 안내: VRAM **24GB 이상**.
- 기본 시뮬레이터와 분리된 `.venv-navila` 사용. 별도 GPU 서버에서도 실행 가능.
- 모델 다운로드용 디스크·인터넷 필요. 체크포인트는 Git에 포함하지 않음.

```bash
# 저장소 루트에서 실행
bash scripts/setup_navila.sh --download
.venv-navila/bin/python scripts/navila_server.py \
  --model-path models/navila-llama3-8b-8f
```

원격 GPU 서버 사용 시 SSH 터널:

```bash
ssh -L 54321:127.0.0.1:54321 USER@GPU_HOST
```

서버 기본 바인딩은 `127.0.0.1`. 터널 연결 후 아래 클라이언트 명령 사용.

## 2. G1 창에서 VLN 실행

별도 터미널에서:

```bash
source .venv/bin/activate
python -m g1_factory.run \
  --zone-route material_storage inspection \
  --navila-url http://127.0.0.1:54321 \
  --instruction "Leave the materials storage area, follow the central aisle to the inspection area, and stop near its entrance." \
  --goal-zone inspection
```

- MuJoCo 창에서 G1 보행·현재 구역·최근 모델 응답 확인.
- `1`: 전체 보기 / `2`: G1 따라가기 / `3`: 1인칭 / `Space`: 일시정지.
- 별도 웹 GUI는 없음. 언어 지시는 실행 인자로 전달.
- 화면 없이 실행: `--headless` 추가. GPU 없는 시뮬레이터 호스트: 명령 앞에 `MUJOCO_GL=osmesa`.
- RGB·깊이 저장: `--record` 추가.
- 영문 지시 권장. 위 문장은 연결 확인용 예시이며 검증된 지시·정답 벤치마크가 아님.

## 3. 시작 위치·평가 목표

| 인자 | 의미 |
|---|---|
| `--scenario` / `--lab` / `--zone-route` | 장면 내 시작 위치·기준 시나리오 선택. 셋 중 하나 |
| `--navila-url` | 고정 경로 대신 모델의 이동 제안 실행 |
| `--instruction` | 모델에게 전달할 언어 지시. VLN에서 필수 |
| `--goal-zone inspection` | 해당 구역의 YAML `goal` 좌표를 평가 정답으로 사용 |
| `--goal-world X Y` | 세계 좌표 기준 평가 정답. `--goal-zone`과 택일 |

- **VLN 성공:** 모델이 `stop` 출력 + 정답 좌표 0.5m 이내 + 충돌·넘어짐 없음.
- `--goal-zone`은 구역 전체에 대한 도착 판정이 아니라 **구역 안의 지정 목표점** 판정.
- 평가 정답·고정 waypoint는 NaVILA에 전달하지 않음. 모델은 RGB 이력과 언어만 받음.
- `--goal-world`는 구역 내부의 장애물 없는 지점이어야 함. 좌표 단위 m.
- 이전 `--goal-local`은 제거됨. 새 설정·명령은 세계 좌표 사용.
- 추론 동안 물리 시간이 정지하는 동기 방식. 추론 지연은 따로 기록하며 실시간 응답 성능과 구분.

| 결과 파일 | 내용 |
|---|---|
| `actions.json` | 모델 원문·파싱한 동작·추론 지연·시뮬레이션 시각 |
| `zone_events.json` | 구역 진입 순서·시각·위치 |
| `summary.json` | 성공 여부·목표 오차·충돌·넘어짐·방문 구역 |
| `frames/` | `--record` 사용 시 G1 시점 RGB·깊이 |

## 모델 인터페이스

```text
POST /infer
입력: {"instruction": "Walk to the inspection area ...", "frames": ["base64 JPEG", ...]}
출력: {"text": "move forward 25 cm", "action": {"kind": "move_forward", "value": 0.25}}
```

| 항목 | 규칙 |
|---|---|
| 영상 | 시간순 RGB 1~8장. 마지막 장이 현재 관측 |
| 부족한 이력 | 공식 벤치마크 방식의 검정 이미지 패딩 |
| 이동 | `move forward 25 cm` 등. `value` 단위 m |
| 회전 | `turn left 15 degrees` / `turn right 15 degrees`. 단위 ° |
| 정지 | `stop` |
| 실행 상한 | 한 번에 전진 0.5m / 회전 30° |
| 잘못된 출력 | 오류 반환. 임의 동작으로 대체하지 않음 |

이동·회전 상한은 본 프로젝트 설정이며 원 논문의 평가 조건과 다름. 뷰어의 경로·상태 오버레이는 모델 RGB에 포함되지 않음.

## 교체 지점·현재 범위

| 연구 대상 | 코드 |
|---|---|
| VLN 모델·응답 해석 | `g1_factory/navila.py`, `scripts/navila_server.py` |
| 구역 간 경로 계획 | `g1_factory/routes.py` |
| waypoint·상대 목표 추종 | `g1_factory/navigation.py` |
| G1 보행 정책 | `g1_factory/locomotion.py` |
| 환경·가구 | `g1_factory/scene.py`, `g1_factory/office.py` |

- 현재 계획기: 고정 중앙 통로 그래프. 위치·방향: 시뮬레이터 정답 pose.
- 후속 연구: 장애물 회피·재계획·SLAM·언어별 성공률·SPL·nDTW. 지시문·정답 경로 데이터셋 별도 필요.
- 공식 NaVILA-Bench: Go2/H1 + Isaac 기반. 이 G1/MuJoCo 연결은 별도 어댑터.
- Warehouse·Office의 형상·카메라 높이는 학습 환경과 다름. 도메인 적응 성능은 후속 평가.

## 버전·출처

- NaVILA 코드: `76b98f233dd0fff05dfcd69435eec6740febff9d`.
- 모델 revision: `b2294e96581454468d6b94f38201f4f965ef48b7`.
- 참고 NaVILA-Bench: `e9d2db12ce5788c0f987d734c0094100b6bc0d3a`.
- 공식 `llava` 로더·전처리·`llama_3` 프롬프트·`model.generate` 사용.
- Transformers 4.37.2 + upstream 패치 사용. 전이 의존성 전체를 잠그는 lockfile은 없음.
- 코드·모델의 이용 조건은 각 배포처 기준. 코드 라이선스를 체크포인트 전체에 적용하지 않음.

[NaVILA 공식 코드](https://github.com/AnjieCheng/NaVILA) · [체크포인트](https://huggingface.co/a8cheng/navila-llama3-8b-8f) · [NaVILA-Bench·GPU 안내](https://github.com/yang-zj1026/NaVILA-Bench) · [참고 추론 서버](https://github.com/yang-zj1026/NaVILA-Bench/blob/e9d2db12ce5788c0f987d734c0094100b6bc0d3a/scripts/vlm_server.py) · [추론 코드 MIT 고지](licenses/NaVILA-Bench-MIT.txt)

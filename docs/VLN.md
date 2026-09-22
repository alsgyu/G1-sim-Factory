# VLN 연결

- 기본 실행: 지정된 좌표 경로 추종. VLN 점수와 구분.
- 선택 모델: **NaVILA `navila-llama3-8b-8f`**. 영상 이력 + 언어 지시 → 이동/회전/정지.
- 보행: 기존 G1 정책 유지. VLN과 보행 정책을 따로 교체.
- 현 상태: 실제 모델을 호출하는 어댑터 제공. GPU 모델 다운로드·추론·공장 VLN 성능은 이 작업 환경에서 미검증.

## 선택 설치

- Ubuntu 22.04 x86_64, Python 3.10, NVIDIA GPU 필요.
- 공식 벤치마크 안내: VRAM **24 GB 이상**. 다른 GPU 서버에서 실행 가능.
- `.venv-navila`에 격리. 기본 시뮬레이터 환경과 별도.
- 모델 용량에 맞는 디스크 공간·인터넷 필요. 체크포인트는 Git에 업로드하지 않음.

```bash
# 저장소 루트에서 실행
bash scripts/setup_navila.sh --download
.venv-navila/bin/python scripts/navila_server.py \
  --model-path models/navila-llama3-8b-8f
```

별도 터미널에서 기본 시뮬레이터 가상환경을 활성화하고 `--navila-url http://127.0.0.1:54321` 사용. 전체 실행 명령은 [README](../README.md) 참조.

원격 GPU 사용 시 서버는 기본값 `127.0.0.1`로 유지하고 SSH 터널 연결:

```bash
ssh -L 54321:127.0.0.1:54321 USER@GPU_HOST
```

## 인터페이스

```text
POST /infer
입력: {"instruction": "Walk to the shelf ...", "frames": ["base64 JPEG", ...]}
출력: {"text": "move forward 25 cm", "action": {"kind": "move_forward", "value": 0.25}}
```

| 항목 | 규칙 |
|---|---|
| 관측 | 시간순 RGB 1~8장. 마지막 장 = 현재 관측 |
| 부족한 이력 | 공식 벤치마크처럼 앞부분을 검정 이미지로 채움 |
| 이동 | `move forward 25 cm` 등. `value` 단위 m |
| 회전 | `turn left 15 degrees` / `turn right 15 degrees`. 단위 ° |
| 정지 | `stop` |
| 실행 상한 | 한 번에 전진 0.5 m / 회전 30° |
| 잘못된 출력 | 오류 반환. 임의 이동·가짜 추론 결과로 대체하지 않음 |

상한 적용은 이 프로젝트의 보수적 실행 설정. 원 논문의 평가 조건과 다름. 영문 지시를 권장하며 한국어 지시 성능은 확인하지 않음.

## 재현성과 범위

- NaVILA 코드: `76b98f233dd0fff05dfcd69435eec6740febff9d`.
- 모델 revision: `b2294e96581454468d6b94f38201f4f965ef48b7`.
- API 참고: NaVILA-Bench `e9d2db12ce5788c0f987d734c0094100b6bc0d3a`.
- 공식 `llava` 로더, 이미지 전처리, `llama_3` 프롬프트, `model.generate` 사용.
- Transformers 4.37.2와 upstream 패치를 사용. 일반 최신 Transformers로 교체하지 않음.
- 선택 설치 스크립트는 핵심 버전을 고정하지만 upstream의 일부 전이 의존성은 완전한 잠금 파일이 아님. 설치 충돌 시 기본 경로 추종 환경은 그대로 사용 가능.
- 공식 NaVILA-Bench는 Go2/H1 + Isaac 기반. **이 G1/MuJoCo 연결은 별도 어댑터**이며 공식 지원 조합이 아님.
- 평범한 VLM·ViNT·NoMaD를 VLN 평가 모델로 대체했다고 주장하지 않음. ViNT/NoMaD는 이미지 목표 기반 시각 내비게이션.
- 공장 색상·선반 형태·카메라 높이는 학습 환경과 다름. 도메인 적응 및 언어 지시별 성공률 평가는 후속 실험.
- 코드와 모델의 이용 조건은 각각 원 저장소와 모델 배포처 기준. 코드 라이선스를 체크포인트 전체에 적용하지 않음.

## 출처

- [NaVILA 공식 코드](https://github.com/AnjieCheng/NaVILA)
- [배포 체크포인트](https://huggingface.co/a8cheng/navila-llama3-8b-8f)
- [공식 NaVILA-Bench: GPU 요구사항·서버 분리](https://github.com/yang-zj1026/NaVILA-Bench)
- [참고한 추론 서버 소스](https://github.com/yang-zj1026/NaVILA-Bench/blob/e9d2db12ce5788c0f987d734c0094100b6bc0d3a/scripts/vlm_server.py)
- [ViNT·NoMaD 공식 코드](https://github.com/robodhruv/visualnav-transformer)
- [추론 코드 MIT 고지](licenses/NaVILA-Bench-MIT.txt)

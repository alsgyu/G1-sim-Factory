# Ubuntu 스테이션 실행 순서: Isaac Sim 5.1 + G1 + 계층형 VLN

`git pull` 이후 위에서부터 순서대로 실행. 저장소 루트에서 실행한다고 가정(`cd ~/G1-sim-test`).
구조·모델 선정 이유: [HVLN.md](HVLN.md) · 토픽·QoS: [ROS2_INTERFACE.md](ROS2_INTERFACE.md)

| 구성 | 실행 위치 |
|---|---|
| Isaac Sim 5.1 + G1 보행 정책 + 센서 (`isaac_sim_node`) | 컨테이너 `nvcr.io/nvidia/isaac-sim:5.1.0` (uid 1234, 저장소 읽기 전용) |
| VLN 스택 (`vln_agent`, `vln_navigator`, `path_follower`, `external_agent`) | 컨테이너 `g1-sim-ros2:humble` |
| VLM 서버 (Qwen3-VL 8B/4B) | 호스트 Ollama `127.0.0.1:11434` |
| 연결 | `--network host`, 같은 `ROS_DOMAIN_ID`, Fast DDS(UDP) `configs/fastdds.xml` |

필요 사양: Ubuntu 22.04/24.04 x86_64, NVIDIA RTX GPU(Isaac Sim 5.1 권장 사양 + Qwen3-VL-8B 약 6GB VRAM), 디스크 여유 약 60GB(Isaac 이미지 + 모델 + 캐시).

## 0. 코드 받기

```bash
git clone https://github.com/alsgyu/G1-sim-test.git ~/G1-sim-test   # 처음 한 번
cd ~/G1-sim-test && git pull
```

## 1. 한 번만 하는 준비

### 1.1 Docker + NVIDIA Container Toolkit (이미 있으면 확인만)

```bash
nvidia-smi
docker run --rm --gpus all ubuntu nvidia-smi
```

위 두 번째 명령이 실패하면 설치:

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

### 1.2 VLN 스택 이미지 · G1 에셋 · Isaac용 장면 USD

```bash
docker build -f docker/Dockerfile.ros2 -t g1-sim-ros2:humble .
docker run --rm -v "$PWD":/ws g1-sim-ros2:humble python3 scripts/fetch_assets.py
docker run --rm -v "$PWD":/ws g1-sim-ros2:humble python3 scripts/export_campus_usd.py --out outputs/isaac
```

- `fetch_assets.py`: Unitree 공식 `unitree_rl_gym`을 고정 커밋으로 받아 `external/`에 저장(G1 URDF·MJCF·보행 정책), 해시 검증.
- `export_campus_usd.py`: MuJoCo 캠퍼스와 **같은 배치**를 `outputs/isaac/campus.usd`(정적 충돌체·재질·표지판 텍스처) + `campus_meta.json`으로 내보냄. 장면 설정을 바꾸면 다시 실행.

### 1.3 Isaac Sim 5.1 컨테이너

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
mkdir -p ~/docker/isaac-sim/{cache/main/ov,cache/main/warp,cache/computecache,config,data/documents,data/Kit,logs,pkg}
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

- Isaac Sim 5.x 컨테이너는 uid 1234로 실행 → 캐시 폴더 소유자를 맞춰야 셰이더 캐시가 저장됨(NVIDIA 컨테이너 안내와 동일). `scripts/run_isaac_sim.sh`가 소유자를 확인함.

### 1.4 VLM 서버 (Ollama + Qwen3-VL)

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen3-vl:8b-instruct
ollama pull qwen3-vl:4b-instruct
curl -s http://127.0.0.1:11434/v1/models
```

VLN의 둘러보기는 영상 4장을 한 번에 보내므로(Ollama에서 장당 약 1,050 토큰) 문맥 길이를 8192로 늘려야 함(**필수**, 기본 4096이면 요청 거절). GPU 메모리를 Isaac Sim과 나눠 쓰므로 모델은 한 번에 하나만 올리도록 설정:

```bash
sudo systemctl edit ollama
```

열린 편집기에 아래 두 줄을 넣고 저장한 뒤 `sudo systemctl restart ollama`:

```text
[Service]
Environment="OLLAMA_CONTEXT_LENGTH=8192" "OLLAMA_MAX_LOADED_MODELS=1" "OLLAMA_KEEP_ALIVE=30m" "OLLAMA_FLASH_ATTENTION=1"
```

### 1.5 실험 실행기 의존성 (호스트 Python)

```bash
sudo apt-get install -y python3-yaml
```

## 2. 부팅할 때마다

대용량 영상(0.9MB RGB + 0.6MB 깊이, 30Hz)을 UDP로 잃지 않도록 소켓 버퍼 상한을 올림:

```bash
sudo sysctl -w net.core.rmem_max=2147483647 net.core.wmem_max=2147483647 net.ipv4.ipfrag_time=3 net.ipv4.ipfrag_high_thresh=134217728
```

## 3. 점검

```bash
docker run --rm --gpus all --network host -v "$PWD":/ws g1-sim-ros2:humble python3 scripts/check_hvln_env.py
docker run --rm --network none -v "$PWD":/ws g1-sim-ros2:humble python3 -m pytest -q
```

- `check_hvln_env.py`: ROS 2·DDS 설정, MuJoCo RGB-D·LiDAR 렌더 속도, Isaac 파일(USD·URDF·Fast DDS 프로파일), VLM 이미지 질의, VLM 4장 요청(둘러보기용 문맥 길이). **FAIL 0개**여야 함.
- `pytest`: 실패 0개(현재 120개 테스트, Mac에서 120 passed). `--network none`은 다른 ROS 그래프와 섞이지 않게 하려는 것.

## 4. Isaac Sim 스모크 테스트 (ROS 없이 G1이 걷는지)

```bash
bash scripts/run_isaac_sim.sh --smoke-test 8
```

- 장면 USD + G1 URDF 로드 → 2초 정지 후 0.3m/s 전진 명령 → 8초 시점에 측정값 한 줄(`SMOKE {"walked_m", "base_z", "upright", "rgb", "depth_valid", "lidar_points", …}`)과 판정 한 줄 출력.
- 통과 기준: 마지막 줄이 `SMOKE PASS` (서 있음 ∧ 0.5m 이상 걸음 ∧ LiDAR 점 1000개 초과).
- 첫 실행은 셰이더 컴파일로 수 분 걸림. 이후에는 캐시 사용.
- `SMOKE FAIL`이면 측정값 줄과 그 위 로그를 그대로 공유하면 원인(넘어짐 = PD·관절 순서, 점 0개 = LiDAR 카메라) 추적 가능.

## 5. 단일 에피소드 (Isaac Sim + VLN, 터미널 2개)

터미널 1, Isaac Sim (자재 창고에서 시작):

```bash
bash scripts/run_isaac_sim.sh --spawn -12 3.8 0
```

`/clock`이 나오면 준비 완료. 확인:

```bash
docker run --rm --network host -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml -v "$PWD":/ws g1-sim-ros2:humble ros2 topic hz /camera/camera/color/image_raw
```

터미널 2, VLN 스택 + 과제 `inspect_conveyor`("조립 라인으로 가서 컨베이어 위에 상자가 놓여 있는지 확인해"):

```bash
docker run --rm --network host --ipc host -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml -v "$PWD":/ws g1-sim-ros2:humble ros2 launch launch/g1_hvln.launch.py sim:=false task:=inspect_conveyor
```

- 결과: `outputs/hvln_inspect_conveyor/` → `summary.json`(성공·지표), `feedback.jsonl`(VLN 결정·생각 전부), `decisions/*.jpg`(매 결정에서 VLM이 본 이미지 + 번호 웨이포인트), `inspection.jpg`(판정 근거), `topic_stats.json`.
- 끝나면 터미널 1은 Ctrl-C. 다른 과제는 그 과제의 시작 위치로 Isaac을 띄움(`configs/vln_tasks.yaml`의 `spawn`).

### 5.1 외부 에이전트로 직접 지시하기

평가기 없이 그래프만 띄우고(`agent:=false`), 다른 터미널에서 지시를 보냄:

```bash
docker run --rm --network host --ipc host -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml -v "$PWD":/ws g1-sim-ros2:humble ros2 launch launch/g1_hvln.launch.py sim:=false agent:=false monitor:=false
```

```bash
docker run --rm --network host -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTRTPS_DEFAULT_PROFILES_FILE=/ws/configs/fastdds.xml -v "$PWD":/ws g1-sim-ros2:humble python3 scripts/send_instruction.py "창고 서쪽 끝에 있는 빨간 소화기함까지 가서 소화기가 들어 있는지 확인해"
```

- 진행 이벤트가 터미널에 출력되고, 확인 이미지는 `inspection.jpg`로 저장. 외부 에이전트 프로그램은 같은 방식으로 `/vln/instruction`에 발행하고 `/vln/feedback`을 구독하면 됨.

## 6. 실험 행렬

먼저 작은 부분으로 확인:

```bash
python3 scripts/hvln_experiment.py --simulator isaac --out outputs/exp_isaac_pilot --conditions vln_8b --seeds 0
```

전체(과제 9 × seed 3 × 조건 3 = 81회 + 토픽 측정 1회):

```bash
python3 scripts/hvln_experiment.py --simulator isaac --out outputs/exp_isaac_$(date +%F)
```

- 에피소드마다 Isaac 컨테이너를 과제 시작 위치로 새로 띄우고(`/clock` 대기 최대 900초), VLN 스택을 실행하고, 끝나면 Isaac 로그(`*_isaac.log`)를 저장하고 정리.
- 중단돼도 같은 명령을 다시 실행하면 끝난 에피소드는 건너뜀.
- 결과: `<out>/results.md`(조건별 성공률·SPL·grounding·판정 정확도·결정 수·지연), `episodes.csv`, `measurement/topic_tables.md`(토픽 크기·주기·QoS 실측 표).
- MuJoCo로 같은 행렬(비교·재현용): `python3 scripts/hvln_experiment.py --out outputs/exp_mujoco_$(date +%F)`.
- 다른 실험을 동시에 돌릴 때는 `--domain-id`를 다르게.

## 7. 문제 해결

| 증상 | 확인·조치 |
|---|---|
| `run_isaac_sim.sh`가 캐시 소유자 안내 후 종료 | 1.3의 `mkdir`·`chown` 실행 |
| Isaac이 `/clock`을 내지 않음 | `docker logs -f g1_isaac`(실행기는 `hvln_isaac_<domain>`), 첫 실행 셰이더 컴파일 대기, `nvidia-smi`로 VRAM 확인 |
| VLN 스택에서 토픽이 안 보임 | 두 컨테이너 모두 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + 같은 `FASTRTPS_DEFAULT_PROFILES_FILE`·`ROS_DOMAIN_ID`, `--network host` |
| 영상 주기가 낮거나 끊김 | 2의 `sysctl` 실행 여부. Fast DDS 기본 설정은 30Hz RGB의 약 43%를 잃음(측정) → `configs/fastdds.xml` 필수 |
| VLM 호출 시간 초과 | `ollama ps`, `nvidia-smi`. 8B와 Isaac이 VRAM을 나눠 씀 → `OLLAMA_MAX_LOADED_MODELS=1` |
| 피드백에 `look-around call failed: HTTP Error 400` | 문맥 길이 부족 → 1.4의 `OLLAMA_CONTEXT_LENGTH=8192` 적용 후 `sudo systemctl restart ollama` |
| 실시간보다 느림 | `summary.json`의 `realtime_factor`. 0.95 미만이면 `--set camera_rate_hz=15.0`으로 낮추고 결과에 명시 |

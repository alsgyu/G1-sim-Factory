# Ubuntu 스테이션 실행 순서: Isaac Sim 5.1 + G1 + 계층형 VLN

**`git pull` 이후 위에서부터 한 단계씩 실행. 각 단계 끝의 "확인"이 통과해야 다음 단계로.** 모든 명령은 저장소 루트에서 실행(`cd ~/Workspace/G1-sim-test`).
구조·모델 선정 이유: [HVLN.md](HVLN.md) · 토픽·QoS: [ROS2_INTERFACE.md](ROS2_INTERFACE.md)

| 구성 | 실행 위치 |
|---|---|
| Isaac Sim 5.1 + G1 보행 정책 + 센서 (`isaac_sim_node`) | 컨테이너 `nvcr.io/nvidia/isaac-sim:5.1.0` (uid 1234, 저장소 읽기 전용) — `scripts/run_isaac_sim.sh` |
| VLN 스택 (`vln_agent`, `vln_navigator`, `path_follower`, `external_agent`) | 컨테이너 `g1-sim-ros2:humble` (내 사용자 권한) — `scripts/g1_docker.sh` |
| VLM 서버 (Qwen3-VL 8B/4B) | 호스트 Ollama `127.0.0.1:11434` |
| 연결 | 호스트 네트워크, 같은 `ROS_DOMAIN_ID`, Fast DDS(UDP) `configs/fastdds.xml` |

필요: Ubuntu 22.04/24.04 x86_64, NVIDIA RTX GPU + 드라이버, 디스크 여유 약 60 GB(Isaac 이미지 약 20 GB + ROS 이미지 약 5 GB + 모델 10 GB + 캐시), 인터넷.

## 0. 현재 상태 확인

```bash
cd ~/Workspace/G1-sim-test && git pull && git log --oneline -1
```

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
```

```bash
docker --version && df -h ~ | tail -1
```

- `nvidia-smi`가 GPU 이름·드라이버·메모리를 출력해야 함. 실패하면 NVIDIA 드라이버부터 설치(`sudo ubuntu-drivers install` 후 재부팅).
- GPU 메모리(VRAM)가 16 GB 미만(예: 노트북 RTX 4080 12 GB)이면 Isaac Sim과 8B 모델을 동시에 올리기 빠듯함 → 8절의 `vln_4b` 조건부터 실행하거나, 8B 실행 시 Ollama가 일부를 CPU로 내림(느려짐).
- `docker: command not found`이면 1.1의 설치부터.

## 1. Docker 준비 (한 번만)

### 1.1 Docker 설치 (없을 때만)

```bash
curl -fsSL https://get.docker.com | sudo sh
```

- snap으로 설치된 Docker(`snap list | grep docker`가 결과를 냄)는 NVIDIA 컨테이너 설정이 다름 → `sudo snap remove docker` 후 위 명령으로 설치.

### 1.2 `permission denied ... /var/run/docker.sock` 해결 (sudo 없이 docker 쓰기)

```bash
sudo usermod -aG docker $USER
```

그다음 **로그아웃 후 다시 로그인**(또는 재부팅). 새 그룹은 새 로그인 세션부터 적용됨. 지금 터미널에서만 바로 쓰려면:

```bash
newgrp docker
```

확인:

```bash
docker run --rm hello-world
```

`Hello from Docker!`가 나오면 통과. 여전히 permission denied면 `groups` 출력에 `docker`가 있는지 확인하고, 없으면 로그아웃·로그인을 다시.

### 1.3 NVIDIA Container Toolkit (컨테이너에서 GPU 사용)

먼저 이미 되는지 확인:

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

GPU 표가 나오면 1.3은 건너뜀. `could not select device driver "" with capabilities: [[gpu]]`가 나오면 설치:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
```

```bash
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

```bash
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

```bash
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

확인: 위의 `docker run --rm --gpus all ubuntu nvidia-smi`가 GPU 표를 출력.

## 2. 저장소 준비 (한 번만. 이후 `git pull` 때는 해당 파일이 바뀐 경우에만 다시)

### 2.1 VLN 스택 이미지 빌드 (10–20분)

```bash
docker build -f docker/Dockerfile.ros2 -t g1-sim-ros2:humble .
```

확인:

```bash
docker images g1-sim-ros2
```

- `docker/Dockerfile.ros2`가 바뀐 `git pull` 뒤에는 다시 빌드(캐시 덕분에 짧음).

### 2.2 Unitree G1 공식 에셋 (고정 커밋, 해시 검증)

```bash
bash scripts/g1_docker.sh python3 scripts/fetch_assets.py
```

확인: 마지막 줄 `Verified 33 assets at 276801e…; manifest: /ws/external/assets_manifest.json`.

- `scripts/g1_docker.sh`는 이 저장소의 컨테이너 명령을 **내 사용자 권한**(만든 파일의 소유자가 나), 호스트 네트워크, GPU로 실행하는 래퍼. `sudo`를 붙이지 말 것(붙이면 root 소유 파일이 생겨 이후 실험 폴더 생성이 실패).

### 2.3 Isaac Sim용 캠퍼스 USD (MuJoCo 장면과 같은 배치)

```bash
bash scripts/g1_docker.sh python3 scripts/export_campus_usd.py --out outputs/isaac
```

확인: 마지막 줄이 `{"usd": "/ws/outputs/isaac/campus.usd", "materials": 187, "box": 1236, … "colliders": 192, "skipped": 0}`, 그리고

```bash
ls outputs/isaac
```

에 `campus.usd  campus_meta.json  mjcf  textures`.

- 장면(`configs/factory.yaml`, `g1_factory/scene.py`·`office.py`)이 바뀐 `git pull` 뒤에는 다시 실행.

## 3. Isaac Sim 5.1 컨테이너 (한 번만)

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

```bash
mkdir -p ~/docker/isaac-sim/{cache/main/ov,cache/main/warp,cache/computecache,config,data/documents,data/Kit,logs,pkg}
```

```bash
sudo chown -R 1234:1234 ~/docker/isaac-sim
```

- Isaac Sim 5.x 컨테이너는 uid 1234로 실행 → 캐시 폴더 소유자를 맞춰야 셰이더 캐시가 저장됨(NVIDIA 안내와 동일). `scripts/run_isaac_sim.sh`가 소유자를 확인하고, 틀리면 위 명령을 안내하고 멈춤.
- `docker pull`이 `unauthorized`로 실패하면 NGC 로그인 후 재시도: `docker login nvcr.io` (사용자명 `$oauthtoken`, 비밀번호 = NGC API 키).

## 4. VLM 서버: Ollama + Qwen3-VL (한 번만)

```bash
curl -fsSL https://ollama.com/install.sh | sh
```

서버 설정(파일 하나 생성): 문맥 8192(**필수**, VLN 둘러보기가 영상 4장을 한 번에 보냄), 모델 하나만 메모리에, 프롬프트 RAM 캐시 끔.

```bash
sudo mkdir -p /etc/systemd/system/ollama.service.d && printf '[Service]\nEnvironment="OLLAMA_CONTEXT_LENGTH=8192"\nEnvironment="OLLAMA_MAX_LOADED_MODELS=1"\nEnvironment="OLLAMA_KEEP_ALIVE=30m"\nEnvironment="OLLAMA_FLASH_ATTENTION=1"\nEnvironment="LLAMA_ARG_CACHE_RAM=0"\n' | sudo tee /etc/systemd/system/ollama.service.d/override.conf
```

```bash
sudo systemctl daemon-reload && sudo systemctl restart ollama
```

모델 받기(8B 약 6.1 GB, 4B 약 3.3 GB):

```bash
ollama pull qwen3-vl:8b-instruct && ollama pull qwen3-vl:4b-instruct
```

확인:

```bash
curl -s http://127.0.0.1:11434/v1/models
```

에 두 모델 이름이 보이고,

```bash
systemctl show ollama --property=Environment
```

에 `OLLAMA_CONTEXT_LENGTH=8192 … LLAMA_ARG_CACHE_RAM=0`이 보이면 통과.

- 문맥 8192가 없으면 둘러보기 요청이 HTTP 400(`exceeds the available context size`)으로 거절됨. 5절 점검이 잡아냄.
- `LLAMA_ARG_CACHE_RAM=0`: Ollama 0.40의 llama-server는 기본으로 프롬프트를 RAM에 최대 8 GB 캐시함(영상 포함 프롬프트 1개 ≈ 340 MB). VLN 프롬프트는 매번 영상이 달라 재사용되지 않으므로 끔 → Isaac Sim과 RAM 경쟁 방지(Mac 실측: llama-server 14 GB → 3 GB).

## 5. 호스트 설정과 점검

실험 실행기(호스트에서 도는 Python) 의존성:

```bash
sudo apt-get install -y python3-yaml
```

대용량 영상(0.9 MB RGB + 0.6 MB 깊이, 30 Hz)을 UDP로 잃지 않도록 소켓 버퍼 상한을 올리고, 재부팅 후에도 유지:

```bash
printf 'net.core.rmem_max=2147483647\nnet.core.wmem_max=2147483647\nnet.ipv4.ipfrag_time=3\nnet.ipv4.ipfrag_high_thresh=134217728\n' | sudo tee /etc/sysctl.d/60-g1-dds.conf && sudo sysctl --system | tail -4
```

환경 점검:

```bash
bash scripts/g1_docker.sh python3 scripts/check_hvln_env.py
```

확인: 마지막 줄 `… PASS, … WARN, 0 FAIL`. 특히 `RGB-D render`(GPU EGL이면 수 ms/프레임), `Isaac file` 4개 PASS, `VLM four-image request (look-around)` PASS.

단위 테스트(다른 ROS 그래프와 섞이지 않게 도메인 99):

```bash
ROS_DOMAIN_ID=99 bash scripts/g1_docker.sh python3 -m pytest -q -p no:cacheprovider
```

확인: `122 passed`(실패 0).

## 6. Isaac Sim 스모크 테스트 (ROS 없이 G1이 서서 걷는지)

```bash
bash scripts/run_isaac_sim.sh --smoke-test 8
```

- 장면 USD + G1 URDF 로드 → 2초 정지 후 0.3 m/s 전진 → 8초 시점 측정값 한 줄(`SMOKE {"walked_m", "base_z", "upright", "rgb", "depth_valid", "lidar_points", …}`)과 판정 한 줄.
- 통과: 마지막 줄 `SMOKE PASS` (서 있음 ∧ 0.5 m 이상 걸음 ∧ LiDAR 점 1000개 초과).
- **첫 실행은 셰이더 컴파일로 5–15분** 걸리고 그동안 출력이 거의 없음. 이후에는 캐시 사용.
- `SMOKE FAIL`이거나 Python 오류가 나면 출력 전체를 그대로 공유(넘어짐 = PD·관절 순서, 점 0개 = LiDAR 카메라, import 오류 = Isaac API).

## 7. 단일 에피소드 (Isaac Sim + VLN, 터미널 2개)

터미널 1 — Isaac Sim (과제 `inspect_conveyor`의 시작 위치: 자재 창고):

```bash
bash scripts/run_isaac_sim.sh --spawn -12 3.8 0
```

`Isaac Sim G1 ready at …`가 나오면 준비 완료. 토픽이 나오는지 확인(다른 터미널):

```bash
G1_DDS=fastdds bash scripts/g1_docker.sh ros2 topic hz /clock
```

`average rate: …`가 나오면 Ctrl-C.

터미널 2 — VLN 스택 + 과제 "조립 라인으로 가서 컨베이어 위에 상자가 놓여 있는지 확인해":

```bash
G1_DDS=fastdds bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py sim:=false task:=inspect_conveyor
```

- 진행 로그: `planned` → `decision`(VLN 선택·생각) … `step_done` → `decision … looked around` → `target_located` → `arrived` → `inspecting` → `done`. 끝나면 launch가 스스로 종료.
- 결과: `outputs/hvln_inspect_conveyor/` → `summary.json`(성공·지표), `feedback.jsonl`(VLN 결정·생각 전부), `decisions/*.jpg`(매 결정에서 VLM이 본 영상 + 번호 웨이포인트), `inspection.jpg`(판정 근거), `topic_stats.json`.
- 끝나면 터미널 1은 Ctrl-C. 다른 과제는 `configs/vln_tasks.yaml`의 그 과제 `spawn`으로 Isaac을 띄우고 `task:=`만 바꿈.
- 8B가 기본. 4B로 보려면 `vlm_model:=qwen3-vl:4b-instruct`를 덧붙임.

### 7.0 VLN 과정을 화면으로 보기

터미널 1에서 `--gui`를 붙이면 Isaac Sim 창이 뜨고, 뷰포트 카메라가 로봇 뒤 위쪽에서 따라감(창 렌더링 때문에 시뮬레이션이 조금 느려짐):

```bash
bash scripts/run_isaac_sim.sh --gui --spawn -12 3.8 0
```

- 터미널 2의 launch 로그에 VLN 결정마다 `feedback: decision {"action", "waypoint", …, "thought": "…"}`로 VLM이 고른 행동과 이유가 실시간으로 나옴.
- VLM이 결정할 때 본 영상(번호 웨이포인트, 둘러보기 4방향)은 결정마다 `outputs/hvln_<과제>/decisions/`에 저장됨 → 이미지 뷰어로 열어 두면 새 장이 계속 추가됨: `eog outputs/hvln_inspect_conveyor/decisions/ &`.
- 실험 행렬(8절)은 화면 없이(headless) 실행. 행렬이 도는 동안에는 GUI 실행을 같이 하지 말 것(GPU 공유).

### 7.1 외부 에이전트로 직접 지시하기

평가기 없이 그래프만 띄우고(`agent:=false`, Ctrl-C까지 계속 실행):

```bash
G1_DDS=fastdds bash scripts/g1_docker.sh ros2 launch launch/g1_hvln.launch.py sim:=false agent:=false monitor:=false
```

다른 터미널에서 지시:

```bash
G1_DDS=fastdds bash scripts/g1_docker.sh python3 scripts/send_instruction.py "창고 서쪽 끝에 있는 빨간 소화기함까지 가서 소화기가 들어 있는지 확인해"
```

- 진행 이벤트가 출력되고 확인 이미지는 `inspection.jpg`로 저장. 외부 에이전트 프로그램은 같은 방식으로 `/vln/instruction`에 발행하고 `/vln/feedback`을 구독(QoS: [ROS2_INTERFACE.md](ROS2_INTERFACE.md) 3·4절).

## 8. 실험 행렬

먼저 작은 부분(8B, seed 0, 과제 9개 = Isaac 기동 포함 약 1–2시간):

```bash
python3 scripts/hvln_experiment.py --simulator isaac --out outputs/exp_isaac_pilot --conditions vln_8b --seeds 0
```

전체(과제 9 × seed 3 × 조건 3 = 81회 + 토픽 측정 1회, 하루 이상):

```bash
python3 scripts/hvln_experiment.py --simulator isaac --out outputs/exp_isaac_$(date +%F)
```

- 에피소드마다 Isaac 컨테이너를 그 과제 시작 위치로 새로 띄우고(`/clock` 대기 최대 900초), VLN 스택을 실행하고, 끝나면 Isaac 로그(`*_isaac.log`)를 저장하고 정리. 시뮬레이터가 멈춘 에피소드는 1회 자동 재시도(첫 시도는 `attempt1_seed*`에 보관).
- 중단돼도 같은 명령을 다시 실행하면 끝난 에피소드는 건너뜀.
- 결과: `<out>/results.md`(조건별 성공률·SPL·grounding·판정 정확도·결정 수·지연), `episodes.csv`, `measurement/topic_tables.md`(토픽 크기·주기·QoS 실측 표).
- 실행 전 명령만 보기: 같은 명령 끝에 `--dry-run`.
- MuJoCo로 같은 행렬(비교·재현용): `python3 scripts/hvln_experiment.py --out outputs/exp_mujoco_$(date +%F)`.
- 다른 실험을 동시에 돌릴 때는 `--domain-id`를 다르게.

## 9. 문제 해결

| 증상 | 원인 → 조치 |
|---|---|
| `permission denied while trying to connect to the docker API at unix:///var/run/docker.sock` | 사용자가 docker 그룹에 없음 → 1.2 (`usermod` 후 로그아웃·로그인) |
| `could not select device driver "" with capabilities: [[gpu]]` | NVIDIA Container Toolkit 없음 → 1.3 |
| `PermissionError` (`outputs/…`, `external/…`) | 예전에 `sudo docker …`로 만든 root 소유 파일 → `sudo chown -R $USER:$USER outputs external` 후 `scripts/g1_docker.sh`로 다시 실행 |
| `run_isaac_sim.sh`가 캐시 소유자 안내 후 종료 | 3절의 `mkdir`·`chown` |
| `Missing outputs/isaac/campus.usd` / `Missing G1 URDF` | 2.3 / 2.2 |
| Isaac `docker pull`이 `unauthorized` | `docker login nvcr.io` (3절) |
| 스모크 테스트가 오래 출력 없음 | 첫 실행 셰이더 컴파일(5–15분). `docker ps`로 컨테이너가 살아 있는지 확인 |
| VLN 스택에서 Isaac 토픽이 안 보임 | 스택을 `G1_DDS=fastdds`로 실행했는지, 두 쪽 `ROS_DOMAIN_ID`가 같은지(기본 0) |
| 영상 주기가 낮거나 끊김 | 5절 `sysctl` 적용 여부(`sysctl net.core.rmem_max`가 2147483647) |
| 피드백에 `look-around call failed: HTTP Error 400` | Ollama 문맥 4096 → 4절 설정 후 `sudo systemctl restart ollama` |
| VLM 호출 시간 초과·매우 느림 | `ollama ps`, `nvidia-smi`. 8B와 Isaac이 VRAM을 나눠 씀 → 4B 조건부터, `OLLAMA_MAX_LOADED_MODELS=1` 확인 |
| 실험이 점점 느려지고 스왑 증가 | `ps -o rss,comm -C llama-server`가 수 GB → 4절 `LLAMA_ARG_CACHE_RAM=0` 적용 여부 |
| 실시간보다 느림 | `summary.json`의 `realtime_factor`. 0.95 미만이면 `--set camera_rate_hz=15.0`으로 낮추고 결과에 명시 |

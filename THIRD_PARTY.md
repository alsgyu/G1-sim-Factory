# 출처·라이선스

- G1 로봇·정책·설정: [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym), BSD-3-Clause. 설치 시 원본 LICENSE 함께 보관.
- 보행 관측·PD 계산: 위 저장소의 `deploy_mujoco.py`를 수정. 원문 고지는 `g1_factory/locomotion.py`에 포함.
- NaVILA 추론 흐름·프롬프트: [NaVILA-Bench](https://github.com/yang-zj1026/NaVILA-Bench), MIT. [고지](RILS/과제/롯데/G1-sim-test/docs/licenses/NaVILA-Bench-MIT.txt).
- NaVILA 코드: [공식 저장소](https://github.com/AnjieCheng/NaVILA). 체크포인트: [a8cheng/navila-llama3-8b-8f](https://huggingface.co/a8cheng/navila-llama3-8b-8f). 기반 모델·가중치의 라이선스는 각 모델 카드 확인.
- 계층형 VLN의 VLM: [Qwen3-VL](https://github.com/QwenLM/Qwen3-VL) Instruct 체크포인트를 사용자가 Ollama 등으로 별도 다운로드. 저장소에 포함하지 않음. 라이선스는 각 모델 카드 확인.
- 센서 메시지 형식 참고: [realsense-ros](https://github.com/IntelRealSense/realsense-ros) 토픽 이름, [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) PointCloud2 필드 배치. 코드는 복사하지 않고 형식만 맞춤.
- SLAM: [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) ROS 2 브랜치(커밋 a4743b0), GPL-2.0. 저장소에 포함하지 않고 Docker 이미지를 빌드할 때 받아서 컴파일(`docker/Dockerfile.ros2`). 이미지를 배포할 때는 GPL-2.0 조건을 따름.
- Livox 메시지 정의: `ros2_ws/src/livox_ros_driver2/msg/`의 `CustomMsg`, `CustomPoint`는 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)(MIT)와 같은 정의만 다시 작성(드라이버 코드는 포함하지 않음).
- MuJoCo: [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco), Apache-2.0.
- Isaac Sim 5.1: NVIDIA 공식 컨테이너 `nvcr.io/nvidia/isaac-sim:5.1.0`를 사용자가 직접 받아 실행(`ACCEPT_EULA=Y` = NVIDIA 라이선스 동의). 저장소에 포함하지 않음. Fast DDS 설정은 Isaac Sim ROS 2 설치 문서의 UDP 프로파일을 기준으로 버퍼만 확대.
- USD 내보내기: [OpenUSD `usd-core`](https://pypi.org/project/usd-core/), Apache-2.0 계열(Modified Apache 2.0).
- VLN 방법: MapGPT(지도 프롬프트), PIVOT·VLMnav(영상 위 후보 표시)의 아이디어를 참고해 이 프로젝트에서 새로 작성. 원 코드는 사용하지 않음.
- 공장 구조·waypoint 평가·HTTP 연결 코드: 이 프로젝트에서 작성.

외부 모델·메시·가중치는 저장소에 재배포하지 않고 고정 버전으로 다운로드.

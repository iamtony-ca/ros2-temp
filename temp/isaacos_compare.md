# Isaac Lab vs Isaac Sim / Isaac OS vs ROS 2 비교

## 1. Isaac Lab vs Isaac Sim

| 구분 | Isaac Sim | Isaac Lab (3.0 기준) |
|---|---|---|
| 정체 | 범용 로봇 시뮬레이터 / 디지털 트윈 | 로봇 학습(RL·IL) 프레임워크 |
| 주 용도 | 센서 시뮬레이션, 합성 데이터 생성(SDG), ROS 연동 검증 | 대규모 병렬 환경에서 정책 학습·평가 |
| 물리엔진 | PhysX | Newton(MJWarp/VBD/MPM), OVPhysX, Isaac Sim PhysX 중 선택 |
| 렌더링 | RTX | Newton Warp 렌더러, OVRTX, Isaac Sim RTX 중 선택 |
| 인터페이스 | Kit GUI, OmniGraph, Python 스크립트 | Python 설정 기반, `isaaclab train/play` CLI |
| ROS 2 | ROS 2 Bridge 내장 | Isaac Sim 경로를 쓸 때만 가능 |
| 상호 관계 | 독립 실행 가능 | 2.x: Isaac Sim 필수 → **3.0: Isaac Sim 선택 사항** |

3.0의 가장 큰 변화는 의존 관계가 끊어졌다는 점입니다. 이제 Isaac Lab은 Isaac Sim 없이 Newton 백엔드만으로 단독 실행할 수 있습니다. 대신 풀 PhysX/RTX, ROS bridge, XR, 라이브스트리밍, Kit 도구가 필요하면 여전히 Isaac Sim을 써야 합니다.

**업무 적용 기준으로 나누면:**
- **Isaac Sim**: Nav2 다중 AMR 검증, UR16e 모션 플래닝 가상환경, 6D pose 학습용 합성 데이터 생성 (ROS 연동과 센서 리얼리즘이 필요한 작업)
- **Isaac Lab**: FOUP 이적재, Wheel 체결 같은 manipulation 정책을 RL로 학습할 때

## 2. Isaac OS vs ROS 2

먼저 한계를 밝혀둡니다. Isaac OS는 아직 공개된 문서나 코드가 없습니다. 그래서 아래 Isaac OS 열은 **"Holoscan 기반"이라는 공식 설명에 Holoscan SDK의 일반적인 특성을 대입한 것**입니다. 실제 Isaac OS 구조와는 다를 수 있습니다.

| 구분 | ROS 2 | Isaac OS (Holoscan 기반, 추정 포함) |
|---|---|---|
| 성격 | 커뮤니티 표준 로봇 미들웨어 | NVIDIA 로봇 실시간 런타임 |
| 실행 모델 | 다수 노드(프로세스) + DDS pub/sub | 오퍼레이터 그래프 + 스케줄러 (Holoscan 방식) |
| 강점 | 분산 구조, 방대한 생태계 (Nav2, MoveIt2, ros2_control, tf2, rosbag) | 저지연 센서 스트리밍, GPU 상주 데이터 흐름, 결정론적 실행 (공식 표방) |
| 실시간성 | RT 커널과 ros2_control로 확보 가능하지만, DDS 통신 자체는 결정론적이지 않음 | 시뮬레이션부터 실로봇까지 결정론적 실시간 성능을 공식 목표로 명시 |
| 하드웨어 | 하드웨어 무관 | NVIDIA 플랫폼 중심 (Jetson Thor, Holoscan Sensor Bridge 사례) |
| 성숙도 | 장기 운영 실적 다수 | 발표 단계 |

NVIDIA의 공식 구분은 이렇습니다. Isaac ROS는 ROS 2와 GPU로 AI 로봇을 만드는 개발자를 위한 것이고, Isaac OS는 Holoscan 기반이며 엔터프라이즈급 애플리케이션용입니다. 즉 ROS 2를 대체한다고 발표한 것은 아닙니다. 두 생태계를 잇는 길도 이미 있습니다. Holoscan 예제 저장소(HoloHub)에 `holoscan_ros2` 브릿지 예제가 있어서, Holoscan 파이프라인과 ROS 2를 함께 쓰는 구성이 가능합니다. 다만 Isaac OS가 이 방식을 공식적으로 채택할지는 확인되지 않았습니다.

**함께 알아둘 점 (ROS 2 버전):** 최신 Isaac ROS 5.0 패키지는 ROS 2 Lyrical 기준으로 설계·테스트되었고, Lyrical의 `rosidl::Buffer` 기능으로 GPU 무복사 전송을 합니다. 반면 4.x 계열은 Jazzy 호환으로 명시되어 있습니다. 부서 스택이 Jazzy라면 Isaac ROS 도입 시 4.x를 쓸지, Lyrical로 올라갈지를 먼저 결정해야 합니다.

**업무 적용 기준으로는:** Nav2, BT.CPP, ros2_control로 쌓은 AMR 자산은 ROS 2에 그대로 둡니다. Isaac OS는 휴머노이드처럼 Jetson Thor 위에서 고속 인식과 제어 루프를 돌리는 영역에서 의미가 생길 수 있습니다. 이 경우에도 상위 미션과 내비게이션은 ROS 2가 맡고, 하위 실시간 루프만 Isaac OS가 맡는 하이브리드 구조가 현실적일 것으로 봅니다. 이 부분은 문서가 공개되기 전까지는 추정입니다.

앞의 정리, 용어집과 합쳐서 부서 공유용 문서 하나로 만들어 드릴까요?

Sources:
- [Isaac Lab v3.0.0 Release Notes](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)
- [Isaac Lab Documentation](https://docs.nvidia.com/isaaclab/3.0/isaac-lab.md)
- [NVIDIA Isaac ROS](https://developer.nvidia.com/isaac/ros)
- [Isaac ROS SIPL Camera (release-5.0)](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_sipl_camera/index.html)
- [isaac_ros_sipl_camera GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_sipl_camera)
- [NVIDIA Forums – HoloHub holoscan_ros2](https://forums.developer.nvidia.com/t/holohub-and-holoscan-sensor-bridge-version-compatibility-for-vb1940-ros-2-example/376015)

##
##
##
##
##
# Isaac Sim 6.2.0 vs Isaac Lab 3.0 / ROS 2 vs Isaac OS 비교

## 1. Isaac Sim 6.2.0 vs Isaac Lab 3.0

| 구분 | Isaac Sim 6.2.0 | Isaac Lab 3.0 |
|---|---|---|
| 정체 | Kit 기반 로봇 시뮬레이터 | 로봇 학습(RL·IL) 프레임워크 |
| 출시 | 2026-10-14 GA | 2026-10-20 GA (v3.0.0) |
| 물리엔진 | PhysX(기본), Newton 1.6.1 + MuJoCo Warp 3.12.0 | Newton(MJWarp·VBD·MPM·Kamino), OVPhysX, Isaac Sim PhysX |
| Newton 실행 | Kit 내부에서 실행 | Kit 없이 단독 실행 |
| 렌더링 | RTX | Newton Warp 렌더러, OVRTX, Isaac Sim RTX |
| ROS 2 | ROS 2 Bridge, ros2_control 확장 | Isaac Sim을 설치한 경우에만 사용 |
| 주 인터페이스 | GUI, WebRTC 스트리밍, Python | `isaaclab train/play` CLI, 설정 기반 |
| 라이선스 | 소스 Apache 2.0 (실행에 필요한 구성요소는 별도 약관) | BSD-3-Clause |
| 상호 관계 | Isaac Lab 3.0 GA를 지원하는 버전 | Isaac Sim은 선택 설치 (`--extra isaacsim`) |

**근거:**
- Isaac Sim 6.2 기본 엔진은 PhysX이고, Docker Compose에서 `ISAACSIM_PHYSICS_ENGINE=newton`으로 Newton을 선택합니다 ([6.2.0 Release Notes](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html)).
- 6.2에 번들된 엔진 버전은 Newton 1.6.1, MuJoCo/MuJoCo Warp 3.12.0이며, Isaac Lab v3.0.0-GA를 지원합니다 ([호환성 표](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/compatibility.html)).
- Isaac Lab 3.0에서 Isaac Sim은 풀 PhysX/RTX, ROS bridge, XR, Kit 도구가 필요할 때만 쓰는 선택 사항입니다 ([v3.0.0 Release](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)).

## 2. ROS 2 vs Isaac OS

| 구분 | ROS 2 | Isaac OS |
|---|---|---|
| 성격 | 오픈소스 로봇 미들웨어 | 오픈소스 런타임 프레임워크 |
| 발표·출시 | Jazzy는 LTS (2024~2029) | 2026-10-21 GTC Berlin 발표 / **출시일 자료 없음** |
| 기반 기술 | DDS 기반 pub/sub (rmw로 교체 가능) | NVIDIA Holoscan |
| 목적 | 분산 노드 간 통신, 로봇 SW 재사용 | AI·센서·제어·로봇 HW를 연결하고, 시뮬레이션부터 실로봇까지 결정론적 실시간 성능 제공 |
| 대상 애플리케이션 | 범용 | 엔터프라이즈급 애플리케이션 (NVIDIA 표현) |
| 실행 모델 / API | 노드·토픽·서비스·액션, C++/Python | **자료 없음** |
| 지원 HW / OS | 하드웨어 무관, Ubuntu 등 | **자료 없음** (사례에 Jetson Thor가 언급될 뿐 공식 지원 목록은 없음) |
| ROS 2 연동 | — | **자료 없음** |
| 라이선스 | 코어 Apache 2.0 | "오픈소스"라고만 명시 / **구체 라이선스 자료 없음** |
| 코드·문서 | 공개 | **자료 없음** (공개 저장소·문서 확인 안 됨) |
| 생태계 | Nav2, MoveIt2, ros2_control 등 | 사례: Humanoid, Hexagon Robotics, Bear Robotics (Jetson Thor, Holoscan Sensor Bridge, Halos와 함께 사용) |

**근거:**
- Isaac OS의 정의, Holoscan 기반이라는 점, 결정론적 실시간이라는 목표, 파트너 사례는 모두 [NVIDIA 공식 보도자료](https://nvidianews.nvidia.com/news/nvidia-advances-isaac-platform-for-general-purpose-robotics-with-new-reference-humanoid-frameworks-simulation-pipelines-and-data)에 나옵니다.
- "Isaac ROS는 ROS 2 기반, Isaac OS는 Holoscan 기반이며 엔터프라이즈급 애플리케이션용"이라는 구분은 [NVIDIA Isaac ROS 페이지](https://developer.nvidia.com/isaac/ros)의 설명입니다.
- 출시일이 발표 외에 명시되지 않았다는 점은 [Blockchain.News 보도](https://blockchain.news/news/nvidia-expands-isaac-robotics-platform-with-humanoid-and-new-tools)에서도 확인됩니다.

**작성 원칙:** Isaac OS 열에는 위 공식 자료에 적힌 내용만 넣었고, 확인되지 않은 항목은 모두 "자료 없음"으로 표시했습니다. ROS 2 열은 공개 표준 사실 기준으로 작성했습니다.

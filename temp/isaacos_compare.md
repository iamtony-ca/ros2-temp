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

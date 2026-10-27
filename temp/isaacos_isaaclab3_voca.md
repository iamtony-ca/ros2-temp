앞선 정리에 나온 기술 용어를 분류별로 모았습니다.

**NVIDIA Isaac 플랫폼 구성요소**
- **Isaac Sim**: Omniverse 기반 로봇 시뮬레이터. PhysX 물리, RTX 렌더링, ROS bridge를 포함합니다.
- **Isaac Lab**: Isaac Sim이나 Newton 위에서 동작하는 로봇 학습 프레임워크(RL, 모방학습)입니다.
- **Isaac OS**: Holoscan 기반 로봇 온보드 실시간 런타임입니다. 이번에 새로 발표되었습니다.
- **Isaac ROS**: GPU 가속 ROS 2 패키지 모음입니다. 무복사 전송 기술인 NITROS를 포함합니다.
- **Isaac GR00T**: 휴머노이드용 파운데이션 모델이자, 데이터 수집부터 배포까지의 워크플로우입니다.
- **Cosmos**: NVIDIA의 월드 파운데이션 모델로, 합성 데이터 생성에 쓰입니다.
- **SimReady**: 물리 속성과 시맨틱 정보가 갖춰진 시뮬레이션용 USD 에셋 규격입니다.
- **Human 1M Dataset**: 3시점 동기 영상과 3D 손 추적이 담긴 사람 행동 데이터셋입니다.
- **Halos**: 자율주행차와 로봇용 풀스택 안전 플랫폼입니다.

**물리, 렌더링, 시뮬레이션 엔진**
- **PhysX**: NVIDIA의 기존 물리엔진입니다. 강체와 관절 시뮬레이션이 중심입니다.
- **Newton**: Warp 기반 오픈소스 물리엔진입니다. 여러 솔버를 선택해서 쓸 수 있습니다.
- **Warp**: Python으로 GPU 커널을 작성하는 NVIDIA 프레임워크입니다.
- **MuJoCo-Warp**: MuJoCo 물리엔진을 Warp로 GPU화한 버전입니다.
- **VBD (Vertex Block Descent)**: 케이블이나 천 같은 변형체를 시뮬레이션하는 솔버입니다.
- **MPM (Material Point Method)**: 액체, 모래 같은 입자형 재료를 시뮬레이션하는 기법입니다.
- **Kit / Kit-less**: Kit은 Omniverse 앱 프레임워크입니다. Kit-less는 Isaac Sim을 띄우지 않고 실행하는 방식입니다.
- **OVPhysX / OVRTX**: PhysX와 RTX 렌더러를 Kit 없이 쓸 수 있도록 분리한 독립 라이브러리입니다.
- **USD (OpenUSD)**: 3D 씬을 기술하는 표준 포맷입니다. Isaac 에셋의 기본 형식입니다.

**학습, 소프트웨어 개념**
- **RL (강화학습)**: 시행착오와 보상을 통해 정책을 학습하는 방법입니다.
- **RSL-RL**: ETH RSL이 만든 RL 라이브러리입니다. PPO 구현이 중심입니다.
- **정책(Policy)**: 관측을 받아 행동을 출력하는 학습된 모델입니다.
- **Sim-to-Real / Sim-to-Sim**: 시뮬레이션에서 학습한 정책을 실제 로봇이나 다른 물리엔진으로 옮기는 것입니다.
- **ProxyArray**: Isaac Lab 3.0의 데이터 래퍼입니다. `.torch`와 `.warp`로 같은 데이터를 복사 없이 접근합니다.
- **LEAPP**: Isaac Lab 3.0의 정책 export·배포 기능입니다. 약어가 무엇의 줄임말인지는 확인하지 못했습니다.
- **uv**: Astral이 만든 Python 패키지·환경 관리 도구입니다. 3.0의 기본 설치 방식입니다.
- **쿼터니언 (x,y,z,w)**: 회전을 나타내는 4원수의 성분 순서입니다. ROS와 tf2도 이 순서를 씁니다.
- **에이전트 스킬**: 코딩 에이전트가 따라가도록 저장소에 포함된 작업 지침입니다.

**하드웨어, 런타임**
- **Holoscan**: 센서 스트리밍 데이터를 저지연으로 처리하는 NVIDIA 엣지 AI SDK입니다.
- **Holoscan Sensor Bridge**: 카메라 같은 센서 데이터를 이더넷으로 GPU에 직접 보내는 브릿지 하드웨어입니다.
- **결정론적 실시간 (Deterministic real-time)**: 같은 입력이면 같은 결과가 정해진 시간 안에 나오도록 보장하는 것입니다.
- **Jetson Thor**: 로봇 온보드용 엣지 AI 컴퓨터입니다. FP4 기준 2,070 TFLOPS, 128GB 통합 메모리입니다.
- **IGX Thor**: 산업·의료용 고신뢰 엣지 플랫폼입니다. Halos의 기반입니다.
- **DGX Station**: 데스크톱형 AI 워크스테이션입니다.

**릴리스, 운영 용어**
- **GA (General Availability)**: 정식 출시 버전입니다.
- **EA (Early Access)**: 정식 출시 전 사전 공개 버전입니다.
- **폐쇄망 (Air-gapped)**: 외부 인터넷과 차단된 사내 네트워크입니다.
- **PoC (Proof of Concept)**: 도입 전에 기술 타당성을 검증하는 것입니다.

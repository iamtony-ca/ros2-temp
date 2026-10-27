# Isaac OS / Isaac Lab 3.0 정리 및 로봇 부서 관점 인사이트

## 1. 한눈에 비교

| 구분 | Isaac Lab 3.0 | Isaac OS |
|---|---|---|
| 정체 | 로봇 학습(RL/IL) 프레임워크 | 로봇 온보드 실시간 런타임 |
| 기반 | Newton / PhysX / (선택) Isaac Sim | NVIDIA Holoscan |
| 상태 | **GA 출시** (v3.0.0, 10/20) | **발표만 확인됨** (코드·문서·출시일 미공개) |
| 위치 | 학습·평가 단계 | 배포·실행 단계 |
| ROS 2 관계 | Isaac Sim 경로에서 ROS bridge 지원 | Isaac ROS(ROS 2 기반)와 별개 제품 |

NVIDIA가 그리는 흐름은 "데이터(SimReady, Human 1M) → 학습(Isaac Lab) → 실행(Isaac OS, Jetson Thor) → 안전(Halos)"입니다. 이 중 지금 실제로 손에 잡히는 것은 Isaac Lab 3.0뿐입니다.

## 2. 로봇 부서 부서 관점 인사이트

**① Isaac Lab 3.0: Manipulation 과제에는 실익, Nav2 다중 AMR 시뮬레이션에는 영향 적음**
- Kit 없이 Newton만으로 학습할 수 있어, OHT Wheel 체결/해체나 FOUP 이적재처럼 접촉이 많은 manipulation RL 실험이 가벼워집니다. 릴리스에 포함된 VBD 케이블 삽입 예제가 이런 접촉 기반 작업의 레퍼런스가 될 수 있습니다.
- 반면 Nav2 기반 20대 AMR 테스트베드는 ROS bridge가 필요하므로 여전히 Isaac Sim 경로를 써야 합니다. 3.0으로 바뀌는 부분은 거의 없습니다.

**② 쿼터니언 순서 변경은 리스크이자 기회**
- 3.0부터 (x,y,z,w) 순서를 쓰는데, 이는 ROS `geometry_msgs/Quaternion`·tf2와 같은 순서입니다. 앞으로 ROS 연동 시 변환 버그가 줄어듭니다.
- 다만 2.x 기반 코드는 전수 점검이 필요합니다. 리셋 자세, 목표 자세, 6D pose 관련 코드가 대상입니다. Isaac Lab 저장소에 `isaaclab-migrating-2x-to-3x` 에이전트 스킬이 포함되어 있어, 사내 코딩 에이전트로 마이그레이션을 보조할 수 있습니다.

**③ 폐쇄망이 가장 큰 도입 장벽**
- 3.0의 기본 설치 방식(`uv run`)은 PyPI에서 의존성을 자동으로 동기화하므로, 폐쇄망에서는 그대로 동작하지 않습니다. 외부에서 Docker 이미지나 wheel 캐시를 만들어 반입하는 절차를 미리 설계해야 합니다.
- SimReady 에셋(2,000종 이상)과 Human 1M 데이터셋은 Hugging Face로 배포됩니다. 따라서 사내 HF 미러 구축 과제와 바로 연결됩니다.
- 스택도 바뀝니다. Isaac Sim 6.2, Python 3.12, CUDA 13이 요구되므로 기존 Isaac Sim 6.0.1 Docker 환경은 유지하고, 3.0은 별도 컨테이너로 PoC하는 편이 안전합니다.

**④ Isaac OS: 지금은 관찰 대상**
- Holoscan 기반이라 현재 부서 스택(ROS 2 Jazzy, Nav2, BT.CPP)을 바로 대체하거나 보완하지 않습니다. ROS 2와 어떻게 연동되는지도 공개된 정보가 없습니다.
- 사용 사례가 Jetson Thor 기반 휴머노이드와 인식 쪽에 집중되어 있습니다. 양수형 휴머노이드 과제가 장기적으로 Thor급 온보드 연산으로 가면 검토할 가치가 있습니다. 하지만 올해 11~12월 시제품·현장 일정에는 영향이 없습니다.

**⑤ Halos는 Fab 내 휴머노이드 도입 시 체크포인트**
- 사람과 같은 공간에서 휴머노이드나 AMR을 운영하려면 안전 인증 이슈가 생깁니다. NVIDIA는 Halos를 IGX Thor 기반 안전 플랫폼으로 내세우고 있고, TÜV SÜD 등 인증 파트너도 언급했습니다. 향후 협력사 사양을 협의할 때 질문 항목으로 넣어둘 만합니다.

## 3. 권장 액션

1. **단기**: Isaac Lab 3.0을 별도 Docker로 반입해 PoC를 진행합니다. 기존 RSL-RL 태스크 1개를 마이그레이션해 소요 공수를 측정합니다.
2. **중기**: Manipulation RL 태스크를 Newton 백엔드에서 PhysX와 교차 검증합니다. 3.0이 PhysX↔Newton 정책 이전 가이드를 제공합니다.
3. **관찰**: Isaac OS 문서와 저장소가 공개되면 ROS 2 연동 여부를 기준으로 재평가합니다.

부서 공유용 문서로 만들어 드릴 수도 있습니다.

Sources:
- [NVIDIA Newsroom – GTC Berlin 보도자료](https://nvidianews.nvidia.com/news/nvidia-advances-isaac-platform-for-general-purpose-robotics-with-new-reference-humanoid-frameworks-simulation-pipelines-and-data)
- [Isaac Lab v3.0.0 Release Notes (GitHub)](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0)
- [Isaac Lab 2.x → 3.0 Migration Guide](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/isaaclab/migrating.html)
- [NVIDIA Isaac ROS 페이지](https://developer.nvidia.com/isaac/ros)

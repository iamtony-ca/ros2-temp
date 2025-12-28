좋습니다. 이제 Nav2의 SmacPlannerHybrid (Hybrid A*) 에 대한 개요 및 각 파라미터 설명과 함께 튜닝 가이드를 한글로 정리해드리겠습니다.

1. SmacPlannerHybrid 개요
SmacPlannerHybrid는 Hybrid-A* 알고리즘을 사용하는 전역 경로 생성기입니다.
다음과 같은 로봇 유형을 지원합니다:

카형 로봇 (Ackermann, 자동차 등)
차동구동 및 임의형상 로봇 (비원형)
다리형 로봇 (Legged robots)
고속 주행 로봇 (곡률 제약이 필요한 경우)
특징 요약:

Dubin과 Reeds-Shepp 모델 기반
선회반경, 후진, 비직선 주행 등의 물리적 제한을 반영한 경로 생성
옵션 smoothing 기능 내장 → 대부분 외부 스무더 불필요
SE(2) 충돌 검사 → 로봇 형상 고려
잠재장 기반 탐색 → 장애물 중심이 아닌 중간 통과 유도
2. 주요 파라미터 설명 및 튜닝 가이드
파라미터 설명 추천값 및 팁

motion_model_for_search	DUBIN 또는 REEDS_SHEPP 선택(REEDS는 후진 포함)	Reeds-Shepp: 정밀 필요시Dubin: 단순 전진 경로
minimum_turning_radius	로봇 최소 선회 반경 (m)	실 로봇 기준, 일반적으로 0.3~0.5
angle_quantization_bins	회전 각도 분해 수 (정밀도)	기본값 64 or 72 추천
reverse_penalty	후진 동작에 대한 패널티 (>= 1.0)	보통 2.0~3.0, 후진 억제 시 ↑
change_penalty	방향 전환 시 패널티 (전→후진)	0.0~0.3, 변경 억제 시 ↑
non_straight_penalty	직선 외 회전 경로 패널티	1.0~1.3, 경로 직선화 유도 시 ↑
cost_penalty	고비용 지역 회피 유도용 패널티	2.0~3.5, 장애물과 거리 확보 시 ↑
rotation_penalty	제자리 회전 패널티 (Lattice 전용)	높은 값 유지 (5.0 이상) 권장
retrospective_penalty	초기 경로보다 이후 경로 우선	0.025 기본, 0~0.1 사이
lookup_table_size	Dubin/Reeds 거리 사전계산 범위 (m)	10~30m
cache_obstacle_heuristic	이전 goal에 대한 장애물 히트맵 캐시 여부	true로 하면 재계산 시 매우 빠름
smooth_path	경로 smoothing 여부	true 유지 권장
debug_visualizations	/expansions, /planned_footprints 토픽 출력	디버깅용 (성능 저하 있음)
downsample_costmap	비용맵 다운샘플링 여부	큰 지도에서 성능 향상 목적
downsampling_factor	다운샘플링 비율 (e.g. 2: 5cm → 10cm)	지도 크기, 속도 목표에 따라 설정
tolerance	goal 근처 도달 허용 오차 (m)	일반적으로 0.2~0.5m
3. 경로 끝 처리: Analytic Expansion 관련 파라미터
파라미터 설명 추천값

analytic_expansion_ratio	goal 근처에서 shortcut 허용 비율	2.0 ~ 4.0
analytic_expansion_max_length	shortcut 허용 최대 거리 (m)	2m ~ 4m (min_turning_radius 기준 4~5배 이상 권장)
analytic_expansion_max_cost	shortcut 중 허용 최대 비용값	100~300
analytic_expansion_max_cost_override	goal 근처에서는 cost 무시할지 여부	goal 주변에서 반드시 접근해야 할 때 true 추천
4. 종료 조건 파라미터
파라미터 설명 예시

max_iterations	최대 노드 확장 수	넓은 맵이면 500,000~1,000,000
max_on_approach_iterations	goal 근처에서의 추가 확장 한도	1000~5000
max_planning_time	전체 계획 시간 제한 (초)	3.0~5.0s
terminal_checking_interval	몇 번 확장마다 goal 취소/타임아웃 확인	500~5000
5. 경로 품질 튜닝 전략 요약
목표 조정할 파라미터

더 직선적인 경로	non_straight_penalty ↑, cost_penalty ↓
후진 최소화	reverse_penalty ↑, change_penalty ↑
장애물과 거리 확보	cost_penalty ↑, minimum_turning_radius ↑
경로 매끄럽게	smooth_path: true, angle_quantization_bins ↑, analytic_expansion_* 적절 설정
성능(속도) 향상	downsample_costmap: true, downsampling_factor: 2~3, cache_obstacle_heuristic: true
6. 추천 기본 예시 설정
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"
  motion_model_for_search: "REEDS_SHEPP"
  minimum_turning_radius: 0.4
  reverse_penalty: 2.5
  change_penalty: 0.1
  non_straight_penalty: 1.2
  cost_penalty: 2.5
  analytic_expansion_ratio: 3.5
  smooth_path: true
  cache_obstacle_heuristic: true
  downsample_costmap: true
  downsampling_factor: 2
필요하면 각 파라미터별 그래픽 예시나 Rviz 디버깅 방법(expansions, planned_footprints topic)도 설명해드릴 수 있습니다. 원하시면 알려주세요.
readme 설명



Smac Planner 한눈에 보기
(ROS 2 Nav2 Global Planner 서버용 플러그인 팩)

이름 핵심 알고리즘 어떤 로봇에 적합? 특징

SmacPlanner2D	2 D A*	원형(원궤적) ⚙️ 차동·전방향	8-방향 격자 검색, 속도 빠름
SmacPlannerHybrid	Hybrid-A*	자동차·Ackermann·다족·고속/곡률제한	Dubins / Reeds-Shepp 모델 지원, 곡률·방향 제약 반영
SmacPlannerLattice	State Lattice	임의 형상(비원형) ⚙️ 차동·전방향·Ackermann·다족	맞춤형 제어 집합(Control set)으로 세밀한 주행 궤적
1. 왜 써야 할까?
속도 : 기본 Hybrid-A* 논문(50-300 ms)보다 훨씬 빠른 2-200 ms 수준
다양성 : 원형부터 장방형·다족 로봇까지 하나의 패키지로 커버
질 높은 경로 
비용(Inflation)·곡률·방향 바이어스를 휴리스틱에 직접 반영 → 더 중앙을 지나고 부드러운 궤적
계산된 경로가 기본적으로 기구학적(kinematic)으로 실행 가능
간단한 Gradient Smoother 내장 (Hybrid·Lattice는 선택적, 2D는 항상 적용)
확장성 : 템플릿 A*·Node 구조라 새로운 로봇 모델 추가가 쉬움
테스트·문서 : C++14, 단위/통합 테스트, doxygen 문서 풍부
2. 건축 자재(라이브러리)
블록 역할

AStar<NodeT>	초고속 템플릿 A* 핵심
CostmapDownsampler	코스트맵 해상도 축소 (N²속도 향상)
CollisionChecker	반경 또는 발자국(footprint) 충돌 확인
Smoother	간단·빠른 경로 평활화
3. 주요 기능 하이라이트
다중 해상도 검색 – 넓은 공간은 거칠게, 좁은 곳은 세밀하게
모션 프리미티브 10× 축소 – 업샘플 과정 없이 세밀한 궤적
사전 계산 휴리스틱 / 모션 테이블 – 반복 재계획 시 20× 가속
비용·방향·후진·변속 패널티 – 파라미터만으로 경로 성향 쉽게 조정
“도달 허용오차(tolerance)” – 목표 근처에서 최선의 경로라도 반환
4. 대표 파라미터 읽어 보기 (Hybrid 예시)
파라미터 의미 팁

tolerance	목표점 도달 오차(m)	0.5 m 추천
downsample_costmap / factor	코스트맵 다운샘플 여부·배율	긴 경로에 유리
motion_model_for_search	DUBIN vs REEDS_SHEPP	후진 필요 여부로 선택
minimum_turning_radius	최소 회전 반경(m)	실제 한계 이상, 환경 고려(0.4 m 기본)
angle_quantization_bins	θ 이산화 개수	64 권장
Penalty 계열	cost_, non_straight_, reverse_, change_	표 범위 안에서 미세 조정
penalty 합리적 범위

Cost 1.7–6.0
Non-Straight 1.0–1.3
Reverse 1.3–5.0
Change 0.0–0.3
5. 실제 적용 & 튜닝 팁
Inflation Layer는 “부드러운 포텐셜 필드” 가 되도록 cost_scaling_factor를 높여라.
통로 중앙을 잘 따라가고 동적 장애물 대응력이 좋아짐.
다운샘플 → 테스트 → 글로벌 코스트맵 해상도 재설정
5 cm 원본에서 2× 다운샘플로 만족하면 글로벌 코스트맵을 10 cm로 바꿔 CPU 절약.
Hybrid/Lattice 최소 회전 반경
로봇 기구학 한계보다 약간 크게 잡으면 경로가 덜 지그재그하고 계산도 빨라짐.
“틈새” 문제로 못 찾을 때
max_iterations 충분히 늘리고, SLAM 맵 작은 구멍은 메워서 아예 통과 불가로 만듦.
또는 footprint padding을 살짝 키워서 구멍을 막아도 OK.
디버깅 : debug_visualizations true → /expansions, /planned_footprints 토픽 확인.
(CPU↑, RViz에서만 켜고 평소엔 꺼두기)
6. 설치 & 설정 예시
sudo apt install ros-<distro>-nav2-smac-planner
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.5
      downsample_costmap: true
      downsampling_factor: 2
      motion_model_for_search: "REEDS_SHEPP"
      minimum_turning_radius: 0.4
      cost_penalty: 2.5
      ...
7. 어떤 로봇이 쓰고 있을까?
실내 배송·서비스 로봇
산업용 물류 로봇
수직 농장 로봇
태양광 패널 점검 로봇 … 등
마무리
Smac Planner는 “빠르고, 범용적이며, 튜닝이 쉬운” Nav2 경로 계획 패키지입니다.

간단한 2D A*부터 기구학 제약이 복잡한 고속 차량까지 한 번에 커버.
기본 파라미터로도 잘 돌지만, Inflation·Penalty·Downsampling·Turning Radius 네 가지만 만져 보면 대부분의 로봇에서 최적의 성능을 빠르게 끌어낼 수 있습니다.
궁금한 점이나 특정 로봇에 대한 세부 튜닝이 필요하면 편하게 물어보세요!







🌟 플래너별 지원 로봇 요약 (쉬운 한국어 설명)
플래너 어떤 로봇에 잘 맞나? 왜 필요한가?

SmacPlannerHybrid	- Ackermann·자동차·카트형 로봇- 고속 주행 또는 최소 회전반경이 중요한 로봇 (속도 때문에 뒷바퀴가 미끄러지거나 짐이 쏟아지는 걸 막아야 할 때)- 모양이 둥글지 않은 차동·전방향 로봇(네모·육각 등) ― 충돌을 자세히 따지는 SE2(자세+위치) 검사 필요할 때- 다족(걷는) 로봇	여러 기구학 제약(회전반경, 전·후진, 방향전환)을 그대로 반영해 실제로 움직일 수 있는 경로를 계산해 주기 때문
SmacPlannerLattice	- 임의 모양(사각·다각형 등) 로봇으로, 세밀한 기구학 검사를 해가며 계획해야 하는 경우- 기본 제공 외에 사용자 정의 로봇 모델을 넣어야 할 때 (차동·Ackermann·전방향 등 지원)	“상태 격자(State Lattice)” 기법으로 다양한 속도·회전 조합을 미리 준비해 두었다가 가장 자연스러운 주행 궤적을 골라 줌
SmacPlanner2D	- 원형 발자국(거의 원에 가깝게 볼 수 있는) 차동·전방향 로봇- 환경 크기에 비해 로봇이 작거나 ‘동그랗다’고 볼 수 있을 때 (예: 복도에서 RC카, 전시회장 큰 로봇)	단순 2D A*지만 코스트(장애물 비용)를 이용해 부드럽고 안전한 경로를 빠르게 찾아 줌
🚀 주요 특징(Features) – 특히 Hybrid A* 개선점
기능 알기 쉬운 설명

10 배 더 작은 모션프리미티브	기존 Hybrid A*는 ‘거친’ 동작을 만든 뒤 업샘플링(세분화)을 했는데, 이 플래너는 처음부터 10배 세밀한 조각으로 탐색해서 업샘플 자체가 필요 없음
다중 해상도 탐색	넓은 공간은 해상도를 낮춰 한 번에 크게, 좁은 곳은 세밀하게 → 계산량 O(N²) 감소
비용 기반 패널티	장애물 근처·급커브·후진 등에 패널티를 줘서 자연스럽고 매끄러운 경로를 바로 얻음 (= 후처리 평활화가 거의 필요 없음)
경량 Gradient Smoother	남은 울퉁불퉁을 빨리 다듬어 주는 미니 평활화 모듈
초고속 A* 템플릿	불필요한 복사·그래프 조회를 없애 원논문보다 더 빠른 계산 속도
사전 계산 휴리스틱·모션표	목표가 비슷하면 재계획을 20 배 이상 빠르게
모션모델 크기 자동조절	코스트맵 해상도·각도 구간(빈)에 맞춰 회전반경 등 내부 파라미터 자동 최적화
목표 근처 ‘최단 비상 경로’	정확히 못 들어가면 허용오차 안에서 가장 가까운 안전 경로라도 반환해 실패 확률 감소
여러 모션모델 지원	Dubins(전진만)·Reeds-Shepp(전·후진) 기본, 새 모델도 쉽게 추가
테스트·문서 풍부	단위·통합 테스트, Doxygen 문서, C++14 모듈화로 재사용성 굿
선택적 휴리스틱 재계산	같은 지도·목표에 대해 불필요한 재계산 생략 → 재계획 20× 가속
‼️ 위 기능 중 다중 해상도·모션모델·Smoother 등은 SmacPlanner2D와 SmacPlannerLattice에도 대부분 적용됩니다.
또, 여기 2D A*는 NavFn의 ‘웨이브프런트 아티팩트’(길이 울컥불컥 끊기는 현상)가 없어서 조금 느려도 경로 품질이 훨씬 좋다는 평가를 받습니다.

📌 정리
Hybrid → 자동차·고속·비원형 로봇 / 복잡한 기구학 제약
Lattice → 비원형 로봇 / 맞춤형 제어집합 / 더 자연스러운 속도·회전 패턴
2D → 원형 로봇 / 간단·빠른 글로벌 경로
세 플래너 모두 속도, 경로 품질, 유연성 면에서 Nav2 기본 플래너 대비 크게 향상됩니다.
환경·로봇 특성에 맞춰 적절히 선택해 보세요!







✨ SmacPlanner 주요 기능 (Features) — Hybrid-A*를 어떻게 개선했을까?
원문 쉬운 한국어 해설

Remove need for upsampling by searching with 10x smaller motion primitives (same as their upsampling ratio).	기존 Hybrid-A*는 “거친” 동작 조각(motion primitive)을 만든 뒤 upsampling으로 세분화했지만, 애초에 10배 짧은 primitive로 탐색하니 업샘플 과정이 아예 필요 없습니다. 계산 단계↓, 품질↑
Multi-resolution search allowing planning to occur at a coarser resolution for wider spaces (O(N²) faster).	넓은 공간에서는 지도 해상도를 자동으로 낮춰서(=큰 셀로) 탐색 → 계산량 N² 축소. 복도·좁은 통로에 들어가면 다시 세밀하게 검색합니다.
Cost-aware penalty functions in search resulting in far smoother plans (further reducing requirement to smooth).	A* 단계에서부터 비용(cost) 기반 패널티를 줍니다. 장애물 가까이·급곡률·후진 등을 덜 선호하도록 하여, 처음부터 부드러운 경로를 뽑아내므로 추가 Smoothing 부담이 줄어듭니다.
Gradient-descent, basic but fast smoother	남은 울퉁불퉁은 경사하강법(gradient descent) 방식의 가벼운 smoother로 빠르게 다듬습니다.
Faster planning than original paper by highly optimizing the template A* algorithm.	A* 템플릿 코드 자체를 최적화해 원 논문보다 훨씬 빠른 계획 시간을 달성했습니다.
Faster planning via custom precomputed heuristic, motion primitive, and other functions.	휴리스틱·모션 테이블 등을 사전 계산해 두어 재계획 시 더욱 가속합니다.
Automatically adjusted search motion model sizes by motion model, costmap resolution, and bin sizing.	사용 중인 모션 모델·코스트맵 해상도·각도 bin 수에 맞춰 primitive 길이·반경을 자동 조정합니다.
Closest path on approach within tolerance if exact path cannot be found or in invalid space.	목표 점이 막혀 있거나 딱 맞는 경로가 없으면, 허용 오차(tolerance) 안에서 가장 근접한 안전 경로를 반환해 실패를 최소화합니다.
Multi-model hybrid searching including Dubin and Reeds-Shepp models. More models may be trivially added.	Dubin(전진만)·Reeds-Shepp(전·후진) 등 여러 모션 모델을 동시 활용하여 탐색합니다. 다른 모델도 쉽게 추가 가능.
High unit and integration test coverage, doxygen documentation.	단위·통합 테스트가 잘 갖춰져 있고, Doxygen 문서도 풍부합니다.
Uses modern C++14 language features and individual components are easily reusable.	C++14 최신 문법을 사용, 각 컴포넌트가 모듈화되어 재사용이 쉽습니다.
Speed optimizations: no data structure graph lookups in main loop, near-zero copy main loop, dynamically generated graph and dynamic programming-based obstacle heuristic, optional recomputation of heuristics for subsequent planning requests of the same goal, etc.	메인 루프에서 그래프 조회를 없애고 복사(overhead)를 최소화, 장애물 휴리스틱을 DP로 빠르게 계산, 같은 목표로 재계획 시 휴리스틱 재사용 옵션 등 수많은 미세 최적화를 적용했습니다.
Templated Nodes and A* implementation to support additional robot extensions.	Node와 A*가 템플릿화돼 있어 새로운 로봇 모델을 손쉽게 확장할 수 있습니다.
Selective re-evaluation of the obstacle heuristic per goal/map or each iteration, which can speed up subsequent replanning 20× or more.	목표·지도 변화가 없으면 장애물 휴리스틱을 다시 계산하지 않아 재계획을 최대 20배 가속합니다.
위 대부분의 기능(다중 해상도, 모션 모델, smoother 등)은 SmacPlanner2D와 SmacPlannerLattice에도 적용됩니다.
특히 2D A* 구현은 NavFn 플래너에서 나타나는 ‘wavefront 지글지글’ 아티팩트 없이 깔끔한 경로를 생성합니다. 약간 느리더라도 경로 품질이 훨씬 좋아요.

🔧 Smoother 관련 추가 설명
예전 버전에는 CG 방식 smoother(원 Hybrid-A* 논문 스타일)가 있었지만,
속도도 느리고, 성공률이 들쭉날쭉 해서 폐지되었습니다.
대신 가볍고 일관적인 gradient-descent 기반 smoother로 교체되었습니다.
이 smoother는 cost-aware(비용 민감)하지 않지만,
Planner 단계의 penalty 함수가 이미 고비용 영역을 피해주므로 큰 문제 없이 동작합니다.
smoothing 과정에서 경로가 장애물과 충돌하면 즉시 중단하여 안전을 보장합니다.
“CG smoother가 꼭 필요해요!”라면 Smoothing Server에 Cost-aware smoother로 여전히 들어 있으니 사용 가능합니다. (계산 시간이 조금 더 듭니다.)
요약

SmacPlanner는 업샘플 없는 세밀 탐색, 다중 해상도, 비용 기반 패널티, 경량 smoother 등으로 Hybrid-A*를 대폭 개선해 더 빠르고, 더 부드럽고, 더 확장성 있는 글로벌 경로 계획을 제공합니다.







📊 성능 지표(Metrics) — 얼마나 빠를까?
구현 실험 맵 크기(셀 수)·각도 bin 평균 소요 시간

원본 Hybrid-A* (Dolgov 논문)	102,400셀 · 72bins	50 – 300 ms
SmacPlanner Hybrid (같은 bin 수)	147,456셀 (1.4 × 더 큼)	2 – 20 ms
〃	344,128셀 (3.3 × 더 큼)	30 – 200 ms
같은 해상도·각도 설정에서도 수 십 배 빠른 결과를 확인했습니다.

75 m 정도 경로를 계산한 실측 예시
(같은 맵, 같은 시작·목표 지점):

SmacPlanner2D (A*) : 243 ms
SmacPlannerHybrid (Hybrid-A*) : 144 ms
SmacPlannerLattice (State Lattice) : 113 ms
참고 : NavFn 플래너 146 ms (경로가 여기저기 끊기는 wavefront 아티팩트 포함)
⚙️ 주요 파라미터 설명 (SmacPlannerHybrid 예시)
YAML 스니펫의 주석을 한국어로 풀었으며, 동일 이름의 파라미터는 SmacPlanner2D 또는 Lattice에서도 비슷한 의미를 가집니다.

파라미터 의미 · 튜닝 팁

tolerance	목표 지점에 정확히 닿지 못해도 이 거리(미터) 안이면 성공으로 간주. 기본 0.5 m
downsample_costmap / downsampling_factor	코스트맵 해상도 축소 활성화 여부와 배율.예) factor 2 + 5 cm costmap ⇒ 10 cm로 축소 → 긴 경로에서 N² 속도 향상
allow_unknown	맵에서 UNKNOWN(미탐사) 영역을 통과할 수 있게 할지
max_iterations	A* 최대 확장 노드 수. 도달 실패 시 탐색을 강제로 종료. -1이면 무한
max_on_approach_iterations	목표 tolerance 안에 들어간 뒤 “마무리” 확장 허용량
terminal_checking_interval	몇 회 확장마다 “취소/타임아웃?”을 체크할지
max_planning_time	전체 플래닝 한계 시간(초). 남은 시간에 따라 smoother·upsample 시간을 자동 분배
motion_model_for_search	DUBIN(전진만) vs REEDS_SHEPP(전·후진)
cost_travel_multiplier	2D 전용. 고비용(장애물 근처) 피하기 가중치. 0 = 완전 무시, 1 이상 추천 (속도 ↔ 안전성 트레이드오프)
angle_quantization_bins	Hybrid / Lattice에서 θ 이산화 개수. 64 bins 기본
analytic_expansion_ratio · analytic_expansion_max_length	목표 근접 시 “직접 연결(analytic expansion)”을 시도할 거리·길이 한계
analytic_expansion_max_cost (_override)	analytic expansion 구간 중 허용할 최대 셀 비용. 목표에 매우 가까우면 override로 완화 가능
minimum_turning_radius	로봇/경로 최소 회전 반경(m). 로봇 한계보다 작게 잡지 말 것
reverse_penalty	Reeds-Shepp에서 후진 동작 가중치(≥ 1). 클수록 후진을 덜 선호
change_penalty	진행 방향 전·후진 전환 패널티 (0 이상)
non_straight_penalty	곡선 주행(직선 아님) 패널티 (≥ 1)
cost_penalty	높은 비용 셀을 휴리스틱에 반영할 때 추가 가중치 (1.3 – 3.5 권장)
retrospective_penalty	경로 뒤쪽(후시점) 기동을 약간 선호하도록 하는 패널티 (0 – 1)
rotation_penalty	Lattice 전용. 제자리 회전 primitive 사용을 억제
lookup_table_size	Dubin/Reeds-Shepp 거리 캐시 길이(m)
cache_obstacle_heuristic	동일 목표에 대해 장애물 휴리스틱 재사용 (고정 맵이면 40× 가속)
allow_reverse_expansion	Lattice 전용. 후진 primitive까지 그래프에 포함할지 (분기 수 2배)
smooth_path	Hybrid·Lattice에서 Gradient Smoother 사용 여부 (2D는 항상 True)
debug_visualizations	/expansions, /planned_footprints 토픽으로 디버그 시각화. 성능 저하 크므로 평소에는 False 권장
smoother.*	Smoother 세부 설정. w_smooth(매끄럽게), w_data(원본 유지) 비율, do_refinement로 3회 반복 정제 등
팁
• cost_penalty, reverse_penalty, non_straight_penalty, change_penalty 네 가지만 조정해도 경로 성향이 크게 변합니다.
• downsampling_factor를 2–3으로 두고 결과가 만족스러우면, 아예 global costmap 해상도를 낮춰 CPU를 절약하세요.





추가 파라미터 설명

📐 SmacPlanner 고급 파라미터 ― 감 잡히는 쉬운 설명
파라미터 “무슨 일을 하나?” 언제·어떻게 쓰면 좋을까? 주의할 점

refinement_num	Smoother가 한 번 다듬은 경로를 다시 잡고 또 다듬는 재귀 횟수. 숫자가 클수록 구불구불이 줄어들고 곡선이 매끈해짐.	- 급격한 지그재그가 남아 있으면 1 → 2 → 3으로 올려 본다. - 보통 2 ~ 3이면 충분.	반복마다 CPU를 다시 사용 → 너무 높이면 계획 시간↑
goal_heading_mode (DEFAULT / BIDIRECTIONAL / ALL_DIRECTION)	“목표에 도착할 때 어떤 방향(θ) 으로 서 있을지” 후보를 자동으로 늘려 줌.	DEFAULT• 사용자가 지정한 θ만 그대로 이용.BIDIRECTIONAL• θ와 θ+180° 두 가지 중 더 싼 경로 선택.• 좁은 복도에서 ‘전진·후진 아무 방향만 도착하면 돼요’ 할 때 유용.ALL_DIRECTION• angle_quantization_bins 로 나뉜 모든 방향을 시도 → “목표 점만 맞추면 방향은 알아서 제일 쉬운 걸로”.• 주차·도킹 같이 어떨 때는 옆으로, 어떨 때는 뒤로 들어가도 되는 작업에 편리.	• BIDIRECTIONAL, ALL_DIRECTION 은 탐색 공간이 커져 계산 시간↑.• ALL_DIRECTION 을 쓸 땐 아래 coarse_search_resolution 로 시간을 줄여 줄 수 있음.
coarse_search_resolution	ALL_DIRECTION 모드일 때 처음엔 큰 간격(거친 해상도) 으로 방향을 훑어보고, 유망한 지점만 세밀 검색(full resolution) 하는 단계 수. 값 = 건너뛰는 bin 수.	예) angle_quantization_bins = 72, coarse_search_resolution = 4 → 최초 탐색은 0·4·8·12… ° 만 검사 → 후보 발견 시 주변 각도를 다시 1° 간격으로 세밀 조정. → 방향 후보 72 → 18개로 줄어 시간 절약	값이 너무 크면 좋은 방향을 놓칠 위험 있으니 2 ~ 6 사이에서 실험해 보세요.
allow_primitive_interpolation	기본 primitive(직진·좌·우)로는 각도가 3칸씩(예) 튀어오를 때, 사이에 1·2칸짜리 보간 primitive를 자동으로 생성. 즉, “0°, ±3칸” 대신 “0°, ±1, ±2, ±3칸 …” 모두 탐색.	- 회전 반경이 작고 bin 수가 많아서 노드 간 점프가 큰 환경에서 지그재그 현상을 줄이기 좋음.- 복도처럼 좁은 공간에서 더 자연스러운 S자 경로가 나옴.	• primitive 수가 늘면 CPU↑.• 이 기능을 켰다면 angle_quantization_bins를 과히 높일 필요 없음.  · 예) 원래 72 bin + jump 3칸 ▶ now 24 bin + interpolation = 기능적으로 동일.
downsample_obstacle_heuristic	휴리스틱(장애물 거리 DP)을 만들 때 costmap을 한번 더 축소 해 계산 시간을 아끼는 기능. 값 True(기본) = 축소 사용.값 False = 원본 해상도 그대로 계산.	- 맵이 크고 복잡하면 True (속도↑, 약간 덜 부드럽지만 Smoother가 커버).- 짧은 이동·좁은 맵 또는 품질이 정말 중요할 때 False 로 해도 가성비가 괜찮음— upfront 계산↑ 대신 total iteration↓.	전체 플래닝 시간은 맵 크기·경로 길이·Inflation 세기에 따라 달라지니 둘 다 켜 보고 벤치마크해 보세요.
🔑 기억할 포인트
Smoother 관련 → refinement_num(몇 번 다듬나)와 w_smooth / w_data 로 자연스러움 ↔ 속도를 맞춘다.
Goal 방향 유연성 → goal_heading_mode 로 “똑바로? 뒤로? 아무 방향?”을 정의하고, coarse_search_resolution 로 계산량을 제어.
모션 primitive 품질 → 간격이 거칠면 allow_primitive_interpolation 로 보간 + angle_quantization_bins 재조정.
휴리스틱 해상도 → downsample_obstacle_heuristic False 로 두면 초기 계산은 무겁지만 전체 경로가 조금 더 자연스러워진다.
이해가 더 필요한 부분이나 실험 결과를 공유해 주시면, 추가로 구체적인 튜닝 방향도 제시해 드릴게요!







use_quadratic_cost_penalty — “비용을 제곱해서 보는 모드”
기본(Linear) Quadratic 모드 (cost²)

셀의 cost가 10 ➜ +10	셀의 cost가 10 ➜ +100
셀의 cost가 1 ➜ +1	셀의 cost가 1 ➜ +1 (거의 차이 없음)
핵심 아이디어

낮은 비용 구간(0 ~ 10 정도)은 무시할 만큼 작게,
높은 비용 구간(150 ~ 253)은 폭탄처럼 크게 보이게 만든다.
1️⃣ 무엇이 달라지나?
효과 직관적 이유

계획 속도 ↑	“안전 통로(0‒10)”와 “위험지역(200+)”의 점수 차이가 급격히 벌어져 휴리스틱이 위험 쪽 브랜치를 일찍 잘라냄(prune).
곡선이 매끄러움	통로 중앙 vs 벽 + 20 cost 차이를 예전엔 집요하게 따졌지만, 지금은 20²=400, 0²=0 → 차이 비중이 상대적으로 줄어 작은 코스트 줄이려 지그재그 하지 않음.
장애물에 살짝 더 가까움	저비용(0~20) 차이가 “거의 0” 취급이라 “벽에서 30 cm 떨어지나 40 cm 떨어지나 비슷” → 가끔 벽을 좀 더 끼고 지나감.
2️⃣ 언제 켜면 좋을까?
상황 권장

창고·복도 등 통로가 넓고 직선 위주	켜기 : 부드럽고 빠른 경로
장애물이 다닥다닥한 좁은 공간	실험 필요 : 벽에 너무 붙을 수도
미세하게 중앙을 맞춰야 하는 로봇 (ex. 좁은 선반 센터라인)	끄기 : Linear가 더 섬세함
3️⃣ 켰을 때 같이 만져야 할 것
파라미터 조정 방향 이유

cost_penalty	1.5 ~ 2.5 정도 조금↑	Quadratic 때문에 낮은 cost가 무시되니, 기본 가중치로 살짝 보강
Inflation Layer cost_scaling_factor	5.0 이상으로 강하게	벽 근처 cost를 더 확실히 키워 “가까이 붙음” 억제
change_penalty, non_straight_penalty	0‒1.1 정도로 낮추거나 OFF	Quadratic 자체가 지그재그를 억제하므로 중복 효과 줄이기
4️⃣ 비유로 더 쉽게
■■■■■ ← 벽 근처 cost 그래프

Linear : 0,1,2,3,4,5,6…
로봇은 “3보다 2가 싸네?” 하면서 중앙을 물고 늘어짐

Quadratic : 0²,1²,2²,3²… = 0,1,4,9…
1 ↔ 4 차이는 작게 느껴지고 30², 50²는 아득히 커서 얼른 안전지대만 확보하고 직진

결과적으로 연필로 한 번만 그린 듯한 깨끗한 곡선이 나오지만, 벽과의 “여유 거리”는 좀 줄어드는 특징이 있습니다.

✅ 정리 한 줄
use_quadratic_cost_penalty = True
→ 비용을 제곱해서 보니 위험 구역은 폭탄, 저비용 구역은 거의 평지로 인식
→ 계획 빨라지고 경로 매끄러워지지만 벽에 약간 더 붙을 수 있으니 cost_penalty & Inflation으로 밸런스 조정!









🌐 Potential Fields (인플레이션 계층을 왜 쓰나요?)
Inflation layer 의 진짜 목적
“벽에서 n cm만 부풀려서( inflate ) 충돌을 막는다”가 전부가 아닙니다.
지도의 모든 위치에 완만한 potential field(잠재 비용 경사) 를 만들어서
로봇이 통로 중앙을 따라가고
가까이 다가오는 사람·로봇 같은 동적 장애물을 자연스럽게 피해 갈 수 있게 합니다.
왜 중요할까?
0-cost(완전 평지) 구역이 넓으면 A*-류 플래너는 “벽 쪽이든 중앙이든 모두 같네?” 하고 경로를 흔들리게 계산합니다.
부드러운 potential field를 두면 “가운데가 더 싸다”는 신호가 생겨 안정적인 중앙 경로가 나옵니다.
실전 튜닝 팁
inflation_layer의 cost_scaling_factor(또는 Nav2 GUI의 “Cost scaling”) 값을 높여 보세요.
거대한 공터라면 중앙 일부를 0-cost로 남겨도 괜찮지만, 복도·통로·랙 사이 통로라면 전체 폭에 완만한 경사를 만들어 주세요.
🔄 Hybrid-A* / State Lattice 의 minimum_turning_radius 설정법
잘못된 생각 올바른 접근

“로봇이 실제로 돌 수 있는 최소 반경 0.25 m니까, 파라미터도 0.25 m!”	환경과 계산 효율도 같이 고려하자.
너무 작은 값(예: 0.2 m)을 주면
Search branch가 과도하게 늘어나 CPU 사용량↑
좁은 각도로 계속 꺾어 “지그재그 경로”가 생김 → smoothing 단계 필요
여유 있게 큰 값(예: 0.4 m-0.6 m)을 주면
가능한 후보가 줄어 계획 속도↑
처음부터 매끄러운 곡선 경로가 나와 후처리 필요↓
단, 실제 기구학 한계보다 작게 설정하는 것은 절대 금지!

권장 기본값

소형 산업 로봇·서비스 로봇: 0.4 m
Simbe, Locus, 작은 Fetch 등에서 테스트해 본 “무난한” 값
복도 폭이 로봇 회전 원보다 훨씬 넓다면 0.5 m 이상으로 늘려도 좋습니다.
Hobby용 TurtleBot-3처럼 아주 작은 로봇도, 실내 장애물이 많지 않다면 0.4 m로 충분히 돌 수 있습니다.
핵심 요약
Potential field: 인플레이션 값을 적당히 키워서 지속적인 비용 경사를 만들면 경로 품질이 확 올라갑니다.
Turning radius: minimum_turning_radius를 차량 한계 ≦ 값 ≦ 환경 폭 범위에서 약간 크게 잡으면 더 빠르고 부드러운 경로를 즉시 얻을 수 있습니다.






🗺️ Costmap Resolution & Downsampling — 쉽게 풀어쓴 가이드
Downsampler 옵션이 왜 필요할까?
Hybrid-A*, State Lattice, 2D A* 모두 costmap downsample 기능을 제공합니다.
큰 공간에서 긴 경로를 계획할 때, 해상도를 낮추면(A→B cm)
확장해야 할 노드 수가 N²만큼 감소 → CPU 시간 대폭 절약
실제 실험에서 해상도 2–3× 낮추면 계획 시간 2–4배 빨라짐
하지만 trade-off
해상도가 낮을수록 작은 장애물이 실제보다 크게 보임
로봇이 벽이나 장애물에 더 멀리서 멈춰야 하므로, 정밀 목표에 붙기가 어려움
결국 “계산 속도 vs. 정밀도”를 직접 테스트해 최적점을 찾아야 함
추천 실전 절차
5 cm costmap(일반적인 Nav2 기본값)으로 시작
downsampling_factor를 2, 3, 4 … 바꿔 보며
“목표점 정확도”가 괜찮은 가장 낮은 factor를 고른다
예: factor 2 × 5 cm → 10 cm 해상도에서 400 ms → 100 ms 미만으로 단축 사례
최적 factor를 찾았으면 global costmap resolution 자체를 그 값으로 변경
이렇게 하면 매번 downsample 계산을 할 필요가 없고, 메모리·CPU를 더 절약
fine-resolution costmap을 “사람 눈” 만족용으로 유지할 필요는 없음
플래너가 여러 개일 때
Planner Server에 2D A*, Lattice, Hybrid A* 등이 함께 등록돼 있다면
가장 해상도에 민감한 플래너(예: 2D A*용) 기준으로 global costmap을 정하고,
Hybrid A* 등에는 downsampler를 적용해 더 거칠게 써도 됨
요약: **“5 cm → 10 cm”**처럼 한두 단계만 해상도를 낮춰도 계획 속도는 기하급수적으로 빨라지고, 대부분의 실내 환경에서 목표 정확도는 충분히 유지됩니다. 직접 factor를 바꿔 가며 속도·정밀도 골든밸런스를 찾아보세요!











⚙️ Penalty Function Tuning — 패널티 값은 왜, 어떻게 바꾸나?
개념 쉬운 설명

Penalty	A*가 후보 경로를 평가할 때 “이 동작은 별로야”라고 추가 가중치를 주는 값. 예) reverse_penalty가 크면 후진을 싫어하고, non_straight_penalty가 크면 급커브를 피함.
기본값	패키지에 포함된 모든 플래너(2D, Hybrid-A*, State Lattice)를 5 cm costmap 기준으로 잘 동작하도록 미리 세팅해 둠. 대부분의 경우 건드릴 필요 없음.
왜 조정이 필요할 수도?	State Lattice는 사용자 정의 minimum control set(속도·회전 primitive 모음)을 마음대로 넣을 수 있어서 로봇·환경에 따라 “적절한 패널티 균형”이 달라질 수 있음.
1️⃣ 패널티 종류 & 권장 범위
이름 의미 “reasonable” 범위

cost_penalty	장애물 비용이 높은 셀을 지날 때 패널티	1.7 – 6.0
non_straight_penalty	직선이 아닌 곡선·커브를 선호도	1.0 – 1.3
change_penalty	전진↔후진 방향 전환 횟수 패널티	0.0 – 0.3※ 0.0 초과 필수
reverse_penalty	후진 자체에 대한 패널티	1.3 – 5.0
모든 값은 1.0 이상이어야 한다(change_penalty만 ≥ 0.0).
범위를 벗어나면 경로 품질이 급격히 나빠지거나 CPU 시간이 늘어날 수 있음.

2️⃣ 실전 튜닝 팁
기본값으로 먼저 테스트 → “경로가 너무 지그재그다” “후진을 너무 많이 쓴다” 같은 불만이 생기면 아래 단계로.
한 번에 하나씩 값 조정
예) 후진이 많음 → reverse_penalty +0.5씩 올려 보기
곡선이 불필요하게 많음 → non_straight_penalty를 1.2 → 1.3
State Lattice는 control set을 바꾸면 패널티 밸런스도 변동 → new set마다 다시 짧게 튜닝.
속도 vs. 품질 트레이드오프 기억
패널티가 높아지면 탐색 범위가 줄어 계산 속도는 빨라지지만
과하면 “돌아갈 데가 없어 실패”할 수도 있으니 범위 안에서만 미세 조정!
3️⃣ Hybrid-A* 특별 메모
최근 수식 개선으로 change_penalty = 0.0(OFF) 상태가 대부분의 로봇에서 좋은 결과를 냅니다.
그래도 전·후진 전환이 잦다고 느끼면 0.1, 0.2 정도를 줘서 억제해 보세요.
핵심 요약

기본값은 5 cm costmap에서 이미 “무난”하다.
State Lattice처럼 control set이 달라지면 패널티도 함께 맞춰 준다.
위 권장 범위 안에서 소폭만 조정해도 경로 스타일을 원하는 대로 바꿀 수 있다.


❌ “분명 갈 수 있는 목표인데 경로를 못 찾는다 / 계산 시간이 너무 길다” 문제 해결 가이드
다음 내용을 적용하기 전에 가장 먼저 확인할 것
max_iterations 값 점검
A*가 확장(Expansions)할 수 있는 최대 노드 수입니다.
예) 창고 전체가 1 km²인데 max_iterations: 5 000이면 턱없이 부족합니다.
목표에 못 가거나 시간이 오래 걸리면 max_iterations를 올려 보세요.
끝없는 탐색도 테스트해 보고 싶다면 -1(무제한) 로 두고 실제 얼마나 걸리는지 확인해 볼 수 있습니다.
그래도 문제가 남아 있다면? → 지도(Map) 품질과 footprint 설정을 의심해 봅니다.

1️⃣ SLAM 지도에 생긴 “틈새(gap)”가 함정일 때
현상 원인

목표는 유효해 보이는데 Planner가 경로를 못 찾거나 100만 iteration까지 쓰고 타임아웃	SLAM 과정에서 생긴 아주 좁은 빈틈(gap)·구멍(hole) 때문.틈이 거의 통과 가능해 보여서 휴리스틱이 그쪽으로 끊임없이 시도하지만, footprint 기준으로는 실제로 통과 불가
왜 이런 일이 생길까?
틈 폭이 로봇보다 살짝 작은데, inflation layer가 그만큼 막아주지 못할 정도로 애매할 때
Planner 입장에선 “가장 짧은 경로”로 보이므로 휴리스틱(heuristic) 이 해당 방향으로 노드를 계속 확장
→ 결국 max_iterations 한도까지 붉은색(확장 실패) 노드만 쌓임
→ 무한히 시도하면 언젠가는 통과 경로가 나오지만, 실시간 로봇에겐 불가능한 시간
2️⃣ 해결 방법
Footprint 크기(or padding) 살짝 늘리기
가장 간단한 임시방편
로봇을 실제보다 살짝 크게 취급하면 문제 틈이 “애초에 통과 불가” 로 표시 → 휴리스틱이 다른 길로 즉시 방향 전환
예시 그림 ② : footprint만 키웠는데 < 10 000 iteration, 110 ms 만에 경로 성공
지도(post-processing) 수정
근본 해결책
틈을 벽(occupied) 으로 칠해 완전히 폐쇄
SLAM 후 세밀한 수동 편집이나, map_cleaner 같은 툴 사용
Inflation 설정 키우기
inflation_layer > inflation_radius 또는 cost_scaling_factor를 살짝 올려 좁은 틈을 cost 값으로 자동 메우기
다만 너무 키우면 실제 운행 시 “좁지만 통과 가능한 통로”까지 막힐 수 있으니 주의
3️⃣ 요약 Check-list
체크 항목 권장 조치

max_iterations	큰 맵이라면 최소 수십~수백만 단위로 설정. 테스트용으로 -1도 활용
지도에 작지만 치명적인 구멍	Footprint padding ↑ → 문제 재현 여부 확인
근본적으로 틈 제거	SLAM 지도 후처리로 gap을 occupied 셀로 채우기
inflation layer가 너무 얇음	inflation_radius, cost_scaling_factor 조금씩 ↑ (주의: 과하면 지나칠 공간 봉쇄)
위 단계를 거치면 “분명 갈 수 있는 목표인데 경로 미탐색” 문제와 장시간 계산/타임아웃 문제를 대부분 해결할 수 있습니다.







✏️ 번역 & 상세 풀이
(영어 기술 용어는 그대로 두고, 문장 구조를 우리말로 풀어썼습니다)

원문 ①
As such, it is recommended if you have sparse SLAM maps, gaps or holes in your map, that you lightly post-process them to fill those gaps or increasing your footprint's padding or radius to make these areas invalid. Without it, it might waste expansions on this small corridor that:
A) you dont want your robot actually using
B) probably isnt actually valid and a SLAM artifact and
C) if there's a more open space, you'd rather it use that.

해설

SLAM으로 만든 지도에 빈틈(gap)·구멍(hole)이 드문드문 있는 경우,
지도 후처리(post-processing) 로 그 틈을 막거나,
footprint padding / radius 값을 살짝 키워서 그 공간을 “애초에 통행 불가” 로 표시하는 편이 좋습니다.
그렇지 않으면 플래너가 그런 좁은 통로에 대해 불필요하게 많은 expansion(노드 확장)을 써버립니다. 이유는
A) 실제로는 로봇이 다니길 원하지 않는 좁은 길이고
B) 대개 오차가 큰 SLAM 노이즈일 가능성이 높으며
C) 더 넓고 안전한 통로가 있는데도 휴리스틱이 “최단거리”만 보고 그 틈으로 억지로 파고들기 때문입니다.
원문 ②
One interesting thing to note from the second figure is that you see a number of expansions in open space. This is due to travel / heuristic values being so similar, tuning values of the penalty weights can have a decent impact there. The defaults are set as a good middle ground between large open spaces and confined aisles (environment specific tuning could be done to reduce the number of expansions for a specific map, speeding up the planner). The planner actually runs substantially faster the more confined the areas of search / environments are -- but still plenty fast for even wide open areas!

해설

두 번째 그림(수정 후)에서도 탁 트인 공간에 expansion(빨간 점)이 꽤 퍼져 있는 걸 볼 수 있습니다.
이는 서로 다른 경로들의 travel cost 와 heuristic 값이 거의 비슷해서, 어느 쪽이든 “좋다”는 판단이 쉽지 않기 때문입니다.
이런 경우 penalty weight(cost_penalty, reverse_penalty 등)를 미세 조정하면 효과가 있습니다.
기본값은 “넓은 공터”와 “좁은 통로” 모두에서 적당히 잘 돌도록 설정돼 있지만,
특정 지도·환경에 맞춰 조정하면 확장 수를 줄여 플래너 속도를 더 끌어올릴 수 있습니다.
재미있는 사실: 공간이 좁을수록(선반 사이, 복도 등) 후보가 한정돼서 계획 속도가 더 빨라집니다.
하지만 공터에서도 기본 설정으로 이미 충분히 빠르니 크게 걱정할 필요는 없습니다.
원문 ③
Sometimes visualizing the expansions is very useful to debug potential concerns (why does this goal take longer to compute, why can't I find a path, etc), should you on rare occasion run into an issue. You can enable the publication of the expansions on the /expansions topic for SmacHybrid with the parameter debug_visualizations: True, beware that it should be used only for debug as it increases a lot the CPU usage.

해설

“왜 이 목표는 이렇게 오래 걸리지?” “왜 경로가 안 나오지?” 같은 문제가 생기면,
expansions 시각화를 켜서 플래너가 어디를 얼마나 탐색했는지 직접 보는 것이 큰 도움이 됩니다.
방법
SmacPlannerHybrid 파라미터 debug_visualizations: True →
RViz에서 /expansions 토픽으로 빨간 점(확장 노드)이 뿌려집니다.
단점
CPU 사용량이 크게 늘어나므로 디버그용으로만 잠깐 사용하세요.
📌 요약 체크리스트
문제 상황 추천 조치

SLAM 지도에 가느다란 틈·구멍	지도 편집으로 메우거나 footprint padding을 살짝↑
넓은 공간에서 확장 노드가 너무 많음	cost_penalty, non_straight_penalty 등 penalty weight 미세 튜닝
원인 분석 필요	debug_visualizations: True로 /expansions 시각화 (일시적으로만)
이렇게 하면 불필요한 연산 낭비를 줄이고, 플래너가 더 빠르고 안정적으로 원하는 경로를 찾도록 만들 수 있습니다.









문제 요약
목표점(Goal pose)이 벽에 너무 가깝다
Hybrid-A*가 마지막에 analytic expansion-곡선을 꽉 붙여서 붙이면서 벽 쪽으로 급히 파고듦
Lidar safety(보통 local planner나 recovery layer) 가 “벽과 너무 근접” 판단 → stuck
원하는 모습 : Image 2 처럼 곡선을 더 완만하게(타원형) 돌려 벽에서 여유를 확보
접근 순서
단계           무엇을 조정?            왜 효과가 있나?

① Cost 로 ‘벽 근처 = 비싸다’ 강조	cost_penalty ↑ & use_quadratic_cost_penalty: True	휴리스틱이 “벽보다 중앙이 싸다”는 사실을 더 강하게 반영 → 경로가 자연히 벽에서 멀어짐
② 차가 ‘더 크게 돈다’고 가정	minimum_turning_radius ↑ (0.4 → 0.6~0.8 m)	모션 primitive 자체가 커지므로 마지막 U-턴이 완만해짐
③ 곡선 너무 많이 쓰지 않게	non_straight_penalty ↑ (1.5 → 2.0~2.3)	급한 곡선/지그재그를 덜 선호
④ analytic expansion이 너무 짧게 벽에 붙는 문제 완화	analytic_expansion_max_length ↑ (3.0 → 4.0) 또는 analytic_expansion_ratio ↓ (3.5 → 2.0)	“목표까지 바로 붙이기” 를 조금 멀리서만 시도 → 입구를 먼저 크게 돌고, 안쪽에서 살살 붙음
⑤ Smoother가 더 세게 펴주도록	Smoother 파라미터 w_smooth: 0.3 → 0.5 w_data: 0.2 → 0.15	기존 궤적을 더 과감히 ‘펼쳐’ 부드럽게 만듦
⑥ 벽 자체의 Inflation	(global_costmap) inflation_radius ↑ , cost_scaling_factor ↑	Planner 이전 단계에서 아예 벽 근처를 더 비싸게 만들어 줌
실제로는 ①+② 만 적용해도 대부분 Image 2에 가깝게 변합니다.

제안 Config 스니펫 (핵심 변경만)
GridBased:
  plugin: "nav2_smac_planner::SmacPlannerHybrid"

  # ── ① 비용 기반 튜닝 ───────────────────────────
  cost_penalty: 3.0              # ← 2.0 ▶ 3.0
  use_quadratic_cost_penalty: True

  # ── ② 회전 반경 키우기 ────────────────────────
  minimum_turning_radius: 0.7    # ← 0.4 ▶ 0.7 (테스트 후 0.6~0.8 조정)

  # ── ③ 곡선 패널티 강화 ─────────────────────────
  non_straight_penalty: 2.1      # ← 1.5 ▶ 2.1

  # ── ④ analytic expansion 완화 ────────────────
  analytic_expansion_ratio: 2.0  # ← 3.5 ▶ 2.0
  analytic_expansion_max_length: 4.0

  # ── ⑤ Smoother 강도 ↑ ─────────────────────────
  smoother:
    w_smooth: 0.5     # ← 0.3
    w_data:   0.15    # ← 0.2
    refinement_num: 2
추가로 확인하면 좋은 것들
항목 확인 & 조정 포인트

Inflation Layer	글로벌 costmap의 inflation_radius 를 로봇 반경 + 0.15 m 이상으로, cost_scaling_factor를 5.0 이상으로 두면 벽 근처 cost gradient가 매끄러워져 planner가 더 멀리 돌아갑니다.
Footprint padding	SLAM 노이즈나 아주 얇은 빈틈 때문에 planner가 벽에 바짝 붙는다면, footprint_padding: 0.03~0.05 m 정도만 주어도 경로가 살짝 떨어집니다.
Local Planner 안전 여유	DWB / TEB 등의 xy_goal_tolerance, yaw_goal_tolerance, inflation_layer 의 robot radius vs safety zone 차이도 같이 맞춰야 stuck false-positive를 줄일 수 있습니다.
시각화 디버그	debug_visualizations: True 로 해보고 /expansions 빨간점이 벽에 몰려 있으면 cost / penalty / inflation 을 더 키워야 한다는 증거입니다.
테스트 순서 권장
①+② 만 적용 → RViz로 새 경로 모양 확인
여전히 벽과 가깝다면 ③(패널티) 조정 → 2.3까지 올려보기
마지막 접속부가 짧게 ‘콕’ 붙으면 ④(analytic expansion) 길이·비율 재조정
그래도 안 된다면 Inflation Layer(global costmap) 값 점검
만족하면 Smoother 강도(⑤) 를 살짝 줄여 CPU 절약
위 단계대로 적용하면 Image 2 처럼 목표 근처를 넉넉히 돌아 들어가는 경로를 얻을 수 있고, Lidar safety에 걸려 Stop & Stuck 되는 현상이 크게 줄어듭니다. 추가 정보나 결과 스크린샷이 있으면 또 알려 주세요!

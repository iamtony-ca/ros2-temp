엔지니어의 포트폴리오나 경력 기술서에서는 단순히 '어떤 모델을 썼다'를 나열하는 것보다, **'왜 이 SOTA(State-of-the-Art) 모델들을 조합했는지'**, 그리고 '구현 과정에서 어떤 한계를 발견하고 어떻게 개선(Troubleshooting) 방향을 도출했는지'를 보여주는 것이 훨씬 매력적입니다.

실제 기업의 로보틱스 비전 연구원 포지션에 제출하기 좋은 형태로, 전문적이고 구조화된 기술서를 작성해 드립니다.

---

### 📄 [프로젝트/업무 경험] 고정밀 로봇 Pick and Place를 위한 SOTA 딥러닝 기반 6D Pose Estimation 파이프라인 구축

**1. 프로젝트 개요 및 목표**

* **목적:** 매니퓰레이터의 정밀한 Pick and Place Task 수행을 위한 실세계(Real-world) 기반 타겟 부품의 6D Pose Estimation 시스템 개발.
* **핵심 과제:** 실시간성보다는 극한의 인식 정밀도(mm, degree 오차 최소화) 확보에 초점을 맞추어, CAD 모델이 주어지는 환경(Seen Object)에 최적화된 딥러닝 파이프라인 설계.

**2. 핵심 파이프라인 아키텍처 (기술 스택)**

* **Sensor & Data:** RGB-D 센서(RealSense / ZED 등) 데이터 및 타겟 객체의 3D CAD 모델.
* **Segmentation (관심 영역 추출):** `SAM6D`
* Zero-shot 기반으로 입력 이미지에서 타겟 객체의 2D Mask를 추출하여 포즈 추정을 위한 탐색 공간(Search Space) 축소.


* **Initial Pose Estimation (초기 포즈 추정):** `FoundationPose`
* 대규모 데이터로 사전 학습된 파운데이션 모델을 도입.
* 딥러닝 네트워크 내에서 CAD 템플릿을 렌더링하고 실제 이미지 특징과 정합하는 Render-and-Compare 방식을 통해 초기 6D 포즈를 강건하게 추정.


* **Pose Refinement (포즈 미세 조정):** `SCFlow2`
* Optical Flow 개념을 3D로 확장한 딥러닝 기반 Pose Refiner 적용.
* FoundationPose의 추정 결과를 초기값으로 삼아, RGB와 고정밀 Depth 데이터를 융합해 최종 기하학적 오차를 SOTA 수준으로 최소화.



**3. 트러블슈팅 및 아키텍처 최적화 설계 (Problem Solving)**

* **이슈 발굴:** SAM6D를 활용한 분할(Segmentation) 과정에서 타겟 부품의 특정 각도(Viewpoint) 및 복잡한 배경 하에서 Masking이 깨지며 전체 파이프라인의 추론 실패로 이어지는 병목 현상(Bottleneck) 식별.
* **원인 분석:** 6D Pose 벤치마크(BOP Challenge 등) 결과를 분석하여, FoundationPose + SCFlow2 조합의 탁월한 포즈 추정 능력(Score 75.2)을 100% 발휘하기 위해서는 앞단의 관측수(Segmentation) 역할이 절대적임을 확인. 범용성을 띈 SAM 모델이 실무의 특정 부품(Instance) 인식에서는 오히려 약점이 될 수 있음을 도출.
* **개선 방향 도출 (Next Step):**
* 파이프라인의 앞단을 범용 모델(SAM6D)에서 타겟 특화 Instance Segmentation 모델(예: YOLO-seg)로 교체하는 하이브리드 구조 제안.
* Isaac Sim 등의 3D 시뮬레이터를 활용해 취약한 각도의 합성 데이터(Synthetic Data)를 집중 생성 및 학습(Fine-tuning)시켜 Segmentation의 강건성을 극대화하는 아키텍처로 고도화 기획.



---

**💡 작성 팁:**
이력서에 위 내용을 넣으실 때는, 단순히 기술만 나열하기보다 본인이 "FoundationPose와 SCFlow2의 SOTA 논문 지표를 직접 분석하여 파이프라인의 타당성을 검증했다"는 점을 어필하시면 '연구 개발 역량'을 아주 높게 평가받을 수 있습니다.

혹시 이 기술서에 추가로 포함하고 싶으신 타겟 부품의 종류(예: 반도체 부품, 금속 가공품 등)나 적용할 로봇 팔(매니퓰레이터)의 기종이 있으신가요? 내용을 더 구체화해 드릴 수 있습니다.

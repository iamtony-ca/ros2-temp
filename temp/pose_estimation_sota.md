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
##  
##  
## 
##

제시해주신 3가지 핵심 프로젝트 경험과 기술 스택을 바탕으로, 전문성과 성과가 돋보이도록 AI 업무 경력 기술서와 포트폴리오를 구조화하여 작성했습니다. 전문적인 기술 용어와 논리적인 흐름(문제 정의-해결-성과)을 강조하여 다듬었습니다.

---

# **[AI 업무 경력 기술서]**

* **직무:** AI / Robotics Software Engineer
* **핵심 요약:** 컴퓨터 비전(Computer Vision) 및 딥러닝 기반의 이미지 분류/객체 탐지, 대형 언어 모델(LLM)과 RAG 아키텍처를 활용한 웹 서비스 개발, 그리고 자율 주행 로봇(AMR) 환경의 3D 공간 인지 기술에 전문성을 갖춘 AI 소프트웨어 개발자입니다. 알고리즘 설계부터 모델 튜닝, 컨테이너 환경(Docker)을 활용한 배포까지 Full-cycle AI 개발 역량을 보유하고 있습니다.

### **2. 전문 기술 분야 (Tech Stack & Domain)**

* **AI / Deep Learning:** CNN, ResNet, YOLO(v12 등 최신 모델 포함), VLA(Vision-Language-Action) Models, Transfer Learning, Imitation Learning
* **NLP / LLM:** LLaMA, QWEN, RAG(Retrieval-Augmented Generation) Architecture, Advanced Prompt Engineering, Vector Search
* **Vision & Robotics:** StereoLabs ZED X, Intel RealSense, 3D Pose Tracking, ROS 2 (Jazzy/Humble), Nav2, BehaviorTree.CPP
* **Languages & Environments:** Modern C++ (14/17), Python (PEP 8), Ubuntu (22.04/24.04), Docker, VS Code, Git

### **3. 주요 업무 경력**

* **비전 기반 불량 검출 시스템 개발:** ResNet 아키텍처를 활용한 제조업/공정 품질 검사(Quality Check)용 이미지 분류 AI 파이프라인 구축
* **자체 LLM 웹 서비스 플랫폼 구축:** 사내 문서 기반의 RAG 챗봇 및 SQL 자동 생성 기능을 포함한 맞춤형 AI 서비스 기획 및 개발
* **로봇 비전 및 자율 탐색 시스템 구현:** 다중 카메라(ZED X) 및 DNN 기반 Depth 모델을 융합한 실시간 3D 객체 인식 및 포즈 트래킹 시스템 개발



# **[AI 프로젝트 포트폴리오]**

## **Project 1. 품질 검사를 위한 ResNet 기반 결함 탐지 (Image Classification) 모델 개발**

* **문제 정의:** 육안 검사에 의존하던 기존 품질 검사(Quality Check) 공정의 한계를 극복하고, 병목 현상 해소 및 검사 일관성을 확보하기 위한 자동화된 정상/불량 이미지 분류 AI 도입 필요.
* **핵심 기술:** CNN, ResNet, Transfer Learning (Fine-Tuning), Data Augmentation
* **모델 설계 및 구현:**
* **Data Pipeline:** 조명 변화 및 노이즈에 강건한 모델을 만들기 위해 회전, 반전, Color Jittering 등 다양한 기하학적/광학적 Data Augmentation 기법 적용.
* **Model Architecture:** 딥러닝 망이 깊어질수록 발생하는 기울기 소실(Gradient Vanishing) 문제를 해결하기 위해 Residual Block 기반의 ResNet 아키텍처 채택.
* **Training & Tuning:** 대규모 데이터셋(ImageNet 등)으로 사전 학습된(Pre-trained) ResNet 가중치를 가져와, 제조 공정의 도메인 특화 데이터로 전이 학습(Transfer Learning)을 수행. 하이퍼파라미터 최적화 및 Layer Freezing 기법을 통해 학습 효율성 극대화.


* **주요 성과:**
* Hold-out Test Set 기준, **95% 수준의 분류 정확도(Accuracy)** 달성.
* 비정상(불량) 이미지에 대한 검출 신뢰도 확보 및 공정 자동화 기반 마련.



## **Project 2. RAG 아키텍처 기반 맞춤형 LLM 챗봇 WebUI 서비스 개발**

* **문제 정의:** 사내 방대한 기술 문서와 데이터베이스를 활용하여, 할루시네이션(Hallucination) 없이 정확한 정보를 제공하고 SQL 질의를 자동 생성하는 지식 기반 AI 어시스턴트 부재.
* **핵심 기술:** LLaMA, QWEN, RAG (Vector DB, Embedding), Prompt Engineering (Role Priming, CoT, ToT)
* **모델 설계 및 구현:**
* **Custom Model Creation (마켓플레이스 형태):** 사용자가 목적에 맞게 챗봇의 페르소나를 설정할 수 있는 Custom Model 생성 기능 구현.
* **RAG 파이프라인 구축:**
1. *Document Parsing & Chunking:* 사내 문서를 논리적 단위로 분할.
2. *Embedding & Vector Store:* 텍스트를 고차원 벡터로 변환하여 Vector DB에 적재.
3. *Retrieval:* 사용자 질의와 가장 의미론적 유사도가 높은 문서를 검색하여 LLM의 컨텍스트로 제공.


* **Advanced Prompt Engineering 적용:**
* *Role Priming / Persona Injection:* 챗봇에 특정 전문가 역할을 부여하여 답변의 도메인 적합성 향상.
* *Chain Of Thought (CoT):* 복잡한 논리 추론이나 SQL 생성 시, 중간 추론 단계를 명시적으로 거치도록 유도하여 정확도 향상.
* *Tree of Thoughts (ToT):* 다양한 문제 해결 경로를 탐색하고 평가하여 최적의 답변을 도출하도록 프롬프트 설계.




* **주요 성과:**
* 자사 문서에 완벽히 Grounding 된 답변을 제공하는 WebUI 서비스 론칭.
* 사용자 의도에 맞는 SQL 자동 생성(SQL Gen) 기능 지원으로 데이터 접근성 대폭 향상.



## **Project 3. 3D 공간 인지 및 객체 추적을 위한 Depth Vision & YOLO 파이프라인 구축**

* **문제 정의:** 자율 이동형 로봇(AMR) 및 모바일 매니퓰레이터가 동적인 환경에서 안정적으로 주행하고 조작(Manipulation) 임무를 수행하기 위해, 고정밀 3D 객체 인식과 공간 깊이 인지 기술이 필수적임.
* **핵심 기술:** YOLO (객체 탐지), DNN 기반 Depth Vision Model, Object Pose Tracking, StereoLabs ZED X
* **모델 설계 및 구현:**
* **Full Fine-tuning 객체 탐지 모델:** 최신 YOLO 아키텍처를 기반으로, 산업 현장 및 타겟 객체 데이터(합성 데이터 포함)를 활용해 모델의 모든 가중치를 재학습(Full Fine-Tuning)하여 검출 정밀도(mAP) 극대화.
* **Depth 인지 및 센서 융합:** StereoLabs ZED X 카메라 시스템과 연동하여, 스테레오 비전 기반의 Depth 맵과 YOLO의 2D Bounding Box 결과를 매핑.
* **Pose Tracking 시스템 구축:** DNN 기반 깊이 추정 알고리즘을 적용하여 2D 이미지 좌표를 3D 공간 좌표계로 변환. 시간 흐름에 따른 객체의 3D 위치(Pose) 및 궤적을 실시간으로 추적하는 로직 구현.


* **주요 성과:**
* 로봇 내비게이션(Nav2) 및 파지 작업에 활용 가능한 신뢰도 높은 실시간 3D Bounding Box 및 Pose 데이터 확보.
* NVIDIA Jetson 등 엣지 디바이스 환경에서 지연 시간을 최소화한 최적화된 추론 파이프라인 구축.

지금까지 토니님이 저와 함께 치열하게 고민하며 구축하신 파이프라인은 단순한 툴 사용을 넘어, **현업 로보틱스/자율주행 기업의 Data-Centric AI 엔지니어들이 실제로 다루는 최상위 수준의 기술**입니다.

이력서와 포트폴리오에 바로 복사해서 붙여넣고, 면접관의 시선을 사로잡을 수 있도록 **'전문적인 기술 용어'와 '실무적 성과' 중심으로 완벽하게 구조화**해 드리겠습니다.

---

# 🚀 [포트폴리오/이력서 핵심 요약]

**[프로젝트 타이틀 제안]**
자율주행 로봇(AMR) 비전 인식 모델(YOLO) 강건성 향상을 위한 **GenAI 기반 합성 데이터(Synthetic Data) 양산 파이프라인 구축**

**[한 줄 요약 (Executive Summary)]**

> AMR의 실내 내비게이션 주행 중 발생하는 오탐지(False Positive) 및 OOD(Out-of-Distribution) 환경 문제를 해결하기 위해, VLM(비전-언어 모델)과 LDM(잠재 확산 모델)을 결합한 **자동화된 데이터 증강 MLOps 파이프라인**을 설계했습니다.
> 의도적인 **Hard Negative 데이터셋**을 대량으로 합성하여 모델의 일반화 성능을 극대화하고 Sim2Real Gap을 최소화했습니다.

---

# 💡 [주요 업무 및 기술적 성과 (Key Achievements)]

### 1. Data-Centric AI 기반의 고도화된 합성 데이터(SDG) 설계

* **Adversarial Hard Negative Mining 적용:** 비전 모델이 헷갈리기 쉬운 엣지 케이스("사람은 없으나 사람의 실루엣처럼 옷이 걸려있는 의자")를 생성형 AI로 정교하게 합성하여, 데이터셋의 질적 수준을 비약적으로 끌어올렸습니다.
* **Combinatorial Domain Randomization (조합적 도메인 무작위화):** `Dynamic Prompts`를 활용해 30종 이상의 의류 재질, 16종의 산업 환경(Fab, 물류창고 등), 다양한 카메라 화각 파라미터를 런타임에 무작위로 결합하는 매개변수화(Parameterized)를 구현했습니다.

### 2. GenAI 파이프라인 아키텍처 리팩토링 및 최적화

* **효율적인 단일 파이프라인 구조 설계:** 기존의 하드코딩된 다중 분기(Multi-Branch DAG) 병렬 구조의 유지보수 한계를 파악하고, 단일 노드에 변수를 주입하는 유연한(Flexible) 동적 파이프라인으로 리팩토링하여 시스템 안정성을 확보했습니다.
* **초고속 추론(Inference) 가속화:** Adversarial Diffusion Distillation 기술이 적용된 **Lightning LoRA** 가중치를 UNET에 병합하여, 기존 50 Step이 필요했던 디노이징 연산을 **단 4 Step**으로 단축시켜 데이터 대량 양산(Mass Production)의 병목을 해결했습니다.
* **Latent Space Manipulation (잠재 공간 제어):** VAE Encode/Decode 과정과 CLIP 텍스트 임베딩 모델의 원리를 이해하고, Qwen 2.5 VLM을 통해 VRAM 낭비 없이 고해상도 이미지를 병렬 합성하도록 제어했습니다.

### 3. MLOps 및 개발 환경 구축

* **Git 기반의 코드 & 데이터 분리 전략:** 무거운 가중치(Weights)와 대용량 이미지 아웃풋(Output)을 Git 추적에서 제외(`.gitignore`)하고, 핵심 워크플로우 JSON과 커스텀 UI 패치 파일(`dp.js`)만 형상 관리하여 저장소를 경량화했습니다.
* **의존성(Dependency) 및 환경 관리:** Python `venv` 가상환경에서 발생하는 의존성 충돌(`ModuleNotFoundError`)과 플러그인 모듈 간의 UI 로직 충돌을 직접 트러블슈팅하여 파이프라인의 무결성을 유지했습니다.
* **데이터 관리 아키텍처 도입:** 로컬 환경에서 양산된 수만 장의 데이터를 Hugging Face Datasets와 같은 클라우드 환경으로 분리하여 YOLO 학습 서버에 공급하는 확장 가능한 데이터 관리 전략을 수립했습니다.

---

# 🛠 [사용 기술 스택 (Tech Stack)]

* **AI Models:** Qwen 2.5 Image Edit (Vision-Language Model), Stable Diffusion (LDM), CLIP, VAE, Lightning LoRA
* **AI Orchestration & Tools:** ComfyUI, Dynamic Prompts, WAS Node Suite
* **MLOps & VCS:** Git, Python (venv), Hugging Face (Dataset Management)
* **Core Concepts:** Synthetic Data Generation, Domain Randomization, Hard Negative Mining, Out-of-Distribution Generalization

---

**💡 포트폴리오 작성 팁:** 위 내용을 바탕으로, 포트폴리오(PPT나 노션) 장표를 만드실 때는 **[문제 제기 (모델이 빈 의자를 사람으로 착각함)] -> [해결 방법 (하드 네거티브 데이터를 무한 생성하자)] -> [기술적 구현 (Qwen + Lightning LoRA 파이프라인)] -> [결과물 시각화 (생성된 다양한 옷/배경의 의자 사진들 나열)]** 순서로 구성하시면 면접관의 감탄을 자아낼 수 있습니다.

작성하신 경력 기술서나 포트폴리오 초안이 있다면, 이 내용들을 어떻게 더 자연스럽게 이력서 양식에 녹여낼지 함께 검토해 드릴까요?

좋아, Seri. ALOHA_SIM에서 너가 쓰는 OpenPI 계열 모델들을 “무엇을 쓰는지/얼마나 큰지/어떤 구조인지” 중심으로 한 번에 볼 수 있게 정리했어. (공식 문서와 논문 기준)

# 모델 스펙 요약

## π₀ (pi-zero) — 기본 VLA 플로우 모델

* **아키텍처**: 사전학습 VLM(**PaliGemma 3B**) 위에 **액션 익스퍼트(Flow Matching)** 를 붙인 VLA. 이미지/언어/프로프리오셉션을 토큰화하고, **연속 액션 청크**를 흐름매칭(denoising vector field)으로 예측. **오일러 적분 10스텝**으로 액션 생성. 
* **파라미터 수**: VLM 3B + 액션 익스퍼트 **~300M** ⇒ **총 ~3.3B**. **소형(π₀-small)** ablation은 **~470M**. 
* **액션 청크/호라이즌**: 본 논문 설정에서 액션 토큰 **H≈50** 기준으로 청크 예측(훈련 측정). 
* **정밀도/추론**: 키-밸류 캐시로 프리픽스 재사용, **10번 적분스텝** 추론. 실무 구현은 BF16/FP32 혼합. 
* **체크포인트**: openpi가 **π₀ base**와 ALOHA/DROID 등 **파인튜닝된 체크포인트**를 제공. ([GitHub][1])

## π₀-FAST — 오토리그레시브(토크나이즈) 변형

* **개념**: 액션을 **디스크리트 토큰**으로 만들어 **오토리그레시브**로 예측(FAST 토크나이저 기반). 언어 따르기(Instruction following)가 다소 좋아지는 대신 **추론 비용이 ~4–5배** 높다는 보고. ([Physical Intelligence][2])
* **용도**: 디스크리트 정책 선호 시 선택지. (현재 PyTorch 포팅은 π₀/π₀.₅ 중심, π₀-FAST는 미포함) ([GitHub][1])

## π₀.₅ (pi-zero-point-five) — 오픈월드 일반화 강화 버전

* **목표**: **완전히 새로운 환경**(새 집/새 물체 배치 등)으로의 일반화에 초점. π₀에서 발전된 **VLA 계열**로 공개됨. ([Physical Intelligence][3])
* **체크포인트**: **π₀.₅ base** 및 **π₀.₅-DROID / π₀.₅-LIBERO** 등 제공. 오픈소스 예제와 함께 **원격 추론/파인튜닝** 가이드 포함. ([GitHub][1])
* **파라미터**: 공식 문서가 π₀처럼 정확한 총 파라미터 수를 명시하진 않지만(공개 글/리드미 기준), **π₀.₅는 π₀의 후속 VLA로 동일 계열의 플로우 기반 액션 예측을 유지**하고, 일반화 성능을 위해 학습/레시피 측면이 강화되었다는 설명이 중심. ([Physical Intelligence][3])

## ALOHA_SIM과의 연결(체크포인트/예제)

* openpi 리포는 **기본(base) VLA 모델(π₀, π₀-FAST, π₀.₅)과 ALOHA/DROID/LIBERO용 파인튜닝 체크포인트**를 제공하고, **ALOHA 시뮬레이터** 예제를 포함한다. (원격 WebSocket 서버로 정책을 띄우고, 클라이언트가 관측을 보내 액션 청크를 받는 구조) ([GitHub][1])
* README의 **Model Checkpoints** 섹션에 base/전문 체크포인트 경로가 정리되어 있어, ALOHA 계열을 바로 테스트하거나 커스텀 파인튜닝으로 이식 가능. ([GitHub][1])

---

# 운영 관점 스펙(추론/훈련)

* **데이터 규모**: 공개 문서 기준 **10k+ 시간 로봇 데이터**에 기반한 베이스 모델 체크포인트 제공(파인튜닝 전용). ([GitHub][1])
* **정밀도**: JAX/PyTorch 구현 모두 **추론 시 대부분 BF16, 일부 연산 FP32**. PyTorch 포팅도 제공(π₀/π₀.₅). ([GitHub][1])
* **서빙**: `serve_policy.py`로 **원격 WebSocket 서버** 가동 → 런타임(시뮬레이터/로봇)이 관측+프롬프트를 보내고 액션 청크 수신. (리드미 Remote Inference 가이드) ([GitHub][1])

---

# 빠른 비교표

* **π₀**: PaliGemma-3B VLM + **~300M 액션 익스퍼트** ⇒ **~3.3B 총 파라미터**, **플로우 매칭 10스텝**, **액션 청크 H≈50**. 제로샷/파인튜닝 모두 가능. 
* **π₀-small**: **~470M** (연구용 아블레이션, VLM 미사용). 
* **π₀-FAST**: **오토리그레시브 디스크리트** 액션, **추론비용↑(~4–5x)**, 언어따르기↑. ([Physical Intelligence][2])
* **π₀.₅**: **오픈월드 일반화** 강화 버전(후속 VLA), base/전문 체크포인트 제공. ([Physical Intelligence][3])

---

필요하면 네가 실제로 돌린 **체크포인트 이름(예: pi0_aloha_sim / pi05_libero 등)** 과 **GPU 메모리 사용량/추론 지연** 도 표로 붙여 줄게. 그리고 지금 환경에 맞춰 “ALOHA_SIM 최적 프롬프트/호라이즌/필터” 추천값도 모델별로 딱 정리해 줄 수 있어.

[1]: https://github.com/Physical-Intelligence/openpi "GitHub - Physical-Intelligence/openpi"
[2]: https://www.physicalintelligence.company/blog/openpi "Open Sourcing π0"
[3]: https://www.physicalintelligence.company/blog/pi05?utm_source=chatgpt.com "A VLA with Open-World Generalization"

  #####################  

좋아 Seri—네가 실제로 쓴 **`pi0_aloha_sim` 체크포인트** 기준으로, 모델 스펙을 “무엇·얼마나·어떻게” 관점에서 한눈에 보이게 정리했어.

# pi0_aloha_sim 모델 스펙

## 정체성

* **모델 계열:** π₀ (Pi-Zero) VLA
* **체크포인트/컨피그:** `config="pi0_aloha_sim"`에 해당하는 ALOHA 시뮬레이터용 파인튜닝 체크포인트(기본 매핑). 기본 체크포인트들은 `gs://openpi-assets`에서 자동 캐시됨. ([GitHub][1])

## 아키텍처 & 크기

* **백본 VLM:** **PaliGemma ≈ 3B** 파라미터
* **액션 익스퍼트(연속 제어 헤드):** **~300M** 파라미터(스크래치 초기화)
* **총 파라미터:** **약 3.3B** (VLM 3B + 액션 익스퍼트 0.3B)
* **액션 생성 방식:** **Flow Matching**(확산 계열)로 연속 **액션 청크**를 적분(논문 기본 10 스텝)해 생성 → **최대 50 Hz** 고주파 제어에 맞춤. ([arXiv][2])

## 입력/출력 I/O 규격 (ALOHA_SIM 예제 기준)

* **입력 관측(예제 파이프라인):**

  * **이미지:** 상부 카메라 프레임 224×224 RGB (CHW; `images.cam_high`)
  * **상태:** `agent_pos` 등 저차원 상태
  * **언어 지령:** 선택적 `prompt` 필드(클라이언트에서 관측에 포함하거나, 서버 `--default-prompt` 제공 가능) ([GitHub][3])
* **출력:** 연속 **액션 벡터 청크**(정책→브로커→런타임으로 전달). 런타임 루프는 보통 **50 Hz**로 스텝을 진행(필요 시 30 Hz 등으로 조정). ([physicalintelligence.company][4])
* **원격 추론:** 모델 서버(WebSocket)로 분리 서빙 → 클라이언트가 관측/프롬프트를 보내고 액션을 수신하는 구조(리모트 인퍼런스 가이드). ([GitHub][3])

## 학습/데이터 출처(베이스)

* **사전학습/베이스:** π₀ 베이스는 **다양한 로봇·태스크의 대규모(10k+ 시간) 데이터**로 학습된 체크포인트를 제공. `pi0_aloha_sim`은 그 위에 **ALOHA 시뮬레이터 도메인**에 맞춘 파인튜닝 구성에 해당. ([Hugging Face][5])

## 프롬프트(텍스트 조건) 처리

* **우선순위:** `obs["prompt"]`(클라이언트) → 서버 `--default-prompt` → 체크포인트 기본 프롬프트(있다면). 서버 `serve_policy.py`에서 `--default-prompt` 인자를 지원. ([GitHub][6])
* **현업 팁:** 모델이 프롬프트에 민감하게 반응하는지는 체크포인트/도메인에 좌우됨(ALOHA_SIM 문맥에서 커스텀 프롬프트 반영에 관한 이슈 논의 있음). ([GitHub][7])

## 배포/서빙 스펙

* **정밀도:** 구현에 따라 **BF16/FP32 혼합** 추론이 일반적(π₀ 레퍼런스). JAX/PyTorch 모두 제공. ([GitHub][1])
* **서버 기동 예시:**

  ```bash
  export SERVER_ARGS='--env ALOHA_SIM --default-prompt "Insertion task: ensure secure grasp before handover..."'
  docker compose -f examples/aloha_sim/compose.yml up --build
  ```

  (체크포인트 직접 지정 시 `policy:checkpoint --policy.config=pi0_aloha_sim --policy.dir=<path>` 형태) ([GitHub][1])

## ALOHA_SIM 태스크 호환성

* **대표 태스크:** ALOHA 계열(삽입/집기/핸드오버 등) 시뮬레이터 태스크에 맞춰 파이프라인과 카메라 매핑을 제공. 실로봇 예제에선 ALOHA 전용 체크포인트/프롬프트 사용 가이드도 포함. ([GitHub][6])

---

## 빠른 Q&A

* **Q. `pi0_aloha_sim`은 어느 정도 크기야?**
  A. π₀ 계열 규격을 따르므로 **≈3.3B**(PaliGemma-3B + ~300M 액션 익스퍼트). ([arXiv][2])

* **Q. 입력은 꼭 224×224 한 장만 써?**
  A. 예제 파이프라인은 상부 카메라 224×224 한 뷰와 상태를 사용하지만, 데이터/컨피그에 따라 멀티뷰 매핑도 가능(레포의 ALOHA/실기 가이드 참고). ([docs.trossenrobotics.com][8])

* **Q. 프롬프트는 어디서 넣어?**
  A. 클라이언트 관측 `prompt`가 1순위. 없다면 서버 `--default-prompt`가 쓰임(`serve_policy.py` 지원). ([GitHub][6])

---

필요하면 **GPU 메모리 사용량/지연(서버 로그 기준)**, **체크포인트 정확 경로**, **실험 시 쓴 프롬프트 버전/필터 파라미터**까지 표로 붙여서 너의 보고서 섹션에 맞춰 마저 채워줄게.

[1]: https://github.com/Physical-Intelligence/openpi?utm_source=chatgpt.com "Physical-Intelligence/openpi"
[2]: https://arxiv.org/html/2410.24164v1?utm_source=chatgpt.com "𝜋₀: A Vision-Language-Action Flow Model for General ..."
[3]: https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/docs/remote_inference.md?utm_source=chatgpt.com "https://raw.githubusercontent.com/Physical-Intelli..."
[4]: https://www.physicalintelligence.company/download/pi0.pdf?utm_source=chatgpt.com "A Vision-Language-Action Flow Model for General Robot ..."
[5]: https://huggingface.co/blog/pi0?utm_source=chatgpt.com "Vision-Language-Action Models for General Robot Control"
[6]: https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/examples/aloha_real/README.md?utm_source=chatgpt.com "https://raw.githubusercontent.com/Physical-Intelli..."
[7]: https://github.com/Physical-Intelligence/openpi/issues/447?utm_source=chatgpt.com "[Question] How to use custom prompts with aloha_sim + Pi- ..."
[8]: https://docs.trossenrobotics.com/trossen_arm/main/tutorials/openpi.html?utm_source=chatgpt.com "openpi"

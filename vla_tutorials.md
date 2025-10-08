ref: https://kimelab.notion.site/KIRIA-25-08-20-21-203875de525f807eb728eb98e76e2382  
  

# **[KIRIA] (25.08.20~21) 비전-언어-액션 모델 기반 로봇 매니퓰레이션**

# 교육 내용

**장소:** 

**일시:** 

**교육목표:**

- 트랜스포머 및 디퓨전 모델 기초 및 실습
- 비전-언어-액션 모델 (OpenVLA) 세팅 및 실습
- LIBERO 벤치마크 기반 OpenVLA 구동 실습

![image.png](attachment:4f80d082-a17a-42b3-9ab3-2a5d5f4a0dc1:image.png)

---

# 강의자료

# 1. 개발환경 구축 (PyTorch)

## Python 및 개발 환경 구축

[Miniconda 설치 가이드 (라이선스 문제 없이 Jupyter Notebook 사용하기!) | 코드잇](https://www.codeit.kr/tutorials/150/miniconda-guide)

## **Installation Guide — Models**

```bash
# Create and activate conda environment
conda create -n ws_mdl python=3.10 -y
conda activate ws_mdl 

# Install PyTorch
# https://pytorch.org/get-started/locally/
conda install pytorch torchvision torchaudio torchtext torchmetrics pytorch-cuda=12.4 tensorboard spacy datasets tokenizers matplotlib ipykernel ipywidgets -c pytorch -c nvidia -y

git clone https://github.com/knowledge-intelligence/KIMe-VLA-Tutorial.git
```

## **Installation Guide — OpenVLA**

- **환경 구성:** PyTorch 2.2.0, torchvision 0.17.0, transformers 4.40.1, tokenizers 0.19.1, timm 0.9.10, and flash-attn 2.5.5

### Set-up: Conda Env

```bash
# Create and activate conda environment
conda create -n openvla python=3.10 -y
conda activate openvla

# Install PyTorch.
# https://pytorch.org/get-started/locally/

# CUDA 12.1 (Conda)
conda install pytorch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 pytorch-cuda=12.1 -c pytorch -c nvidia -y 
# OR
# CUDA 12.1 (Pip) -Optional
pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu121 -q
```

### Set-up: Openvla Repo

```bash
# Clone and install the openvla repo
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .
# openvla pip 설치이후 Pytorch CPU 버전으로 downgrade 현상 체크
# openvla pip 설치이후 CUDA 12.1 (Conda) 설치 추천
conda install nvidia/label/cuda-12.1.0::cuda-nvcc
```

### Set-up(for Ubuntu): Ninja & Flash Attention 2

```bash
# Install Flash Attention 2 for training (https://github.com/Dao-AILab/flash-attention)
#   =>> If you run into difficulty, try `pip cache remove flash_attn` first
pip install packaging ninja
# ninja --version; echo $?  # Verify Ninja --> should return exit code "0"
# Linux GCC 필수 (윈도우에서는 MSVC 필요) 
pip install "flash-attn==2.5.5" --no-build-isolation
```

## **Installation Guide — Gr00t**

1. Git Repo 복사

```bash
git clone https://github.com/NVIDIA/Isaac-GR00T
cd Isaac-GR00T
```

1. Conda 환경 구성 (Python 3.10)
    
    > ⚠️CUDA 12.4 버전 필수 (flash-attn 때문에)
    > 

```bash
conda create -n gr00t python=3.10
conda activate gr00t
pip install --upgrade setuptools
pip install -e .[base]
conda install nvidia/label/cuda-12.4.0::cuda-nvcc
pip install --no-build-isolation flash-attn==2.7.1.post4
```

# 2. Autoregressive (Transformer) 모델

[Autoregressive (Transformer) 모델 기초 이론](https://www.notion.so/Autoregressive-Transformer-24f875de525f8190a6d4ed32f8b8ffe1?pvs=21)

[Transformer — Python 코드 설명 (tformer > model.py)](https://www.notion.so/Transformer-Python-tformer-model-py-24f875de525f8138a732ef6c31d791f9?pvs=21)

[Transformer — Python 코드 설명 (Training)](https://www.notion.so/Transformer-Python-Training-24f875de525f814c9381eb3fb81864f4?pvs=21)

[Transformer — Python 코드 기반 실습 (훈련) Check Point](https://www.notion.so/Transformer-Python-Check-Point-24f875de525f81d1b253cd9d721d2d82?pvs=21)

# 3. Diffusion 모델

[Diffusion 모델 기초 이론](https://www.notion.so/Diffusion-24f875de525f81839591feaa1fe7a392?pvs=21)

[Diffusion — Python 코드 기반 실습 (Sprite & MNIST)](https://www.notion.so/Diffusion-Python-Sprite-MNIST-24f875de525f8137888fe7f6ccf5f78e?pvs=21)

# 4. OpenVLA & LIBERO 소개 및 개발환경 구축

[OpenVLA 및 LIBERO 환경 구축](https://www.notion.so/OpenVLA-LIBERO-24f875de525f813984b9f84b75e5d5fb?pvs=21)

[OpenVLA 소개](https://www.notion.so/OpenVLA-24f875de525f81abb3f7d05a605dd86f?pvs=21)

[LIBERO 소개](https://www.notion.so/LIBERO-24f875de525f81678c1ac06a756f4e45?pvs=21)

# 5. OpenVLA 및 LIBERO 실습

[FastAPI 구축 실습](https://www.notion.so/FastAPI-24f875de525f81eaa068d001fa8aecae?pvs=21)

[LIBERO+OpenVLA Jupyter 실습](https://www.notion.so/LIBERO-OpenVLA-Jupyter-24f875de525f8182bbd9d8aee7ddbb08?pvs=21)

# 6.  GR00T N1.5 구동 실습

[GR00T N1.5 Tutorial](https://www.notion.so/GR00T-N1-5-Tutorial-24f875de525f817d92f2f5d42a459434?pvs=21)
#####################################  

# Autoregressive (Transformer) 모델 기초 이론

# **트랜스포머 개요**

Recurrent Neural Networks (RNNs)

- Long Short-Term Memory (LSTM) networks
- Gated Recurrent Units (GRUs)

![image.png](attachment:61ad044d-5033-4fb9-8473-33a6b3fc3bba:image.png)

**RNN의 문제점**

- **긴 시퀀스의 느린 계산**: 각 토큰을 순차적으로 처리하는 for-loop로 인해 시간이 오래 걸림.
- **기울기 소실/폭발**: 손실 함수의 미분을 체인 룰로 계산 시, 숨겨진 층이 많아질수록 기울기가 매우 작거나 커져 CPU/GPU의 정밀도 한계로 인해 모델 파라미터 업데이트가 비효율적임.
- **초기 정보 접근 어려움**: 긴 의존성 체인으로 인해 초기 토큰의 숨겨진 상태 영향이 시퀀스 후반부에서 약화되어 장거리 의존성 유지가 어려움.

**트랜스포머 — 인코더**와 **디코더** 블록으로 **해결**

- **Transformer 모델 키 포인트**
    - Embedding
    - Encoding
    - Attention
    - Self-Attention
    - Masked
    - Encoder-Decoder
    - Multi-Head

![image.png](attachment:1a63e6eb-294d-441a-96a5-2b3884e4cd16:image.png)

# **Encoder**

![image.png](attachment:5debec99-728f-454c-a0d7-26ae3c85fc72:dff6e8cf-f172-4eb4-956b-5ba53e15786d.png)

## **Input Embeddings**

![image.png](attachment:2cf74eb7-f0c9-472c-9ce7-9c6bcb2b5abc:image.png)

**트랜스포머 인코더의 입력 처리 과정**

- **입력 문장**: 예: “The quick brown fox jumps over” (6단어 문장).
- **토큰화**: 문장을 개별 토큰으로 나눔.
- **입력 ID 생성**: 각 토큰을 어휘 사전 내 위치를 나타내는 숫자로 매핑.
- **임베딩 변환**: 입력 ID를 512차원 벡터(임베딩)로 변환.
- **임베딩 학습**: 임베딩은 고정되지 않고, 모델이 단어 의미를 학습하며 훈련 과정에서 조정됨.
- **입력 ID 고정**: 입력 ID는 훈련 중에도 변하지 않음.

## **Positional Encoding**

![image.png](attachment:5f775783-5808-42a4-9bd1-589be17ff7fc:image.png)

**위치 인코딩(Position Encoding)의 역할**

- **단어 임베딩의 한계**: 단어의 의미는 전달하지만, 문장에서 단어의 위치 정보는 제공하지 않음.
- **위치 정보의 필요성**: 단어의 문장 내 위치를 알려주어, 가까운 단어는 더 관련 있고 먼 단어는 덜 관련 있음을 모델이 이해하도록 함.
- **기능**: 각 단어의 위치에 대한 공간 정보를 제공하여 문장 구조의 패턴 인식을 도움.

![image.png](attachment:0d93b3f0-3f6d-400a-b205-96ff03ca1ef9:image.png)

**트랜스포머의 위치 인코딩 계산**

- **두 가지 공식 사용**: 원래 트랜스포머 논문에 따라, 짝수 위치와 홀수 위치에 각각 다른 공식을 적용.
- **한 번 계산, 재사용**: 위치 인코딩은 한 번만 계산되며, 훈련과 추론 시 모든 문장에 재사용됨.

## **Multi-Head Attention**

![image.png](attachment:1d1a9a8b-f1e2-4906-afb4-f574e8f4b404:image.png)

**셀프 어텐션(Self-Attention) 개요**

- **정의**: 트랜스포머 이전에 존재했던 메커니즘으로, 트랜스포머에서 멀티헤드 어텐션으로 발전.
- **기능**: 단어 간 관계를 학습해 모델이 문맥을 이해하도록 돕는다.
- **입력 임베딩**: 단어의 의미를 포착.
- **위치 인코딩**: 문장에서 각 단어의 위치 정보를 제공.
- **입력 행렬 사용**: 입력 행렬(6,512)을 Q(쿼리), K(키), V(값)으로 3번 사용.
- **계산 과정**:
    1. Q(6,512)와 Kᵀ(512,6)를 곱함.
    2. 결과에 √dₖ로 나눔.
    3. 소프트맥스(softmax) 적용.
- **값(`score`)의 의미**: 단어 간 관계의 강도를 나타내는 점수.
- **트랜스포머 학습**: 단어 간 상호작용의 강도를 점수로 표현해 이해.

![image.png](attachment:2b25fbb1-6653-4da8-8d81-d56c01845ca1:image.png)

**트랜스포머의 셀프 어텐션 메커니즘 요약**

- **어텐션 매트릭스 생성**:
    - V 행렬과 곱하여 어텐션 매트릭스를 생성.
    - 어텐션 매트릭스의 차원은 초기 행렬과 동일.
    - 결과 임베딩은 단어의 의미, 위치, 그리고 다른 단어와의 관계를 포함.
- **셀프 어텐션의 특징**:
    - **순열 불변성**: 입력 순서에 관계없이 동일한 결과를 생성.
    - **대각선 값 강조**: 대각선 방향의 값이 일반적으로 가장 높음.
- **특정 위치 상호작용 제어**:
    - 소프트맥스 적용 전, 원치 않는 위치의 값을 -∞로 설정 가능.
    - 이를 통해 모델이 특정 위치 간 상호작용을 학습하지 않도록 조정.

![image.png](attachment:6eff2eaa-49ef-4766-afa1-49d37f4331cd:image.png)

**멀티-헤드 어텐션의 동작 과정**

- **입력 임베딩 처리**:
    - 입력 임베딩(seq, d_model)을 복사해 Q, K, V 3개 행렬 생성.
    - 각 행렬에 가중치 행렬(Wꟴ, Wᴷ, Wⱽ, 크기: d_model, d_model)을 곱해 Q’, K’, V’(seq, d_model) 생성.
- **헤드 분할**:
    - Q’, K’, V’를 d_model 차원에서 h(헤드 수, 여기서는 4)로 나눔.
    - dₖ = d_model/h로 각 헤드의 차원 계산(dₖ = dᵥ).
- **어텐션 계산**:
    - 각 헤드에서 논문의 어텐션 공식으로 작은 행렬(head1, head2, head3, head4, 크기: seq, dᵥ) 계산.
- **헤드 병합**:
    - 각 헤드의 결과를 dᵥ 차원에서 연결(concatenate).
    - 연결된 행렬에 가중치 행렬(h*dᵥ, d_model)을 곱해 최종 출력(seq, d_model) 생성.

**멀티-헤드 어텐션의 역할**

- **분할 계산**: Q’, K’, V’를 직접 계산하지 않고, 여러 헤드로 나누어 작은 행렬로 어텐션 계산.
- **다양한 관점 학습**: 각 헤드는 단어의 다른 측면(예: 명사, 동사, 부사 등)을 학습.
    - 예: head1은 단어를 명사로, head2는 동사로 해석하며 관계 학습.

**장점**: 단어의 다양한 맥락을 동시에 파악해 더 풍부한 표현 학습.

## **ADD & Norm (Layer Normalization)**

![image.png](attachment:466845ac-8b70-49ac-9f5c-a9500f0184c6:image.png)

**배치 정규화(Batch Normalization) 요약**

- **입력**: n개의 아이템 배치, 각 아이템은 임베딩과 같은 특징(feature)을 가짐.
- **계산**: 각 아이템의 특징에 대해 독립적으로 평균과 분산을 계산.
- **정규화**: 각 값을 다음 식으로 변환:
    - 곱셈 파라미터(𝛾 또는 𝜶)로 스케일링.
    - 덧셈 파라미터(𝛽)로 이동.
- **수치 안정성**: 분모가 0에 가까워지지 않도록 작은 값(𝜖)을 추가.

# **Decoder**

![image.png](attachment:6652250a-202f-4c45-aa9b-ee7655386c38:d24733bb-5581-4346-b46c-0231f4880d7a.png)

## **Masked Multi-Head Attention**

![image.png](attachment:42b48f28-eedf-40dc-9eab-576412773ac9:image.png)

**트랜스포머의 인과성 보장 및 멀티헤드 어텐션 요약**

- **인과성 보장**
    - 모델이 현재 위치의 출력이 이전 위치의 단어에만 의존하도록 함.
    - 미래 단어를 보지 못하게 **마스킹** 적용.
    - 멀티헤드 어텐션에서 주대각선 위 값들을 **음의 무한대**로 대체 후 소프트맥스 적용.
- **인코더와 디코더의 상호작용**
    - 인코더 출력: **키(Key)**와 **값(Value)** 생성.
    - 디코더의 멀티헤드 어텐션:
        - **크로스 셀프 어텐션** 사용.
        - 키와 값은 인코더에서, 쿼리 행렬은 디코더 입력(마스크된 멀티헤드 어텐션 처리 후)에서 생성.

## **Feed Forward**

**피드포워드 블록의 동작 요약**

- **선형 변환 1**: 입력 텐서(형상: batch_size, seq, d_model)를 완전 연결 선형 층에 통과시켜 더 큰 차원의 텐서(형상: batch_size, seq, d_ff)로 변환.
- **ReLU 활성화**: 첫 번째 선형 층의 출력을 ReLU 함수에 요소별로 적용해 비선형성을 추가.
- **선형 변환 2**: ReLU 출력 텐서를 또 다른 선형 층에 통과시켜 원래 차원(batch_size, seq, d_model)으로 되돌림.
- **특징**: 피드포워드 블록은 복잡한 비선형 변환을 학습하며, 시퀀스의 각 위치에 독립적으로 적용되어 병렬 처리 가능하고 효율적임.

# **Training**

![image.png](attachment:6874c962-3bf9-4ed5-9cb8-6e95ff246dab:image.png)

**영어-프랑스어 번역을 위한 트랜스포머 동작 과정**

- **입력 준비**
    - 영어 문장(“I love you so much”)에 시작 토큰(SOS)과 종료 토큰(EOS)을 추가.
    - 문장을 입력 임베딩으로 변환 후, 위치 인코딩 추가.
- **인코더 처리**
    - 입력을 인코더에 전달.
    - 출력: (시퀀스 길이, d_model) 크기의 행렬.
    - 각 단어의 의미, 위치, 단어 간 관계를 포함한 임베딩 생성.
- **디코더 처리**
    - 시작 토큰(SOS)을 디코더에 입력.
    - 출력(오른쪽으로 이동)을 임베딩으로 변환, 위치 인코딩 추가.
    - 마스크된 멀티-헤드 어텐션에 전달.
    - 인코더 출력(쿼리와 키) + 마스크된 어텐션 출력(값)을 디코더의 멀티-헤드 어텐션에 입력.
    - 출력: (시퀀스 길이, d_model) 크기의 행렬.
- **출력 생성**
    - 디코더 출력을 선형 층으로 (시퀀스 길이, vocab_size)로 변환.
    - 소프트맥스 함수로 각 토큰의 확률 계산.
    - 예측된 프랑스어 문장(“je t’aime tellement”) 생성.
- **학습**
    - 모델 출력과 목표 프랑스어 문장을 비교해 손실 계산.
    - 역전파로 모델 가중치 업데이트.
    - 전체 과정은 단일 시간 단계에서 수행.

# **Inference**

![image.png](attachment:da876da5-c058-454d-b62b-c71b2b9b3963:image.png)

**트랜스포머 번역 과정 요약**

- **인코더 동작**
    - 입력: 문장의 시작(SOS)과 끝(EOS) 토큰을 포함한 영어 문장.
    - 출력: 인코더는 출력 행렬을 생성.
    - 특징: 동일한 영어 문장이므로 매 타임스텝에서 재계산 불필요.
- **디코더 동작 (타임스텝 1)**
    - 입력: SOS 토큰만 디코더에 입력.
    - 과정: 디코더 출력 → 선형 층 → 소프트맥스 함수 적용 → 가장 높은 확률의 어휘 토큰 선택.
    - 결과: 번역된 문장의 첫 번째 토큰 생성.
- **디코더 동작 (타임스텝 2 이후)**
    - 입력: 이전 타임스텝에서 예측된 토큰을 디코더 입력에 추가.
    - 과정: 디코더에 입력하여 다음 토큰 생성, EOS 토큰이 나올 때까지 반복.
- **디코딩 전략**
    - **그리디 서치**: 각 단계에서 가장 높은 소프트맥스 확률의 단어를 선택.
        - 단점: 최적의 결과를 보장하지 않음.
    - **빔 서치**: 각 단계에서 상위 B개의 단어를 고려, 가능한 다음 단어 조합을 평가하여 상위 B개의 시퀀스 유지.
        - 장점: 여러 시퀀스를 동시에 탐색해 더 나은 결과 생성.

# 참고자료

[Mastering Transformer: Detailed Insights into Each Block](https://medium.com/@sayedebad.777/mastering-transformer-detailed-insights-into-each-block-and-their-math-4221c6ee0076)

##################################################  

# Transformer — Python 코드 설명 (tformer > model.py)

![image.png](attachment:1a63e6eb-294d-441a-96a5-2b3884e4cd16:image.png)

# **Input Embeddings**

```python
class InputEmbeddings(nn.Module):
    def __init__(self, d_model: int, vocab_size: int):
        super().__init__()
        self.d_model = d_model
        self.vocab_size = vocab_size
        self.embedding = nn.Embedding(vocab_size, d_model)

    def forward(self, x):
        return self.embedding(x) * math.sqrt(self.d_model)
```

**입력 임베딩 생성 과정**

- **문장 → 입력 ID 변환**: 문장을 단어 사전 내 각 단어의 위치를 나타내는 숫자(입력 ID)로 변환.
- **입력 ID → 임베딩 매핑**: 각 입력 ID를 512차원 벡터인 임베딩으로 변환.

### 🔹 기본 개념

`InputEmbeddings` 클래스는 PyTorch의 `nn.Module`을 상속받은 클래스입니다.

(※ `nn.Module`은 모든 신경망 모듈의 기본 클래스입니다.)

---

### 🔸 생성자 (`__init__` 메서드)

### 📥 입력 파라미터

1. **`d_model`**: 임베딩 벡터의 차원 (예: 512, 768 등)
2. **`vocab_size`**: 단어 집합(어휘)의 크기 (고유한 토큰 개수)

### 🧱 내부 속성

- `self.d_model`: 임베딩 차원을 저장
- `self.vocab_size`: 어휘 크기를 저장
- `self.embedding`: `nn.Embedding`을 이용해 임베딩 레이어 초기화
    
    → 각 토큰을 `d_model` 차원의 벡터로 변환해줌
    

---

### 🔸 `forward` 메서드 (순전파)

### 📥 입력

- **`x`**: 토큰 인덱스(예: 문장을 토큰화한 후 얻은 ID들)

### ⚙️ 동작

- `self.embedding(x)`: 입력된 토큰 인덱스를 임베딩 벡터로 변환
    
    → 출력 형태: `(배치 크기, 시퀀스 길이, d_model)`
    
- `math.sqrt(self.d_model)`: 임베딩 벡터에 **√d_model**을 곱해줌
    
    → 학습 시 **gradient 안정화**를 위해 사용되는 트랜스포머의 일반적인 기법
    
    → 위치 인코딩 등 다른 벡터와 더할 때 **분산을 일정하게 유지**하려는 목적
    

### ✅ 핵심 요약

- 입력 토큰 ID를 고차원 임베딩 벡터로 변환
- 변환된 벡터는 √d_model로 스케일링됨
- 트랜스포머 모델에서 입력값으로 사용됨

# **Positional Encoding**

```python
class PositionalEncoding(nn.Module):
    def __init__(self, d_model: int, seq: int, dropout: float) -> None:
        super().__init__()
        self.d_model = d_model
        self.seq = seq
        self.dropout = nn.Dropout(dropout)

        # Create a matrix of shape (seq, d_model)
        pe = torch.zeros(seq, d_model)

        # Create a vector of shape (seq)
        position = torch.arange(0, seq, dtype=torch.float).unsqueeze(1) # (seq, 1)

        # Create a vector of shape (d_model)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-math.log(10000.0) / d_model)) # (d_model / 2)

        # Apply sine to even indices
        pe[:, 0::2] = torch.sin(position * div_term) # sin(position * (10000 ** (2i / d_model))

        # Apply cosine to odd indices
        pe[:, 1::2] = torch.cos(position * div_term) # cos(position * (10000 ** (2i / d_model))

        # Add a batch dimension to the positional encoding
        pe = pe.unsqueeze(0) # (1, seq, d_model)

        # Register the positional encoding as a buffer
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + (self.pe[:, :x.shape[1], :]).requires_grad_(False) # (batch, seq, d_model)
        return self.dropout(x)
```

**위치 인코딩(Position Encoding)의 역할과 구현**

- **목적**: 문장 내 각 단어의 위치 정보를 모델에 전달.
- **방법**: 단어 임베딩과 동일한 크기의 벡터를 추가해 위치 정보 표현.
    - **벡터 생성**: 트랜스포머 논문의 두 공식 사용.
        - 짝수 위치: 첫 번째 공식 적용.
        - 홀수 위치: 두 번째 공식 적용.
    - **벡터 크기**: 예: 단어 3개일 때, 최대 시퀀스 길이까지 각 위치에 대해 512차원 벡터 생성.
- **수치 안정성**: 로그 공간(log space) 사용 (e^{ln(n)} = n).

### 🔹 기본 개념

이 클래스는 **입력 임베딩에 위치 정보를 더해주는 역할**을 합니다.

트랜스포머는 순서 정보가 없기 때문에, 이 위치 인코딩이 필수입니다.

---

### 🔸 생성자 (`__init__` 메서드)

### 📥 입력 파라미터

1. **`d_model`**: 임베딩 벡터의 차원
2. **`seq`**: 입력 시퀀스의 최대 길이 (예: 문장의 최대 토큰 수)
3. **`dropout`**: 드롭아웃 비율 (과적합 방지)

### 🧱 내부 속성

- `self.d_model`: 임베딩 차원을 저장
- `self.seq`: 최대 시퀀스 길이 저장
- `self.dropout`: 지정된 비율로 드롭아웃 레이어 생성

---

### 🔸 위치 인코딩 행렬 생성

### 📐 행렬 구성

- `pe`: `(seq, d_model)` 크기의 0으로 채워진 텐서 (위치 인코딩 저장용)
- `position`: `0 ~ seq-1`까지의 위치 인덱스를 (seq, 1) 형태로 저장
- `div_term`: 스케일링 용도로 사용되는 값
    - 계산식: `10000^(2i / d_model)`
    - 여기서 i는 임베딩 차원 인덱스

### 🎵 공식 적용 (논문 기반)

- `pe[:, 0::2]`: 짝수 인덱스에는 **사인(sin)** 함수 값 저장
- `pe[:, 1::2]`: 홀수 인덱스에는 **코사인(cos)** 함수 값 저장

---

### 🔸 배치 차원 추가 및 버퍼 등록

- `pe = pe.unsqueeze(0)`
    
    → 크기를 `(1, seq, d_model)`로 변경 → 배치 처리를 위해
    
- `self.register_buffer('pe', pe)`
    
    → 학습 가능한 파라미터는 아니지만 모델 상태에 저장됨
    
    → GPU 등으로 모델 이동 시 자동 포함됨
    

---

### 🔸 순전파 (`forward` 메서드)

### ⚙️ 동작 과정

1. `x + positional encoding`
    - 입력 임베딩 x에 위치 인코딩을 더함
    - `requires_grad_(False)`로 위치 인코딩은 **학습되지 않도록 설정**
2. `return self.dropout(x)`
    - 드롭아웃 적용 → **과적합 방지**
    - 

### ✅ 핵심 요약

- 위치 정보를 임베딩에 더해 순서를 인식할 수 있게 함
- 사인/코사인 함수를 이용해 규칙적인 패턴으로 인코딩
- 드롭아웃을 통해 학습 안정성 향상

# **Add Norm (Layer Normalization)**

```python
class LayerNormalization(nn.Module):

    def __init__(self, features: int, eps:float=10**-6) -> None:
        super().__init__()
        self.eps = eps
        self.alpha = nn.Parameter(torch.ones(features)) # alpha (multiplicative) is a learnable parameter
        self.bias = nn.Parameter(torch.zeros(features)) # bias (additive) is a learnable parameter

    def forward(self, x):
        # x: (batch, seq, hidden_size)
        # Keep the dimension for broadcasting
        mean = x.mean(dim = -1, keepdim = True) # (batch, seq, 1)
        # Keep the dimension for broadcasting
        std = x.std(dim = -1, keepdim = True) # (batch, seq, 1)
        # eps is to prevent dividing by zero or when std is very small
        return self.alpha * (x - mean) / (std + self.eps) + self.bias
```

### 🔹 기본 개념

- *레이어 정규화(Layer Normalization)**는 입력 데이터를 정규화하여 학습을 더 안정적으로 만드는 기법입니다.

트랜스포머에서는 **모든 서브레이어(예: Self-Attention, Feedforward)** 뒤에 레이어 정규화를 적용합니다.

---

### 🔸 내부 속성 (Attributes)

1. **`self.eps`**
    - 정규화 과정에서 **0으로 나누는 것 방지**를 위한 아주 작은 값 (예: 1e-5)
2. **`self.alpha`**
    - 학습 가능한 **스케일(scale)** 파라미터
    - 초기값은 `1`이고, 모양은 `(features,)`
3. **`self.bias`**
    - 학습 가능한 **이동(shift)** 파라미터
    - 초기값은 `0`, 모양은 `(features,)`

---

### 🔸 순전파 (`forward` 메서드) 과정

### 📥 입력

- `x`: 모양이 `(batch_size, seq_len, hidden_size)`인 텐서
    
    (하나의 배치 내 문장들의 임베딩 벡터)
    

---

### 📊 평균 계산

```python
mean = x.mean(dim=-1, keepdim=True)
```

- **마지막 차원(hidden_size)** 기준으로 평균 계산
- 모양: `(batch_size, seq_len, 1)`
- `keepdim=True`는 **브로드캐스팅을 쉽게 하기 위해** 차원을 유지

---

### 📉 표준편차 계산

```python
std = x.std(dim=-1, keepdim=True)
```

- 마지막 차원 기준으로 표준편차 계산
- 모양은 평균과 동일하게 `(batch_size, seq_len, 1)`

---

### 🔄 정규화

```python
normed = (x - mean) / (std + self.eps)
```

- 평균을 빼고, 표준편차 + ε로 나누어 정규화
- **분산이 너무 작을 때의 불안정성**을 `eps`로 방지

---

### 🔧 스케일 & 시프트

```python
output = self.alpha * normed + self.bias
```

- 정규화된 결과를 `self.alpha`로 **스케일 조정**
- `self.bias`로 **값 이동**
- 이 둘은 **학습 가능한 파라미터**라 모델이 최적의 정규화 형태를 학습할 수 있음
- 

### ✅ 핵심 요약

- 마지막 차원(hidden_size)을 기준으로 평균과 표준편차를 계산하여 정규화
- 정규화된 값에 **스케일(alpha)** 과 **이동(bias)** 을 적용
- 학습 안정성과 속도 향상에 매우 중요

# **Feed Forward Layer**

```python
class FeedForwardBlock(nn.Module):

    def __init__(self, d_model: int, d_ff: int, dropout: float) -> None:
        super().__init__()
        self.linear_1 = nn.Linear(d_model, d_ff) # w1 and b1
        self.dropout = nn.Dropout(dropout)
        self.linear_2 = nn.Linear(d_ff, d_model) # w2 and b2

    def forward(self, x):
        # (batch, seq, d_model) --> (batch, seq, d_ff) --> (batch, seq, d_model)
        return self.linear_2(self.dropout(torch.relu(self.linear_1(x))))
```

**트랜스포머 논문의 행렬 설명**

- **행렬 W1**: 차원 512 → 2048 (입력 임베딩을 확장).
- **행렬 W2**: 차원 2048 → 512 (확장된 표현을 다시 축소).

### 🔹 기본 개념

트랜스포머에서 각 위치(position)의 토큰에 대해 독립적으로 적용되는 **두 개의 선형 계층(Fully Connected Layers)**

→ **비선형 변환**을 통해 모델의 표현력을 높여줌.

---

### 🔸 내부 속성 (Attributes)

1. **`self.linear_1`**
    - 첫 번째 선형 계층
    - 입력 차원 `d_model` → 은닉 차원 `d_ff`로 변환
    - 역할: **특징 확장 (feature expansion)**
2. **`self.linear_2`**
    - 두 번째 선형 계층
    - `d_ff` → 다시 `d_model`로 차원 축소
    - 역할: **원래 차원으로 복원**

---

### 🔸 순전파 (`forward` 메서드) 과정

### ① 첫 번째 선형 변환

```python
self.linear_1(x)
```

- 입력 텐서의 shape: `(batch, seq_len, d_model)`
- 출력: `(batch, seq_len, d_ff)`
- 의미: **차원을 넓혀 더 풍부한 표현 생성**

---

### ② ReLU 활성화

```python
torch.relu(self.linear_1(x))
```

- 비선형 함수 적용 (음수는 0으로, 양수는 그대로)
- 표현에 **비선형성**을 더해줌 → 더 복잡한 함수 근사 가능

---

### ③ 드롭아웃 (Dropout) *(생략되어 있었지만 대부분 포함됨)*

```python
self.dropout(...)
```

- 훈련 시 일부 뉴런을 임시로 꺼서 **과적합 방지**

---

### ④ 두 번째 선형 변환

```python
self.linear_2(...)
```

- 차원을 다시 `d_model`로 축소
- shape: `(batch, seq_len, d_model)`
- 원래 차원으로 되돌리면서 정보 정제

---

### 🔚 최종 출력

```python
return self.linear_2(self.dropout(torch.relu(self.linear_1(x))))
```

- 출력 모양: `(batch_size, seq_len, d_model)`
- 각 토큰 위치에 대해 독립적으로 처리됨

---

### ✅ 핵심 요약

- `d_model → d_ff → d_model` 구조의 **두 개의 선형 계층**
- ReLU로 **비선형성 부여**
- 드롭아웃으로 **과적합 방지**
- 트랜스포머의 **표현력을 높이는 핵심 구성 요소**

# **Multi-Head Attention**

```python
class MultiHeadAttentionBlock(nn.Module):

    def __init__(self, d_model: int, h: int, dropout: float) -> None:
        super().__init__()
        self.d_model = d_model # Embedding vector size
        self.h = h # Number of heads
        # Make sure d_model is divisible by h
        assert d_model % h == 0, "d_model is not divisible by h"

        self.d_k = d_model // h # Dimension of vector seen by each head
        self.w_q = nn.Linear(d_model, d_model, bias=False) # Wq
        self.w_k = nn.Linear(d_model, d_model, bias=False) # Wk
        self.w_v = nn.Linear(d_model, d_model, bias=False) # Wv
        self.w_o = nn.Linear(d_model, d_model, bias=False) # Wo
        self.dropout = nn.Dropout(dropout)

    @staticmethod
    def attention(query, key, value, mask, dropout: nn.Dropout):
        d_k = query.shape[-1]
        # Just apply the formula from the paper
        # (batch, h, seq, d_k) --> (batch, h, seq, seq)
        attention_scores = (query @ key.transpose(-2, -1)) / math.sqrt(d_k)
        if mask is not None:
            # Write a very low value (indicating -inf) to the positions where mask == 0
            attention_scores.masked_fill_(mask == 0, -1e9)
        attention_scores = attention_scores.softmax(dim=-1) # (batch, h, seq, seq) # Apply softmax
        if dropout is not None:
            attention_scores = dropout(attention_scores)
        # (batch, h, seq, seq) --> (batch, h, seq, d_k)
        # return attention scores which can be used for visualization
        return (attention_scores @ value), attention_scores

    def forward(self, q, k, v, mask):
        query = self.w_q(q) # (batch, seq, d_model) --> (batch, seq, d_model)
        key = self.w_k(k) # (batch, seq, d_model) --> (batch, seq, d_model)
        value = self.w_v(v) # (batch, seq, d_model) --> (batch, seq, d_model)

        # (batch, seq, d_model) --> (batch, seq, h, d_k) --> (batch, h, seq, d_k)
        query = query.view(query.shape[0], query.shape[1], self.h, self.d_k).transpose(1, 2)
        key = key.view(key.shape[0], key.shape[1], self.h, self.d_k).transpose(1, 2)
        value = value.view(value.shape[0], value.shape[1], self.h, self.d_k).transpose(1, 2)

        # Calculate attention
        x, self.attention_scores = MultiHeadAttentionBlock.attention(query, key, value, mask, self.dropout)

        # Combine all the heads together
        # (batch, h, seq, d_k) --> (batch, seq, h, d_k) --> (batch, seq, d_model)
        x = x.transpose(1, 2).contiguous().view(x.shape[0], -1, self.h * self.d_k)

        # Multiply by Wo
        # (batch, seq, d_model) --> (batch, seq, d_model)
        return self.w_o(x)
```

### 🔹 기본 개념

멀티헤드 어텐션은 **입력 정보를 여러 관점(head)** 에서 동시에 주의(attention)하여 더 풍부한 표현을 만드는 구조입니다.

트랜스포머의 핵심 구성요소 중 하나로, **Self-Attention**을 여러 번 병렬로 수행합니다.

---

### 🔸 내부 속성 (Attributes)

1. **`self.d_model`**
    - 전체 임베딩 차원 (예: 512, 768 등)
2. **`self.h`**
    - 어텐션 헤드 수 (예: 8개, 12개)
    - 입력 벡터를 이 수만큼 나누어 병렬 처리함
3. **`self.d_k = d_model // h`**
    - **각 헤드가 담당하는 차원 수**
    - 예: d_model이 512이고 h가 8이면, 각 헤드는 64차원 처리
4. **`self.w_q, self.w_k, self.w_v`**
    - 각각 쿼리(Q), 키(K), 밸류(V)로 입력을 변환하는 선형 레이어
5. **`self.w_o`**
    - 모든 헤드의 출력을 결합한 뒤, 최종 출력으로 변환하는 선형 레이어

---

### 🔸 어텐션 계산 과정 (`attention` 메서드)

### 📥 입력

- **query, key, value**: 각 입력 시퀀스의 변환된 벡터
- **mask (선택 사항)**: 특정 위치를 무시할 때 사용 (ex. padding, causal masking)

---

### ① 어텐션 스코어 계산

```python
scores = (query @ key.transpose(-2, -1)) / math.sqrt(d_k)
```

- Q와 K의 내적을 통해 유사도 계산
- `√d_k`로 나누어 **그래디언트 안정화**

---

### ② 마스크 적용 (선택)

- mask가 주어지면, **마스크된 위치에 큰 음수(-inf)**를 넣어 softmax 후 무시되게 만듦

---

### ③ 어텐션 가중치 & 출력 계산

```python
weights = softmax(scores)
output = weights @ value
```

- 소프트맥스로 가중치 계산 후, V와 곱해 최종 어텐션 출력 생성

---

### 🔸 멀티헤드 처리 흐름

### ① 입력 분할

```python
(batch, seq_len, d_model) → (batch, h, seq_len, d_k)
```

- Q, K, V를 각각 `h`개의 헤드로 나눔
- `.view()`와 `.transpose()` 등을 통해 shape 조정

### ② 병렬로 어텐션 수행

- 각 헤드에서 독립적으로 attention 계산

### ③ 헤드 결합

```python
(batch, h, seq_len, d_k) → (batch, seq_len, d_model)
```

- 여러 헤드의 출력을 concat (병합)
- `.transpose()` 후 `.contiguous().view()`로 원래 차원 복원

### ④ 최종 선형 변환

- `self.w_o`를 사용해 최종 출력 생성
- shape: `(batch, seq_len, d_model)`

---

### ✅ 핵심 요약

| 단계 | 설명 |
| --- | --- |
| 입력 선형변환 | Q, K, V 생성 (linear layers) |
| 분할 | 여러 헤드로 나눔 (h개) |
| 어텐션 수행 | 각 헤드에서 scaled dot-product attention |
| 결합 | 모든 헤드 결과를 합쳐 하나로 |
| 최종 변환 | 선형 레이어를 통해 출력 |

# **Residual Connection**

```python
class ResidualConnection(nn.Module):

        def __init__(self, features: int, dropout: float) -> None:
            super().__init__()
            self.dropout = nn.Dropout(dropout)
            self.norm = LayerNormalization(features)

        def forward(self, x, sublayer):
            return x + self.dropout(sublayer(self.norm(x)))
```

### 🔹 기본 개념

트랜스포머에서는 **잔차 연결(Residual Connection)** 과 **레이어 정규화(LayerNorm)** 를 결합하여 학습 안정성과 성능을 향상시킵니다.

- 구조:
    
    **`x → LayerNorm → Sublayer → Dropout → + x (Residual)`**
    

---

### 🔸 `forward` 메서드 개요

### 📥 입력

- `x`: 입력 텐서 (예: 어텐션 또는 FFN 이전의 입력)
- `sublayer`: 함수 형태로 전달되는 **서브레이어**
    
    예: `MultiHeadAttention`, `FeedForward`, 등
    

---

### 🔸 단계별 처리 과정

---

### ① 정규화

```python
self.norm(x)
```

- 입력 `x`에 대해 **Layer Normalization** 수행
- 학습 안정화 및 속도 향상

---

### ② 서브레이어 적용

```python
sublayer(self.norm(x))

```

- 정규화된 값을 **sublayer** (예: 어텐션 또는 FFN)에 전달
- 서브레이어는 입력을 변환함

---

### ③ 드롭아웃

```python
self.dropout(sublayer(...))
```

- **과적합 방지**를 위한 드롭아웃 적용
- 학습 중 일부 뉴런 무작위 제거

---

### ④ 잔차 연결 (Residual Connection)

```python
x + self.dropout(...)
```

- **원래 입력 x**와 **서브레이어의 출력**을 더함
- 모델이 필요 시 **"아무 것도 하지 않기" (identity mapping)** 를 학습할 수 있도록 도와줌
- **딥러닝에서 그래디언트 소실(Vanishing Gradient)** 문제를 줄여줌

---

### 🔚 최종 출력

- 정규화 → 서브레이어 → 드롭아웃 → 잔차 연결 후 결과 반환

```python
return x + self.dropout(sublayer(self.norm(x)))
```

---

### ✅ 핵심 요약

| 단계 | 설명 |
| --- | --- |
| `self.norm(x)` | 정규화로 학습 안정화 |
| `sublayer(...)` | 어텐션 or FFN 등 변환 수행 |
| `self.dropout(...)` | 일부 뉴런 제거로 과적합 방지 |
| `x + ...` | 입력 + 출력 → **잔차 연결로 성능 향상** |

# Encoder Block

```python
class EncoderBlock(nn.Module):

    def __init__(self, features: int, self_attention_block: MultiHeadAttentionBlock, feed_forward_block: FeedForwardBlock, dropout: float) -> None:
        super().__init__()
        self.self_attention_block = self_attention_block
        self.feed_forward_block = feed_forward_block
        self.residual_connections = nn.ModuleList([ResidualConnection(features, dropout) for _ in range(2)])

    def forward(self, x, src_mask):
        x = self.residual_connections[0](x, lambda x: self.self_attention_block(x, x, x, src_mask))
        x = self.residual_connections[1](x, self.feed_forward_block)
        return x
```

### 🔹 기본 개념

트랜스포머 인코더의 한 블록은 다음 두 가지 주요 구성 요소로 이루어져 있습니다:

1. **멀티헤드 자기어텐션 (Multi-Head Self-Attention)**
2. **포지션-와이즈 피드포워드 네트워크 (Feed-Forward Network)**

각 구성 요소에는 **정규화(LayerNorm)** 와 **잔차 연결(Residual Connection)** 이 함께 사용됩니다.

---

### 🔸 속성 설명 (Attributes)

1. **`self.self_attention_block`**
    - 멀티헤드 **자기어텐션** 블록 저장
2. **`self.feed_forward_block`**
    - **피드포워드 네트워크** 저장 (2개의 선형 계층 + ReLU)
3. **`self.residual_connections`**
    - **ResidualConnection 인스턴스 2개를 리스트로 보관**
    - 하나는 어텐션용, 하나는 FFN용
    - 형식: `[ResidualConnection, ResidualConnection]`

---

### 🔸 순전파 과정 (`forward` 메서드)

### ✅ 1단계: 자기어텐션 + 잔차 연결

```python
x = self.residual_connections[0](
    x,
    lambda x: self.self_attention_block(x, x, x, src_mask)
)
```

- `x`는 입력 텐서
- 먼저 **LayerNorm → Self-Attention → Dropout** 적용
- 원래 입력 `x`와 결과를 더해서 **잔차 연결**

🔹 `lambda x: ...` 를 쓰는 이유:

- `ResidualConnection` 안에서 정규화된 `x`를 받아 어텐션 블록에 넘기기 위해
- `query = key = value = x` 형태로 자기어텐션 수행

---

### ✅ 2단계: 피드포워드 네트워크 + 잔차 연결

```python
x = self.residual_connections[1](x, self.feed_forward_block)
```

- 첫 번째 출력 `x`를 다시 정규화 후 FFN에 통과
- 결과를 원래 입력 `x`에 더함 (잔차 연결)

---

### 🔚 최종 출력

- 어텐션 → 잔차 연결
- FFN → 잔차 연결
- 두 단계를 거친 최종 텐서를 반환

```python
return x
```

---

### ✅ 핵심 요약

| 순서 | 구성 요소 | 설명 |
| --- | --- | --- |
| ① | **자기어텐션** | 입력 간 상호작용 학습 |
| ② | **잔차 + 정규화** | 학습 안정화 |
| ③ | **피드포워드 네트워크** | 위치별 독립적 비선형 변환 |
| ④ | **잔차 + 정규화** | 정보 보존 및 안정화 |

# Encoder (Stack)

```python
class Encoder(nn.Module):

    def __init__(self, features: int, layers: nn.ModuleList) -> None:
        super().__init__()
        self.layers = layers
        self.norm = LayerNormalization(features)

    def forward(self, x, mask):
        for layer in self.layers:
            x = layer(x, mask)
        return self.norm(x)
```

### 🔹 기본 개념

전체 **인코더**는 여러 개의 **인코더 레이어(블록)** 를 **순차적으로 쌓은 구조**입니다.

각 레이어는 다음을 포함합니다:

- 멀티헤드 자기어텐션
- 피드포워드 네트워크
- 레이어 정규화 + 잔차 연결

---

### 🔸 Feed Forward 과정 (Encoder의 forward 메서드)

### ✅ 1단계: 인코더 레이어들을 순차적으로 통과

```python
for layer in self.layers:
    x = layer(x, mask)
```

- `self.layers`: 여러 개의 인코더 레이어가 담긴 리스트
- 각 레이어에서 입력 `x`는 다음과 같은 과정을 거칩니다:
    - LayerNorm → MultiHeadAttention → Residual
    - LayerNorm → FFN → Residual
- 레이어를 통과할 때마다 `x`가 갱신됨

---

### ✅ 2단계: 최종 레이어 정규화

```python
x = self.norm(x)
```

- 모든 인코더 레이어를 통과한 후, **마지막으로 한 번 더 정규화(LayerNorm)**
- 모델이 출력 전에 안정된 표현을 생성하도록 돕는 역할

---

### ✅ 3단계: 출력 반환

```python
return x
```

- 정규화된 최종 인코더 출력을 반환
- 이 출력은 이후 디코더(decoder)나 다른 태스크(예: 분류)에 사용됨

---

### ✅ 핵심 요약

| 단계 | 설명 |
| --- | --- |
| `for layer in self.layers:` | 인코더 레이어를 반복하면서 `x`를 업데이트 |
| `x = self.norm(x)` | 최종 출력값 정규화 |
| `return x` | 인코더의 최종 출력 반환 |

# Decoder Block

```python
class DecoderBlock(nn.Module):

    def __init__(self, features: int, self_attention_block: MultiHeadAttentionBlock, cross_attention_block: MultiHeadAttentionBlock, feed_forward_block: FeedForwardBlock, dropout: float) -> None:
        super().__init__()
        self.self_attention_block = self_attention_block
        self.cross_attention_block = cross_attention_block
        self.feed_forward_block = feed_forward_block
        self.residual_connections = nn.ModuleList([ResidualConnection(features, dropout) for _ in range(3)])

    def forward(self, x, encoder_output, src_mask, tgt_mask):
        x = self.residual_connections[0](x, lambda x: self.self_attention_block(x, x, x, tgt_mask))
        x = self.residual_connections[1](x, lambda x: self.cross_attention_block(x, encoder_output, encoder_output, src_mask))
        x = self.residual_connections[2](x, self.feed_forward_block)
        return x
```

### 🔹 기본 개념

디코더의 한 블록은 다음 **세 가지 서브레이어**로 구성되어 있습니다:

1. **자기어텐션 (Self-Attention)**
2. **크로스어텐션 (Cross-Attention)** – 인코더 출력에 대한 어텐션
3. **피드포워드 네트워크 (Feed Forward Network)**

각 서브레이어는 **잔차 연결(Residual Connection)** 과 **정규화(LayerNorm)** 와 함께 사용됩니다.

---

### 🔸 파라미터 설명

| 이름 | 설명 |
| --- | --- |
| `self_attention_block` | 타겟 시퀀스 내부에서 자기어텐션을 수행 |
| `cross_attention_block` | 인코더의 출력에 대해 어텐션 수행 (크로스어텐션) |
| `feed_forward_block` | 포지션-와이즈 피드포워드 네트워크 |
| `src_mask` | 소스 시퀀스에 대한 마스크 |
| `tgt_mask` | 타겟 시퀀스에 대한 마스크 (미래 토큰 가리기 등) |

---

### 🔸 속성 설명 (Attributes)

- `self.self_attention_block`: 자기어텐션 블록 저장
- `self.cross_attention_block`: 인코더 출력에 대한 크로스어텐션 블록 저장
- `self.feed_forward_block`: FFN 저장
- `self.residual_connections`: ResidualConnection 3개 (각 서브레이어용)

---

### 🔸 순전파 과정 (`forward` 메서드)

### ✅ 1단계: **자기어텐션 + 잔차 연결**

```python
x = self.residual_connections[0](
    x,
    lambda x: self.self_attention_block(x, x, x, tgt_mask)
)
```

- 자기 시퀀스 내부 토큰 간의 관계를 학습
- 미래 정보 방지를 위해 **tgt_mask** 적용
- LayerNorm → Self-Attention → Dropout → Residual

---

### ✅ 2단계: **크로스어텐션 + 잔차 연결**

```python
x = self.residual_connections[1](
    x,
    lambda x: self.cross_attention_block(x, encoder_output, encoder_output, src_mask)
)
```

- 인코더 출력(소스 시퀀스)에 대한 어텐션
- 각 디코더 위치가 인코더의 모든 위치를 참고 가능
- LayerNorm → Cross-Attention → Dropout → Residual

---

### ✅ 3단계: **피드포워드 + 잔차 연결**

```python
x = self.residual_connections[2](x, self.feed_forward_block)
```

- 위치별 독립적 변환 수행
- LayerNorm → FFN → Dropout → Residual

---

### ✅ 출력 반환

```python
return x
```

- 세 서브레이어를 모두 거친 최종 디코더 블록 출력 반환

---

### ✅ 핵심 요약

| 순서 | 구성 요소 | 설명 |
| --- | --- | --- |
| ① | **Self-Attention + Residual** | 미래 토큰을 가리는 자기어텐션 |
| ② | **Cross-Attention + Residual** | 인코더 출력에 대한 어텐션 |
| ③ | **Feed Forward + Residual** | 각 위치별 비선형 변환 |
| 🔄 | 각 단계마다 LayerNorm → Dropout → Residual 적용 |  |

# Decoder (Stack)

```python
class Decoder(nn.Module):

    def __init__(self, features: int, layers: nn.ModuleList) -> None:
        super().__init__()
        self.layers = layers
        self.norm = LayerNormalization(features)

    def forward(self, x, encoder_output, src_mask, tgt_mask):
        for layer in self.layers:
            x = layer(x, encoder_output, src_mask, tgt_mask)
        return self.norm(x)
```

### 🔹 기본 개념

전체 **디코더**는 여러 개의 **디코더 블록(DecoderBlock)** 으로 구성되어 있으며,

각 블록은 다음을 포함합니다:

- 자기어텐션 (Self-Attention)
- 크로스어텐션 (Cross-Attention)
- 피드포워드 네트워크 (Feed Forward)
- 잔차 연결 + 정규화

---

### 🔸 순전파 과정 (forward 메서드)

### ✅ 1단계: 디코더 블록들을 순차적으로 통과

```python
for layer in self.layers:
    x = layer(x, encoder_output, src_mask, tgt_mask)
```

- `self.layers`: 여러 개의 디코더 레이어가 담긴 리스트
- 각 디코더 레이어는 다음을 수행:
    - 타겟 시퀀스에 대한 자기어텐션 (`tgt_mask` 사용 → 미래 토큰 가리기)
    - 인코더 출력에 대한 크로스어텐션 (`src_mask` 사용)
    - 피드포워드 네트워크
    - 각 과정마다 **정규화 + 잔차 연결**

---

### ✅ 2단계: 최종 레이어 정규화

```python
x = self.norm(x)
```

- 모든 디코더 레이어를 통과한 후 마지막으로 LayerNorm 적용
- 모델이 출력 전에 안정된 표현을 생성하도록 돕는 역할

---

### ✅ 3단계: 출력 반환

```python
return x
```

- 정규화된 최종 디코더 출력 반환
- 이 출력은 최종적으로 **소프트맥스 → 다음 토큰 예측** 등에 사용됨

---

### ✅ 핵심 요약

| 단계 | 설명 |
| --- | --- |
| `for layer in self.layers:` | 디코더 레이어들을 반복하면서 `x`를 순차 처리 |
| `tgt_mask` | 디코더가 미래 토큰을 보지 못하도록 마스킹 |
| `src_mask` | 인코더 출력 중 일부 토큰에 대한 어텐션을 차단 |
| `self.norm(x)` | 마지막 정규화 |
| `return x` | 디코더의 최종 출력 반환 |

# **Projection Layer**

```python
class ProjectionLayer(nn.Module):

    def __init__(self, d_model, vocab_size) -> None:
        super().__init__()
        self.proj = nn.Linear(d_model, vocab_size)

    def forward(self, x) -> None:
        # (batch, seq, d_model) --> (batch, seq, vocab_size)
        return self.proj(x)
```

**ProjectionLayer의 역할**

- **기능**: 트랜스포머 모델의 디코더 출력을 어휘 공간(vocabulary space)으로 매핑.
- **목적**: 다음 단어 생성 또는 예측 가능하도록 함.
- **특징**: 간단하지만 트랜스포머에서 핵심적인 구성 요소.

**트랜스포머의 출력층(Output Layer) 요약:**

- **`self.proj`**:
    - **역할**: 트랜스포머 모델의 결과(`d_model` 차원)를 전체 단어의 개수(`vocab_size`)로 변환해주는 선형 계층(nn.Linear).
    - **기능**: 입력 데이터를 어휘(단어) 공간으로 투영(projection)하여 각 단어에 대한 점수(logit)를 계산.
- **`forward` 메소드 (순전파 과정)**:
    - **입력**: 디코더로부터 받은 텐서 `x`.
        - **형태**: `(batch, seq, d_model)` (배치 크기, 시퀀스 길이, 모델 차원).
    - **처리**: 입력 텐서 `x`가 `self.proj` 선형 계층을 통과.
        - **과정**: 입력 텐서와 `self.proj`의 가중치 행렬을 곱하고 편향(bias)을 더하는 선형 변환 수행.
    - **출력**: 변환된 텐서.
        - **형태**: `(batch, seq, vocab_size)` (배치 크기, 시퀀스 길이, 어휘 크기).
        - **의미**: 시퀀스 내 각 위치마다 전체 어휘에 있는 각 단어에 대한 **점수(logit)** 벡터가 생성됨. 이 점수들을 통해 어떤 단어가 다음에 올 확률이 높은지 알 수 있습니다.

# **Transformer**

```python
class Transformer(nn.Module):

    def __init__(self, encoder: Encoder, decoder: Decoder, src_embed: InputEmbeddings, tgt_embed: InputEmbeddings, src_pos: PositionalEncoding, tgt_pos: PositionalEncoding, projection_layer: ProjectionLayer) -> None:
        super().__init__()
        self.encoder = encoder
        self.decoder = decoder
        self.src_embed = src_embed
        self.tgt_embed = tgt_embed
        self.src_pos = src_pos
        self.tgt_pos = tgt_pos
        self.projection_layer = projection_layer

    def encode(self, src, src_mask):
        # (batch, seq, d_model)
        src = self.src_embed(src)
        src = self.src_pos(src)
        return self.encoder(src, src_mask)

    def decode(self, encoder_output: torch.Tensor, src_mask: torch.Tensor, tgt: torch.Tensor, tgt_mask: torch.Tensor):
        # (batch, seq, d_model)
        tgt = self.tgt_embed(tgt)
        tgt = self.tgt_pos(tgt)
        return self.decoder(tgt, encoder_output, src_mask, tgt_mask)

    def project(self, x):
        # (batch, seq, vocab_size)
        return self.projection_layer(x)
```

**트랜스포머의 핵심 구성 요소 및 작동 방식 요약:**

---

### **1. 초기화 (`__init__`)**

- **역할**: 트랜스포머 모델을 사용할 준비를 합니다.
- **포함**: **인코더**, **디코더**, **임베딩(Embedding) 계층**, **위치 인코딩(Positional Encoding) 계층**, 그리고 **투영(Projection) 계층**과 같은 필수 구성 요소들을 미리 설정합니다.
- **목적**: 입력 시퀀스를 인코딩하고 디코딩하는 전체 과정을 위한 기본 골격을 마련합니다.

---

### **2. 인코딩 (`encode`)**

- **역할**: 입력으로 들어온 **원본 시퀀스**를 처리합니다.
- **과정**:
    1. 입력된 단어(토큰)들을 숫자로 변환하고, 이 숫자들을 의미 있는 **임베딩 벡터**로 바꿉니다.
    2. 단어의 **순서 정보**를 유지하기 위해 **위치 인코딩**을 임베딩에 추가합니다.
    3. 이 정보를 **인코더**에 넣어 원본 시퀀스의 압축된 **표현(encoded representation)**을 만듭니다.

---

### **3. 디코딩 (`decode`)**

- **역할**: **목표 시퀀스** (번역하려는 결과물 또는 다음 단어)를 처리합니다.
- **과정**:
    1. 목표 시퀀스의 단어들을 **임베딩**하고 **위치 인코딩**을 추가합니다.
    2. 이 정보와 함께 **인코더의 출력**을 **디코더**에 입력합니다.
    3. 디코더는 이 두 정보를 바탕으로 최종 예측을 생성하는 데 사용될 **디코딩된 표현(decoded representation)**을 출력합니다.

---

### **4. 투영 (`project`)**

- **역할**: **디코더의 출력**을 실제 단어로 변환합니다.
- **과정**: 디코더에서 나온 연속적인 숫자 값들을 **어휘(단어) 공간**에 있는 특정 단어로 매핑합니다.
- **목적**: 디코더의 추상적인 결과물을 사람이 이해할 수 있는 구체적인 **단어 예측**으로 바꾸는 핵심 단계입니다.

# **Build Transformer**

```python
def build_transformer(src_vocab_size: int, tgt_vocab_size: int, src_seq: int, tgt_seq: int, d_model: int=512, N: int=6, h: int=8, dropout: float=0.1, d_ff: int=2048) -> Transformer:
    # Create the embedding layers
    src_embed = InputEmbeddings(d_model, src_vocab_size)
    tgt_embed = InputEmbeddings(d_model, tgt_vocab_size)

    # Create the positional encoding layers
    src_pos = PositionalEncoding(d_model, src_seq, dropout)
    tgt_pos = PositionalEncoding(d_model, tgt_seq, dropout)

    # Create the encoder blocks
    encoder_blocks = []
    for _ in range(N):
        encoder_self_attention_block = MultiHeadAttentionBlock(d_model, h, dropout)
        feed_forward_block = FeedForwardBlock(d_model, d_ff, dropout)
        encoder_block = EncoderBlock(d_model, encoder_self_attention_block, feed_forward_block, dropout)
        encoder_blocks.append(encoder_block)

    # Create the decoder blocks
    decoder_blocks = []
    for _ in range(N):
        decoder_self_attention_block = MultiHeadAttentionBlock(d_model, h, dropout)
        decoder_cross_attention_block = MultiHeadAttentionBlock(d_model, h, dropout)
        feed_forward_block = FeedForwardBlock(d_model, d_ff, dropout)
        decoder_block = DecoderBlock(d_model, decoder_self_attention_block, decoder_cross_attention_block, feed_forward_block, dropout)
        decoder_blocks.append(decoder_block)

    # Create the encoder and decoder
    encoder = Encoder(d_model, nn.ModuleList(encoder_blocks))
    decoder = Decoder(d_model, nn.ModuleList(decoder_blocks))

    # Create the projection layer
    projection_layer = ProjectionLayer(d_model, tgt_vocab_size)

    # Create the transformer
    transformer = Transformer(encoder, decoder, src_embed, tgt_embed, src_pos, tgt_pos, projection_layer)

    # Initialize the parameters
    for p in transformer.parameters():
        if p.dim() > 1:
            nn.init.xavier_uniform_(p)

    return transformer
```

**트랜스포머 모델 구축 (`build_transformer`) 요약:**

- **역할**: 시퀀스-투-시퀀스(Sequence-to-Sequence) 작업을 위한 트랜스포머 모델을 생성합니다.
- **입력 매개변수**:
    - **입력(원본) 및 출력(목표) 어휘 크기**: 모델이 다룰 수 있는 단어의 총 개수.
    - **시퀀스 길이**: 처리할 문장의 최대 길이.
    - **모델 차원 (d_model)**: 모델 내부에서 데이터를 표현하는 벡터의 크기.
    - **계층 수 (num_layers)**: 인코더와 디코더에 쌓을 블록의 개수.
    - **어텐션 헤드 수 (num_attention_heads)**: 어텐션 메커니즘에서 병렬로 정보를 처리할 헤드의 개수.
    - **드롭아웃 비율 (dropout_rate)**: 과적합 방지를 위한 드롭아웃 적용 비율.
    - **피드포워드 신경망 차원 (feed_forward_dimensions)**: 인코더/디코더 블록 내 피드포워드 네트워크의 차원.
- **생성 구성 요소**:
    - **임베딩 계층**: 단어를 벡터로 변환.
    - **위치 인코딩 계층**: 단어의 위치 정보를 추가.
    - **인코더 및 디코더 블록**: 실제 인코딩 및 디코딩 연산을 수행하는 핵심 모듈.
    - **투영 계층**: 디코더 출력을 최종 단어 예측으로 변환.
- **최종 작업**: 모델의 모든 매개변수를 초기화하고, 완성된 트랜스포머 모델 객체를 반환합니다.

# 참고자료

[Building a Transformer from Scratch: A Step-by-Step Guide](https://medium.com/@sayedebad.777/building-a-transformer-from-scratch-a-step-by-step-guide-a3df0aeb7c9a)

https://github.com/ES7/Transformer-from-Scratch


################################## 

# Transformer — Python 코드 설명 (Training)

# **Dataset (tformer > dataset.py)**

```python
import torch
import torch.nn as nn
from torch.utils.data import Dataset

class TranslationDataset(Dataset):

    def __init__(self, ds, tokenizer_src, tokenizer_tgt, src_lang, tgt_lang, seq):
        super().__init__()
        self.seq = seq

        self.ds = ds
        self.tokenizer_src = tokenizer_src
        self.tokenizer_tgt = tokenizer_tgt
        self.src_lang = src_lang
        self.tgt_lang = tgt_lang

        self.sos_token = torch.tensor([tokenizer_tgt.token_to_id("[SOS]")], dtype=torch.int64)
        self.eos_token = torch.tensor([tokenizer_tgt.token_to_id("[EOS]")], dtype=torch.int64)
        self.pad_token = torch.tensor([tokenizer_tgt.token_to_id("[PAD]")], dtype=torch.int64)

    def __len__(self):
        return len(self.ds)

    def __getitem__(self, idx):
        src_target_pair = self.ds[idx]
        src_text = src_target_pair['translation'][self.src_lang]
        tgt_text = src_target_pair['translation'][self.tgt_lang]

        # Transform the text into tokens
        enc_input_tokens = self.tokenizer_src.encode(src_text).ids
        dec_input_tokens = self.tokenizer_tgt.encode(tgt_text).ids

        # Add sos, eos and padding to each sentence
        enc_num_padding_tokens = self.seq - len(enc_input_tokens) - 2  # We will add <s> and </s>
        # We will only add <s>, and </s> only on the label
        dec_num_padding_tokens = self.seq - len(dec_input_tokens) - 1

        # Make sure the number of padding tokens is not negative. If it is, the sentence is too long
        if enc_num_padding_tokens < 0 or dec_num_padding_tokens < 0:
            raise ValueError("Sentence is too long")

        # Add <s> and </s> token
        encoder_input = torch.cat(
            [
                self.sos_token,
                torch.tensor(enc_input_tokens, dtype=torch.int64),
                self.eos_token,
                torch.tensor([self.pad_token] * enc_num_padding_tokens, dtype=torch.int64),
            ],
            dim=0,
        )

        # Add only <s> token
        decoder_input = torch.cat(
            [
                self.sos_token,
                torch.tensor(dec_input_tokens, dtype=torch.int64),
                torch.tensor([self.pad_token] * dec_num_padding_tokens, dtype=torch.int64),
            ],
            dim=0,
        )

        # Add only </s> token
        label = torch.cat(
            [
                torch.tensor(dec_input_tokens, dtype=torch.int64),
                self.eos_token,
                torch.tensor([self.pad_token] * dec_num_padding_tokens, dtype=torch.int64),
            ],
            dim=0,
        )

        # Double check the size of the tensors to make sure they are all seq long
        assert encoder_input.size(0) == self.seq
        assert decoder_input.size(0) == self.seq
        assert label.size(0) == self.seq

        return {
            "encoder_input": encoder_input,  # (seq)
            "decoder_input": decoder_input,  # (seq)
            "encoder_mask": (encoder_input != self.pad_token).unsqueeze(0).unsqueeze(0).int(), # (1, 1, seq)
            "decoder_mask": (decoder_input != self.pad_token).unsqueeze(0).int() & causal_mask(decoder_input.size(0)), # (1, seq) & (1, seq, seq),
            "label": label,  # (seq)
            "src_text": src_text,
            "tgt_text": tgt_text,
        }

def causal_mask(size):
    mask = torch.triu(torch.ones((1, size, size)), diagonal=1).type(torch.int)
    return mask == 0
```

**트랜스포머 학습용 데이터셋 클래스 요약:**

이 데이터셋 클래스는 번역 작업을 위해 트랜스포머 모델에 필요한 데이터를 준비합니다. 토큰화, 패딩, 특수 토큰 처리, 마스킹 등을 통해 모델이 올바른 형식의 입력을 받도록 보장합니다.

---

### **1. 초기화 (`__init__`)**

- **역할**: 데이터셋을 설정하고 필요한 도구들을 준비합니다.
- **매개변수**:
    1. `ds`: 원본 및 목표 언어 쌍을 포함하는 실제 데이터셋.
    2. `tokenizer_src`: 원본 언어 텍스트를 토큰화하는 도구.
    3. `tokenizer_tgt`: 목표 언어 텍스트를 토큰화하는 도구.
    4. `src_lang`: 원본 언어 식별자 (예: 'en' for English).
    5. `tgt_lang`: 목표 언어 식별자 (예: 'it' for Italian).
    6. `seq`: 모든 문장을 고정된 길이로 맞추기 위한 시퀀스(문장) 길이.
- **추가 설정**: 목표 언어를 위한 특수 토큰 (`[SOS]` (문장 시작), `[EOS]` (문장 끝), `[PAD]` (패딩))을 초기화합니다.

---

### **2. 길이 반환 (`__len__`)**

- **역할**: 데이터셋에 있는 샘플의 총 개수를 반환합니다.

---

### **3. 아이템 가져오기 (`__getitem__`)**

- **역할**: 주어진 인덱스 `idx`에 해당하는 데이터 샘플을 가져와 처리합니다.
- **처리 과정**:
    - **텍스트 추출**: 데이터셋에서 원본 및 목표 텍스트를 가져옵니다.
    - **텍스트 토큰화**: 각 텍스트를 해당 토크나이저로 잘게 쪼개어 토큰으로 만듭니다.
    - **패딩 계산**: 정해진 `self.seq` 길이에 맞추기 위해 필요한 패딩 토큰의 수를 계산합니다.
    - **문장 길이 검증**: 특수 토큰 추가 후에도 문장이 `self.seq` 길이를 초과하지 않는지 확인합니다.
    - **인코더 입력 생성**: `[SOS]` 토큰 + 인코딩된 원본 토큰들 + `[EOS]` 토큰 + 패딩 토큰들을 연결하여 만듭니다.
    - **디코더 입력 생성**: `[SOS]` 토큰 + 인코딩된 목표 토큰들 + 패딩 토큰들을 연결하여 만듭니다.
    - **레이블 생성**: 인코딩된 목표 토큰들 + `[EOS]` 토큰 + 패딩 토큰들을 연결하여 만듭니다 (모델이 예측해야 할 정답).
    - **마스크 생성**:
        - 실제 토큰과 패딩 토큰을 구분하기 위한 **인코더/디코더 입력 마스크**를 만듭니다.
        - 디코더가 미래의 정보를 미리 보지 못하도록 **인과(causal) 마스크**를 디코더 입력에 적용합니다.
- **반환**: 인코더 입력, 디코더 입력, 마스크들, 레이블, 그리고 원본 텍스트들을 포함하는 딕셔너리를 반환합니다.

---

### **4. 인과 마스크 함수 (`causal_mask`)**

- **역할**: 디코더가 훈련 중에 현재 위치 이전의 토큰들만 참조하도록 제한하는 마스크를 생성합니다.
- **생성 방법**: 대각선을 제외한 상삼각 행렬을 1로 채운 다음, 0이 허용된 위치(이전 위치를 참조할 수 있는 위치)를 나타내는 불리언 마스크로 변환합니다. 이는 인과 관계를 강제하여 디코더가 순차적으로 단어를 생성하도록 돕습니다.

# **Configuration (tformer > config.py)**

```python
from pathlib import Path

def get_config():
    return {
        "batch_size": 8,
        "num_epochs": 20,
        "lr": 10**-4,
        "seq": 350,
        "d_model": 512,
        "datasource": 'opus_books',
        "lang_src": "en",
        "lang_tgt": "it",
        "model_folder": "weights",
        "model_basename": "tmodel_",
        "preload": "latest",
        "tokenizer_file": "tokenizer_{0}.json",
        "experiment_name": "runs/tmodel"
    }

def get_weights_file_path(config, epoch: str):
    model_folder = f"{config['datasource']}_{config['model_folder']}"
    model_filename = f"{config['model_basename']}{epoch}.pt"
    return str(Path('.') / model_folder / model_filename)

# Find the latest weights file in the weights folder
def latest_weights_file_path(config):
    model_folder = f"{config['datasource']}_{config['model_folder']}"
    model_filename = f"{config['model_basename']}*"
    weights_files = list(Path(model_folder).glob(model_filename))
    if len(weights_files) == 0:
        return None
    weights_files.sort()
    return str(weights_files[-1])
```

### **1. 설정 가져오기 (`get_config`) 함수**

- **역할**: 모델 학습에 필요한 모든 설정 매개변수를 담은 딕셔너리를 반환합니다.
- **주요 매개변수 설명**:
    - `batch_size`: 한 번의 학습 반복에서 처리되는 샘플의 수.
    - `num_epochs`: 전체 학습 데이터셋이 모델을 통과하는 횟수.
    - `lr`: 옵티마이저(optimizer)의 학습률(learning rate).
    - `seq`: 입력 데이터의 시퀀스(문장) 길이.
    - `d_model`: 모델의 차원 (예: 입력에서 특징의 수).
    - `datasource`: 데이터셋 소스의 이름 (예: ‘opus_books’).
    - `lang_src`: 원본 언어 코드 (예: ‘en’ for English).
    - `lang_tgt`: 목표 언어 코드 (예: ‘it’ for Italian).
    - `model_folder`: 모델 가중치 파일이 저장될 폴더.
    - `model_basename`: 저장될 모델 파일의 기본 이름.
    - `preload`: 가장 최신 가중치 파일을 로드할지 여부 (예: ‘latest’).
    - `tokenizer_file`: 토크나이저 파일 이름의 템플릿.
    - `experiment_name`: 실험 실행 이름 (로깅 및 추적에 사용됨).

---

### **2. 가중치 파일 경로 가져오기 (`get_weights_file_path`) 함수**

- **역할**: 설정 정보와 에포크 번호를 기반으로 모델 가중치 파일의 전체 경로를 생성합니다.
- **입력**:
    - `config`: `get_config` 함수에서 반환된 설정 딕셔너리.
    - `epoch`: 에포크 번호를 나타내는 문자열 (예: ‘1’, ‘2’ 등).
- **과정**: 현재 디렉토리 (`Path('.')`), 모델 폴더 (데이터 소스 및 모델 폴더 이름으로 구성), 모델 파일 이름 (모델 기본 이름과 에포크 번호로 구성)을 조합하여 파일 경로를 만듭니다.
- **반환**: 결과 파일 경로를 문자열로 반환합니다.

---

### **3. 최신 가중치 파일 경로 찾기 (`latest_weights_file_path`) 함수**

- **역할**: 지정된 모델 폴더에서 가장 최신 가중치 파일을 찾습니다.
- **입력**:
    - `config`: `get_config` 함수에서 반환된 설정 딕셔너리.
- **과정**:
    1. 모델 폴더 경로와 파일 이름 패턴(와일드카드 ‘*’ 포함)을 구성합니다.
    2. `Path.glob`를 사용하여 해당 패턴과 일치하는 모든 파일을 모델 폴더에서 검색합니다.
    3. 가중치 파일이 없으면 `None`을 반환합니다.
    4. 파일이 발견되면, 파일 이름을 기준으로 정렬한 후, 가장 최신 파일(정렬된 목록의 마지막 파일)의 경로를 반환합니다.
- **반환**: 가장 최신 가중치 파일의 경로를 문자열로 반환합니다.

# **Training (tformer > trainpy)**

```python
from model import build_transformer
from dataset import TranslationDataset, causal_mask
from config import get_config, get_weights_file_path, latest_weights_file_path

import torchtext.datasets as datasets
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split
from torch.optim.lr_scheduler import LambdaLR

import warnings
from tqdm import tqdm
import os
from pathlib import Path

# Huggingface datasets and tokenizers
from datasets import load_dataset
from tokenizers import Tokenizer
from tokenizers.models import WordLevel
from tokenizers.trainers import WordLevelTrainer
from tokenizers.pre_tokenizers import Whitespace

import torchmetrics
from torch.utils.tensorboard import SummaryWriter
```

### `get_all_sentences` 함수

```python
def get_all_sentences(ds, lang):
    for item in ds:
        yield item['translation'][lang]
```

**`get_all_sentences` 함수 요약:**

- **역할**: 주어진 데이터셋(`ds`)에서 특정 언어(`lang`)의 문장들을 하나씩 반환하는 제너레이터(generator) 함수입니다.
- **작동 방식**:
    - 데이터셋의 각 항목을 반복합니다.
    - 각 항목은 `translation`이라는 키를 가지고 있어야 하며, 이 키 아래에 여러 언어로 된 번역들이 포함되어 있습니다.
    - 지정된 `lang`에 해당하는 문장을 **`yield`** 키워드를 사용하여 하나씩 "생성"합니다.
- **장점**: `yield`를 사용함으로써, 모든 문장을 한꺼번에 메모리에 로드하는 대신 필요할 때마다 하나씩 생성하여 **메모리 효율성**을 높입니다.

### `get_or_build_tokenizer` 함수

```python
def get_or_build_tokenizer(config, ds, lang):
    tokenizer_path = Path(config['tokenizer_file'].format(lang))
    if not Path.exists(tokenizer_path):
        # Most code taken from: https://huggingface.co/docs/tokenizers/quicktour
        tokenizer = Tokenizer(WordLevel(unk_token="[UNK]"))
        tokenizer.pre_tokenizer = Whitespace()
        trainer = WordLevelTrainer(special_tokens=["[UNK]", "[PAD]", "[SOS]", "[EOS]"], min_frequency=2)
        tokenizer.train_from_iterator(get_all_sentences(ds, lang), trainer=trainer)
        tokenizer.save(str(tokenizer_path))
    else:
        tokenizer = Tokenizer.from_file(str(tokenizer_path))
    return tokenizer
```

`get_or_build_tokenizer` 함수 요약:

- **역할**: 주어진 언어에 대한 토크나이저 파일을 확인하고, 존재하지 않으면 새로 구축 및 학습하여 저장하고, 존재하면 기존 파일을 로드하여 반환합니다.
- **주요 구성 요소 및 과정**:
    1. **토크나이저 객체 생성**:
        - `Tokenizer(models.WordLevel(unk_token="[UNK]"))`: 새로운 `Tokenizer` 객체를 생성합니다. 단어 수준(WordLevel) 모델을 사용하며, 모르는 단어는 `[UNK]` 토큰으로 처리하도록 설정합니다.
    2. **전처리(Pre-tokenizer) 설정**:
        - `tokenizer.pre_tokenizer = pre_tokenizers.Whitespace()`: 텍스트를 공백(`whitespace`) 기준으로 분리하도록 전처리기를 설정합니다.
    3. **학습 도구(Trainer) 설정**:
        - `WordLevelTrainer(special_tokens=["[UNK]", "[PAD]", "[SOS]", "[EOS]"], min_frequency=2)`: `WordLevelTrainer`를 생성합니다.
            - **특수 토큰**: `[UNK]` (알 수 없는 단어), `[PAD]` (패딩), `[SOS]` (문장 시작), `[EOS]` (문장 끝)을 정의합니다.
            - **최소 빈도**: 최소 2번 이상 나타나는 단어만 어휘에 포함시키도록 설정합니다.
    4. **토크나이저 학습 (파일이 없는 경우)**:
        - `tokenizer.train_from_iterator(get_all_sentences(ds, lang), trainer=trainer)`: `get_all_sentences` 함수를 통해 데이터셋에서 해당 언어의 문장들을 하나씩 가져와 토크나이저를 학습시킵니다.
        - `tokenizer.save(str(tokenizer_path))`: 학습된 토크나이저를 지정된 파일 경로에 저장합니다.
    5. **토크나이저 로드 (파일이 있는 경우)**:
        - `Tokenizer.from_file(str(tokenizer_path))`: 이미 존재하는 토크나이저 파일에서 토크나이저를 로드합니다.
- **반환 값**: 새로 구축하고 학습했거나 기존 파일에서 로드된 토크나이저 객체를 반환합니다. 이 토크나이저는 이후 텍스트 처리 작업(예: 단어를 숫자로, 숫자를 단어로 변환)에 사용됩니다.

### `get_ds` 함수

```python
def get_ds(config):
    # It only has the train split, so we divide it overselves
    ds_raw = load_dataset(f"{config['datasource']}", f"{config['lang_src']}-{config['lang_tgt']}", split='train')

    # Build tokenizers
    tokenizer_src = get_or_build_tokenizer(config, ds_raw, config['lang_src'])
    tokenizer_tgt = get_or_build_tokenizer(config, ds_raw, config['lang_tgt'])

    # Keep 90% for training, 10% for validation
    train_ds_size = int(0.9 * len(ds_raw))
    val_ds_size = len(ds_raw) - train_ds_size
    train_ds_raw, val_ds_raw = random_split(ds_raw, [train_ds_size, val_ds_size])

    train_ds = TranslationDataset(train_ds_raw, tokenizer_src, tokenizer_tgt, config['lang_src'], config['lang_tgt'], config['seq'])
    val_ds = TranslationDataset(val_ds_raw, tokenizer_src, tokenizer_tgt, config['lang_src'], config['lang_tgt'], config['seq'])

    # Find the maximum length of each sentence in the source and target sentence
    max_len_src = 0
    max_len_tgt = 0

    for item in ds_raw:
        src_ids = tokenizer_src.encode(item['translation'][config['lang_src']]).ids
        tgt_ids = tokenizer_tgt.encode(item['translation'][config['lang_tgt']]).ids
        max_len_src = max(max_len_src, len(src_ids))
        max_len_tgt = max(max_len_tgt, len(tgt_ids))

    print(f'Max length of source sentence: {max_len_src}')
    print(f'Max length of target sentence: {max_len_tgt}')

    train_dataloader = DataLoader(train_ds, batch_size=config['batch_size'], shuffle=True)
    val_dataloader = DataLoader(val_ds, batch_size=1, shuffle=True)

    return train_dataloader, val_dataloader, tokenizer_src, tokenizer_tgt
```

**데이터 준비 및 로드 과정 요약:**

---

### **1. 데이터셋 로드**

- **설정**: 설정(`config`)에서 지정된 데이터 소스 이름과 번역 작업을 위한 언어 쌍(예: 'en-it')을 가져옵니다.
- **실행**: 해당 데이터 소스에서 데이터셋을 로드합니다. 이때, `split='train'`을 지정하여 **학습용 분할(training split)**만 로드합니다.

---

### **2. 토크나이저 구축**

- **설정**: 설정 딕셔너리, 이전에 로드된 원본 데이터셋, 그리고 원본 및 목표 언어를 `get_or_build_tokenizer` 함수에 전달합니다.
- **실행**: 이 함수는 해당 언어에 대한 토크나이저가 이미 존재하는지 확인하고, 없으면 새로 구축하고 학습시켜 저장하며, 존재하면 기존 토크나이저를 로드하여 반환합니다.

---

### **3. 데이터셋 분할**

- **실행**: 로드된 원본 데이터셋을 계산된 크기(예: 훈련용 90%, 검증용 10%)에 따라 **훈련 세트(training set)**와 **검증 세트(validation set)**로 무작위 분할합니다.

---

### **4. 번역 데이터셋 생성**

- **클래스 활용**: `dataset.py` 파일의 `TranslationDataset` 클래스를 호출합니다.
- **매개변수 전달**: 분할된 원본 훈련 및 검증 데이터셋, 원본 및 목표 언어 토크나이저를 전달합니다. 또한, 설정에서 가져온 원본 언어, 목표 언어, 그리고 시퀀스 길이(문장 최대 길이)도 함께 전달합니다.

---

### **5. 최대 문장 길이 찾기**

- **과정**: 원본 데이터셋의 각 항목을 반복하면서 원본 및 목표 문장을 인코딩하여 토큰 ID를 가져옵니다.
- **목적**: 현재 문장 길이가 기존의 최대 길이보다 길면 업데이트하며, 최종적으로 원본 및 목표 문장의 **최대 길이**를 출력합니다. (이는 모델 설계 또는 디버깅에 유용할 수 있습니다.)

---

### **6. 데이터 로더 생성**

- **역할**: 훈련 및 검증 데이터셋을 위한 반복 가능한 데이터 로더를 생성합니다.
- **설정**: 설정에서 훈련 데이터 로더의 배치 크기를 가져옵니다.
- **성능 향상**: 학습 성능 향상을 위해 데이터셋을 **섞습니다(shuffle)**.

---

### **7. 반환 값**

- **결과**: 최종적으로 훈련 데이터 로더, 검증 데이터 로더, 그리고 원본 및 목표 언어 토크나이저를 반환합니다. 이 결과물들은 트랜스포머 모델 학습에 직접 사용됩니다.

### `get_model` 함수

```python
def get_model(config, vocab_src_len, vocab_tgt_len):
    model = build_transformer(vocab_src_len, vocab_tgt_len, config["seq"], config['seq'], d_model=config['d_model'])
    return model
```

**트랜스포머 모델 빌드 함수 요약:**

---

`build_transformer` 함수는 트랜스포머 모델을 만드는 과정을 간단하게 감싸는(wrapping) 함수입니다.

- **역할**: 모델 생성의 복잡한 세부 사항을 숨기고, 설정(configuration)과 어휘 크기(vocabulary sizes)만 입력으로 받아 트랜스포머 모델을 준비시킵니다.
- **반환**: 학습 및 평가에 바로 사용할 수 있는 트랜스포머 모델 객체를 반환합니다.

### `train_model` 함수

```python
def train_model(config):
    # Define the device
    device = "cuda" if torch.cuda.is_available() else "mps" if torch.has_mps or torch.backends.mps.is_available() else "cpu"
    print("Using device:", device)
    if (device == 'cuda'):
        print(f"Device name: {torch.cuda.get_device_name(device.index)}")
        print(f"Device memory: {torch.cuda.get_device_properties(device.index).total_memory / 1024 ** 3} GB")
    elif (device == 'mps'):
        print(f"Device name: <mps>")
    else:
        print("NOTE: If you have a GPU, consider using it for training.")
        print("      On a Windows machine with NVidia GPU, check this video: https://www.youtube.com/watch?v=GMSjDTU8Zlc")
        print("      On a Mac machine, run: pip3 install --pre torch torchvision torchaudio torchtext --index-url https://download.pytorch.org/whl/nightly/cpu")
    device = torch.device(device)

    # Make sure the weights folder exists
    Path(f"{config['datasource']}_{config['model_folder']}").mkdir(parents=True, exist_ok=True)

    train_dataloader, val_dataloader, tokenizer_src, tokenizer_tgt = get_ds(config)
    model = get_model(config, tokenizer_src.get_vocab_size(), tokenizer_tgt.get_vocab_size()).to(device)
    # Tensorboard
    writer = SummaryWriter(config['experiment_name'])

    optimizer = torch.optim.Adam(model.parameters(), lr=config['lr'], eps=1e-9)

    # If the user specified a model to preload before training, load it
    initial_epoch = 0
    global_step = 0
    preload = config['preload']
    model_filename = latest_weights_file_path(config) if preload == 'latest' else get_weights_file_path(config, preload) if preload else None
    if model_filename:
        print(f'Preloading model {model_filename}')
        state = torch.load(model_filename)
        model.load_state_dict(state['model_state_dict'])
        initial_epoch = state['epoch'] + 1
        optimizer.load_state_dict(state['optimizer_state_dict'])
        global_step = state['global_step']
    else:
        print('No model to preload, starting from scratch')

    loss_fn = nn.CrossEntropyLoss(ignore_index=tokenizer_src.token_to_id('[PAD]'), label_smoothing=0.1).to(device)

    for epoch in range(initial_epoch, config['num_epochs']):
        torch.cuda.empty_cache()
        model.train()
        batch_iterator = tqdm(train_dataloader, desc=f"Processing Epoch {epoch:02d}")
        for batch in batch_iterator:

            encoder_input = batch['encoder_input'].to(device) # (b, seq)
            decoder_input = batch['decoder_input'].to(device) # (B, seq)
            encoder_mask = batch['encoder_mask'].to(device) # (B, 1, 1, seq)
            decoder_mask = batch['decoder_mask'].to(device) # (B, 1, seq, seq)

            # Run the tensors through the encoder, decoder and the projection layer
            encoder_output = model.encode(encoder_input, encoder_mask) # (B, seq, d_model)
            decoder_output = model.decode(encoder_output, encoder_mask, decoder_input, decoder_mask) # (B, seq, d_model)
            proj_output = model.project(decoder_output) # (B, seq, vocab_size)

            # Compare the output with the label
            label = batch['label'].to(device) # (B, seq)

            # Compute the loss using a simple cross entropy
            loss = loss_fn(proj_output.view(-1, tokenizer_tgt.get_vocab_size()), label.view(-1))
            batch_iterator.set_postfix({"loss": f"{loss.item():6.3f}"})

            # Log the loss
            writer.add_scalar('train loss', loss.item(), global_step)
            writer.flush()

            # Backpropagate the loss
            loss.backward()

            # Update the weights
            optimizer.step()
            optimizer.zero_grad(set_to_none=True)

            global_step += 1

        # Run validation at the end of every epoch
        run_validation(model, val_dataloader, tokenizer_src, tokenizer_tgt, config['seq'], device, lambda msg: batch_iterator.write(msg), global_step, writer)

        # Save the model at the end of every epoch
        model_filename = get_weights_file_path(config, f"{epoch:02d}")
        torch.save({
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'global_step': global_step
        }, model_filename)
```

**트랜스포머 모델 핵심 학습 로직 요약:**

이 함수는 트랜스포머 모델의 핵심 학습 로직을 담당합니다. 장치(CPU/GPU) 선택, 모델 초기화, 옵티마이저 설정, 학습 루프 실행, 그리고 Tensorboard를 이용한 학습 진행 상황 로깅 등을 처리합니다.

---

### **1. 장치 확인 및 설정**

- **판단**: "cuda"(GPU), "mps"(Multi-Process Service), 또는 "cpu" 중에서 사용 가능한 장치를 결정합니다.
- **정보 출력**: "cuda"를 사용하는 경우, 장치 정보를 출력합니다.

---

### **2. 모델 가중치 폴더 생성**

- **목적**: 모델 가중치를 저장할 폴더가 없는 경우 새로 생성합니다.

---

### **3. 데이터셋 가져오기**

- **처리**: 데이터셋을 로드하고, 이를 학습 세트와 검증 세트로 분할합니다.
- **도구 준비**: 원본 및 목표 언어에 대한 토크나이저를 구축합니다.

---

### **4. 모델 초기화**

- **생성**: `get_model` 함수를 사용하여 트랜스포머 모델을 생성합니다.
- **장치 이동**: 생성된 모델을 설정된 장치(GPU/CPU)로 이동시킵니다.

---

### **5. Tensorboard 초기화**

- **도구 생성**: 학습 진행 상황을 시각화하기 위해 `SummaryWriter`를 생성합니다.

---

### **6. 옵티마이저 초기화**

- **선택**: `Adam` 옵티마이저를 사용하며, 설정된 학습률(learning rate)로 초기화합니다.

---

### **7. 사전 학습된 모델 로드 (선택 사항)**

- **조건부 실행**: 설정(`config`)에 지정된 경우, 사전 학습된 모델 가중치와 옵티마이저 상태를 로드하여 이어서 학습할 수 있도록 합니다.

---

### **8. 손실 함수 정의**

- **선택**: `CrossEntropyLoss`를 사용하며, 레이블 스무딩(label smoothing)을 적용합니다.

---

### **9. 학습 루프 (`Training Loop`)**

- **에포크 반복**: 전체 에포크 수만큼 반복합니다.
- **배치 반복**: 각 에포크 내에서 학습 데이터 로더의 배치들을 반복 처리합니다.
- **순전파**: 인코더, 디코더, 그리고 투영(projection) 계층을 통해 순전파(forward pass)를 수행합니다.
- **손실 계산 및 역전파**: 손실을 계산하고, 역전파(backpropagation)를 통해 모델의 가중치를 업데이트합니다.
- **로깅**: Tensorboard를 사용하여 손실 값을 로깅합니다.
- **에포크 종료 시**:
    - 각 에포크가 끝날 때마다 **검증(validation)**을 실행하여 모델의 성능을 평가합니다.
    - 모델 가중치를 저장합니다.

# TensorBoard 실행

```bash
tensorboard --logdir=./BasicModel/Transformer/runs
```

# Tformer_Tutorial.ipynb 실습

# 참고자료

[Training a Transformer Model from Scratch](https://medium.com/@sayedebad.777/training-a-transformer-model-from-scratch-25bb270f5888)

https://github.com/ES7/Transformer-from-Scratch

```bash
cd /home/kime/ws_openvla ; /usr/bin/env /home/kime/miniconda3/envs/ws_mdl/bin/python /home/kime/ws_openvla/KIMe_BasicModel/Transformer/tformer/train.py 
```

##################################  

# Transformer — Python 코드 기반 실습 (훈련) Check Point

[Transformer_Train_CheckPT.zip](attachment:9de9c604-b327-46c2-8283-9ad828ec597b:Transformer_Train_CheckPT.zip)

폴더 `lemon-mint/weights`

[tmodel_00.pt](attachment:9fed9a68-d6bd-41ff-892d-1471f519304d:tmodel_00.pt)

[tmodel_10.pt](attachment:4ef90b1e-fb63-4df0-a2ba-ecf458c442a7:tmodel_10.pt)

[tmodel_20.pt](attachment:36574036-c477-4bbc-b0ba-c20e53058c9c:tmodel_10.pt)

[tmodel_29.pt](attachment:c55f499b-133e-4367-a78d-2dcd36dcb71b:tmodel_29.pt)

폴더 `runs/tmodel`

[events.out.tfevents.1748068267.kime-A6000.3472412.0](attachment:d89e2f0e-a605-4258-acbd-d3d26290db56:events.out.tfevents.1748068267.kime-A6000.3472412.0)

###############################  

# Diffusion 모델 기초 이론

**작성:** `초기` July 30, 2025 

# **Diffusion 모델 개요**

## **Generative vs. Discriminative**

- Generative models learn the data distribution

![image.png](attachment:66b704c2-6653-4a3a-83d8-e9bff8df1d1d:image.png)

## **Generative Models — Learning to generate data**

![image.png](attachment:832e53cd-6bca-443e-8dd7-cec9462bbc01:image.png)

## Different **Generative Models**

![image.png](attachment:c678bb9c-cf32-47eb-84fe-bcaa28ee33d5:image.png)

# **Diffusion 모델 기초 이론**

## Diffusion Models as Stacking VAEs

![image.png](attachment:1c53c049-398f-463a-8428-13da91daabd8:image.png)

![image.png](attachment:f6278021-546f-489b-9f22-d0ba44c79dad:image.png)

### Forward diffusion process — stacking fixed VAE encoders

- **Adding Gaussian noise according to schedule $𝛽_t$:**
    
    ![image.png](attachment:381605c6-7545-4aea-bd69-f4433341cade:image.png)
    
    **Forward Process (Step):** $q(\mathbf{x}_t | \mathbf{x}_{t-1}) = \mathcal{N}(\mathbf{x}_t; \sqrt{1 - \beta_t}\mathbf{x}_{t-1}, \beta_t \mathbf{I})$
    
    - $\mathcal{N}(\mathbf{x}t; \sqrt{1 - \beta_t}\mathbf{x}{t-1}, \beta_t \mathbf{I})$
        - 다변량 정규 분포(Multivariate Normal Distribution)
        - xt는 분포를 따르는 변수
        - μ는 분포의 평균(Mean) 벡터
        - Σ는 분포의 공분산(Covariance) 행렬
    
    **Forward Process (Total):** $q(\mathbf{x}_{1:T} | \mathbf{x}0) = \prod_{t=1}^{T} q(\mathbf{x}_t | \mathbf{x}_{t-1})$
    

- **Sampling of $x_𝑡$ at arbitrary timestep 𝑡 in closed form:**
    
    ![image.png](attachment:eedbac94-248c-40c3-b662-6c7ec5f5542c:c989a06f-72c0-4050-892e-e8c02bf6d569.png)
    
    ![image.png](attachment:584c34fa-8a32-44d6-947e-0b485eeccb5d:image.png)
    
    ![image.png](attachment:0161afdd-96b3-4517-aa52-6cfad97a7dcc:image.png)
    
    ![image.png](attachment:f643e7a6-8724-4ca1-8721-ebd22019b2a2:image.png)
    
    ![image.png](attachment:d99fb4f3-ea4b-4b2d-aa84-15af6ef6e592:image.png)
    

### Reverse denoising process — stacking `learnable` VAE decoders

![image.png](attachment:dac7f402-8b29-47ef-9794-8683e80374ef:image.png)

- Sample
    
    ![image.png](attachment:6e17c0e2-3855-4fb1-bdc7-6eb8394f2a11:image.png)
    
- Iteratively sample
    
    ![image.png](attachment:c0c5d3af-bfcb-46a7-8417-cf336b52417d:image.png)
    
- $q(\textbf{x}_{t-1}|\textbf{x}_{t})$ not directly tractable — But can be estimated with a Gaussian distribution if $𝛽_𝑡$ is small at each step
    
    ![image.png](attachment:77d45f03-da32-4f27-b01d-1a77d57dc0b4:image.png)
    
    ![image.png](attachment:9dda50c1-8c89-4b8f-9ed7-cfe6e7c52c65:image.png)
    
    ![image.png](attachment:06bdeaea-0a72-433a-b2ed-b51125deb740:image.png)
    
    ![image.png](attachment:d0988e32-2d48-48c2-871b-fbc7a696bab5:image.png)
    

### **Recap: Variational Autoencoders**

- **Solution:** having intermediate latent variables to reduce the gap of each step
    - Hierarchical VAEs (Stacking VAEs)
        - Each step, the decoder removes part of the noise
        - Provides a seed model closer to final distribution
            
            ![image.png](attachment:0a9b58ec-73ff-44db-bfcb-ca77fca5f8d3:image.png)
            
            ![image.png](attachment:5716afac-7559-41d2-a678-1880ad55c6be:image.png)
            
        - Each step incrementally recovers the final distribution
            
            ![image.png](attachment:cec49299-d73d-4f71-8565-cd233f45fbde:image.png)
            
- **Limitations of VAEs**
    - Decoder must transform a standard Gaussian all the way to the target distribution in one-step
        
        ![image.png](attachment:e3707ce3-4c97-4956-b42b-0ba1eb27ce43:image.png)
        
        - Often too large a gap
        - Blurry results are generated
            
            ![image.png](attachment:e023fdf3-358c-4fe8-a7c7-0792cc95d7a3:image.png)
            
- Training: maximize the ELBO (Evidence Lower Bound)
    
    ![image.png](attachment:8efb52de-60fc-419d-be30-cf0662188500:image.png)
    
    ![image.png](attachment:05c242e4-df15-42b1-9a55-ebd50a91c08a:image.png)
    
    ![image.png](attachment:ef21f705-7709-4efa-a448-bc59789af889:image.png)
    
- Decoder: a generative model that transforms a Gaussian variable z to real data
- Encoder: an inference model that approximates the posterior 𝑞(𝑧|𝑥)
- VAEs: a likelihood-based generative model

### Diffusion models are special cases of Stacking VAEs

![image.png](attachment:799d8ab2-fbaa-41b9-9ce0-052a8090efd9:image.png)

- Diffusion models use fixed inference encoders
    
    ![image.png](attachment:82d2dfee-ab70-421c-8d6f-dbf87e117c7d:image.png)
    
- In VAEs, encoders are learned with KL-divergence between the posterior and the prior
- Suffers from the ‘posterior-collapse’ issue :
    - Posterior collapse는 잠재 변수 $z$의 근사 후방 분포 $q(z|x)$가 사전 분포 $p(z)$에 지나치게 가까워져서 잠재 변수가 데이터의 정보를 거의 반영하지 않게 되는 현상입니다.
    - 잠재 변수 $z$가 데이터 $x$에 대해 독립적이 되어 의미 있는 표현을 학습하지 못합니다.
    - 모델이 생성 품질이 떨어지거나, 잠재 공간이 활용되지 않아 다양성이 감소합니다.

---

## Diffusion Models: Training, Sampling

- Final Objective
    
    ![image.png](attachment:dc8d42ee-9a04-425b-9fb5-121d8e561614:image.png)
    
- In DDPM, simplified version
    
    ![image.png](attachment:85811f63-0c74-4a1f-a542-fc36b1e5b008:image.png)
    
    ![image.png](attachment:833dd5ab-d7a2-4017-bb3b-cd9b6ba8dcb3:image.png)
    

---

## Many Steps in Diffusion (Sampling)

- Slow in generation
- In Training, we randomly sample one time step
- But in inference, we must transit from T to 0
    - 1000 steps
    - extremely slow for raw images/signals
        
        ![image.png](attachment:fe5bac93-8d07-4df4-868b-5886803f95d7:image.png)
        
        ![image.png](attachment:d93ba831-a496-47f3-a59d-4be123696d90:image.png)
        
        ![image.png](attachment:166f718c-2f7e-4d23-8808-caea88ce102b:image.png)
        
- DDIM with Fewer Steps Sampling
    
    ![image.png](attachment:9ab7b49f-d2aa-48f4-a7ae-184fec0732f3:image.png)
    

### DDPM vs DDIM 차이점

Diffusion 모델에서 샘플 생성을 위한 **reverse diffusion process**를 방법

- **DDPM**(Denoising Diffusion Probabilistic Models)
- **DDIM**(Denoising Diffusion Implicit Models)

하지만 두 모델은 **샘플링 과정의 방식**과 **속도**, 그리고 **샘플 다양성** 측면에서 중요한 차이를 가지고 있음.

---

### ✅ 1. DDPM (Denoising Diffusion Probabilistic Models)

**제안자**: Ho et al. (2020)

**핵심 아이디어**:

DDPM은 forward 과정에서 점점 더 노이즈를 추가해가며 데이터를 정규 분포로 만든 뒤, 이를 역으로 제거하는 **확률적 reverse process**를 통해 샘플을 생성함.

### 🔁 Forward Process:

- 데이터 $x_0$에 점진적으로 노이즈를 추가해서 $x_1, x_2, ..., x_T$까지 생성
- 노이즈는 다음 식으로 추가됨:
    
    $q(x_t | x_{t-1}) = \mathcal{N}(x_t; \sqrt{1 - \beta_t} x_{t-1}, \beta_t I)$
    

### 🔄 Reverse Process:

- 학습된 신경망 $\epsilon_\theta(x_t, t)$이 $x_t$에서 원래의 noise $\epsilon$을 예측
- 역방향 샘플링은 다음과 같은 **확률적** 형태:
    
    $p_\theta(x_{t-1} | x_t) = \mathcal{N}(x_{t-1}; \mu_\theta(x_t, t), \Sigma_\theta(x_t, t))$
    
- 한 스텝마다 Gaussian 샘플링이 있음 → **느림**

### ❗️⭐특징:

- **높은 샘플 품질**
- **샘플링이 느림** (보통 1000 스텝 이상 필요)
- **다양성 높음** (확률적 과정이므로 다양한 샘플 가능)

---

### ✅ 2. DDIM (Denoising Diffusion Implicit Models)

**제안자**: Song et al. (2021)

**핵심 아이디어**:

DDIM은 DDPM과 동일한 학습 방식을 유지하면서도, 샘플링을 **결정론적(Deterministic)**으로 바꿔서 더 빠르게 만드는 방식

### 🔁 Forward Process:

- DDPM과 동일하게 진행

### 🔄 Reverse Process:

- 확률적 Gaussian 샘플링이 아닌, **ODE 기반의 결정론적 역방향 과정**
- 다음과 같이 noise-free trajectory를 통해 샘플링:
    
    $x_{t-1} = \sqrt{\bar{\alpha}_{t-1}} \left( \frac{x_t - \sqrt{1 - \bar{\alpha}_t} \cdot \epsilon_\theta(x_t, t)}{\sqrt{\bar{\alpha}_t}} \right) + \sqrt{1 - \bar{\alpha}_{t-1}} \cdot \epsilon_\theta(x_t, t)$
    
- **Gaussian noise를 제거**해 결정론적 결과 생성
- **샘플링 스텝 수를 줄일 수 있음** (e.g., 1000 → 50 스텝 등)

### ❗️⭐특징:

- **샘플링 속도 빠름** (수십 스텝으로도 고품질 샘플 생성 가능)
- **결정론적 생성** (같은 조건이면 항상 같은 샘플)
- **다양성 부족** → 하지만 optional noise 추가로 다양성 회복 가능

---

### 🔍 핵심 비교 요약

| 항목 | **DDPM** | **DDIM** |
| --- | --- | --- |
| 샘플링 방식 | 확률적 (Stochastic) | 결정론적 (Deterministic) |
| 샘플링 속도 | 느림 (보통 1000 스텝) | 빠름 (10~50 스텝 가능) |
| 다양성 | 높음 (매번 다른 결과) | 낮음 (같은 조건이면 같은 결과) |
| 학습 과정 | 동일 (noise prediction 방식) | 동일 |
| 활용성 | 고품질이지만 느림 | 트레이드오프 조절 가능 (품질 vs 속도) |

# **Diffusion 모델 — 실습 키 포인트**

- Sampling
- Network (UNet)
- Training
- Controling (Context)
- Speeding Up (DDPM vs DDIM)

## **Diffusion 모델 — Why?**

![image.png](attachment:fdf8c017-2fca-4f7a-a937-697b95414794:image.png)

![image.png](attachment:2d29c96a-9caa-4173-9c77-c0621676ae8b:image.png)

![image.png](attachment:ae147e44-ba18-44e7-9d52-078d7d1d4e30:image.png)

![image.png](attachment:324fd25f-73a9-4422-bb64-e438d4f529d7:image.png)

## **Diffusion 모델 —** Sampling

![image.png](attachment:1257f4db-7363-45e2-95e6-59144b668d75:image.png)

![image.png](attachment:75ee2741-a7f8-4efd-98ad-668d44a2126d:image.png)

![image.png](attachment:39381f54-7fe0-44eb-a86a-53013cb5b427:image.png)

## **Diffusion 모델 —** Neural Network

![image.png](attachment:05a3b71a-0043-4c20-be4b-e7054a51a404:image.png)

![image.png](attachment:fae5b6f9-244e-4084-af0d-500345053d2f:image.png)

## **Diffusion 모델 —** Training

![image.png](attachment:2238cf84-fc8c-4621-96de-170ae92e3ab7:image.png)

![image.png](attachment:edbced9f-d42a-4564-8395-1258a9083be3:image.png)

![image.png](attachment:c7a4961e-e048-4391-b458-5d8e2b55ae8a:image.png)

![image.png](attachment:fdc0143d-c60c-433a-a778-58c4bbc77828:image.png)

## **Diffusion 모델 —** Controlling

![image.png](attachment:3c7d545f-cd19-4e5e-8d78-8449ddc87286:image.png)

![image.png](attachment:5168403e-0f88-42a1-a659-7fe957365810:image.png)

![image.png](attachment:0576b914-c760-4618-95eb-31f2680b1dbc:image.png)

## **Diffusion 모델 —** Speeding Up

![image.png](attachment:c288281a-67fc-44f6-8891-cb88ba3d3ff0:image.png)

![image.png](attachment:8914f088-1821-40db-9bbf-6bb256a382fa:image.png)

---

# 참고자료

cs.cmu(F24) — lec24.diffusion.pdf — https://deeplearning.cs.cmu.edu/F24/document/slides/lec24.diffusion.pdf

####################################  

# Diffusion — Python 코드 기반 실습 (Sprite & MNIST)

# **Lab 1 - Sampling (L1_Sampling.ipynb)**

![image.png](attachment:10a2c874-b8fa-441d-be1c-1e8bd050d84f:9cfd4a18-be19-4b70-a4d0-c0cb28c74acf.png)

```python
class ContextUnet(nn.Module):
    def __init__(self, in_channels, n_feat=256, n_cfeat=10, height=28):  # cfeat - context features
        super(ContextUnet, self).__init__()

        # number of input channels, number of intermediate feature maps and number of classes
        self.in_channels = in_channels
        self.n_feat = n_feat
        self.n_cfeat = n_cfeat
        self.h = height  #assume h == w. must be divisible by 4, so 28,24,20,16...

        # Initialize the initial convolutional layer
        self.init_conv = ResidualConvBlock(in_channels, n_feat, is_res=True)

        # Initialize the down-sampling path of the U-Net with two levels
        self.down1 = UnetDown(n_feat, n_feat)        # down1 #[10, 256, 8, 8]
        self.down2 = UnetDown(n_feat, 2 * n_feat)    # down2 #[10, 256, 4, 4]
        
         # original: self.to_vec = nn.Sequential(nn.AvgPool2d(7), nn.GELU())
        self.to_vec = nn.Sequential(nn.AvgPool2d((4)), nn.GELU())

        # Embed the timestep and context labels with a one-layer fully connected neural network
        self.timeembed1 = EmbedFC(1, 2*n_feat)
        self.timeembed2 = EmbedFC(1, 1*n_feat)
        self.contextembed1 = EmbedFC(n_cfeat, 2*n_feat)
        self.contextembed2 = EmbedFC(n_cfeat, 1*n_feat)

        # Initialize the up-sampling path of the U-Net with three levels
        self.up0 = nn.Sequential(
            nn.ConvTranspose2d(2 * n_feat, 2 * n_feat, self.h//4, self.h//4), # up-sample  
            nn.GroupNorm(8, 2 * n_feat), # normalize                       
            nn.ReLU(),
        )
        self.up1 = UnetUp(4 * n_feat, n_feat)
        self.up2 = UnetUp(2 * n_feat, n_feat)

        # Initialize the final convolutional layers to map to the same number of channels as the input image
        self.out = nn.Sequential(
            nn.Conv2d(2 * n_feat, n_feat, 3, 1, 1), # reduce number of feature maps   #in_channels, out_channels, kernel_size, stride=1, padding=0
            nn.GroupNorm(8, n_feat), # normalize
            nn.ReLU(),
            nn.Conv2d(n_feat, self.in_channels, 3, 1, 1), # map to same number of channels as input
        )

    def forward(self, x, t, c=None):
        """
        x : (batch, n_feat, h, w) : input image
        t : (batch, n_cfeat)      : time step
        c : (batch, n_classes)    : context label
        """
        # x is the input image, c is the context label, t is the timestep, context_mask says which samples to block the context on

        # pass the input image through the initial convolutional layer
        x = self.init_conv(x)
        # pass the result through the down-sampling path
        down1 = self.down1(x)       #[10, 256, 8, 8]
        down2 = self.down2(down1)   #[10, 256, 4, 4]
        
        # convert the feature maps to a vector and apply an activation
        hiddenvec = self.to_vec(down2)
        
        # mask out context if context_mask == 1
        if c is None:
            c = torch.zeros(x.shape[0], self.n_cfeat).to(x)
            
        # embed context and timestep
        cemb1 = self.contextembed1(c).view(-1, self.n_feat * 2, 1, 1)     # (batch, 2*n_feat, 1,1)
        temb1 = self.timeembed1(t).view(-1, self.n_feat * 2, 1, 1)
        cemb2 = self.contextembed2(c).view(-1, self.n_feat, 1, 1)
        temb2 = self.timeembed2(t).view(-1, self.n_feat, 1, 1)
        #print(f"uunet forward: cemb1 {cemb1.shape}. temb1 {temb1.shape}, cemb2 {cemb2.shape}. temb2 {temb2.shape}")

        up1 = self.up0(hiddenvec)
        up2 = self.up1(cemb1*up1 + temb1, down2)  # add and multiply embeddings
        up3 = self.up2(cemb2*up2 + temb2, down1)
        out = self.out(torch.cat((up3, x), 1))
        return out

```

## 🧠 ContextUnet 클래스 정의

이 부분은 **컨텍스트 조건부 U-Net** 구조를 구현한 것입니다. 확산 모델에서 사용될 핵심 신경망입니다.

### 구조 요약:

- `ResidualConvBlock`: 잔차 구조로 특징 추출
- `UnetDown`, `UnetUp`: 다운샘플링, 업샘플링 경로
- `EmbedFC`: 타임스텝(timestep) 및 조건(context)을 임베딩
- 컨텍스트와 타임스텝 정보를 임베딩하여 중간 특성과 결합
- 마지막에 원본 이미지와 같은 차원의 출력을 생성

## Hyper Parameter 설정

```python
# hyperparameters

## diffusion hyperparameters
timesteps = 500
beta1 = 1e-4
beta2 = 0.02

## network hyperparameters
device = torch.device("cuda:0" if torch.cuda.is_available() else torch.device('cpu'))
n_feat = 64 # 64 hidden dimension feature
n_cfeat = 5 # context vector is of size 5
height = 16 # 16x16 image
save_dir = './weights/'

# training hyperparameters
batch_size = 100
n_epoch = 32
lrate=1e-3
```

## 확산 스케줄 계산

```python
## construct DDPM noise schedule
b_t = (beta2 - beta1) * torch.linspace(0, 1, timesteps + 1, device=device) + beta1
a_t = 1 - b_t
ab_t = torch.cumsum(a_t.log(), dim=0).exp()    
ab_t[0] = 1
```

노이즈 추가/제거에 사용되는 변수들을 계산합니다:

- `b_t`, `a_t`, `ab_t`: 각각 시간에 따른 노이즈 비율, 잔여 신호 비율, 누적 신호 보존 비율

## 노이즈 추가

```python
# removes the predicted noise (but adds some noise back in to avoid collapse)
def denoise_add_noise(x, t, pred_noise, z=None):
    if z is None:
        z = torch.randn_like(x)
    noise = b_t.sqrt()[t] * z
    mean = (x - pred_noise * ((1 - a_t[t]) / (1 - ab_t[t]).sqrt())) / a_t[t].sqrt()
    return mean + noise
```

## 샘플링 (DDPM)

```python
# sample using standard algorithm
@torch.no_grad()
def sample_ddpm(n_sample, save_rate=20):
    # x_T ~ N(0, 1), sample initial noise
    samples = torch.randn(n_sample, 3, height, height).to(device)  

    # array to keep track of generated steps for plotting
    intermediate = [] 
    for i in range(timesteps, 0, -1):
        print(f'sampling timestep {i:3d}', end='\r')

        # reshape time tensor
        t = torch.tensor([i / timesteps])[:, None, None, None].to(device)

        # sample some random noise to inject back in. For i = 1, don't add back in noise
        **z = torch.randn_like(samples) if i > 1 else 0**

        eps = nn_model(samples, t)    # predict noise e_(x_t,t)
        samples = denoise_add_noise(samples, i, eps, z)
        if i % save_rate ==0 or i==timesteps or i<8:
            intermediate.append(samples.detach().cpu().numpy())

    intermediate = np.stack(intermediate)
    return samples, intermediate
```

## 올바르지 않은 샘플링 (DDPM)

```python
**# incorrectly sample without adding in noise**
@torch.no_grad()
def sample_ddpm_incorrect(n_sample):
    # x_T ~ N(0, 1), sample initial noise
    samples = torch.randn(n_sample, 3, height, height).to(device)  

    # array to keep track of generated steps for plotting
    intermediate = [] 
    for i in range(timesteps, 0, -1):
        print(f'sampling timestep {i:3d}', end='\r')

        # reshape time tensor
        t = torch.tensor([i / timesteps])[:, None, None, None].to(device)

        # don't add back in noise
        **z = 0**

        eps = nn_model(samples, t)    # predict noise e_(x_t,t)
        samples = denoise_add_noise(samples, i, eps, z)
        if i%20==0 or i==timesteps or i<8:
            intermediate.append(samples.detach().cpu().numpy())

    intermediate = np.stack(intermediate)
    return samples, intermediate
```

# **Lab 2 - Training  (L2_Training.ipynb)**

## 데이터 로드

```python
# load dataset and construct optimizer
dataset = CustomDataset("./dataset/L2_sprites_1788_16x16.npy", "./dataset/L2_sprite_labels_nc_1788_16x16.npy", transform, null_context=False)
dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=1)
optim = torch.optim.Adam(nn_model.parameters(), lr=lrate)
```

## 이미지에 노이즈 추가

```python
# perturbs an image to a specified noise level
def perturb_input(x, t, noise):
    return ab_t.sqrt()[t, None, None, None] * x + (1 - ab_t[t, None, None, None]).sqrt() * noise
```

## Training 프로세스

```python
# training without context code (Need GPU)

# set into train mode
nn_model.train()

for ep in range(n_epoch):
    print(f'epoch {ep}')
    
    # linearly decay learning rate
    optim.param_groups[0]['lr'] = lrate*(1-ep/n_epoch)
    
    pbar = tqdm(dataloader, mininterval=2 )
    for x, _ in pbar:   # x: images
        optim.zero_grad()
        x = x.to(device)
        
        # perturb data
        noise = torch.randn_like(x)
        t = torch.randint(1, timesteps + 1, (x.shape[0],)).to(device) 
        x_pert = perturb_input(x, t, noise)
        
        # use network to recover noise
        pred_noise = nn_model(x_pert, t / timesteps)
        
        # loss is mean squared error between the predicted and true noise
        loss = F.mse_loss(pred_noise, noise)
        loss.backward()
        
        optim.step()

    # save model periodically
    if ep%4==0 or ep == int(n_epoch-1):
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
        torch.save(nn_model.state_dict(), save_dir + f"model_{ep}.pth")
        print('saved model at ' + save_dir + f"model_{ep}.pth")
```

# **Lab 3 - Context**

**조건부(Conditional) 이미지 생성**이며, Context를 반영한 생성 결과를 얻는 것이 목적임.

## Context를 추가한 Training 프로세스

```python
# training with context code (Need GPU)
# set into train mode
nn_model.train()

for ep in range(n_epoch):
    print(f'epoch {ep}')
    
    # linearly decay learning rate
    optim.param_groups[0]['lr'] = lrate*(1-ep/n_epoch)
    
    pbar = tqdm(dataloader, mininterval=2 )
    **for x, c in pbar:   # x: images  c: context**
        optim.zero_grad()
        x = x.to(device)
        **c = c.to(x)**
        
        # randomly mask out c
        context_mask = torch.bernoulli(torch.zeros(c.shape[0]) + 0.9).to(device)
        **c = c * context_mask.unsqueeze(-1)**
        
        # perturb data
        noise = torch.randn_like(x)
        t = torch.randint(1, timesteps + 1, (x.shape[0],)).to(device) 
        x_pert = perturb_input(x, t, noise)
        
        # use network to recover noise
        **pred_noise = nn_model(x_pert, t / timesteps, c=c)**
        
        # loss is mean squared error between the predicted and true noise
        loss = F.mse_loss(pred_noise, noise)
        loss.backward()
        
        optim.step()

    # save model periodically
    if ep%4==0 or ep == int(n_epoch-1):
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
        torch.save(nn_model.state_dict(), save_dir + f"context_model_{ep}.pth")
        print('saved model at ' + save_dir + f"context_model_{ep}.pth")
```

## Context를 추가한 Sampling 프로세스

```python
# sample with context using standard algorithm
@torch.no_grad()
def sample_ddpm_context(n_sample, context, save_rate=20):
    # x_T ~ N(0, 1), sample initial noise
    samples = torch.randn(n_sample, 3, height, height).to(device)  

    # array to keep track of generated steps for plotting
    intermediate = [] 
    for i in range(timesteps, 0, -1):
        print(f'sampling timestep {i:3d}', end='\r')

        # reshape time tensor
        t = torch.tensor([i / timesteps])[:, None, None, None].to(device)

        # sample some random noise to inject back in. For i = 1, don't add back in noise
        z = torch.randn_like(samples) if i > 1 else 0

        **eps = nn_model(samples, t, c=context)**    # predict noise e_(x_t,t, ctx)
        samples = denoise_add_noise(samples, i, eps, z)
        if i % save_rate==0 or i==timesteps or i<8:
            intermediate.append(samples.detach().cpu().numpy())

    intermediate = np.stack(intermediate)
    return samples, intermediate
```

## 특정 Context 기반 Sampling

```python
# user defined context
ctx = torch.tensor([
    # hero, non-hero, food, spell, side-facing
    [1,0,0,0,0],  
    [1,0,0,0,0],    
    [0,0,0,0,1],
    [0,0,0,0,1],    
    [0,1,0,0,0],
    [0,1,0,0,0],
    [0,0,1,0,0],
    [0,0,1,0,0],
]).float().to(device)
samples, _ = sample_ddpm_context(ctx.shape[0], ctx)
show_images(samples)
```

## 특정 Mixed Context 기반 Sampling

```python
# mix of defined context
ctx = torch.tensor([
    # hero, non-hero, food, spell, side-facing
    [1,0,0,0,0],      #human
    [1,0,0.6,0,0],    
    [0,0,0.6,0.4,0],  
    [1,0,0,0,1],  
    [1,1,0,0,0],
    [1,0,0,1,0]
]).float().to(device)
samples, _ = sample_ddpm_context(ctx.shape[0], ctx)
show_images(samples)
```

## 🔍모델 아키텍처 – `ContextUnet` 핵심 아이디어

- **U-Net 기반 구조**에 **timestep**과 **context label**을 embedding하여 이미지 생성에 반영.
- `context`는 클래스 정보를 담은 벡터 (예: one-hot)이며, 이를 통해 원하는 속성을 가진 이미지를 생성할 수 있음.

```python
x → init_conv → down1 → down2 → to_vec
                  ↓           ↓
              + time/context embeddings
                  ↓           ↓
               up0 → up1 → up2 → output

```

- `EmbedFC`: Fully Connected layer로 context와 timestep을 embedding
- `up1`, `up2`: context와 timestep의 embedding을 곱/합하여 추가 정보 반영
- context 없이 생성할 수도 있도록, `if c is None:` 처리

# **Lab 4 - Fast Sampling (DDIM)**

## DDIM Denois 프로세스

```python
# define sampling function for DDIM   
# removes the noise using ddim
def denoise_ddim(x, t, t_prev, pred_noise):
    ab = ab_t[t]
    ab_prev = ab_t[t_prev]
    
    x0_pred = ab_prev.sqrt() / ab.sqrt() * (x - (1 - ab).sqrt() * pred_noise)
    dir_xt = (1 - ab_prev).sqrt() * pred_noise

    return x0_pred + dir_xt
```

## DDIM Sampling 프로세스

```python
# sample quickly using DDIM
@torch.no_grad()
def sample_ddim(n_sample, n=20):
    # x_T ~ N(0, 1), sample initial noise
    samples = torch.randn(n_sample, 3, height, height).to(device)  

    # array to keep track of generated steps for plotting
    intermediate = [] 
    step_size = timesteps // n
    for i in range(timesteps, 0, -step_size):
        print(f'sampling timestep {i:3d}', end='\r')

        # reshape time tensor
        t = torch.tensor([i / timesteps])[:, None, None, None].to(device)

        eps = nn_model(samples, t)    # predict noise e_(x_t,t)
        samples = denoise_ddim(samples, i, i - step_size, eps)
        intermediate.append(samples.detach().cpu().numpy())

    intermediate = np.stack(intermediate)
    return samples, intermediate
```

## Context를 추가한 DDIM Sampling 프로세스

```python
# fast sampling algorithm with context
@torch.no_grad()
def sample_ddim_context(n_sample, context, n=20):
    # x_T ~ N(0, 1), sample initial noise
    samples = torch.randn(n_sample, 3, height, height).to(device)  

    # array to keep track of generated steps for plotting
    intermediate = [] 
    step_size = timesteps // n
    for i in range(timesteps, 0, -step_size):
        print(f'sampling timestep {i:3d}', end='\r')

        # reshape time tensor
        t = torch.tensor([i / timesteps])[:, None, None, None].to(device)

        eps = nn_model(samples, t, c=context)    # predict noise e_(x_t,t)
        samples = denoise_ddim(samples, i, i - step_size, eps)
        intermediate.append(samples.detach().cpu().numpy())

    intermediate = np.stack(intermediate)
    return samples, intermediate
```


############################  
# OpenVLA 및 LIBERO 환경 구축

**최초 작성:** May 19, 2025

**업데이트:** 1차 July 30, 2025

# **Installation — OpenVLA (Linux & Windows)**

- **환경 구성:** PyTorch 2.2.0, torchvision 0.17.0, transformers 4.40.1, tokenizers 0.19.1, timm 0.9.10, and flash-attn 2.5.5

### Set-up: Conda Env

```bash
# Create and activate conda environment
conda create -n openvla python=3.10 -y
conda activate openvla

# Install PyTorch.
# https://pytorch.org/get-started/locally/

# CUDA 12.1 (Conda)
conda install pytorch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 pytorch-cuda=12.1 -c pytorch -c nvidia -y 
# OR
# CUDA 12.1 (Pip) -Optional
pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu121 -q
```

### Set-up: Openvla Repo

```bash
# Clone and install the openvla repo
git clone https://github.com/openvla/openvla.git
cd openvla
pip install -e .
# openvla pip 설치이후 Pytorch CPU 버전으로 downgrade 현상 체크
# openvla pip 설치이후 CUDA 12.1 (Conda) 설치 추천
conda install nvidia/label/cuda-12.1.0::cuda-nvcc
```

### Set-up(for Ubuntu): Ninja & Flash Attention 2

```bash
# Install Flash Attention 2 for training (https://github.com/Dao-AILab/flash-attention)
#   =>> If you run into difficulty, try `pip cache remove flash_attn` first
pip install packaging ninja
# ninja --version; echo $?  # Verify Ninja --> should return exit code "0"
# Linux GCC 필수 (윈도우에서는 MSVC 필요) 
pip install "flash-attn==2.5.5" --no-build-isolation
```

### Set-up(for Windows): Ninja & Flash Attention 2

```bash
# 윈도우 flash-attn 설치 (이전 설치 진행 전)

# 1. nvcc 설치
# https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/index.html
# nvcc 실행 확인

# 2. (Option) MS Visual Studio 설치 필요 가능

```

![image.png](attachment:bd205b7c-0c87-4e40-91b6-6844d7d14631:image.png)

# Inference Test **— OpenVLA (Linux & Windows)**

**Source code: vla-scripts/extern/verify_openvla.py**

### VRAM 16GB 정도 — (BFLOAT16 + FLASH-ATTN MODE)

```python
# === BFLOAT16 + FLASH-ATTN MODE ===
print("[*] Loading in BF16 with Flash-Attention Enabled")
vla = AutoModelForVision2Seq.from_pretrained(
    MODEL_PATH,
    attn_implementation="flash_attention_2",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True,
).to(device)
```

```python
# === BFLOAT16 MODE ===
inputs = processor(prompt, image).to(device, dtype=torch.bfloat16)
```

### VRAM 6GB 정도 — (4-BIT QUANTIZATION MODE)

```bash
# If needed
sudo apt install python3-pip
# Install bitsandbytes
pip install bitsandbytes
```

```python
from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig
```

```python
# === 4-BIT QUANTIZATION MODE (`pip install bitsandbytes`) :: [~6GB of VRAM Passive || 7GB of VRAM Active] ===
print("[*] Loading in 4-Bit Quantization Mode")
vla = AutoModelForVision2Seq.from_pretrained(
    MODEL_PATH,
    attn_implementation="flash_attention_2",
    torch_dtype=torch.float16,
    quantization_config=BitsAndBytesConfig(load_in_4bit=True),
    low_cpu_mem_usage=True,
    trust_remote_code=True,
)
```

```python
# === 8-BIT/4-BIT QUANTIZATION MODE ===
inputs = processor(prompt, image).to(device, dtype=torch.float16)
```

<aside>
💡

**ERROR #1** - Related to “pip install bitsandbytes”

> A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.
Some module may need to rebuild instead e.g. with 'pybind11>=2.12'.
> 

**SOL**

```bash
# pip install again (to downgrade as numpy 1.26.4)
cd openvla
pip install -e .
```

</aside>

<aside>
💡

**ERROR #2**

> ValueError: `.to` is not supported for `4-bit` or `8-bit` bitsandbytes models. Please use the model as it is, since the model has already been set to the correct devices and casted to the correct `dtype`.
> 

**SOL**

```bash
pip install accelerate==1.1.1
```

</aside>

---

# Install — **LIBERO**

### Clone and install the LIBERO repo

```bash
git clone https://github.com/Lifelong-Robot-Learning/LIBERO.git
cd LIBERO
pip install -e .
cd ..
```

### Install other required packages for OpenVLA

```bash
cd openvla
pip install -r experiments/robot/libero/libero_requirements.txt
```

<aside>
💡

**ERROR #1** - Related to “pip install bitsandbytes”

> A module that was compiled using NumPy 1.x cannot be run in
NumPy 2.2.6 as it may crash. To support both 1.x and 2.x
versions of NumPy, modules must be compiled with NumPy 2.0.
Some module may need to rebuild instead e.g. with 'pybind11>=2.12'.
> 

**SOL**

```bash
# pip install again (to downgrade as numpy 1.26.4)
cd openvla
pip install -e .
```

</aside>

# **Launching LIBERO Evaluations (Only Linux)**

### Set-up — 4-bit 양자화 모드

`openvla/experiments/robot/libero` 폴더 > `run_libero_eval.py` 

GenerateConfig 클래스 > `load_in_4bit: bool = False` → `load_in_4bit: bool = True`

```bash
# Launch LIBERO-Spatial evals
python experiments/robot/libero/run_libero_eval.py \
  --model_family openvla \
  --pretrained_checkpoint openvla/openvla-7b-finetuned-libero-spatial \
  --task_suite_name libero_spatial \
  --center_crop True
  
python experiments/robot/libero/run_libero_eval.py --model_family openvla --pretrained_checkpoint openvla/openvla-7b-finetuned-libero-spatial --task_suite_name libero_spatial --center_crop True

# Launch LIBERO-Object evals
python experiments/robot/libero/run_libero_eval.py \
  --model_family openvla \
  --pretrained_checkpoint openvla/openvla-7b-finetuned-libero-object \
  --task_suite_name libero_object \
  --center_crop True

# Launch LIBERO-Goal evals
python experiments/robot/libero/run_libero_eval.py \
  --model_family openvla \
  --pretrained_checkpoint openvla/openvla-7b-finetuned-libero-goal \
  --task_suite_name libero_goal \
  --center_crop True

# Launch LIBERO-10 (LIBERO-Long) evals
python experiments/robot/libero/run_libero_eval.py \
  --model_family openvla \
  --pretrained_checkpoint openvla/openvla-7b-finetuned-libero-10 \
  --task_suite_name libero_10 \
  --center_crop True
```

<aside>
💡

Custom dataset folder

> Do you want to specify a custom path for the dataset folder? (Y/N): N
> 
</aside>

<aside>
💡

Error #1

```bash
pip install robosuite==1.4
```

</aside>

<aside>
💡

`robosuite` Installation for Windows ([참고](https://robosuite.ai/docs/installation.html#installing-on-windows))

https://robosuite.ai/docs/installation.html#installing-on-windows

</aside>

---

# 참고자료

[GitHub - openvla/openvla: OpenVLA: An open-source vision-language-action model for robotic manipulation.](https://github.com/openvla/openvla?tab=readme-ov-file#getting-started)

[CUDA Installation Guide for Microsoft Windows — Installation Guide Windows 12.9 documentation](https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/index.html)


################################  

# OpenVLA 소개

최초작성 : May 19, 2025 

[OpenVLA: An Open-Source Vision-Language-Action Model](https://openvla.github.io/)

[openvla_teaser_video.mp4](attachment:b5fbb0a8-8ea4-492e-9f2a-f8038a3f5076:openvla_teaser_video.mp4)

![image.png](attachment:e475d3cf-9573-4e59-9853-a85dea39d267:image.png)

![image.png](attachment:3a116deb-5858-4b79-b05d-d4400e95834d:image.png)

## **OpenVLA 소개 및 특징**

- **모델 개요**: 7B 파라미터 오픈소스 비전-언어-액션 모델(VLA)
- **데이터**: Open X-Embodiment 데이터셋의 970k 로봇 에피소드로 사전 훈련
- **성능**: 일반 로봇 조작 정책에서 새로운 최고 성능 달성
- **다기능성**: 여러 로봇 제어 기본 지원
- **적응성**: 파라미터 효율적 미세 조정으로 새로운 로봇 설정에 빠르게 적응
- **오픈소스**: OpenVLA 체크포인트 및 PyTorch 훈련 파이프라인 완전 공개
- **접근성**: HuggingFace에서 모델 다운로드 및 미세 조정 가능

###########################################  

# LIBERO 소개

최초 작성: May 19, 2025 

[LIBERO – LIBERO](https://libero-project.github.io/main.html)

![image.png](attachment:b0635f31-4c2a-4296-8ffa-a0cad642ae29:image.png)

## LIBERO 요약

- **(기본)** 사전 훈련된 후 배치 **(추가)** 인간 사용자와 함께 **평생 학습**하여 개인화된 구현 에이전트 필수
- **연구 과제**: 의사결정에서의 평생 학습(LLDM, lifelong learning in decision making) 중요
- **문제점**: LLDM 연구를 위한 적절한 테스트베드 부족
- **제안**: LIBERO, 평생 로봇 학습에 특화된 벤치마크 제공
    - **특징**: 지속적으로 확장 가능한 벤치마크
    - **목표**: 지식 전이를 연구하는 공통 플랫폼 제공
- **기대 효과**: 기계학습 및 로보틱스 커뮤니티가 새로운 평생 학습 알고리즘 개발 및 평가 가능

## LIBERO이 LLDM에 특화된 점

- **지속적 학습 환경 제공**: LIBERO는 로봇이 **인간과의 상호작용**을 통해 새로운 기술과 지식을 **지속적으로 학습**할 수 있는 **동적 테스트베드**를 제공. LLDM의 핵심인 시간에 걸친 적응과 개선을 지원.
- **지식 전이 초점**: LIBERO는 로봇이 **기존 지식**을 **새로운 작업**이나 환경에 **전이시키는 능력**을 **평가**하도록 설계됨. LLDM에서 필수적인 요소로, 로봇이 다양한 상황에서 효율적으로 학습할 수 있도록 도움.
- **확장 가능한 벤치마크**: LIBERO는 새로운 작업, 시나리오, 데이터셋을 추가하며 지속적으로 **확장 가능**함.
- **현실적 문제 반영**: LIBERO는 로봇이 실제 **인간과의 협업**에서 직면할 수 있는 **복잡한 의사결정 문제**를 포함함. 실세계에서의 평생 학습 요구사항을 충족하도록 설계됨.
- **커뮤니티 협업 촉진**: 알고리즘 비교·평가를 위한 **표준화된 플랫폼** 제공.

##################################  

FastAPI 구축 실습
최초 작성: @May 19, 2025​
업데이트: 1차 @July 30, 2025​
Serving OpenVLA models over a REST API (FastAPI) 
Install — FastAPI
pip install fastapi uvicorn
​
Test @ 4-bit 양자화 모드 & FastAPI Server
Source code: vla-scripts/deploy.py [참고]
수정: Import 구문
from transformers import AutoModelForVision2Seq, AutoProcessor, BitsAndBytesConfig
​
수정: class OpenVLAServer::__Init__
# self.vla = AutoModelForVision2Seq.from_pretrained(
#     self.openvla_path,
#     attn_implementation=attn_implementation,
#     torch_dtype=torch.bfloat16,
#     low_cpu_mem_usage=True,
#     trust_remote_code=True,
# ).to(self.device)

print("[*] Loading in 4-Bit Quantization Mode")
self.vla = AutoModelForVision2Seq.from_pretrained(
    self.openvla_path,
    attn_implementation=attn_implementation,
    torch_dtype=torch.float16,
    quantization_config=BitsAndBytesConfig(load_in_4bit=True),
    low_cpu_mem_usage=True,
    trust_remote_code=True,
)
​
수정: class OpenVLAServer::predict_action
# inputs = self.processor(prompt, Image.fromarray(image).convert("RGB")).to(self.device, dtype=torch.bfloat16)

# === 8-BIT/4-BIT QUANTIZATION MODE ===
inputs = self.processor(prompt, Image.fromarray(image).convert("RGB")).to(self.device, dtype=torch.float16)
​
실행
# at VS Code Terminal
conda activate openvla
cd ~/ws_openvla

python openvla/vla-scripts/deploy.py
​
Install — gradio
pip install gradio
​
Test @ Gradio 시각화
Source code: gradio_app.py
코드 — gradio_app.py
import gradio as gr
import requests
import json_numpy
import numpy as np
from PIL import Image

# Gradio 클라이언트와 서버 간 데이터 포맷 처리
json_numpy.patch()

# REST API 서버 엔드포인트
API_URL = "http://localhost:8000/act"

def predict_action(image, instruction, unnorm_key=None):
    # 업로드된 이미지를 numpy 배열로 변환
    image_array = np.array(image)

    # 요청 데이터(payload) 생성
    payload = {
        "image": image_array,
        "instruction": instruction,
    }

    if unnorm_key:
        payload["unnorm_key"] = unnorm_key

    # 서버에 POST 요청
    response = requests.post(API_URL, json=payload)
    
    # 서버 응답 확인
    if response.status_code == 200:
        return response.json()
    else:
        return f"Error {response.status_code}: {response.text}"

# Gradio 인터페이스 구성
with gr.Blocks() as gradio_app:
    gr.Markdown("# OpenVLA Robot Action Prediction")
    gr.Markdown(
        "Provide an image of the robot workspace and an instruction to predict the robot's action. "
        "You can either upload an image or provide a base64-encoded image via API."
    )

    with gr.Row():
        with gr.Column(scale=1):
            instruction_input = gr.Textbox(label="Instruction", placeholder="e.g., Pick up the remote")
            unnorm_key_input = gr.Textbox(label="Unnorm Key (Optional)", placeholder="e.g., bridge_orig")
            image_input = gr.Image(type="pil", label="Upload Image")
            submit_btn = gr.Button("Submit")

        with gr.Column(scale=1):
            output_action = gr.Textbox(label="Robot Action (X, Y, Z, Roll, Pitch, Yaw, Gripper)", interactive=False, lines=8)
    

    # 예측 함수 연결
    submit_btn.click(
        fn=predict_action,
        inputs=[image_input, instruction_input, unnorm_key_input],
        outputs=[output_action]
    )

    # 예제 제공
    gr.Examples(
        examples=[
            ["Place the red vegetable in the silver pot.", "bridge_orig", "./KIMe_OpenVLA/images/example1.jpeg"],
            ["Pick up the remote", "bridge_orig", "./OpenVLA_Tutorial/images/example2.jpeg"]
        ],
        inputs=[instruction_input, unnorm_key_input, image_input]
    )

gradio_app.launch()
​
실행
# at VS Code Terminal
conda activate openvla
cd ~/ws_openvla

python OpenVLA_Tutorial/gradio_app.py
​
Test Images

Place the red vegetable in the silver pot.

Pick up the remote.
############################  

# LIBERO+OpenVLA Jupyter 실습

# VSCode 설정 (lanuch.json & settings.json)

**lanuch.json**

```json
{
    // Use IntelliSense to learn about possible attributes.
    // Hover to view descriptions of existing attributes.
    // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python Debugger: Current File",
            "type": "debugpy",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "env": {"PYTHONPATH": "${workspaceFolder}${pathSeparator}${workspaceFolder}/openvla${pathSeparator}${env:PYTHONPATH}"}
        }
    ]
}
```

**settings.json**

```json
{
    "python.analysis.extraPaths": [
        "./openvla"
    ],
    "python.autoComplete.extraPaths": [
        "./openvla"
    ]
}
```

# Jupyter 실습

**OpenVLA_Tutorial 폴더 내**

- 01_LIBERO.ipynb
- 02_OpenVLA_on_LIBERO.ipynb
    
    ```bash
    # ipywidgets 모듈 오류시
    conda install -c conda-forge ipywidgets
    ```

##################################  

# GR00T N1.5 Tutorial

**작성 일자:** 최초 작성 July 30, 2025

---

# **Installation Guide**

1. Git Repo 복사

```bash
git clone https://github.com/NVIDIA/Isaac-GR00T
cd Isaac-GR00T
```

1. Conda 환경 구성 (Python 3.10)
    
    > ⚠️CUDA 12.4 버전 필수 (flash-attn 때문에)
    > 

```bash
conda create -n gr00t python=3.10
conda activate gr00t
pip install --upgrade setuptools
pip install -e .[base]
conda install nvidia/label/cuda-12.4.0::cuda-nvcc
pip install --no-build-isolation flash-attn==2.7.1.post4
```

# **Getting started with this repo**

- **Jupyter 노트북**과 상세 **문서**는 [`./getting_started`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started) 폴더에서 확인 가능
- **유틸리티 스크립트**는 [`./scripts`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/scripts) 폴더 있음.
- SO-101 로봇에서 모델을 미세 조정하는 방법 ([HuggingFace](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning) 참고)
    
    [Post-Training Isaac GR00T N1.5 for LeRobot SO-101 Arm](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning)
    

## **1. Data Format & Loading**

- 데이터를 로드하고 처리하기 위해 [Hugging Face의 LeRobot 데이터](https://github.com/huggingface/lerobot)를 사용하지만, 더 상세한 양식(modality) 및 주석 스키마("LeRobot 호환 데이터 스키마", "LeRobot compatible data schema")를 활용함.
- LeRobot 데이터셋의 예시는 `./demo_data/robot_sim.PickNPlace`에 저장되어 있음. 추가적인 [`modality.json`](https://www.google.com/search?q=%5Bhttps://github.com/NVIDIA/Isaac-GR00T/blob/main/demo_data/robot_sim.PickNPlace/meta/modality.json%5D(https://github.com/NVIDIA/Isaac-GR00T/blob/main/demo_data/robot_sim.PickNPlace/meta/modality.json)) 파일도 포함됨.
- 데이터셋 형식에 대한 자세한 설명은 [`getting_started/LeRobot_compatible_data_schema.md`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/LeRobot_compatible_data_schema.md) 참고
- [`EmbodimentTag`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/4_deeper_understanding.md#embodiment-action-head-fine-tuning)을 통해 multiple embodiments  지원
- `LeRobotSingleDataset` 클래스를 사용하여 데이터를 로드 가능
    
    ```python
    from gr00t.data.dataset import LeRobotSingleDataset
    from gr00t.data.embodiment_tags import EmbodimentTag
    from gr00t.data.dataset import ModalityConfig
    from gr00t.experiment.data_config import DATA_CONFIG_MAP
    
    # get the data config
    data_config = DATA_CONFIG_MAP["fourier_gr1_arms_only"]
    
    # get the modality configs and transforms
    modality_config = data_config.modality_config()
    transforms = data_config.transform()
    
    # This is a LeRobotSingleDataset object that loads the data from the given dataset path.
    dataset = LeRobotSingleDataset(
        dataset_path="demo_data/robot_sim.PickNPlace",
        modality_configs=modality_config,
        transforms=None,  # we can choose to not apply any transforms
        embodiment_tag=EmbodimentTag.GR1, # the embodiment to use
    )
    
    # This is an example of how to access the data.
    dataset[5]
    ```
    
- GR00T N1.5 모델과 연동하기 위해 데이터를 로드하고 처리하는 방법 **튜토리얼**
- [`getting_started/0_load_dataset.ipynb`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/0_load_dataset.ipynb)
- 위와 동일한 내용의 **실행 가능한 스크립트**
    - [`scripts/load_dataset.py`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/scripts/load_dataset.py)
- Dataset 로드 실행
    
    ```bash
    python scripts/load_dataset.py --dataset-path ./demo_data/robot_sim.PickNPlace
    ```
    

## **2. Inference**

- GR00T N1.5 모델: [Huggingface](https://huggingface.co/nvidia/GR00T-N1.5-3B)
- 교차 구현(cross embodiment) dataset 예시: [demo_data/robot_sim.PickNPlace](https://github.com/NVIDIA/Isaac-GR00T/blob/main/demo_data/robot_sim.PickNPlace)

### **2.1 Inference with PyTorch**

```python
from gr00t.model.policy import Gr00tPolicy
from gr00t.data.embodiment_tags import EmbodimentTag

# 1. Load the modality config and transforms, or use above
modality_config = ComposedModalityConfig(...)
transforms = ComposedModalityTransform(...)

# 2. Load the dataset
dataset = LeRobotSingleDataset(.....<Same as above>....)

# 3. Load pre-trained model
policy = Gr00tPolicy(
    model_path="nvidia/GR00T-N1.5-3B",
    modality_config=modality_config,
    modality_transform=transforms,
    embodiment_tag=EmbodimentTag.GR1,
    device="cuda"
)

# 4. Run inference
action_chunk = policy.get_action(dataset[0])
```

- 추론 파이프라인을 구축 Jupyter 노트북 튜토리얼:
    - [`getting_started/1_gr00t_inference.ipynb`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/1_gr00t_inference.ipynb)
- **추론 서비스** 실행 가능
    - **서버 모드**
        
        ```
        python scripts/inference_service.py --model-path nvidia/GR00T-N1.5-3B --server
        ```
        
    - **클라이언트 모드** (to send requests to the server)
        
        ```
        python scripts/inference_service.py  --client
        ```
        

### **2.2 Inference with Python TensorRT (Optional)**

- ONNX 및 TensorRT를 사용하여 추론하려면, [`deployment_scripts/README.md`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/deployment_scripts/README.md) 참조

## **3. Fine-Tuning**

- **미세 조정(finetuning) Jupyter 노트북 :** [`getting_started/2_finetuning.ipynb`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/2_finetuning.ipynb)
- 미세 조정(finetuning) 스크립트 실행:
    
    ```bash
    # first run --help to see the available arguments
    python scripts/gr00t_finetune.py --help
    
    # then run the script
    python scripts/gr00t_finetune.py --dataset-path ./demo_data/robot_sim.PickNPlace --num-gpus 1
    ```
    
    **Note**: **RTX 4090** 그래픽 카드를 사용하여 모델을 미세 조정(finetuning)하는 경우, `gr00t_finetune.py`를 실행할 때 **`--no-tune_diffusion_model`** 플래그를 반드시 추가해야 함. 이 플래그를 사용하지 않으면 **CUDA 메모리 부족(CUDA out of memory)** 오류가 발생할 수 있음.
    
- Hugging Face Sim 데이터 릴리즈에서 **샘플 데이터셋**을 [다운로드](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim)
    
    ```bash
    huggingface-cli download  nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim \
      --repo-type dataset \
      --include "gr1_arms_only.CanSort/**" \
      --local-dir $HOME/gr00t_dataset
    ```
    

- 모델 미세 조정 시, **배치 크기를 최대로 늘리고 20k 스텝 동안 학습**을 권장
- 하드웨어 성능 고려 사항
    - **미세 조정 성능**: 최적의 미세 조정을 위해 **H100 노드 1개 또는 L40 노드 1개**를 사용함. A6000, RTX 4090과 같은 다른 하드웨어 구성도 작동하지만, 수렴하는 데 더 오래 걸릴 수 있음.
    - **LoRA 미세 조정**: LoRA 미세 조정을 위해 **A6000 GPU 2개 또는 RTX 4090 GPU 2개**를 사용함.
    - **추론 성능**: 실시간 추론의 경우, 대부분의 최신 GPU는 단일 샘플 처리시 비슷한 성능을 보임. 벤치마크 결과, L40과 RTX 4090 간의 추론 속도 차이는 미미함.
- 새로운 형태(new embodiment)의 미세 조정에 대한 내용은 [`getting_started/3_0_new_embodiment_finetuning.md`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/3_0_new_embodiment_finetuning.md) 참고

### **Choosing the Right Embodiment Head**

![robots-banner](https://github.com/NVIDIA/Isaac-GR00T/raw/main/media/robots-banner.png)

**GR00T N1.5는 다양한 로봇 구성에 최적화된 세 가지 사전 학습된 embodiment head를 제공함.**

- **`EmbodimentTag.GR1`**: 절대 조인트 공간 제어(absolute joint space control)를 사용하는 **손**을 가진 휴머노이드 로봇
- **`EmbodimentTag.OXE_DROID`**: EEF(delta end-effector) 제어를 사용하는 단일 암 로봇
- **`EmbodimentTag.AGIBOT_GENIE1`**: 절대 조인트 공간 제어를 사용하는 **그리퍼**를 가진 휴머노이드 로봇
- **`EmbodimentTag.NEW_EMBODIMENT`**: (사전 학습되지 않음) 새로운 로봇 embodiment에 대한 미세 조정
- 관측 및 동작 공간에 대한 자세한 정보는 [`EmbodimentTag`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/4_deeper_understanding.md#embodiment-action-head-fine-tuning) 참고

### **Sim Env: [robocasa-gr1-tabletop-tasks](https://github.com/robocasa/robocasa-gr1-tabletop-tasks)**

- Sample dataset for finetuning: [다운로드](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim)
- For Simulation Evaluation, [robocasa-gr1-tabletop-tasks](https://github.com/robocasa/robocasa-gr1-tabletop-tasks) 참고

## **4. Evaluation**

- 모델의 오프라인 평가를 수행하기 위해, 데이터셋에 대해 모델을 평가하고 그 결과를 시각적으로 보여주는 스크립트
    
    ```bash
    python scripts/eval_policy.py --plot --model_path nvidia/GR00T-N1.5-3B
    ```
    
- 또는, 새로 학습된 모델을 **클라이언트-서버 모드**로 실행 가능
    
    **새로 훈련된 모델 실행:**
    
    ```
    python scripts/inference_service.py --server \
        --model-path <MODEL_PATH> \
        --embodiment-tag new_embodiment
        --data-config <DATA_CONFIG>
    ```
    
    **Offline Evaluation 스크립트 실행:**
    
    ```
    python scripts/eval_policy.py --plot \
        --dataset-path <DATASET_PATH> \
        --embodiment-tag new_embodiment \
        --data-config <DATA_CONFIG>
    ```
    
    - Offline Evaluation 스크립트를 실행하면 **실제 값 (Ground Truth)과 예측된 행동 (Predicted actions) 간의 그래프**을 볼 수 있음. **행동의 정규화되지 않은 MSE (평균 제곱 오차)** 값도 제공됨.
    - 이 결과는 **정책(policy)이 해당 데이터셋에서 얼마나 잘 작동하는지**에 대한 좋은 지표가 됨.

# **Jetson Deployment**

A detailed guide for deploying GR00T N1.5 on Jetson is available in [`deployment_scripts/README.md`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/deployment_scripts/README.md).

Here's comparison of E2E performance between PyTorch and TensorRT on Orin

- Jetson에 GR00T N1.5를 배포하는 자세한 가이드는 [`deployment_scripts/README.md`](https://github.com/NVIDIA/Isaac-GR00T/blob/main/deployment_scripts/README.md) 참고
- Orin에서 **PyTorch와 TensorRT 간의 E2E(End-to-End) 성능 비교**
    
    ![orin-perf](https://github.com/NVIDIA/Isaac-GR00T/raw/main/media/orin-perf.png)
    
- 모델 지연 시간은 `trtexec`를 사용하여 배치 크기 1로 측정되었음.
    
    
    | **Model Name** | **Orin benchmark perf (ms)** | **Precision** |
    | --- | --- | --- |
    | Action_Head - process_backbone_output | 5.17 | FP16 |
    | Action_Head - state_encoder | 0.05 | FP16 |
    | Action_Head - action_encoder | 0.20 | FP16 |
    | Action_Head - DiT | 7.77 | FP16 |
    | Action_Head - action_decoder | 0.04 | FP16 |
    | VLM - ViT | 11.96 | FP16 |
    | VLM - LLM | 17.25 | FP16 |
    
    **참고:** 파이프라인 내 **모듈 지연 시간**(예: DiT 블록)은 위의 벤치마크 표에 있는 **모델 지연 시간보다 약간 더 길음.** 이는 해당 모듈(예: Action_Head - DiT)의 지연 시간이 표의 모델 지연 시간뿐만 아니라 **PyTorch에서 TensorRT로 데이터를 전송하고 다시 TensorRT에서 PyTorch로 결과를 반환하는 오버헤드**까지 포함하기 때문임.
    

---

# 참고자료

[NVIDIA Isaac GR00T](https://developer.nvidia.com/isaac/gr00t)

https://github.com/NVIDIA/Isaac-GR00T/

https://github.com/NVIDIA/Isaac-GR00T/

[Post-Training Isaac GR00T N1.5 for LeRobot SO-101 Arm](https://huggingface.co/blog/nvidia/gr00t-n1-5-so101-tuning)

  



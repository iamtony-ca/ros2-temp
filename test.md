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

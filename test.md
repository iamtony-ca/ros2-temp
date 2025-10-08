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

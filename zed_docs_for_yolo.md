제공해주신 공식 문서는 **ZED SDK와 최신 YOLO 모델(Custom 포함)을 결합하여, 단순 2D 검출을 넘어 3D 위치 추적(Localization & Tracking)까지 구현하는 방법**을 설명하고 있습니다.

사용자님의 환경(Jetson AGX Orin, ROS 2, Custom Model)에 맞춰, 해당 문서의 핵심 내용을 상세하게 분석해 드립니다.

---

### 1. 개요 (Introduction)

이 패키지의 핵심은 **"2D YOLO + ZED Depth = 3D Object Detection"**입니다.

* **기능:** YOLO 모델(딥러닝)이 이미지에서 물체의 2D 박스()를 찾으면, ZED SDK가 이를 받아 Depth Map과 융합하여 **3D 좌표()와 3D Bounding Box**를 계산하고, 물체가 움직여도 놓치지 않도록 **트래킹(Tracking)**까지 수행합니다.
* **성능:** **TensorRT** 프레임워크를 기반으로 동작하도록 설계되어 있습니다. 사용자님의 **Jetson AGX Orin**에서 극강의 추론 속도(Inference Speed)를 보장합니다.
* **지원 모델:** YOLOv5, v6, v7, v9, v10, v11, v12 등 대부분의 최신 YOLO 아키텍처를 지원합니다.
* *핵심:* 꼭 공식 COCO 데이터셋 모델이 아니어도 됩니다. **같은 아키텍처(구조)를 사용하여 사용자님만의 데이터로 학습시킨 Custom Model**도 완벽하게 호환됩니다.



### 2. 설치 요구사항 (Installation)

사용자님의 Jetson 환경에는 이미 대부분 설치되어 있을 것입니다.

* **ZED SDK & Python API:** 기본 설치됨.
* **TensorRT:** JetPack에 포함되어 있으며, ZED SDK 설치 시 AI 모듈과 연동됩니다.
* **OpenCV & CUDA:** Jetson 필수 라이브러리로 이미 설치되어 있음.

### 3. 작업 워크플로우 (Workflow) - **[가장 중요]**

ZED SDK가 YOLO 모델을 구동하는 방식은 크게 두 가지가 있습니다. 문서에서는 **첫 번째 방식(Recommended)**을 강력히 권장합니다.

#### **방식 1: [Recommended] Native YOLO Loading (`CUSTOM_YOLOLIKE_BOX_OBJECTS`)**

이 방식은 **"ZED SDK가 알아서 다 해주는 모드"**입니다.

* **동작 원리:**
1. 사용자님은 학습된 모델을 **ONNX 포맷**으로 변환하여 ZED SDK에 경로만 던져줍니다.
2. ZED SDK 내부에서 이 ONNX 파일을 읽고, Jetson에 최적화된 **TensorRT 엔진 파일(.engine)을 자동으로 생성**합니다.
3. 추론(Inference)부터 3D 변환까지 SDK 내부 파이프라인에서 최적화된 상태로 돌아갑니다.


* **설정 파라미터:** `OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS`
* **장점:** 별도의 Python/C++ 추론 코드를 짤 필요가 없습니다. 성능이 가장 빠르고 구현이 쉽습니다.
* **호환성:** YOLO 아키텍처의 출력 텐서 구조를 자동으로 분석하여 동작하므로, 지원되는 버전(v5~v12)과 구조만 같다면 Custom Model도 바로 작동합니다.

#### **방식 2: [Advanced] External Inference (Custom Ingest)**

이 방식은 **"내가 직접 추론하고 결과만 ZED에게 넘겨주는 모드"**입니다.

* **동작 원리:**
1. 사용자님이 별도의 코드(예: `ultralytics` 라이브러리 사용)로 이미지를 받아 추론을 돌립니다.
2. 나온 2D 박스 결과를 `zed.ingest_custom_box_objects()` 함수를 통해 ZED SDK에 주입(Ingest)합니다.
3. ZED SDK는 받은 2D 박스에 Depth만 입혀서 3D로 만듭니다.


* **언제 쓰는가?:** ZED SDK가 자동으로 지원하지 않는 아주 특이한 모델 구조를 쓰거나, 추론 전후에 복잡한 전처리가 필요할 때 사용합니다. (YOLO를 쓰신다면 굳이 이 방식을 쓸 필요는 없습니다.)

### 4. 필수 준비 단계: ONNX 내보내기 (Exporting to ONNX)

ZED SDK는 PyTorch 모델 파일(`.pt`)을 직접 읽지 못합니다. 반드시 **ONNX 포맷(`.onnx`)**으로 변환해야 합니다.

* **절차:** 학습이 완료된 Custom YOLO 모델(`.pt`)을 YOLO 공식 리포지토리의 `export.py` 등을 사용하여 ONNX로 변환합니다.
* **주의사항:** 내보낼 때 ZED SDK가 이해할 수 있는 형태여야 합니다. (보통은 기본 export 옵션을 사용하면 되지만, 때로는 NMS(Non-Maximum Suppression) 레이어를 포함할지 말지 옵션을 신경 써야 할 수도 있습니다. 이는 다음 단계 문서인 "YOLO ONNX Export"에서 다룹니다.)

### 5. 요약 및 사용자님을 위한 조언

사용자님은 현재 **ROS 2** 환경에서 작업 중이시므로, 이 문서의 내용은 **`zed_wrapper`의 파라미터 설정**과 직결됩니다.

1. **목표:** Custom YOLO 모델 학습 완료 (`best.pt`).
2. **변환:** `best.pt` → `best.onnx` 로 변환. (Opset version 등 호환성 확인 필요)
3. **ROS 2 적용 (방식 1 사용):**
* `zed_wrapper` 파라미터에서 `object_detection_model`을 **`CUSTOM_YOLOLIKE`** (또는 유사한 명칭)로 설정.
* `onnx_model_path`에 변환한 `best.onnx` 파일 경로 지정.


4. **실행:** 노드를 실행하면, 처음 1회 실행 시 터미널에 "Building TensorRT engine..." 같은 로그가 뜨면서 몇 분 정도 걸립니다. 그 이후에는 실시간으로 Custom Object Detection + 3D Tracking이 수행됩니다.

이 공식 문서는 **"복잡하게 추론 코드 짜지 말고, ONNX로만 변환해서 우리 SDK에 넣어라. 그럼 우리가 TensorRT로 가속해서 3D 결과까지 다 줄게"**라고 말하고 있는 것입니다.

################################  
#################################  
######################################  
제공해주신 공식 문서는 **ZED SDK에서 제공하는 YOLO 샘플 코드들의 구조와 사용법**에 대한 안내서입니다.

사용자님께서 ZED X와 Jetson AGX Orin을 사용 중이시므로, 이 문서는 **"어떤 샘플 코드를 참고하여 내 프로젝트(ROS 2)에 적용할 것인가?"**를 결정하는 데 매우 중요합니다.

각 섹션별로 빠짐없이 상세하게 설명해 드리겠습니다.

---

### 1. 개요 (Introduction)

이 샘플들은 **TensorRT** 프레임워크를 사용하여 YOLO 모델을 최적화하고 실행하는 방법을 보여줍니다.

* **핵심 흐름:**
1. ZED SDK가 이미지를 캡처합니다.
2. YOLO 모델이 **2D Bounding Box**를 검출합니다.
3. ZED SDK가 이 2D 박스 정보를 받아 **3D 정보(위치 x,y,z 및 3D 박스)**를 추출하고 **트래킹(추적)**을 수행합니다.


* **참고:** TensorRT 외에도 OpenCV DNN이나 PyTorch를 사용하는 예제도 있지만, Jetson 환경에서는 **TensorRT**가 성능상 가장 유리하므로 이 문서의 샘플들이 표준입니다.

### 2. 지원 버전 및 실행 방식 (YOLO v5 ~ v12 with TensorRT & C++)

ZED SDK는 YOLO v5부터 최신 v12까지 폭넓게 지원합니다. 실행 방식은 크게 두 가지(**Recommended** vs **Advanced**)로 나뉩니다.

#### **방식 A: [Recommended] Native Inference (자체 추론)**

* **설명:** ZED SDK API의 `OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS` 모드를 사용하는 방식입니다.
* **특징:**
* **자동 최적화:** 사용자님은 **ONNX 파일**만 준비하면 됩니다. ZED SDK가 내부적으로 TensorRT 엔진 변환, 최적화, 추론을 알아서 수행합니다.
* **가장 쉬움:** 별도의 추론 코드를 짤 필요가 없습니다.
* **범용성:** 출력 텐서의 구조를 SDK가 자동으로 분석하므로, 미래에 나올 모델이라도 구조가 비슷하면 호환됩니다.


* **ROS 2 사용자님께:** `zed_wrapper`에서 파라미터로 모델 경로만 잡아주면 되는 방식이 바로 이 방식입니다. **이 방식을 사용하시는 것을 강력 추천합니다.**

#### **방식 B: [Advanced] External Inference (외부 추론)**

* **설명:** 사용자가 직접 외부 코드(Custom C++/Python)로 TensorRT 추론을 돌리고, 나온 결과(2D 박스)를 ZED SDK에 **주입(Ingest)**하는 방식입니다.
* **용도:** ZED SDK가 자동으로 구조를 파악하지 못하는 **매우 특수한 모델**이나, 추론 과정 커스터마이징이 필수적인 경우에만 사용합니다. 유지보수가 어렵습니다.

### 3. 필수 선행 작업: ONNX 내보내기 (Exporting to ONNX)

ZED SDK(Native 방식)를 사용하려면 `.pt` (PyTorch) 파일은 사용할 수 없습니다. 반드시 **ONNX 포맷**으로 변환해야 합니다.

* **호환성:** YOLO 공식 리포지토리(Ultralytics 등)의 `export.py`를 사용하여 변환해야 하며, ZED SDK 버전에 따라 호환되는 Export 옵션이 다를 수 있습니다. (이는 별도 문서인 'YOLO ONNX Export' 페이지를 참고해야 합니다.)
* **모델:** COCO 데이터셋으로 학습된 기본 모델뿐만 아니라, **사용자님이 직접 학습시킨 커스텀 모델**도 사용 가능합니다.

### 4. 제공되는 샘플 폴더 설명 (Sample Folders)

GitHub 리포지토리에 있는 폴더별 역할입니다. 사용자님의 상황에 맞춰 무엇을 봐야 할지 알려드립니다.

#### **① `cpp_tensorrt_yolo_onnx_native` (★ 추천)**

* **언어:** C++
* **방식:** **Native Inference (방식 A)**
* **내용:** ONNX 파일을 로드하여 ZED SDK 내부에서 추론부터 3D 트래킹까지 한 번에 수행하는 가장 깔끔한 예제입니다.
* **활용:** ROS 2 Wrapper가 내부적으로 이 방식을 사용합니다. C++로 커스텀 노드를 짠다면 이 코드를 참고하세요.

#### **② `python_tensorrt_yolo_onnx_native**`

* **언어:** Python
* **방식:** **Native Inference (방식 A)**
* **내용:** 위 C++ 예제의 파이썬 버전입니다.
* **활용:** 파이썬으로 빠르게 프로토타이핑하거나 로직을 테스트할 때 유용합니다.

#### **③ `cpp_tensorrt_yolo_onnx` (Advanced)**

* **언어:** C++
* **방식:** External Inference (방식 B)
* **내용:** 외부에서 TensorRT 코드를 직접 구현하고, 결과 박스를 `ingest_custom_box_objects()` 함수로 ZED SDK에 넣어주는 예제입니다.

#### **④ `pytorch_yolov8` (Python Only)**

* **언어:** Python
* **방식:** External Inference (Ultralytics 라이브러리 직접 사용)
* **내용:** TensorRT 변환 과정 없이, `ultralytics` 패키지(PyTorch)를 그대로 실행해서 나온 결과를 ZED SDK에 주입하는 예제입니다.
* **장점:** ONNX 변환 없이 `.pt` 파일을 바로 쓸 수 있어 테스트가 매우 편합니다.
* **단점:** TensorRT 최적화를 거치지 않으므로 **속도가 느립니다.** 실전 주행용보다는 **모델 학습 직후 성능 검증용**으로 적합합니다.

### 5. 주요 기능 및 학습 (Features & Training)

* **기능:** 샘플을 실행하면 OpenGL 뷰어에 3D Point Cloud와 함께 감지된 물체의 **3D Bounding Box**가 그려집니다. 클래스(Class)와 신뢰도(Confidence) 임계값을 실시간으로 조절할 수 있습니다.
* **학습:** YOLOv8 등 최신 프레임워크로 학습한 **Custom Model**을 그대로 가져와서 사용할 수 있습니다. (Ultralytics의 커스텀 데이터셋 학습 튜토리얼 링크 제공)

---

### 💡 사용자님을 위한 요약 및 적용 가이드

사용자님은 **Jetson AGX Orin + ROS 2 + Custom YOLO** 환경이므로, 다음 순서로 진행하시면 됩니다.

1. **모델 준비:** 학습된 `best.pt`를 준비합니다.
2. **변환:** 이를 `best.onnx`로 변환합니다. (dynamic batch 등의 옵션 확인 필요)
3. **ROS 2 적용:** 별도의 코딩(샘플 코드 복사) 없이, **`zed_wrapper`의 파라미터 설정만으로 가능**합니다.
* 공식 문서의 **[Recommended] Native Inference** 방식이 `zed_wrapper`에 이미 구현되어 있기 때문입니다.
* `object_detection_model: CUSTOM_YOLOLIKE`
* `onnx_model_path: /path/to/best.onnx`



이 샘플 코드들은 ROS 2를 쓰지 않고 C++/Python 단독으로 테스트할 때 참고하시면 되며, ROS 2 환경에서는 위 파라미터 설정만으로 동일한 기능을 수행할 수 있습니다.
#####################################  
###########################################  
############################################  
###############################################  
제공해주신 공식 문서는 **"학습된 YOLO 모델(PyTorch .pt)을 ZED SDK가 이해할 수 있는 ONNX 포맷으로 변환(Export)하는 방법"**에 대한 기술 문서입니다.

사용자님께서 **Custom YOLO 모델을 학습**시킨 후, 이를 **Jetson AGX Orin의 ROS 2 환경**에서 ZED와 연동하기 위해 **반드시 거쳐야 하는 필수 과정**입니다.

문서의 내용을 **핵심 개념**, **버전별 변환 명령어**, **주의사항**으로 나누어 상세히 설명해 드립니다.

---

### 1. 개요 및 워크플로우 (Introduction & Workflow)

ZED SDK는 PyTorch 모델 파일(`.pt`)을 직접 읽지 못합니다. 따라서 중간 단계인 **ONNX** 포맷으로 변환해주면, ZED SDK가 이를 읽어 **TensorRT 엔진(.engine)**으로 자동 변환하여 최적화합니다.

* **지원 모델:** YOLO v5, v6, v7, v8, v9, v10, v11, v12 (Ultralytics 및 각 공식 리포지토리)
* **주요 기능:**
* COCO 데이터셋(80개 클래스) 기본 모델 사용 가능.
* **사용자 정의 데이터셋(Custom Class)으로 학습된 모델 사용 가능.** (사용자님의 목표)


* **전체 흐름:**
1. **Train:** YOLO 모델 학습 (`best.pt` 생성)
2. **Export:** `.pt` 파일을 `.onnx` 파일로 변환 (이 문서의 핵심)
3. **Load:** ZED SDK(또는 ROS 2 Wrapper)에서 ONNX 파일을 로드 → SDK가 TensorRT 엔진 자동 생성 및 3D 추론 시작.



---

### 2. 버전별 Export 방법 (상세 가이드)

사용하시는 YOLO 버전에 따라 변환 명령어와 라이브러리가 다릅니다. 현재 가장 대중적인 **Ultralytics (v8, v11, v12 등)**를 중심으로 설명해 드립니다.

#### **A. Ultralytics 패키지 (YOLO v5, v8, v10, v11, v12)**

가장 최신 방식이며 통합된 인터페이스를 제공합니다.

1. **설치:**
```bash
pip install -U ultralytics

```


2. **Export 명령어 (터미널 CLI 사용):**
학습된 모델(`best.pt` 등)이 있는 곳에서 아래 명령어를 실행합니다.
* **기본 명령어 (정적 크기):**
```bash
# 예시: YOLOv8 nano 모델
yolo export model=yolov8n.pt format=onnx simplify=True dynamic=False imgsz=608

```


* **Custom 모델 사용 시 (사용자님 케이스):**
```bash
# 학습된 가중치 파일 경로 지정
yolo export model=path/to/your/best.pt format=onnx simplify=True dynamic=False imgsz=640

```




3. **주요 파라미터 해설:**
* `format=onnx`: 출력 포맷을 ONNX로 지정.
* `simplify=True`: **[중요]** ONNX 모델의 불필요한 연산자를 제거하여 TensorRT 변환 성공률과 속도를 높입니다.
* `dynamic=False`: **[권장]** 입력 이미지 크기를 고정합니다. TensorRT 최적화에 유리하며, ZED 카메라는 해상도가 고정되어 있으므로 `False`가 성능상 좋습니다.
* `imgsz=640`: 학습할 때 사용했던 이미지 크기(Image Size)와 **동일하게** 맞춰야 합니다. (보통 640이나 512, 1280 등)



#### **B. YOLOv6 (Meituan)**

별도의 리포지토리를 사용합니다.

1. **설치:** `git clone` 후 `requirements.txt` 설치.
2. **Export 명령어:**
```bash
python ./deploy/ONNX/export_onnx.py \
    --weights best_ckpt.pt \
    --img 640 \
    --batch 1 \
    --simplify

```



#### **C. YOLOv7 (WongKinYiu)**

**[주의]** YOLOv7은 Export 시 **매우 중요한 제약 사항**이 있습니다.

1. **설치:** `git clone` 후 `requirements.txt` 설치.
2. **Export 명령어:**
```bash
python export.py --weights ./yolov7_custom.pt --grid --simplify --topk-all 100 --iou-thres 0.65 --conf-thres 0.35 --img-size 640 640

```


3. **⛔ 절대 주의사항 (Critical Note):**
* 명령어에 **`--end2end` 옵션을 절대 넣으면 안 됩니다.**
* *이유:* `--end2end`를 넣으면 NMS(Non-Maximum Suppression) 과정이 모델 안에 포함되어 버립니다. ZED SDK는 **Raw Bounding Box**를 받아서 직접 3D 매칭을 수행해야 하므로, NMS가 포함된 모델을 읽지 못하거나 성능이 떨어집니다.



#### **D. YOLOv5 (Legacy Ultralytics)**

구버전 리포지토리를 사용하는 경우입니다. (최신 `ultralytics` 패키지로도 v5 export가 가능하므로 A방식을 추천합니다.)

* 명령어: `python export.py --weights custom.pt --include onnx --imgsz 640`

---

### 3. 기술적 핵심 요약 (Technical Key Points)

이 문서에서 사용자님이 꼭 챙겨야 할 **"성공적인 변환을 위한 3가지 조건"**입니다.

1. **`simplify=True` (단순화):**
* ONNX 그래프를 단순화해야 Jetson의 TensorRT가 에러 없이 엔진을 빌드할 수 있습니다. 필수 옵션입니다.


2. **`imgsz` (이미지 크기 일치):**
* 모델 학습 시 사용한 해상도(예: 640x640)와 Export 할 때의 해상도가 다르면 추론 정확도가 박살 납니다. 학습 설정값을 꼭 확인하세요.


3. **`dynamic=False` (정적 배치 권장):**
* 공식 문서 예제에는 `dynamic=False`가 기본입니다.
* 로봇/임베디드 환경에서는 입력 크기가 변하지 않으므로, 정적(Static) 크기로 고정하는 것이 TensorRT 최적화 및 메모리 관리 측면에서 유리합니다.



---

### 4. ROS 2 프로젝트 적용 가이드 (User Context)

이제 이 문서를 바탕으로 사용자님이 수행해야 할 작업 순서입니다.

1. **모델 학습:** PC 등에서 YOLO 모델을 학습시켜 `best.pt`를 얻습니다.
2. **ONNX 변환 (PC 또는 Jetson):**
```bash
# 예: Ultralytics 패키지 사용, 640 사이즈 학습 모델
yolo export model=best.pt format=onnx simplify=True dynamic=False imgsz=640

```


3. **파일 이동:** 생성된 `best.onnx` 파일을 Jetson의 적당한 경로(예: `~/ros2_ws/src/zed_ros2_wrapper/zed_wrapper/config/`)로 복사합니다.
4. **ROS 2 파라미터 설정 (`common_stereo.yaml` 또는 `zedx.yaml`):**
* `object_detection`:
* `od_enabled`: `true`
* `model_type`: `CUSTOM_YOLOLIKE` (이게 핵심입니다)
* `model_path`: `/path/to/your/best.onnx` (변환한 파일 경로)




5. **실행:** `ros2 launch ...` 실행 시, ZED SDK가 자동으로 `best.onnx`를 읽어 `best.engine`을 생성하고 3D Object Detection을 시작합니다.

이 과정만 거치면, 복잡한 C++ 코딩 없이도 사용자님의 Custom YOLO 모델이 ZED 카메라 위에서 3D로 작동하게 됩니다.
####################################  
####################################  
######################################  
######################################  
제공해주신 문서는 **"사용자가 직접 학습시킨 Custom YOLO 모델(ONNX)을 ZED ROS 2 Wrapper에 통합하여 사용하는 전체 가이드"**입니다.

이 문서는 사용자가 `.pt` 파일을 ONNX로 변환하는 것부터, ROS 2 파라미터를 설정하고, 최종적으로 Rviz2에서 결과를 확인하는 것까지의 워크플로우를 다루고 있습니다.

단계별로 빠짐없이 상세하게 설명해 드립니다.

---

### 1. 전제 조건 및 모델 준비 (Prerequisites & Export)

사용자 모델을 ZED SDK가 인식하려면 반드시 **ONNX 포맷**이어야 하며, 올바르게 Export 되어야 합니다.

* **필수 조건:**
* ONNX 포맷일 것.
* ZED SDK와 호환될 것.


* **Export 방법 (Ultralytics YOLOv8 예시):**
문서에서는 `ultralytics` 패키지를 사용한 CLI 명령어를 제공합니다.
```bash
pip install -U ultralytics
yolo export model=yolov8s.pt format=onnx simplify=True dynamic=False imgsz=512

```


* **`simplify=True`:** TensorRT 엔진 변환 성공률을 높이기 위해 모델 구조를 단순화합니다.
* **`dynamic=False`:** 입력 이미지 크기를 고정합니다. (임베디드 환경 최적화)
* **`imgsz=512`:** 학습 시 사용했던 해상도와 일치시켜야 합니다.



---

### 2. ROS 2 기본 설정 (`common_stereo.yaml`)

기본 설정 파일(`common_stereo.yaml` 혹은 `zedx.yaml`)에서 **"나는 Custom 모델을 쓰겠다"**라고 선언해야 합니다.

```yaml
object_detection:
  detection_model: 'CUSTOM_YOLOLIKE_BOX_OBJECTS'  # [핵심] 커스텀 YOLO 모델 사용 선언
  enable_tracking: true                           # 객체 추적 기능 활성화

```

* **`enable_tracking: true`:** 이 옵션을 켜면, 물체가 잠시 가려지거나(Occlusion) 카메라 밖으로 나가도 ZED SDK의 알고리즘이 궤적을 예측하여 추적합니다.
* **추적 상태 (Tracking State):** 결과 메시지(`ObjectsStamped`)에 포함되며 다음 값을 가집니다.
* `0 (OFF)`: 추적 불가 (유효하지 않음)
* `1 (OK)`: 정상 추적 중
* `2 (SEARCHING)`: 가려짐 발생, 궤적 예측 중
* `3 (TERMINATE)`: 추적 종료 예정



---

### 3. 커스텀 모델 상세 설정 파일 생성 (`custom_object_detection.yaml`)

이 단계가 **가장 중요**합니다. ONNX 파일 경로와 클래스별 세부 설정을 담은 **새로운 YAML 파일**을 만들어야 합니다. (기존 템플릿 복사 권장)

#### **A. 헤더 설정 (Global Settings)**

```yaml
/**:
  ros__parameters:
    object_detection:
      custom_onnx_file: '/path/to/your/best.onnx'  # [필수] ONNX 파일 절대 경로
      custom_onnx_input_size: 640                   # 학습 시 사용한 입력 해상도
      custom_class_count: 80                        # 모델의 클래스 개수 (Custom은 1~N개)

```

#### **B. 클래스별 상세 설정 (Class-Specific Config)**

각 클래스(`class_0`, `class_1`...)마다 필터링 및 추적 옵션을 개별적으로 설정할 수 있습니다.

* **기본 정보:**
* `label`: 객체 이름 (예: 'person', 'robot')
* `model_class_id`: ONNX 모델 내부의 클래스 ID (0부터 시작)
* `enabled`: `true`/`false`. 해당 클래스를 검출할지 여부.


* **신뢰도 및 필터링:**
* `confidence_threshold`: 이 값(0~99)보다 낮은 검출 결과는 무시합니다.
* **[3D 필터링]** (Ghost 데이터 제거에 유용):
* `max_box_width_meters`, `min_box_width_meters`: 3D 상의 실제 너비 제한.
* `max_box_height_meters`, `min_box_height_meters`: 3D 상의 실제 높이 제한.
* *팁: 로봇이 '사람'만 인식해야 한다면, 사람의 키나 너비를 벗어나는 물체는 여기서 필터링 가능합니다.*




* **추적 보정 (Tracking Hints):**
* `is_grounded`: `true`면 물체가 지면에 붙어 움직인다고 가정 (로봇, 사람 등). 추적 성능 향상.
* `is_static`: `true`면 물체가 고정되어 있다고 가정.
* `tracking_timeout`: 물체가 시야에서 사라졌을 때, 몇 초까지 기억할지 설정. (-1은 무제한)
* `object_acceleration_preset`: 예상되는 물체의 가속도 (`LOW`, `MEDIUM`, `HIGH`). 사람이라면 `LOW`~`MEDIUM`.



---

### 4. 노드 실행 (Launching)

설정한 `custom_object_detection.yaml` 파일을 로드하며 노드를 실행합니다.

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zedx \
    custom_object_detection_config_path:='/path/to/custom_object_detection.yaml'

```

* **참고:** 경로를 지정하지 않으면 패키지 내 기본 `config/custom_object_detection.yaml`을 읽습니다.

---

### 5. 최적화 및 런타임 제어 (Optimization & Runtime)

* **최초 실행 시 지연 (Optimization):**
* 처음 실행하면 ZED SDK가 ONNX를 읽어 **TensorRT 엔진(.engine)을 빌드**합니다.
* 이 과정은 GPU 성능에 따라 **수 분(few minutes)** 걸릴 수 있으며, 터미널에 `Please wait while the AI model is being optimized...` 로그가 뜹니다.
* 한 번 빌드되면 `/usr/local/zed/resources` 등에 저장되어 다음부터는 즉시 실행됩니다.


* **기능 끄기/켜기:**
* `common_stereo.yaml`의 `od_enabled: true`로 자동 시작 가능.
* 서비스(`~/enable_obj_det`)를 호출하여 실행 중에 켜거나 끌 수 있습니다.



---

### 6. 결과 시각화 (Rviz2 & ZedOdDisplay)

검출 결과는 `zed_interfaces/ObjectsStamped`라는 커스텀 메시지로 발행됩니다. 이를 Rviz2에서 제대로 보려면 **전용 플러그인**이 필요합니다.

* **플러그인:** `zed-ros2-examples` 리포지토리에 있는 **`ZedOdDisplay`** 플러그인을 설치해야 합니다.
* **Rviz2 설정:**
* Add -> `ZedOdDisplay` 추가.
* **Topic:** 객체 검출 토픽 선택 (`/zed/zed_node/obj_det/objects` 등).
* **Show Bounding Boxes:** 3D 박스 시각화.
* **Show Labels:** `<tracking_id>-<label>` 형태로 표시됨. (예: `5-person`)


* **QoS 정책:** 데이터가 안 보인다면 `Reliability Policy`를 `Best Effort`, `Durability Policy`를 `Volatile`로 변경해 보세요.

---

### 💡 요약: 사용자님이 해야 할 일

1. 학습된 모델을 `yolo export ...` 명령어로 **ONNX 변환**.
2. `config/custom_object_detection.yaml` 파일을 새로 만들어서, **ONNX 경로**와 **클래스 정보** 입력.
3. `common_stereo.yaml`에서 `detection_model: 'CUSTOM_YOLOLIKE_BOX_OBJECTS'` 설정.
4. Launch 파일 실행 시 `custom_object_detection_config_path`에 2번 파일 경로 지정.
5. 첫 실행 시 TensorRT 빌드 시간 기다리기.
6. ###############################################
   ################################################
###############################################
##################################################
####################################################
제공해주신 공식 문서는 **"사용자가 직접 만든 2D 객체 검출기(Custom Detector)를 ZED SDK와 연동하여 3D 정보를 얻는 방법"**에 대한 기술 문서입니다.

이 기능은 **"나는 ZED가 제공하는 기본 모델 말고, 내가 학습시킨 YOLO 모델을 쓰고 싶은데, 3D 위치랑 트래킹 기능은 ZED SDK꺼를 쓰고 싶어!"**라는 사용자(바로 지금의 사용자님)를 위해 만들어졌습니다.

핵심 내용을 빠짐없이 상세하게 설명해 드립니다.

---

### 1. 개요 (Introduction)

* **목적:** 어떤 2D Bounding Box 검출기(YOLO, SSD, EfficientDet 등)라도 상관없이, 그 결과(2D 박스)만 ZED SDK에 넣어주면 **3D 위치, 3D Bounding Box, 트래킹 정보**를 계산해 줍니다.
* **작동 원리:**
1. **User Side:** Custom Detector가 이미지에서 2D 박스()를 찾습니다.
2. **ZED SDK Side:** 사용자가 넣어준 2D 박스에 **Depth Map(깊이 정보)**을 매핑하여 3D 좌표()를 계산하고, **Positional Tracking 모듈**을 이용해 로봇이 움직여도 물체를 추적(Tracking)합니다.



---

### 2. 워크플로우 (Workflow) - [중요]

전체 과정은 다음과 같이 순환합니다.

1. **이미지 획득 (`grab`):** ZED 카메라에서 이미지를 가져옵니다.
2. **추론 (Inference):** 가져온 이미지를 사용자님의 Custom Detector(YOLO 등)에 넣어 2D 박스를 얻습니다.
* *주의:* 추론을 저해상도에서 했다면, ZED에 넣기 전에 원본 해상도(Native Resolution)로 좌표를 변환(Rescale)해야 합니다.


3. **주입 (`ingest`):** 얻은 2D 박스 정보를 `ingestCustomBoxObjects` 함수를 통해 ZED SDK에 밀어 넣습니다.
4. **회수 (`retrieve`):** ZED SDK가 계산을 마친 3D 객체 정보(위치, 속도, 트래킹 ID 등)를 `retrieveObjects` 함수로 받아옵니다.

---

### 3. 설정 방법 (Configuration)

ZED SDK를 초기화할 때, **"나 지금 Custom Model 쓸 거야"**라고 알려줘야 합니다.

* **파라미터 설정:**
* `detection_model`: 반드시 **`CUSTOM_BOX_OBJECTS`**로 설정해야 합니다. (이게 핵심입니다.)
* `enable_tracking`: `true`로 설정하면 물체에 ID를 부여하고 추적합니다.
* `enable_mask_output`: `true`로 설정하면 2D 마스크(Segmentation)도 계산합니다.


* **주의사항:** Custom Mode에서는 ZED SDK 내부의 NMS(Non-Maximum Suppression)나 Score Threshold 필터링이 **작동하지 않습니다.** (사용자가 추론 단계에서 이미 필터링해서 넣어야 합니다.)

---

### 4. 데이터 주입 구조체 (`CustomBoxObjectData`)

Custom Detector가 찾은 정보를 ZED SDK에 넣을 때 사용하는 데이터 포맷입니다.

* **`unique_object_id`:** (선택) 외부에서 관리하는 ID가 있다면 넣고, 없으면 SDK가 알아서 관리하게 둡니다.
* **`probability`:** 검출 신뢰도 (Score). 트래킹 정확도 향상에 쓰입니다.
* **`label`:** 클래스 ID (예: 사람=0, 차=1).
* **`bounding_box_2d`:** [좌상단 A, 우상단 B, 우하단 C, 좌하단 D] 순서의 4개 점 좌표.
* **`is_grounded`:** `true`로 설정하면 "이 물체는 바닥 위에 있다"고 가정하여 위치 추정 정확도를 높입니다. (로봇, 사람 등에 유용)

---

### 5. 결과 데이터 (`sl::ObjectData`)

`retrieveObjects`로 받아온 결과에는 다음 정보가 들어있습니다.

* **`id`:** 객체의 고유 추적 ID. (프레임이 지나도 같은 물체면 ID 유지)
* **`position`:** 3D 좌표 (). 카메라 왼쪽 눈 기준입니다.
* **`velocity`:** 3D 속도 벡터.
* **`tracking_state`:** 현재 추적 상태 (OK, OFF, SEARCHING 등).
* **`raw_label`:** 사용자님이 넣어준 원래 클래스 ID.
* **`bounding_box`:** 3D Bounding Box를 구성하는 8개의 3D 점 좌표.

---

### 💡 사용자님을 위한 요약 및 적용 포인트

이 문서는 C++/Python 코드 레벨에서의 설명이지만, 사용자님은 **ROS 2 Wrapper**를 사용 중이시므로 아래 내용만 기억하시면 됩니다.

1. **ROS 2 파라미터:** `common_stereo.yaml`에서 `detection_model`을 `CUSTOM_YOLOLIKE_BOX_OBJECTS`로 설정하면, Wrapper가 내부적으로 이 문서의 워크플로우(Ingest -> Retrieve)를 자동으로 수행합니다.
2. **학습:** Custom YOLO 모델을 학습시킬 때, **메모리와 연산량(Inference Speed)**을 고려하여 모델 아키텍처(n, s, m 등)를 선정해야 합니다. (Jetson에서의 실시간성을 위해)
3. **좌표계:** ZED SDK가 주는 3D 좌표는 기본적으로 **"Left Camera Optical Frame"** 기준일 수 있으므로, ROS 2에서 `frame_id`를 잘 확인해야 합니다. (보통 Wrapper가 `base_link`나 `map` 좌표계로 변환해서 줍니다.)

즉, **"ROS 2 Wrapper를 쓴다면 이 복잡한 코딩 과정(Ingest, Retrieve 등)은 다 생략하고, ONNX 파일 경로만 잘 잡아주면 된다"**는 것이 결론입니다.
###################################  
######################################  
##########################################  
############################################  
제공해주신 공식 문서는 **ZED ROS 2 Wrapper에서 기본 제공 모델 또는 사용자 커스텀 모델을 사용하여 객체 검출(Object Detection)을 수행하는 방법**에 대한 가이드입니다.

사용자님께서 앞서 질문하신 "Custom YOLO 모델 적용"과 직결되는 내용이며, **모델 선택부터 Rviz2 시각화까지**의 설정을 다룹니다.

빠짐없이 상세하게 설명해 드립니다.

---

### 1. 사용 가능한 감지 모델 (Detection Models)

ZED ROS 2 Wrapper는 크게 **내장 모델(Internal)**과 **사용자 모델(Custom)** 두 가지 방식을 지원합니다.

| 모델 옵션 | 설명 | 특징 |
| --- | --- | --- |
| **`MULTI_CLASS_BOX_FAST`** | 사람, 차량 등 일반 객체 감지 | 속도 빠름, 정확도 낮음 |
| **`MULTI_CLASS_BOX_MEDIUM`** | 사람, 차량 등 일반 객체 감지 | 속도와 정확도의 균형 (기본값) |
| **`MULTI_CLASS_BOX_ACCURATE`** | 사람, 차량 등 일반 객체 감지 | 정확도 높음, 속도 느림 |
| **`PERSON_HEAD_BOX_FAST`** | 사람의 **머리**만 특화하여 감지 | 군중(Crowd) 환경에서 사람 위치 파악에 유리 |
| **`PERSON_HEAD_BOX_ACCURATE`** | 사람의 **머리**만 특화하여 감지 | 위 모델보다 정확하지만 느림 |
| **`CUSTOM_YOLOLIKE_BOX_OBJECTS`** | **[사용자님 선택]** Custom YOLO | **사용자가 만든 ONNX 파일 필요.** 내부적으로 TensorRT로 추론 수행. |

* **참고:** `MULTI_CLASS` 및 `PERSON_HEAD` 모델은 SDK가 인터넷에서 자동으로 다운로드하여 사용합니다.
* **사용자님:** 앞서 논의한 Custom YOLO를 쓰시려면 설정 파일에서 모델을 반드시 **`CUSTOM_YOLOLIKE_BOX_OBJECTS`**로 지정해야 합니다.

---

### 2. 객체 검출 활성화 방법 (Enable Object Detection)

객체 검출 모듈을 켜는 방법은 **자동(시작 시)**과 **수동(런타임)** 두 가지가 있습니다.

#### **방법 A: 자동 실행 (설정 파일 수정)**

ROS 2 노드가 켜질 때 자동으로 감지를 시작하게 하려면, `common_stereo.yaml` (또는 `zedx.yaml`) 파일을 수정합니다.

```yaml
object_detection:
    od_enabled: true  # false -> true로 변경

```

#### **방법 B: 수동 실행 (서비스 호출)**

노드가 실행 중일 때 터미널에서 명령어를 날려 켜거나 끌 수 있습니다.

* **켜기:** `ros2 service call /<node_name>/enable_obj_det std_srvs/srv/SetBool "{data: True}"`
* **끄기:** `ros2 service call /<node_name>/enable_obj_det std_srvs/srv/SetBool "{data: False}"`

---

### 3. 최초 실행 시 주의사항 (Optimization) - [매우 중요]

ZED SDK는 AI 모델을 돌리기 위해 **NVIDIA TensorRT** 엔진을 사용합니다.

* **현상:** 처음 `od_enabled: true`로 설정하고 실행하면, 터미널에 다음과 같은 로그가 뜨면서 **몇 분 동안 멈춘 것처럼 보일 수 있습니다.**
```text
[ZED][INFO] Please wait while the AI model is being optimized for your graphics card

```


* **이유:** ONNX 모델(또는 내장 모델)을 현재 GPU(Jetson AGX Orin)에 최적화된 `.engine` 파일로 변환(Build)하는 과정입니다.
* **위치:** 변환된 파일은 `/usr/local/zed/resources` 폴더에 저장됩니다.
* **결과:** 이 과정은 **최초 1회만** 수행되며, 다음 실행부터는 저장된 엔진을 불러오므로 즉시 실행됩니다. **절대 강제 종료하지 말고 기다리세요.**

---

### 4. 결과 시각화 (Rviz2 & ZedOdDisplay Plugin)

검출된 결과는 `zed_interfaces/ObjectsStamped`라는 커스텀 토픽으로 나옵니다. 이를 Rviz2에서 시각적으로 확인하려면 **전용 플러그인(`ZedOdDisplay`)**이 필요합니다.

* **설치:** `zed-ros2-examples` 패키지를 빌드하면 플러그인이 생성됩니다.
* **Rviz2 사용법:** `Add` 버튼 -> `zed_ros2_examples` -> `ZedOdDisplay` 선택.

#### **주요 플러그인 파라미터 설명**

1. **Topic:** 시각화할 토픽 선택 (예: `/zed/zed_node/obj_det/objects`).
2. **QoS 설정 (History, Reliability, Durability):**
* **매우 중요합니다.** 토픽이 발행되는데 Rviz에서 안 보인다면 보통 이 설정이 안 맞아서 그렇습니다.
* 권장값:
* History: `Keep Last`
* Reliability: `Best Effort` (데이터 손실 감수하고 최신 데이터 우선)
* Durability: `Volatile` (지나간 데이터 저장 안 함)




3. **Transparency:** 3D 박스나 모델의 투명도 조절 (0~1).
4. **Show Labels:** 물체 위에 이름(Label)을 띄울지 여부.
5. **Show Bounding Boxes:** 3D 박스를 그릴지 여부.
6. **Label Scale:** 라벨 글자 크기 조절.

---

### 💡 요약 및 사용자님 적용 가이드

이 문서를 통해 사용자님은 다음을 확신하고 진행하시면 됩니다.

1. **모델 설정:** `common_stereo.yaml`에서 `detection_model`을 `CUSTOM_YOLOLIKE_BOX_OBJECTS`로 설정해야 사용자 모델이 로드됩니다.
2. **첫 실행:** "왜 안 켜지지?" 하고 끄지 마시고, **TensorRT 최적화가 끝날 때까지(길게는 5~10분)** 기다리셔야 합니다.
3. **시각화:** Rviz2에서 결과를 볼 때, 데이터가 안 뜨면 **QoS 설정을 `Best Effort`로 변경**해 보세요.
##########################################
###############################################
################################################
##################################################
현재 **SVGA (960 x 600)** 해상도를 사용하고 계시다면, YOLO 학습 이미지 사이즈(`imgsz`)는 **`512`**를 가장 강력하게 추천합니다.

그 이유를 **픽셀 매칭 효율성**과 **4대 카메라 구동 부하** 관점에서 명확히 설명해 드립니다.

---

### 1. 왜 512가 최적의 선택인가?

YOLO 모델은 입력 이미지를 정사각형(Square)으로 받습니다. ZED SDK는 직사각형인 SVGA 이미지를 정사각형 모델에 넣기 위해 **"가로(긴 쪽)를 기준으로 리사이징 + 위아래 여백(Letterbox)"** 방식을 사용합니다.

#### **옵션 A: `imgsz=640` (보통 많이 쓰는 값)**

* **변환 과정:** 가로 960px을 640px로 줄입니다. (약 66% 축소)
* **세로 해상도:** 원본 세로 600px도 66% 줄어들어 약 **400px**이 됩니다.
* **실제 데이터:** 모델 입력(640x640) 중, 실제 이미지는 `640 x 400`이고 나머지 `240`픽셀 높이는 검은색 여백(Padding)입니다.
* **판단:** SVGA 원본 해상도에 비해 640은 약간 과한 감이 있습니다. 하지만 디테일은 가장 좋습니다.

#### **옵션 B: `imgsz=512` (추천)**

* **변환 과정:** 가로 960px을 512px로 줄입니다. (약 53% 축소)
* **세로 해상도:** 원본 세로 600px도 53% 줄어들어 약 **320px**이 됩니다.
* **실제 데이터:** 모델 입력(512x512) 중, 실제 이미지는 `512 x 320`이고 나머지는 여백입니다.
* **판단:** SVGA 화질 자체가 아주 높지 않기 때문에, 512로 줄여도 **객체 인식률에 큰 차이가 없으면서 연산량은 대폭 줄어듭니다.**

---

### 2. 4대 동시 구동 시 성능 차이 (Jetson AGX Orin 기준)

사용자님은 카메라를 4대나 돌려야 합니다. 이게 핵심 결정 요인입니다.

* **연산량 비교:**
*  픽셀
*  픽셀
* **차이:** 512 사이즈가 640보다 **픽셀 수가 약 36% 적습니다.**


* **결론:**
* 카메라 1대라면 640도 괜찮지만, **4대라면 512를 써야 GPU 여유 자원을 36%나 더 확보**할 수 있습니다.
* 이 여유 자원은 앞서 구현한 `ROR 필터`나 `ROS 2 통신` 처리에 쓰는 것이 시스템 전체 안정성에 훨씬 이득입니다.



---

### 3. 최종 가이드: 이렇게 설정하세요

#### **Step 1. YOLO 학습 및 Export**

학습시킬 때부터 512 사이즈로 학습시키는 것이 가장 좋고, 만약 이미 큰 사이즈로 학습했다면 Export라도 512로 하세요.

```bash
# 512 사이즈로 Export (Jetson 부하 최소화)
yolo export model=best.pt format=onnx simplify=True dynamic=False imgsz=512

```

#### **Step 2. ROS 2 설정 (`custom_object_detection.yaml`)**

ZED 설정 파일에도 512라고 명시해야 내부적으로 리사이징을 512에 맞춰서 수행합니다.

```yaml
/**:
  ros__parameters:
    object_detection:
      custom_onnx_file: '/path/to/best_512.onnx'
      custom_onnx_input_size: 512       # [중요] 여기도 512로 설정

```

### 요약

* **SVGA (960x600)** 환경에서 4대 카메라를 돌린다면 **`imgsz=512`**가 **속도와 정확도의 황금비**입니다.
* 만약 512로 했는데 "작은 물체(멀리 있는 사람 등)" 인식이 너무 안 된다면 그때만 640으로 올리세요. 하지만 90%의 경우 512로 충분합니다.
#######################################3  

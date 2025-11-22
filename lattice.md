반갑습니다\! ROS 2 Nav2 Jazzy 및 Smac Planner 전문가로서 도와드리겠습니다.

Nav2의 **Smac Planner Hybrid A**\*는 미리 계산된 "동작 궤적(Primitives)"을 사용하여 경로를 계획합니다. 사용자가 공유해주신 `readme.md`는 바로 이 **동작 궤적 파일(.json)을 생성하는 도구**에 대한 설명입니다.

요청하신 대로 **1) README 내용 설명**과 **2) 후진(Reverse) Motion만 생성하도록 튜닝하는 방법**을 정리해 드리겠습니다.

-----

### 1\. Lattice Primitive Generator README 설명

이 도구는 로봇이 이동 가능한 최소 단위의 경로 조각(Primitive)들을 미리 계산하여 JSON 파일로 만들어주는 Python 스크립트입니다.

#### **주요 내용 요약**

  * **About (개요):**

      * CMU의 논문을 기반으로 State Lattice Planner(Smac Planner)가 사용할 \*\*최소 제어 집합(Control Set)\*\*을 생성합니다.
      * 미리 계산된 궤적을 사용하므로 실시간 경로 계획 시 연산 속도가 빠릅니다.

  * **Parameters (핵심 파라미터 - `config.json`에서 설정):**

      * `motion_model`: 로봇의 기구학적 모델입니다.
          * `ackermann`: 전진 및 후진 (자동차/지게차 형태).
          * `diff`: 차동 구동 (전진 + 제자리 회전). **(주의: diff 모델은 기본 생성기에서 후진 궤적을 포함하지 않을 수 있습니다. ackermann을 사용하여 후진을 유도하는 것이 유리할 수 있습니다.)**
          * `omni`: 전 방향 이동.
      * `turning_radius`: 로봇의 \*\*최소 회전 반경(m)\*\*입니다. 이 값보다 작은 회전은 생성되지 않습니다.
      * `grid_resolution`: 격자 해상도(m)입니다. **Costmap의 해상도와 반드시 일치**해야 합니다. (예: 0.05)
      * `num_of_headings`: 로봇의 방향(Yaw)을 몇 개로 쪼갤지 결정합니다. (보통 16개 권장, 8의 배수여야 함).

  * **How it Works (작동 원리):**

      * **각도 이산화 (Angle Discretization):** 로봇이 격자(Grid) 점에 정확히 도착할 수 있도록 각도를 나눕니다.
      * **궤적 생성 (Trajectory Generator):** 시작점과 끝점을 잇는 곡선 및 직선 궤적을 수학적으로 계산합니다.
      * **Lattice Generator:** 원점에서 시작하여 가능한 모든 경로를 탐색(Wavefront)하고, 중복되거나 불필요한 경로를 제거하여 최적의 집합만 남깁니다. 이후 대칭성(Symmetry)을 이용해 1사분면의 데이터를 뒤집어 전체 방향의 궤적을 만듭니다.

-----

### 2\. "후진(Reverse) Motion"만 생성 및 튜닝하는 방법

기본적으로 이 생성기는 "전진"과 "후진"을 모두 포함하거나, 모델에 따라 "전진"만 생성하도록 설계되어 있습니다. **"후진만"** 하는 설정 옵션은 따로 없으므로, \*\*생성된 결과물(JSON)을 후처리(Filtering)\*\*하는 방법을 사용해야 합니다.

전문가로서 추천하는 절차는 다음과 같습니다.

#### **Step 1: `config.json` 설정 (생성 준비)**

후진 궤적을 포함시키기 위해 `motion_model`을 `ackermann`으로 설정하는 것이 가장 확실합니다. (차동 구동 로봇이라도 후진 궤적 확보를 위해 이 모델을 사용하여 생성할 수 있습니다.)

```json
{
  "motion_model": "ackermann", 
  "turning_radius": 0.4,       
  "grid_resolution": 0.05,     
  "stopping_threshold": 5,
  "num_of_headings": 16
}
```

  * **Tip:** 후진 주행은 제어가 까다로울 수 있으므로 `turning_radius`를 실제 스펙보다 약간 여유 있게 잡는 것이 안전합니다.

#### **Step 2: Primitives 생성 실행**

터미널에서 생성기를 실행합니다.

```bash
python3 generate_motion_primitives.py --config config.json --output all_primitives.json
```

#### **Step 3: JSON 파일 필터링 (핵심: 후진만 남기기)**

생성된 `all_primitives.json` 파일에는 전진과 후진 궤적이 섞여 있습니다. 여기서 **전진 궤적을 삭제**해야 합니다.

  * **원리:** JSON 데이터 내의 각 primitive는 `poses` 리스트를 가집니다.
      * `poses`의 마지막 좌표 $(x, y, \theta)$를 확인합니다.
      * 로봇 좌표계 기준 \*\*$x > 0$ (양수)\*\*이면 전진입니다.
      * 로봇 좌표계 기준 \*\*$x < 0$ (음수)\*\*이면 후진입니다.

이 작업을 수동으로 하기엔 데이터가 너무 많으므로, 아래의 간단한 Python 스크립트를 사용하여 **후진 궤적만 남긴 파일**로 변환하세요.

```python
import json

# 1. 생성된 원본 파일 로드
input_file = 'all_primitives.json'
output_file = 'backward_only_primitives.json'

with open(input_file, 'r') as f:
    data = json.load(f)

original_primitives = data['primitives']
backward_primitives = []

print(f"Total primitives before: {len(original_primitives)}")

# 2. 필터링 로직
for prim in original_primitives:
    # 궤적의 마지막 포즈 확인 (x, y, theta)
    last_pose = prim['poses'][-1]
    x_end = last_pose[0]
    
    # x가 음수(후진)인 경우 혹은 제자리 회전(거의 0)만 허용
    # 주의: Ackermann 모델은 제자리 회전이 없지만, 혹시 모를 오차 고려
    if x_end < -0.001: 
        backward_primitives.append(prim)

# 3. 메타데이터 업데이트 (궤적 개수 수정)
data['lattice_metadata']['number_of_trajectories'] = len(backward_primitives)
data['primitives'] = backward_primitives

print(f"Total primitives after (Backward Only): {len(backward_primitives)}")

# 4. 새로운 파일로 저장
with open(output_file, 'w') as f:
    json.dump(data, f, indent=4)

print(f"Saved to {output_file}")
```

#### **Step 4: Nav2 적용**

1.  생성된 `backward_only_primitives.json` 파일을 로봇의 패키지 경로로 옮깁니다.
2.  `nav2_params.yaml` 파일에서 Smac Planner 설정을 수정합니다.

<!-- end list -->

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      downsampler_costmap: false
      # ... 기타 설정 ...
      motion_model_for_search: "DUBINS" # 또는 REEDS_SHEPP
      angle_quantization_bins: 16
      lattice_filepath: "/path/to/backward_only_primitives.json" # 경로 지정
```

### **전문가 조언 (주의사항)**

1.  **Analytic Expansion (해석적 확장) 주의:** Smac Planner는 격자 탐색(A\*) 외에도 목표 지점 근처에서 `Analytic Expansion`을 수행하여 경로를 빠르게 찾습니다.
      * 만약 이 기능이 켜져 있고 비율(`analytic_expansion_ratio`)이 높으면, Lattice 파일에 후진만 있어도 **마지막 구간에서 전진 경로(Reeds-Shepp 곡선 등)를 생성**해버릴 수 있습니다.
      * 완벽한 후진을 원한다면 `nav2_params.yaml`에서 `analytic_expansion_ratio`를 `None`으로 끄거나 매우 신중하게 튜닝해야 합니다. 하지만 이를 끄면 목표 지점 도달 정밀도가 떨어질 수 있습니다.
2.  **Blind Spot:** 후진만 가능한 로봇은 센서(Lidar/Depth Camera)가 뒤쪽에 충분히 커버되는지 확인이 필수입니다. Nav2는 Costmap을 기반으로 충돌을 체크하므로 뒤쪽 시야가 없으면 위험합니다.

이 설명이 도움이 되셨나요? 다음 단계로 **위에서 작성해드린 필터링 스크립트를 실행하여 실제 파일을 만들어드리는 것**도 가능합니다. 필요하시면 말씀해주세요.


###################  
######################  
#################  
좋은 접근입니다. 생성된 Lattice Primitive가 실제로 의도한 대로(후진으로) 동작하는지, 부드럽게 연결되는지 눈으로 확인하는 것은 매우 중요합니다.

두 가지 방법을 제안합니다.
**1. 생성기 자체 기능 사용 (가장 간단)**
**2. 커스텀 Python 스크립트 사용 (필터링된 파일 확인용)**

-----

### 방법 1. 생성기 자체의 시각화 기능 사용 (README 기반)

공유해주신 `readme.md`에 따르면, 생성 스크립트 실행 시 `--visualizations` 플래그를 추가하면 생성된 궤적을 이미지로 저장해줍니다.

**실행 명령어:**

```bash
# 이미지를 저장할 폴더 경로를 지정합니다 (예: ./vis_output)
python3 generate_motion_primitives.py --config config.json --output primitives.json --visualizations ./vis_output
```

**결과:**

  * 지정한 폴더(`vis_output`) 안에 `.png` 이미지 파일들이 생성됩니다.
  * `all_trajectories.png`: 모든 궤적을 겹쳐서 보여줍니다.
  * 각도별 궤적 이미지들도 별도로 생성될 수 있습니다.

-----

### 방법 2. 커스텀 Python 뷰어 (추천)

**이 방법을 강력 추천합니다.**
이유:

1.  앞서 말씀드린 \*\*"후진만 남긴 필터링된 JSON 파일"\*\*을 시각화하여 검증할 수 있습니다.
2.  \*\*방향(화살표)\*\*을 표시하여 실제로 후진하는지 명확히 볼 수 있습니다.

아래 코드를 `view_primitives.py`로 저장하고 실행해 보세요.

```python
import json
import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# 설정: 확인하고 싶은 JSON 파일 경로 입력
FILE_PATH = 'backward_only_primitives.json' 
# ==========================================

def visualize_primitives(file_path):
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: 파일을 찾을 수 없습니다. 경로를 확인하세요: {file_path}")
        return

    primitives = data['primitives']
    metadata = data['lattice_metadata']
    
    print(f"Visualizing {len(primitives)} trajectories from {file_path}")
    print(f"Motion Model: {metadata.get('motion_model', 'Unknown')}")

    plt.figure(figsize=(10, 10))
    
    # 좌표축 설정 (로봇은 (0,0)에 위치, 앞쪽이 +x)
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    
    # 로봇 위치 표시 (원점)
    plt.plot(0, 0, 'ro', markersize=10, label='Robot Start (0,0)')
    # 로봇 전방 방향 화살표
    plt.arrow(0, 0, 0.2, 0, head_width=0.05, head_length=0.05, fc='r', ec='r')

    for i, prim in enumerate(primitives):
        poses = prim['poses']
        # x, y 좌표 추출
        x = [p[0] for p in poses]
        y = [p[1] for p in poses]
        
        # 궤적 그리기 (투명도 조절하여 겹쳐보이게)
        line, = plt.plot(x, y, alpha=0.4, linewidth=1)
        
        # **중요**: 방향 확인을 위해 끝점에 화살표 추가
        # 끝점의 yaw를 사용하여 화살표 방향 계산
        end_x, end_y, end_yaw = poses[-1]
        
        # 화살표 그리기 (궤적 색상과 동일하게)
        plt.arrow(end_x, end_y, 
                  0.05 * np.cos(end_yaw), 0.05 * np.sin(end_yaw),
                  head_width=0.02, head_length=0.03, 
                  fc=line.get_color(), ec=line.get_color(), alpha=0.8)

    plt.title(f"Nav2 Lattice Primitives Visualization\nFile: {file_path}")
    plt.xlabel("X (meter) - Forward(+)/Backward(-)")
    plt.ylabel("Y (meter)")
    plt.grid(True)
    plt.axis('equal') # x, y 비율 1:1 고정
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # 필요한 라이브러리: matplotlib, numpy
    # pip install matplotlib numpy
    visualize_primitives(FILE_PATH)
```

### 시각화 결과 해석 가이드 (전문가 팁)

이 코드를 실행했을 때 다음 사항을 중점적으로 확인하세요.

1.  **X축 음수 영역 분포:**
      * 모든 선들이 원점(0,0)을 기준으로 \*\*왼쪽(X \< 0)\*\*으로 뻗어나가야 합니다.
      * 오른쪽(X \> 0)으로 가는 선이 하나라도 있다면 "전진" 궤적이 섞여 있는 것입니다.
2.  **화살표 방향:**
      * 궤적 끝에 달린 화살표가 로봇이 바라보는 방향(Heading)입니다.
      * 후진 주행 시, 로봇 몸체는 뒤로 가지만 **헤딩(Yaw) 자체는 궤적의 접선 방향과 반대일 수도 있고(Ackermann 후진), 로봇의 뒷면을 기준으로 정의될 수도 있습니다.**
      * Smac Planner에서는 보통 후진 시에도 **로봇의 전면(Front)이 가리키는 방향**이 Yaw 값입니다. 따라서 궤적은 뒤로 가지만(-x), 화살표(Yaw)는 여전히 앞쪽 비슷한 방향을 가리키거나, 회전하면서 변할 것입니다.
3.  **부채꼴 모양 (Fan Shape):**
      * 궤적들이 부채꼴처럼 고르게 퍼져 있어야 합니다. 특정 각도로만 쏠려 있다면 장애물 회피 성능이 떨어집니다.

혹시 시각화 실행에 문제가 있거나, 결과 해석이 애매하면 결과 이미지를 설명해주시거나 상황을 알려주세요. 추가로 분석해드리겠습니다.
#####################  
############################  
############################  
##########################  
이 에러는 현재 실행 중인 환경(특히 **Docker 컨테이너**나 **VS Code의 터미널** 환경)이 그래픽 창(GUI)을 띄울 수 없는 상태(Headless)이거나, Matplotlib의 백엔드가 `Agg`(이미지 생성 전용, 비대화형)로 설정되어 있어서 발생합니다.

사용자님의 환경이 **Docker**와 **Ubuntu Server(GUI 없음)** 기반일 가능성이 높으므로, 창을 띄우는 `plt.show()` 대신 \*\*이미지 파일로 저장하는 `plt.savefig()`\*\*를 사용하는 것이 가장 확실한 해결책입니다.

코드를 다음과 같이 수정해 보세요.

### 수정된 `view_primitives.py` (이미지 저장 방식)

`plt.show()` 부분을 주석 처리하고 `plt.savefig()`를 추가했습니다.

```python
import json
import matplotlib
# GUI가 없는 환경(Docker 등)에서도 동작하도록 Agg 백엔드 강제 설정
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# 설정: 확인하고 싶은 JSON 파일 경로 입력
FILE_PATH = 'backward_only_primitives.json' 
OUTPUT_IMG_NAME = 'primitives_result.png' # 저장할 이미지 파일 이름
# ==========================================

def visualize_primitives(file_path):
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: 파일을 찾을 수 없습니다. 경로를 확인하세요: {file_path}")
        return

    primitives = data['primitives']
    metadata = data['lattice_metadata']
    
    print(f"Visualizing {len(primitives)} trajectories from {file_path}")
    
    plt.figure(figsize=(10, 10))
    
    # 좌표축 설정 (로봇은 (0,0)에 위치, 앞쪽이 +x)
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    
    # 로봇 위치 표시 (원점)
    plt.plot(0, 0, 'ro', markersize=10, label='Robot Start (0,0)')
    # 로봇 전방 방향 화살표 (붉은색)
    plt.arrow(0, 0, 0.2, 0, head_width=0.05, head_length=0.05, fc='r', ec='r')

    for i, prim in enumerate(primitives):
        poses = prim['poses']
        x = [p[0] for p in poses]
        y = [p[1] for p in poses]
        
        # 궤적 그리기
        line, = plt.plot(x, y, alpha=0.4, linewidth=1)
        
        # 방향 확인을 위해 끝점에 화살표 추가
        end_x, end_y, end_yaw = poses[-1]
        
        plt.arrow(end_x, end_y, 
                  0.05 * np.cos(end_yaw), 0.05 * np.sin(end_yaw),
                  head_width=0.02, head_length=0.03, 
                  fc=line.get_color(), ec=line.get_color(), alpha=0.8)

    plt.title(f"Nav2 Lattice Primitives Visualization\nFile: {file_path}")
    plt.xlabel("X (meter) - Forward(+)/Backward(-)")
    plt.ylabel("Y (meter)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()

    # [수정됨] 화면에 출력하는 대신 파일로 저장합니다.
    # plt.show() 
    plt.savefig(OUTPUT_IMG_NAME)
    print(f"Success! Image saved to '{OUTPUT_IMG_NAME}'")

if __name__ == "__main__":
    visualize_primitives(FILE_PATH)
```

### 해결 방법 요약 및 추가 팁

1.  **코드 수정 (위 코드 사용):**

      * `matplotlib.use('Agg')`: 코드가 시작될 때 명시적으로 비대화형 백엔드를 사용하도록 설정합니다.
      * `plt.savefig('파일명.png')`: 창을 띄우는 대신 결과물을 이미지 파일로 저장합니다.
      * 실행 후 생성된 `primitives_result.png` 파일을 VS Code 탐색기에서 클릭하여 확인하시면 됩니다.

2.  **VS Code 팁 (Jupyter 활용):**

      * Python 스크립트(`.py`) 대신 \*\*Jupyter Notebook (`.ipynb`)\*\*을 사용하시면 VS Code 내부에서 이미지를 바로 렌더링해서 보여줍니다. 데이터 시각화 작업 시에는 노트북 환경이 훨씬 편리할 수 있습니다.

수정된 코드로 다시 실행해 보시고, 생성된 이미지에서 **모든 궤적이 X축 왼쪽(음수, 뒤쪽)으로 뻗어 있는지** 확인해 보시기 바랍니다.

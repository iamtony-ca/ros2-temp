**네, 정확합니다! 바로 그겁니다.**

사용자님의 이해대로 `/map` 토픽과 `/lane_mask` (사용자가 정한 이름) 토픽이 **동시에 발행(Publish)** 됩니다. 마치 포토샵에서 여러 레이어를 겹쳐서 보는 것과 똑같은 원리입니다.

혼동을 없애기 위해 **ROS 2 노드 구조가 어떻게 돌아가는지** 명확하게 교통정리 해드릴게요.

---

### 1. 두 개의 "Map Server"가 동시에 돌아갑니다

Nav2에서는 `nav2_map_server`라는 프로그램을 여러 개 띄울 수 있습니다. 이름만 다르게 해서요.

1. **Main Map Server (기존):**
* **노드 이름:** `map_server`
* **하는 일:** 벽과 빈 공간이 그려진 `map.yaml`을 읽음
* **발행 토픽:** `/map`
* **누가 씀?:** AMCL(위치 추정), Costmap(Static Layer)


2. **Mask Map Server (새로 추가):**
* **노드 이름:** `mask_server` (이름은 짓기 나름)
* **하는 일:** 중앙 분리대가 그려진 `lane_mask.yaml`을 읽음
* **발행 토픽:** `/lane_mask`
* **누가 씀?:** Costmap(Filter Layer) via Filter Info Server



즉, 로봇 내부에서는 **두 개의 지도가 겹쳐서 동시에 존재**하게 됩니다.

---

### 2. [중요] Launch 파일 수정 없이는 작동 안 합니다!

`params.yaml`에 설정을 적었다고 해서 자동으로 서버가 실행되는 건 아닙니다. **Launch 파일에서 "두 번째 맵 서버"를 실행시켜주는 코드를 추가**해야 합니다.

보통 `bringup_launch.py` 또는 `localization_launch.py` 같은 곳에 맵 서버를 실행하는 부분이 있을 텐데, 그 아래에 **마스크용 서버를 하나 더 추가**해야 합니다.

#### Launch 파일 수정 예시 (Python)

```python
from launch_ros.actions import Node

# 1. 기존 맵 서버 (건드리지 않음)
map_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    output='screen',
    parameters=[configured_params], # map.yaml 경로 포함
    remappings=[('/map', '/map')]
)

# 2. [추가] 마스크용 맵 서버 (새로 추가해야 함!)
mask_server_node = Node(
    package='nav2_map_server',
    executable='map_server',
    name='mask_server',             # 노드 이름이 달라야 함!
    output='screen',
    parameters=[configured_params], # lane_mask.yaml 경로는 params.yaml에서 가져옴
    # remappings가 없으면 params.yaml의 topic_name을 따름
)

# 3. [추가] 필터 정보 서버 (새로 추가해야 함!)
costmap_filter_info_server_node = Node(
    package='nav2_map_server',
    executable='costmap_filter_info_server',
    name='costmap_filter_info_server',
    output='screen',
    emulate_tty=True,
    parameters=[configured_params]
)

# 4. [매우 중요] Lifecycle Manager 목록에 추가
# Nav2 노드들은 Lifecycle 관리자가 "켜져라(Active)" 해야 작동합니다.
lifecycle_nodes = [
    'map_server', 
    'amcl',
    'mask_server',                # <-- 목록에 추가 필수
    'costmap_filter_info_server'  # <-- 목록에 추가 필수
]

```

---

### 3. 전체 데이터 흐름도

이렇게 설정하면 데이터가 아래와 같이 흐릅니다.

1. **`map_server` (Node)** ➡️ `/map` (Topic) ➡️ **Global Costmap (Static Layer)**
* "여기 벽이 있어." (기본 지도)


2. **`mask_server` (Node)** ➡️ `/lane_mask` (Topic) ⬇️
3. **`costmap_filter_info_server` (Node)** ➡️ `/costmap_filter_info` (Topic) ➡️ **Global Costmap (Filter Layer)**
* "이 마스크 위치는 비용을 10배로 해." (가중치 정보)


4. **Global Costmap (Final)**
* 1번과 3번 정보를 합쳐서 최종 경로 생성



### 결론

네, **`/map`과 `/lane_mask`는 동시에 Pub 되는 것이 정상이며 필수적**입니다.

* Launch 파일에서 **새로운 Node(`mask_server`)를 추가**했는지,
* **Lifecycle Manager 목록**에 그 노드 이름을 넣었는지

이 두 가지만 확인하시면 완벽하게 동작할 겁니다.

###############################  
################################  
#################################  

제공해주신 Nav2 문서는 **Costmap Filter의 개념, 구조, 그리고 설정 방법**을 아주 상세하게 설명하고 있습니다. 이 내용을 바탕으로 사용자님의 **"가상의 중앙 분리대(Virtual Median Strip)"** 구현에 필요한 핵심 내용을 요약하고, **문서 내용에 맞춰 제 기존 답변을 더 정교하게 수정(최적화)**해 드리겠습니다.

---

### 1. 문서 내용 핵심 요약

Nav2 문서는 Costmap Filter를 **"지도 위에 그림을 그려서 로봇의 행동을 제어하는 기능"**이라고 정의합니다.

#### 작동 구조 (Architecture)

로봇이 이 기능을 쓰려면 **3박자**가 맞아야 합니다.

1. **Filter Mask (그림 파일):** 행동을 제어할 구역을 색칠한 지도 파일 (`.pgm`).
2. **Servers (정보 전달):**
* `Map Server`: 마스크 그림 파일을 읽어서 배포.
* `Costmap Filter Info Server`: "이 그림은 금지 구역용이야" 또는 "이 그림은 속도 제한용이야" 같은 메타 정보(`type`, 변환 공식)를 배포.


3. **Costmap Plugin (실행):** 실제 Nav2의 Global/Local Costmap에 붙어서, 위 정보를 받아 비용(Cost)을 실시간으로 수정하는 플러그인.

#### 핵심 플러그인: `KeepoutFilter`

문서에서는 `KeepoutFilter`를 집중적으로 설명합니다.

* **주 용도:** 진입 금지 구역(Keep-out zones).
* **응용 용도:** **가중치 영역(Weighted areas layer).**
* 문서의 이 부분이 사용자님에게 가장 중요합니다: *"KeepoutFilter can also act as a 'weighted areas layer' by setting the OccupancyGrid to something between [1-99] non-occupied values."*
* 즉, **완전한 검은색(100)이 아니라 회색(1~99)으로 칠하면, 벽이 아니라 '진흙탕(비싼 길)'이 된다**는 뜻입니다.



---

### 2. 문서 내용을 반영한 "수정된" 최적 가이드

제 이전 답변에서는 "범용 CostmapFilter + Multiplier(수식)" 방식을 제안드렸으나, **Nav2 공식 문서의 표준인 `KeepoutFilter`를 사용하는 방식**으로 약간 수정하는 것이 호환성과 안정성 면에서 더 좋습니다. (문서에 따르면 `KeepoutFilter`는 `multiplier` 파라미터를 무시하고 픽셀 값을 그대로 쓴다고 명시되어 있습니다.)

사용자님의 목표인 **"넘을 수 있는 중앙 분리대"**를 **공식 문서 방식**으로 구현하려면 아래와 같이 하시면 됩니다.

#### A. 마스크 이미지 제작 (가장 중요!)

`multiplier`로 튜닝하는 대신, **이미지 색상의 진하기**로 비용을 조절합니다.

* **배경 (통행 가능):** 흰색 (값 0)
* **중앙 분리대 (가상의 비싼 길):** **진한 회색 (Dark Gray)**
* 완전 검은색(`0x00`)은 벽(Lethal)이 됩니다.
* **추천 색상:** `0x40` ~ `0x60` (십진수 64~96 정도)
* GIMP에서 Lightness를 조절하여 **"벽보다는 연하지만, 바닥보다는 훨씬 진한"** 색으로 중앙선을 그립니다.



#### B. Launch 파일 구성 (문서의 `Step 2` 참고)

문서에 있는 긴 파이썬 코드가 바로 제가 말씀드린 **"별도의 노드 추가"** 부분입니다.

* `mask_server` (문서상의 `keepout_filter_mask_server`)
* `costmap_filter_info_server` (문서상의 `keepout_costmap_filter_info_server`)
* 이 두 노드를 실행하고 `Lifecycle Manager`에 등록해야 합니다.

#### C. `params.yaml` 설정 (KeepoutFilter 사용)

문서의 가이드에 따라 `KeepoutFilter` 플러그인을 사용하되, **동적 장애물 회피를 위해 중앙선을 '회색'으로 처리했으므로 로봇은 그곳을 '지나갈 수는 있지만 싫어하는 곳'으로 인식**하게 됩니다.

```yaml
# 1. Global Costmap에 필터 적용
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"] # 이름 변경
      
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter" # 공식 플러그인 사용
        enabled: True
        filter_info_topic: "/costmap_filter_info"

# 2. 서버 설정 (문서 가이드 준수)
costmap_filter_info_server:
  ros__parameters:
    type: 0                    # Keepout Filter는 type 0 사용
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/lane_mask"
    base: 0.0                  # 문 서: KeepoutFilter는 이 값을 무시하지만 0.0으로 둠
    multiplier: 1.0            # 문 서: KeepoutFilter는 이 값을 무시하지만 1.0으로 둠

```

### 3. 결론: 무엇이 바뀌었나?

문서를 보니 제 이전 답변(Multiplier 사용)보다 **더 심플하고 표준적인 방법**이 명확해졌습니다.

1. **튜닝 방법 변경:** `params.yaml`에서 숫자를 바꾸는 게 아니라, **마스크 이미지(`pgm`)에서 중앙선의 색깔(명도)을 바꿉니다.**
* 로봇이 중앙을 너무 자주 침범하면? -> **더 진한 회색**으로 칠하세요.
* 로봇이 막혔는데도 중앙을 안 넘어가면? -> **더 연한 회색**으로 칠하세요.


2. **안전성:** 이 방식은 Nav2가 공식적으로 지원하는 `KeepoutFilter`의 "Weighted Area" 기능을 사용하는 것이므로 버그 없이 가장 확실하게 동작합니다.

**최종 추천:**
문서에 있는 `Step 4`의 실행 명령어를 참고하여, **"중앙에 회색 선이 그려진 마스크"**를 만들고 `KeepoutFilter`를 적용하세요. 이것이 **"유연한 중앙 분리대"**를 만드는 정석입니다.
###############################  
######################################  
#############################################  
제공해주신 데모 코드는 **Nav2의 표준 필터 실행 구조**를 아주 잘 보여주고 있습니다. 이 코드를 기반으로, 사용자님의 목표인 **"넘을 수 있는 중앙 분리대(Weighted Median Strip)"**를 구현하기 위한 **맞춤형 수정 가이드**를 단계별로 정리해 드립니다.

핵심은 **"코드는 그대로 쓰되, 설정(YAML)과 이미지(PGM)만 바꿔서 벽을 진흙탕으로 만드는 것"**입니다.

---

### 1단계: 마스크 이미지 제작 (Paint the "Soft Wall")

제공해주신 코드는 `mode: scale`을 사용하고 있습니다. 이는 이미지의 **명도(밝기)**를 0~100 사이의 점유율(Occupancy) 값으로 변환해 줍니다.

* **파일:** `median_mask.pgm` (기존 `keepout_mask.pgm` 대신)
* **그리는 법:**
* **배경 (통로 양옆):** **흰색 (255)** → Cost 0 (자유롭게 주행)
* **중앙 분리대:** **진한 회색 (Dark Gray, 값 약 50~80)**
* **검은색(0)**으로 칠하면 **절대 못 지나가는 벽(Lethal)**이 됩니다.
* **회색**으로 칠해야 **"비싸지만 지나갈 수 있는(High Cost)"** 영역이 됩니다.


* *Tip:* GIMP나 포토샵에서 명도(K값)를 70~80% 정도로 설정해서 선을 그으세요.



---

### 2단계: Mask YAML 수정 (`median_mask.yaml`)

데모 코드의 `keepout_mask.yaml`을 복사해서 아래와 같이 수정합니다.

```yaml
image: median_mask.pgm  # 위에서 만든 회색 중앙선 이미지
mode: scale             # [핵심] 명암을 비용으로 변환하는 모드
resolution: 0.05        # [중요] 사용 중인 메인 지도(map.yaml)와 똑같이 맞추세요!
origin: [-10.0, -10.0, 0.0] # [중요] 메인 지도와 똑같이 맞추세요!
negate: 0
occupied_thresh: 1.0    # 1.0으로 설정하여 완전 검은색만 100(벽)으로 인식되게 함 (안전장치)
free_thresh: 0.0        # 0.0으로 설정

```

---

### 3단계: Params YAML 수정 (`median_params.yaml`)

데모 코드의 `keepout_params.yaml`을 복사해서 아래와 같이 수정합니다.
여기서 `type: 0` (Keepout Filter)을 그대로 쓰지만, **마스크가 회색이기 때문에 Weighted Filter처럼 동작**하게 됩니다.

```yaml
costmap_filter_info_server:
  ros__parameters:
    use_sim_time: true
    type: 0                       # Keepout Filter 모드 사용
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/median_filter_mask" # 토픽 이름 구분 (권장)
    base: 0.0                     # Keepout은 이 값을 무시하지만 0.0 유지
    multiplier: 1.0               # Keepout은 이 값을 무시하지만 1.0 유지

filter_mask_server:
  ros__parameters:
    use_sim_time: true
    frame_id: "map"
    topic_name: "/median_filter_mask" # 위 mask_topic과 일치시켜야 함
    yaml_filename: "median_mask.yaml" # 2단계에서 만든 파일 경로

```

---

### 4단계: 실행 (Launch File 활용)

제공해주신 `costmap_filter_info.launch.py`는 수정할 필요가 **전혀 없습니다.** 아주 잘 짜여진 코드입니다.
사용자님의 메인 런치 파일(예: `bringup_launch.py`)에서 이 파일을 **Include** 하기만 하면 됩니다.

**`my_robot_bringup.launch.py` 예시:**

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ... (기존 코드들) ...

    # [추가] Costmap Filter 실행 (중앙 분리대)
    # nav2_costmap_filters_demo 패키지가 설치되어 있다고 가정하거나,
    # 해당 launch 파일을 내 패키지로 복사해서 경로를 지정하세요.
    
    # 예시: 내 패키지에 launch 파일을 복사해뒀다면
    my_pkg_dir = get_package_share_directory('my_robot_package')
    filter_launch_path = os.path.join(my_pkg_dir, 'launch', 'costmap_filter_info.launch.py')
    
    # 파라미터 파일 경로 정의
    params_file_path = os.path.join(my_pkg_dir, 'params', 'median_params.yaml')
    mask_file_path = os.path.join(my_pkg_dir, 'maps', 'median_mask.yaml')

    costmap_filter_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(filter_launch_path),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'true',
            'params_file': params_file_path, # 3단계 파일
            'mask': mask_file_path           # 2단계 파일
        }.items()
    )

    ld.add_action(costmap_filter_cmd)
    
    return ld

```

---

### 5단계: Nav2 메인 파라미터 적용 (`nav2_params.yaml`)

마지막으로 로봇이 이 필터를 사용하도록 `nav2_params.yaml`의 **Global Costmap**에 등록합니다.

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # plugins 리스트의 '맨 뒤'에 추가하는 것을 잊지 마세요!
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"]
      
      # ... (다른 레이어 설정) ...

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: True
        filter_info_topic: "/costmap_filter_info" # 3단계에서 설정한 토픽과 일치

```

---

### 최종 요약

1. **이미지:** 중앙선을 **회색(Dark Gray)**으로 칠하세요. (검은색 X)
2. **YAML:** `mode: scale`을 사용하여 회색을 '높은 비용'으로 인식시키세요.
3. **Launch:** 제공된 데모 Launch 파일을 그대로 호출하되, **내가 만든 yaml 파일 경로**만 인자로 넘기세요.
4. **Config:** `global_costmap`에 `keepout_filter` 플러그인을 추가하세요.

이렇게 하면 **Nav2 데모 코드의 구조를 100% 재활용**하면서, 기능만 **"완전 차단(Wall)"**에서 **"유연한 차단(Weighted Lane)"**으로 영리하게 바꿀 수 있습니다.

###################################  
####################################  
######################################  
현재 제공해주신 `bringup_launch.py`, `navigation_launch.py`, `localization_launch.py` 중에서 **수정이 필요한 파일은 오직 `bringup_launch.py` 하나뿐입니다.**

나머지 `navigation_launch.py`와 `localization_launch.py`는 건드릴 필요가 없습니다. Costmap Filter 서버들은 독립적인 노드(Map Server의 일종)이기 때문에, `bringup` 단계에서 별도로 실행시켜주는 것이 구조상 가장 깔끔합니다.

### 수정 가이드

이전에 보여주신 **`nav2_costmap_filters_demo/launch/costmap_filter_info.launch.py`** 파일의 내용을 **`filters_launch.py`** 라는 이름으로 저장하여 님 패키지의 launch 폴더에 넣어두었다고 가정하고 작성하겠습니다.

---

### [nav2_bringup/launch/bringup_launch.py] 수정본

주석 **`# [추가]`** 라고 표시된 부분만 추가하시면 됩니다.

```python
# ... (기존 import 문들) ...

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # ... (기존 변수 선언들) ...
    
    # [추가] 필터 관련 변수 선언
    # 님 패키지 이름을 여기에 넣으세요 (예: 'my_robot_bringup')
    # 만약 nav2_costmap_filters_demo 패키지가 설치되어 있다면 그대로 둬도 됩니다.
    # 하지만 보통 커스텀 패키지 내에 두는 것이 좋습니다.
    # filters_dir = get_package_share_directory('my_robot_bringup') 
    
    # 여기서는 nav2_costmap_filters_demo 패키지를 예시로 사용 (없다면 본인 패키지로 경로 수정 필수)
    try:
        filters_dir = get_package_share_directory('nav2_costmap_filters_demo')
        filters_launch_dir = os.path.join(filters_dir, 'launch')
    except:
        filters_dir = launch_dir # 임시 (실제 경로 확인 필요)

    # [추가] Launch Configuration 변수
    use_filters = LaunchConfiguration('use_filters')
    filter_params_file = LaunchConfiguration('filter_params_file')
    filter_mask_file = LaunchConfiguration('filter_mask_file')

    # ... (기존 remappings, params_file 처리 로직) ...

    # ... (기존 declare_namespace_cmd 등 선언) ...

    # [추가] 필터 관련 Launch Argument 선언
    declare_use_filters_cmd = DeclareLaunchArgument(
        'use_filters', default_value='True',
        description='Whether to enable costmap filters'
    )

    declare_filter_params_file_cmd = DeclareLaunchArgument(
        'filter_params_file',
        # 기본값은 본인의 median_params.yaml 경로로 설정
        default_value=os.path.join(bringup_dir, 'params', 'keepout_params.yaml'),
        description='Full path to the Costmap Filter params file'
    )

    declare_filter_mask_file_cmd = DeclareLaunchArgument(
        'filter_mask_file',
        # 기본값은 본인의 median_mask.yaml 경로로 설정
        default_value=os.path.join(bringup_dir, 'maps', 'keepout_mask.yaml'),
        description='Full path to the Costmap Filter mask file'
    )

    # ... (기존 declare 명령어들) ...

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            
            # ... (기존 nav2_container Node) ...
            
            # ... (기존 SLAM IncludeLaunchDescription) ...

            # ... (기존 Localization IncludeLaunchDescription) ...

            # [추가] Costmap Filters 실행 (Localization 다음, Navigation 이전에 배치 추천)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # 아까 저장해둔 filters_launch.py 파일 경로
                    os.path.join(filters_launch_dir, 'costmap_filter_info.launch.py')
                ),
                condition=IfCondition(use_filters),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': filter_params_file, # 필터 전용 파라미터 전달
                    'mask': filter_mask_file,          # 필터 마스크 전달
                    'use_composition': use_composition,
                    'container_name': 'nav2_container', # 컴포지션 사용 시 같은 컨테이너에 로드
                }.items(),
            ),

            # ... (기존 Navigation IncludeLaunchDescription) ...
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # ... (기존 ld.add_action 들) ...

    # [추가] 필터 관련 Argument 등록
    ld.add_action(declare_use_filters_cmd)
    ld.add_action(declare_filter_params_file_cmd)
    ld.add_action(declare_filter_mask_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld

```

### 적용 포인트 요약

1. **파일 위치:** `bringup_launch.py`만 수정합니다.
2. **경로 확보:** `costmap_filter_info.launch.py`가 있는 패키지 경로(`get_package_share_directory`)를 정확히 지정해야 합니다. (본인의 로봇 패키지에 복사해두는 것을 추천)
3. **인자 전달:** `params_file`(Nav2 메인 파라미터)과 `filter_params_file`(필터 서버용 파라미터)이 섞이지 않도록 주의하세요. 필터 런치 파일에는 필터용 파라미터만 넘겨야 합니다.
4. **실행 순서:** `bringup_cmd_group` 리스트 안에서 `Localization` 실행 후, `Navigation` 실행 전에 필터를 실행하는 것이 논리적으로 가장 깔끔합니다.

이제 이 `bringup_launch.py`를 실행할 때 `filter_mask_file` 경로에 아까 만든 **회색 중앙선 마스크 yaml**을 지정해주면 모든 시스템이 연동됩니다.
#####################  
#########################  
##########################  
네, **충분히 하나로 통합할 수 있습니다!** 👍

원래 데모 코드에서는 관리를 편하게 하려고 파일을 분리해 둔 것이지만, ROS 2의 파라미터 파일 구조상 **노드 이름(Node Name)**만 다르다면 하나의 YAML 파일(`nav2_params.yaml`)에 모두 몰아넣어도 아무런 문제가 없습니다.

오히려 **파일 하나로 모든 설정을 관리**하고 싶다면 통합하는 것이 훨씬 깔끔할 수 있습니다.

통합하는 방법과 그에 따른 Launch 파일 수정 사항을 알려드리겠습니다.

---

### 1. `nav2_params.yaml`에 내용 합치기

기존 `nav2_params.yaml` 파일을 열어보시면 `amcl:`, `bt_navigator:`, `controller_server:` 처럼 각 노드의 이름이 최상위 키(Key)로 되어 있을 겁니다.

여기에 **필터 관련 서버 2개의 설정**을 그대로 복사해서 붙여넣으시면 됩니다. (들여쓰기 주의: 최상위 레벨에 위치해야 함)

**[통합된 nav2_params.yaml 예시]**

```yaml
amcl:
  ros__parameters:
    # ... (기존 설정) ...

bt_navigator:
  ros__parameters:
    # ... (기존 설정) ...

# ... (기존 nav2 노드들 설정) ...

# =========================================================
# [추가] Costmap Filter 관련 설정 (여기에 붙여넣기)
# =========================================================

costmap_filter_info_server:
  ros__parameters:
    use_sim_time: true
    type: 0                       # Keepout Filter (Weighted 모드 활용)
    filter_info_topic: "/costmap_filter_info"
    mask_topic: "/median_filter_mask" 
    base: 0.0
    multiplier: 1.0

filter_mask_server:
  ros__parameters:
    use_sim_time: true
    frame_id: "map"
    topic_name: "/median_filter_mask"
    yaml_filename: "median_mask.yaml"  # 경로는 launch에서 관리하거나 절대경로 추천

```

---

### 2. `bringup_launch.py` 수정 (중요!)

파라미터 파일이 하나로 합쳐졌으니, Launch 파일에서 **"필터용 파라미터 파일 경로"**를 따로 받을 필요가 없어졌습니다. 그냥 **메인 `params_file`을 필터 실행 시에도 똑같이 넘겨주면 됩니다.**

아까 작성해 드린 `bringup_launch.py` 수정본에서 아래 부분만 살짝 바꾸면 됩니다.

**[수정 전]**

```python
    declare_filter_params_file_cmd = DeclareLaunchArgument(
        'filter_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'keepout_params.yaml'), # 별도 파일
        description='Full path to the Costmap Filter params file'
    )
    
    # ... (생략) ...

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource( ... ),
                launch_arguments={
                    # ...
                    'params_file': filter_params_file, # 별도 변수 사용
                    # ...
                }.items(),
            ),

```

**[수정 후: 통합된 파일 사용]**

```python
    # 1. declare_filter_params_file_cmd 삭제 가능 (필요 없음)

    # ... (생략) ...

            # [추가] Costmap Filters 실행
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(filters_launch_dir, 'costmap_filter_info.launch.py')
                ),
                condition=IfCondition(use_filters),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    
                    # [핵심 변경] 필터 전용 파일 대신, 메인 params_file을 그대로 전달
                    'params_file': params_file, 
                    
                    'mask': filter_mask_file,
                    'use_composition': use_composition,
                    'container_name': 'nav2_container',
                }.items(),
            ),

```

---

### 3. 주의사항: `yaml_filename` 경로 문제

`nav2_params.yaml`에 설정을 통합할 때 딱 하나 주의할 점은 **마스크 yaml 파일의 경로(`yaml_filename`)**입니다.

```yaml
filter_mask_server:
  ros__parameters:
    # ...
    # 이 부분은 Launch 파일에서 'mask' 인자로 덮어쓰기(Overwrite) 되므로
    # 여기 적힌 값은 사실상 무시되거나 기본값으로 쓰입니다.
    yaml_filename: "median_mask.yaml" 

```

`costmap_filter_info.launch.py` 내부를 보면, Launch Argument로 받은 `mask` 값이 파라미터 파일의 설정을 덮어쓰도록 되어 있습니다(`RewrittenYaml`).

따라서:

1. **`nav2_params.yaml`에 적는 `yaml_filename`**은 비워두거나 아무거나 적어도 큰 상관없습니다.
2. 중요한 건 **`bringup_launch.py`를 실행할 때 `filter_mask_file` 인자에 올바른 경로가 들어가느냐**입니다.

### 요약

1. **통합 가능:** `nav2_params.yaml` 맨 아래에 `costmap_filter_info_server`와 `filter_mask_server` 항목을 복사/붙여넣기 하세요.
2. **Launch 수정:** `bringup_launch.py`에서 필터 실행 시 `params_file` 인자에 **메인 Nav2 파라미터 변수(`params_file`)**를 그대로 넘겨주세요.
3. **관리 효율:** 이렇게 하면 파일 하나로 모든 파라미터 튜닝이 가능해져서 관리가 훨씬 편해집니다. 추천합니다! 👍

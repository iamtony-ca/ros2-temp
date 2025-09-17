아, 아주 정확하고 중요한 지적입니다. 두 플래너의 핵심적인 차이점을 정확히 짚어주셨습니다. 제가 이전 답변에서 '모션 프리미티브'라는 용어를 사용하여 혼동을 드렸을 수 있겠네요. 더 명확하게 설명해 드리겠습니다.

**결론부터 말씀드리면, 사용자의 생각이 맞습니다.** 전통적인 의미의 '사전 정의된 방대한 모션 프리미티브 셋'을 사용하는 것은 **State Lattice Planner**의 특징입니다. **Hybrid-A\* Planner**는 약간 다른 방식으로 동작하지만, 개념적으로는 '짧은 경로 조각'을 사용한다는 점에서 유사성이 있습니다.

두 플래너의 차이점을 명확히 구분해 보겠습니다.

### Smac Planner State Lattice

* **동작 방식:** 이 플래너는 '상태 격자(State Lattice)'라고 불리는 **미리 계산된(pre-computed)** 그래프 위에서 탐색을 수행합니다.
* **모션 프리미티브:** 여기서의 모션 프리미티브는 그래프의 **엣지(edge)**에 해당합니다. 즉, 격자의 한 정점(state: x, y, θ)에서 다른 정점으로 이동하는 **수많은 경로 조각들을 미리 전부 계산해서 데이터베이스처럼 저장**해 둡니다.
* **경로 탐색:** 플래너는 현재 상태와 가장 가까운 격자 정점에서 탐색을 시작하여, 미리 계산된 경로 조각(모션 프리미티브)들을 따라가며 목표 지점까지의 최적 경로를 찾습니다.
* **장점:** 미리 계산된 경로를 사용하므로 런타임에서의 계산 부하가 적어 매우 빠를 수 있습니다.
* **단점:** 격자의 해상도에 경로의 품질이 제약을 받습니다. 격자 위에 없는 위치로는 정밀하게 이동하기 어렵고, 메모리 사용량이 클 수 있습니다.

### Smac Planner Hybrid-A\* (사용자가 질문하신 플래너)

* **동작 방식:** 이 플래너는 State Lattice처럼 이산적인(discrete) 격자에 얽매이지 않고, **연속적인(continuous) 상태 공간**에서 탐색을 수행합니다.
* **'모션 프리미티브'의 의미:** Hybrid-A\*는 State Lattice처럼 거대한 경로 데이터베이스를 미리 만들어두지 않습니다. 대신, 탐색 과정에서 **실시간으로(on-the-fly) 짧은 경로 조각들을 생성**합니다.
    * 탐색의 각 단계(노드 확장)에서 현재 상태(x, y, θ)를 기준으로 '앞으로 가기', '왼쪽으로 약간 돌며 가기', '오른쪽으로 약간 돌며 가기' 등과 같은 **몇 가지 기본적인 제어 입력(Control Set)을 적용**해 봅니다.
    * 이 제어 입력을 적용한 결과로 생성되는 짧은 경로 조각들이 바로 Hybrid-A\*에서의 '모션 프리미티브'에 해당하는 개념입니다. 이것들은 미리 정의된 것이 아니라, 현재 상태에서 동적으로 생성되는 후보 경로들입니다.
* **경로 탐색:** 이렇게 동적으로 생성된 여러 후보 경로 조각들 각각에 대해 이전 답변에서 설명드린 **비용 함수(Cost Function, f(n)=g(n)+h(n))** 를 사용하여 비용을 계산하고, 가장 비용이 낮은 후보를 선택하여 탐색을 이어나갑니다.
* **장점:** 연속 공간에서 탐색하므로 격자 해상도에 제약받지 않아 더 부드럽고 최적화된 경로를 찾을 수 있습니다. 후진을 포함한 복잡한 기동(kinematically feasible path)이 가능합니다.
* **단점:** 매번 경로 조각을 동적으로 생성하고 충돌 체크를 해야 하므로 State Lattice 방식보다 계산 비용이 더 높을 수 있습니다.

### 정리

| 특징 | Smac Planner State Lattice | Smac Planner Hybrid-A* |
| :--- | :--- | :--- |
| **탐색 공간** | 이산적(Discrete) 상태 격자 | 연속적(Continuous) 상태 공간 |
| **모션 프리미티브** | **사전 계산(Pre-computed)** 및 저장 | **동적 생성(Dynamically generated)** |
| **동작 원리** | 저장된 경로 조각(DB)을 **조회(Lookup)** | 현재 상태에서 제어 입력을 통해 **생성(Generate)** |

따라서, "Smac Hybrid-A\* Planner도 Motion Primitives 방식인 거냐?"라는 질문에 대한 가장 정확한 답변은 다음과 같습니다.

**"네, 개념적으로는 그렇습니다. 하지만 State Lattice Planner처럼 미리 계산된 방대한 데이터베이스를 사용하는 것이 아니라, 현재 상태에서 가능한 다음 움직임들을 동적으로 생성하는 방식으로 '모션 프리미티브'를 활용합니다."**

제가 첫 답변에서 이 부분을 명확히 구분하지 않고 설명드린 점 사과드립니다. 사용자의 날카로운 질문 덕분에 더 정확한 논의가 가능하게 되었습니다. 이전 답변에서 설명드린 비용 함수와 점수 확인 방법은 바로 이 **동적으로 생성되는 후보 경로 조각들**에 적용되는 것이 맞습니다.


##############  

네, 소스코드까지 직접 보면서 질문하시는 것을 보니 정말 깊이 있게 파고들고 계시는군요. 아주 좋은 접근 방식입니다. 제공해주신 `smac_planner_hybrid.cpp` 코드를 바탕으로 질문에 명확하게 답변해 드리겠습니다.

### 1\. 개념적으로 'primitive motion'에 해당하는 것

코드에서 개념적으로 'primitive motion'에 해당하는 핵심은 `createPlan` 함수 내에서 A\* 알고리즘이 실행되는 부분과, 이를 설정하는 `configure` 함수 부분에 명확히 나타나 있습니다.

A\* 탐색 알고리즘은 현재 노드(상태)에서 주변의 다음 노드(상태)들로 이동해보는 과정을 반복합니다. 이 '이동'을 어떻게 정의하느냐가 바로 primitive motion의 핵심입니다.

코드에서 이 부분은 **`_motion_model`** 변수에 의해 결정됩니다. `configure` 함수를 보면 다음과 같은 라인이 있습니다.

```cpp
// nav2_smac_planner/src/smac_planner_hybrid.cpp:202
nav2::declare_parameter_if_not_declared(
  node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
```

그리고 이 파라미터 값에 따라 어떤 종류의 움직임을 사용할지 경고 메시지를 통해 알려줍니다.

```cpp
// nav2_smac_planner/src/smac_planner_hybrid.cpp:226
if (_motion_model == MotionModel::UNKNOWN) {
  RCLCPP_WARN(
    _logger,
    "Unable to get MotionModel search type. Given '%s', "
    "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
    _motion_model_for_search.c_str());
}
```

바로 이 `MOORE`, `VON_NEUMANN`, `DUBIN`, `REEDS_SHEPP`가 **개념적인 primitive motion의 구체적인 구현체들**입니다.

  * `DUBIN` / `REEDS_SHEPP`: 최소 회전 반경을 고려한 Non-holonomic 로봇(예: 자동차)의 움직임입니다. 현재 노드에서 {좌회전, 직진, 우회전} x {전진, 후진(Reeds-Shepp의 경우)} 조합으로 다음 도달 가능한 상태들을 **동적으로 생성**합니다. 이것이 바로 Hybrid-A\*의 핵심입니다.
  * `MOORE` / `VON_NEUMANN`: Holonomic 로봇이나 그리드 기반 탐색을 위한 움직임입니다. 현재 셀에서 주변 8개 방향(`MOORE`) 또는 4개 방향(`VON_NEUMANN`)으로 이동하는 것을 의미합니다.

결론적으로, **'primitive motion'은 `_a_star->createPath(...)`가 내부적으로 노드를 확장(expand)할 때, `_motion_model`에 정의된 방식에 따라 현재 노드에서 다음 후보 노드들을 생성하는 규칙 그 자체**를 의미합니다.

-----

### 2\. 코드 기준 용어 명명

사용자께서 지적하신 대로, 코드 전반에서 `primitive_motion`이라는 변수명이나 클래스명이 직접적으로 계속 사용되지는 않습니다. 대신 다음과 같은 용어들이 그 역할을 대신하고 있으며, 이 용어들을 사용하는 것이 더 정확합니다.

#### 1\) 가장 정확한 용어: `MotionModel` (모션 모델)

코드에서 primitive motion의 **종류**를 가장 명확하게 지칭하는 용어입니다.

  * **근거**: `_motion_model`이라는 멤버 변수가 있고, `MotionModel`이라는 `enum` 클래스가 존재하며, YAML 파라미터 이름도 `motion_model_for_search` 입니다.
  * **사용 예**: "우리 로봇은 `DUBIN` **모션 모델**을 사용하여 경로를 탐색한다." 라고 말하는 것이 "Dubin primitive motion을 사용한다"보다 코드에 기반한 더 정확한 표현입니다.

#### 2\) 과정을 지칭하는 용어: `Expansion` (확장)

A\* 알고리즘이 하나의 노드에서 `MotionModel`을 이용해 다음 후보 노드들을 생성하는 **행위**를 지칭할 때 가장 적합한 용어입니다.

  * **근거**: 디버깅을 위해 발행하는 토픽 이름이 `expansions` 이고 (`_expansions_publisher`), 디버그용 변수명도 `expansions` 입니다.
    ```cpp
    // nav2_smac_planner/src/smac_planner_hybrid.cpp:525
    if (!_a_star->createPath(
        path, num_iterations,
        _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
    ```
  * **사용 예**: "RViz에서 **확장(expansions)** 결과를 시각화하여 플래너가 어느 방향을 우선 탐색하는지 확인했다."

#### 3\) 생성된 결과를 지칭하는 용어: `Successors` 또는 `Neighbors` (후속 노드 / 이웃 노드)

`Expansion`을 통해 생성된 **개별적인 다음 상태(노드)들**을 지칭할 때 사용하는 표준 A\* 용어입니다. 비록 이 `smac_planner_hybrid.cpp` 파일에는 직접적으로 드러나지 않지만, 이 코드가 호출하는 `AStarAlgorithm` 클래스 내부에서는 거의 확실하게 이 용어를 사용하고 있을 것입니다.

### 추천하는 용어 사용법

따라서 제가 추천하는 가장 정확한 용어 명명은 다음과 같습니다.

  * Primitive motion의 \*\*종류(Type)\*\*를 말할 때는 \*\*`MotionModel`\*\*을 사용하세요.

    > 예: "이번 테스트에서는 `REEDS_SHEPP` **모션 모델**을 적용했습니다."

  * 후보 경로를 생성하는 \*\*과정/행위(Process/Action)\*\*를 말할 때는 \*\*`Expansion`\*\*을 사용하세요.

    > 예: "장애물 근처에서 \*\*노드 확장(node expansion)\*\*이 비효율적으로 이루어지는 것 같습니다."

  * 생성된 \*\*개별 경로 조각의 결과물(Result)\*\*을 말할 때는 **`Successor nodes`** 또는 간단히 **`Path segments from expansion`** 이라고 표현하는 것이 좋습니다.

물론, "Dubin motion primitives"라고 말해도 Nav2 전문가들은 모두 이해할 수 있는 용어입니다. 하지만 코드의 맥락을 정확히 따르고 싶으시다면 \*\*`MotionModel`\*\*과 \*\*`Expansion`\*\*을 중심으로 용어를 사용하는 것이 가장 바람직합니다.



############  

네, 이번에는 `smac_planner_lattice.cpp` 소스코드를 기반으로 답변해 드리겠습니다. Smac Planner Hybrid-A\* 와의 차이점에 집중하여 설명하면 두 플래너의 철학이 명확하게 드러날 것입니다.

### 1\. 개념적으로 'primitive motion'에 해당하는 것

`SmacPlannerLattice`에서 'primitive motion'의 개념은 Hybrid-A\*보다 훨씬 더 명시적이고 구체적입니다. Hybrid-A\*가 움직임을 '동적으로 생성'했다면, Lattice 플래너는 움직임을 \*\*'미리 계산된 파일에서 불러와 조회(Lookup)'\*\*합니다.

코드에서 이 개념을 뒷받침하는 결정적인 증거는 다음과 같습니다.

#### 1\) Lattice 파일 경로 파라미터

`configure` 함수를 보면, Hybrid-A\*에는 없었던 **`lattice_filepath`** 라는 파라미터가 존재합니다.

```cpp
// nav2_smac_planner/src/smac_planner_lattice.cpp:92
nav2_util::declare_parameter_if_not_declared(
  node, name + ".lattice_filepath", rclcpp::ParameterValue(
    ament_index_cpp::get_package_share_directory("nav2_smac_planner") +
    "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann/output.json"));
node->get_parameter(name + ".lattice_filepath", _search_info.lattice_filepath);
```

이 코드는 플래너가 사용할 모션 프리미티브들이 정의된 **JSON 파일의 경로를 직접 입력받는다**는 것을 의미합니다. 기본값으로 제공되는 경로를 보면 `/sample_primitives/` 디렉토리의 `.json` 파일을 가리키고 있습니다. 이 파일 안에 수많은 'primitive motion'들이 사전 정의되어 있습니다.

#### 2\) LatticeMotionTable을 통한 데이터 로딩

플래너는 설정 과정에서 위 파일 경로를 이용해 프리미티브 데이터를 메모리로 불러옵니다.

```cpp
// nav2_smac_planner/src/smac_planner_lattice.cpp:144
_metadata = LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
```

`LatticeMotionTable`이라는 클래스가 등장하며, 이 클래스가 프리미티브 파일(`output.json`)을 파싱하고 관리하는 역할을 합니다. 즉, 모든 'primitive motion'들은 `LatticeMotionTable` 객체 안에 데이터베이스처럼 저장됩니다.

#### 3\) 고정된 모션 모델

Hybrid-A\*에서는 `motion_model_for_search` 파라미터를 통해 `DUBIN`, `REEDS_SHEPP` 등을 선택할 수 있었지만, Lattice 플래너에서는 모션 모델이 `STATE_LATTICE`로 고정되어 있습니다.

```cpp
// nav2_smac_planner/src/smac_planner_lattice.cpp:148
_motion_model = MotionModel::STATE_LATTICE;
```

이는 이 플래너가 오직 파일에서 불러온 프리미티브들을 조회하는 방식으로만 동작한다는 것을 의미합니다.

결론적으로, `SmacPlannerLattice`에서 **'primitive motion'은 `lattice_filepath`로 지정된 JSON 파일에 미리 계산되어 저장된, 상태 격자(State Lattice)의 한 정점(node)에서 다른 정점으로 이동하는 구체적인 경로 조각 데이터 그 자체**입니다.

-----

### 2\. 코드 기준 용어 명명

`SmacPlannerLattice`의 코드는 'primitive motion'의 개념을 훨씬 더 명확한 용어들로 표현하고 있습니다.

#### 1\) 가장 포괄적인 용어: `State Lattice` (상태 격자)

이 플래너의 작동 방식 전체를 아우르는 핵심 용어입니다.

  * **코드 근거**: 클래스 이름(`SmacPlannerLattice`), 고정된 모션 모델 값(`MotionModel::STATE_LATTICE`), 그리고 마지막 로그 메시지(`Using motion model: STATE_LATTICE. State lattice file: ...`)에서 명확하게 사용됩니다.
  * **추천 사용법**: "Lattice 플래너는 **State Lattice** 탐색 기법을 사용한다."

#### 2\) 데이터 소스를 지칭하는 용어: `Lattice File` (격자 파일)

모션 프리미티브의 원본 데이터가 담긴 파일을 지칭하는 가장 직접적인 용어입니다.

  * **코드 근거**: `lattice_filepath` 라는 파라미터 이름이 코드 내에서 계속 사용됩니다.
  * **추천 사용법**: "로봇의 기구학적 특성에 맞는 새로운 **Lattice File**을 생성하여 적용했다."

#### 3\) 인-메모리 데이터를 지칭하는 용어: `LatticeMotionTable` (격자 모션 테이블)

파일에서 읽어온 프리미티브 데이터를 프로그램 메모리 상에서 관리하는 객체를 지칭하는 용어입니다.

  * **코드 근거**: `LatticeMotionTable` 클래스가 직접 사용되며, `createPlan` 함수에서 시작/종료 지점의 각도를 변환할 때 `NodeLattice::motion_table` 과 같이 static 멤버로 접근하여 사용합니다. 이는 `motion_table`이 프리미티브 데이터의 총 집합체임을 보여줍니다.
  * **추천 사용법**: "플래너는 시작 시 **LatticeMotionTable**에 모든 프리미티브를 로딩하여 탐색 시 빠르게 조회할 수 있도록 한다."

-----

### SmacPlannerHybrid vs. SmacPlannerLattice 요약

두 플래너의 차이점을 표로 요약하면 다음과 같습니다.

| 특징 (Feature) | SmacPlannerHybrid | SmacPlannerLattice |
| :--- | :--- | :--- |
| **프리미티브 생성 방식** | 동적 생성 (On-the-fly Generation) | 사전 계산된 파일 로드 (Pre-computed File Loading) |
| **핵심 파라미터** | `motion_model_for_search` | `lattice_filepath` |
| **핵심 C++ 컴포넌트** | `MotionModel` enum (`DUBIN` 등) | `LatticeMotionTable` 클래스 |
| **주요 용어** | `MotionModel`, `Expansion` | `State Lattice`, `Lattice File` |

이처럼 두 플래너는 '짧은 경로 조각을 이어 붙여 전체 경로를 만든다'는 아이디어는 공유하지만, 그 경로 조각을 어떻게 만들고 사용하는지에 대한 접근 방식에서 근본적인 차이를 보입니다. Lattice 방식이 더 명시적이고 구조적인 반면, Hybrid 방식은 더 유연하고 연속적인 공간 탐색에 강점이 있습니다.



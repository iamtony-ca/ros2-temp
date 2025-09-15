네, 알겠습니다. 요청하신 대로 네임스페이스를 `amr_bt_nodes`로, 클래스 이름을 `StatefulRecoveryRobinControl`로 변경하여 전체 코드를 다시 제공해 드리겠습니다.

아래 코드는 바로 컴파일하여 플러그인으로 사용할 수 있도록 완벽한 형태로 작성되었습니다.

-----

### 1\. `stateful_recovery_robin_control.hpp` (헤더 파일)

```cpp
#ifndef AMR_BT_NODES__STATEFUL_RECOVERY_ROBIN_CONTROL_HPP_
#define AMR_BT_NODES__STATEFUL_RECOVERY_ROBIN_CONTROL_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

/**
 * @class StatefulRecoveryRobinControl
 * @brief A stateful, cycling control node designed for recovery sequences.
 *
 * This node ticks its children sequentially based on its internal state.
 * Its behavior is a hybrid between a Fallback and a stateful Sequence.
 *
 * - Rule 1 & 6 (Stateful & Cycling): It remembers the last executed child index
 * and will cycle back to the first child after the sequence is complete. It is
 * reset to the first child when halt() is called (e.g., at the start of a new action).
 *
 * - Rule 2 & 4 (Execution Policy): It attempts to find a successful child within a
 * single tick. If a child fails, it immediately tries the next child in the
 * same tick. It only stops execution in a tick if a child returns SUCCESS or RUNNING,
 * or if the final child fails.
 *
 * - Rule 3 (Success Condition): If any child returns SUCCESS, this node
 * immediately returns SUCCESS.
 *
 * - Rule 5 (Failure Condition): This node returns FAILURE only if the VERY LAST
 * child in the sequence is ticked and it returns FAILURE.
 */
class StatefulRecoveryRobinControl : public BT::ControlNode
{
public:
  explicit StatefulRecoveryRobinControl(const std::string & name, const BT::NodeConfiguration & config);

  StatefulRecoveryRobinControl() = delete;

  /**
   * @brief The main execution logic for the node.
   * @return The status of the node after execution.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Called when the node is halted. Resets the internal state.
   */
  void halt() override;

  /**
   * @brief Defines the ports for this node.
   * @return A list of ports. This node has no ports.
   */
  static BT::PortsList providedPorts() {return {};}

private:
  unsigned int current_child_idx_{0};
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__STATEFUL_RECOVERY_ROBIN_CONTROL_HPP_
```

### 2\. `stateful_recovery_robin_control.cpp` (구현 파일)

```cpp
#include "amr_bt_nodes/stateful_recovery_robin_control.hpp"

#include <string>

namespace amr_bt_nodes
{

StatefulRecoveryRobinControl::StatefulRecoveryRobinControl(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

void StatefulRecoveryRobinControl::halt()
{
  // halt() is called when the tree is reset (e.g., new goal received).
  // This ensures that for every new action, we start from the first child.
  ControlNode::halt();
  current_child_idx_ = 0;
}

BT::NodeStatus StatefulRecoveryRobinControl::tick()
{
  const size_t num_children = children_nodes_.size();

  if (num_children == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // This while loop allows the node to try multiple children within a single tick if they fail.
  while (current_child_idx_ < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      case BT::NodeStatus::RUNNING:
        // If a child is still running, the entire node is still running.
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        // A child succeeded. Return SUCCESS for this tick.
        // Increment the index to prepare for the *next* time this node is ticked.
        current_child_idx_++;
        // If we have completed the cycle, wrap around.
        if (current_child_idx_ == num_children) {
          current_child_idx_ = 0;
        }
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
        {
          // A child failed. Check if it was the last one in the sequence.
          if (current_child_idx_ == num_children - 1) {
            // The last child failed. This entire recovery cycle fails.
            // Reset index for the next full attempt and return FAILURE.
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
          }
          // It was not the last child. Increment index and continue the while loop
          // to try the next child immediately in this same tick.
          current_child_idx_++;
        }
        break;

      default:  // SKIPPED
        // Treat SKIPPED the same as FAILURE.
        if (current_child_idx_ == num_children - 1) {
            current_child_idx_ = 0;
            return BT::NodeStatus::FAILURE;
        }
        current_child_idx_++;
        break;
    }  // end switch
  }  // end while

  // This part of the code should not be reachable, but serves as a safeguard.
  halt();
  return BT::NodeStatus::FAILURE;
}

}  // namespace amr_bt_nodes
```

### 3\. BT 플러그인 등록

위 코드를 빌드한 후, BehaviorTreeFactory에 플러그인으로 등록해야 합니다. 일반적으로 패키지의 `..._register.cpp` 파일에 다음 코드를 추가합니다.

```cpp
#include "behaviortree_cpp/bt_factory.h"
#include "amr_bt_nodes/stateful_recovery_robin_control.hpp" // 새로 만든 노드의 헤더

// 이 함수는 BehaviorTreeFactory의 클래스 로더에 의해 호출됩니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  // 다른 노드 등록...
  factory.registerNodeType<amr_bt_nodes::StatefulRecoveryRobinControl>("StatefulRecoveryRobinControl");
}
```

### 4\. BT XML에서 사용법

XML에서는 클래스 이름과 동일하게 태그를 사용하면 됩니다.

```xml
<StatefulRecoveryRobinControl name="DefaultFailureRecoveryActions">
  <SubTree name="ClearPair@Default" ... />
  <Wait name="DefaultWait" ... />
  <SubTree name="Maneuver_PlanA@Default" ... />
  <SubTree name="Maneuver_PlanB@Default" ... />
</StatefulRecoveryRobinControl>
```

이제 이 코드를 프로젝트에 통합하여 사용하시면 됩니다.


##################################
네, 그럼요. `StatefulRecoveryRobinControl` 소스코드를 기반으로, 이전에 함께 정리했던 6가지 규칙이 코드의 각 부분에서 어떻게 구현되었는지 자세하게 설명해 드리겠습니다.

### 개요: 설계 철학

이 노드는 **"상태를 기억하는 제어 노드"** 입니다. 가장 중요한 상태는 `current_child_idx_`라는 정수 변수 하나이며, 이 변수를 통해 "이번에 어떤 복구 전략을 시도할 차례인가?"를 기억합니다. 모든 로직은 이 변수를 읽고, 실행하고, 업데이트하는 과정으로 이루어집니다.

-----

### `stateful_recovery_robin_control.hpp` (헤더 파일) 분석

```cpp
class StatefulRecoveryRobinControl : public BT::ControlNode
{
public:
  // ... 생성자 ...
  BT::NodeStatus tick() override;
  void halt() override;
  // ...

private:
  // 이 노드의 "기억"을 담당하는 유일한 멤버 변수입니다.
  // 다음에 실행할 자식 노드의 인덱스를 저장합니다.
  unsigned int current_child_idx_{0};
};
```

  * **`class ... : public BT::ControlNode`**: 이 노드가 자식들을 거느리며 실행 흐름을 제어하는 '제어 노드'임을 나타냅니다.
  * **`current_child_idx_{0}`**: 이 노드의 핵심입니다. `unsigned int` 타입의 이 변수 하나가 **규칙 1 (Stateful)** 을 구현합니다. `_idx`가 0으로 초기화되므로, 노드가 처음 생성되면 항상 첫 번째 자식부터 시작합니다.

-----

### `stateful_recovery_robin_control.cpp` (구현 파일) 분석

#### `halt()` 메소드: 상태 초기화

```cpp
void StatefulRecoveryRobinControl::halt()
{
  // BT가 중단되거나 완료될 때 호출됩니다.
  ControlNode::halt();
  // 노드의 기억(상태)을 깨끗이 지웁니다.
  current_child_idx_ = 0;
}
```

  * 이 함수는 **규칙 6 (순환 및 초기화)** 의 일부를 담당합니다.
  * `nav2_bt_navigator`는 하나의 네비게이션 Action(임무)이 끝나거나, 새로운 Action이 들어와서 기존 것을 취소할 때 BT 전체에 `halt()` 신호를 보냅니다.
  * `current_child_idx_ = 0;` 라인은 이 신호를 받았을 때 인덱스를 0으로 되돌려, 다음번 새로운 임무에서 복구가 필요할 때 무조건 첫 번째 자식부터 다시 시작하도록 보장합니다.

#### `tick()` 메소드: 핵심 실행 로직

```cpp
BT::NodeStatus StatefulRecoveryRobinControl::tick()
{
  // (자식 노드가 없는 경우 등 예외 처리) ...
  
  setStatus(BT::NodeStatus::RUNNING);

  // 이 while 루프가 "실패 시 다음 자식 즉시 실행" 로직의 핵심입니다.
  while (current_child_idx_ < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    switch (child_status) {
      // ...
    }
  }
  // ...
}
```

  * **`while (current_child_idx_ < num_children)`**: 이 `while` 루프가 **규칙 2와 4 (실패 시 계속 진행)** 를 구현하는 핵심 장치입니다. 자식이 `FAILURE`를 반환하면 `switch`문에서 `break`를 통해 이 `while` 루프의 다음 순회로 넘어가, `current_child_idx_`가 증가된 채로 다음 자식을 **같은 `tick` 안에서** 실행하게 됩니다.

-----

`switch (child_status)` 내부를 자세히 보겠습니다.

##### `case BT::NodeStatus::SUCCESS:`

```cpp
case BT::NodeStatus::SUCCESS:
  // 규칙 3: 자식이 성공하면, 우리도 성공을 반환.
  current_child_idx_++;
  if (current_child_idx_ == num_children) {
    current_child_idx_ = 0; // 규칙 6: 순환
  }
  return BT::NodeStatus::SUCCESS;
```

  * 이 블록은 **규칙 3 (성공 조건)** 을 완벽하게 구현합니다.
  * 자식이 `SUCCESS`를 반환하면, `return BT::NodeStatus::SUCCESS;`를 통해 `tick()` 함수와 `while` 루프를 즉시 종료합니다. 이로써 "tick 당 하나의 성공적인 액션"이 보장됩니다.
  * `current_child_idx_++`와 `if`문은 다음번 `tick`을 위해 실행할 자식의 인덱스를 미리 준비해두는 역할을 합니다. 마지막 자식이었다면 0으로 되돌려 **규칙 6 (순환)** 을 만족시킵니다.

##### `case BT::NodeStatus::FAILURE:`

```cpp
case BT::NodeStatus::FAILURE:
  {
    // 규칙 5: 마지막 자식이 실패했는지 확인.
    if (current_child_idx_ == num_children - 1) {
      current_child_idx_ = 0; // 다음 사이클을 위해 리셋
      return BT::NodeStatus::FAILURE;
    }
    
    // 규칙 4: 중간 자식이 실패하면, 다음 자식을 시도.
    current_child_idx_++;
  }
  break; // << 중요: break를 통해 while 루프의 다음 반복으로 넘어감
```

  * 이 블록은 **규칙 4 (중간 실패)** 와 **규칙 5 (최종 실패)** 를 구현합니다.
  * `if (current_child_idx_ == num_children - 1)`: 현재 실패한 자식이 마지막 자식인지 확인합니다.
      * **만약 마지막 자식이면**, `return BT::NodeStatus::FAILURE;`를 통해 `tick()` 함수를 즉시 종료하고 최종 실패를 알립니다. `current_child_idx_`를 0으로 리셋하여 다음 사이클을 준비합니다. 이것이 **규칙 5**의 구현입니다.
      * **만약 중간 자식이면**, `current_child_idx_++`로 인덱스만 증가시키고 `break`를 실행합니다. `break`는 `switch`문만 빠져나가게 하고, `while` 루프는 계속되기 때문에 **같은 `tick` 안에서 바로 다음 자식을 시도**하게 됩니다. 이것이 **규칙 4**의 정확한 구현입니다.

### 실행 시나리오 예시

1.  **복구 시도 1:** `tick()` 호출 (`idx=0`). `ClearCostmaps` 실행 -\> `SUCCESS` 반환.

      * `StatefulRecoveryRobinControl`은 `idx`를 1로 만들고 `SUCCESS` 반환.

2.  **복구 시도 2:** 주행 실패 후 `tick()` 다시 호출 (`idx=1`). `Wait` 실행 -\> `SUCCESS` 반환.

      * `StatefulRecoveryRobinControl`은 `idx`를 2로 만들고 `SUCCESS` 반환.

3.  **복구 시도 3:** 주행 실패 후 `tick()` 다시 호출 (`idx=2`). `ManeuverA` 실행 -\> `FAILURE` 반환.

      * `switch`문 `FAILURE` 케이스 진입. 마지막 자식이 아니므로 `idx`를 3으로 만들고 `while` 루프 계속.
      * **같은 tick 안에서** `ManeuverB` 실행 -\> `FAILURE` 반환.
      * `switch`문 `FAILURE` 케이스 진입. **마지막 자식이므로** `idx`를 0으로 리셋하고 `FAILURE` 반환.
      * `StatefulRecoveryRobinControl` 최종 `FAILURE` 반환.

이처럼, 제공해 드린 코드는 사용자님께서 상세하게 정의해주신 6가지 규칙을 모두 정확하게 따르도록 설계되었습니다.

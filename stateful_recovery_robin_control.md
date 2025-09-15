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

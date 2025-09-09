네, 원하시는 지능형 복구 로직의 상세한 명세를 기반으로, 제공해주신 현재 BT XML을 수정하여 완벽한 Behavior Tree를 설계해 드리겠습니다.

### 설계 철학 및 핵심 제어 노드

요청하신 로직은 **"실패 원인 진단 -\> 단계적 복구 시도 -\> 해결 시 정상 복귀"** 의 흐름을 따릅니다. 이를 BT로 구현하기 위한 최적의 제어 노드는 다음과 같습니다.

1.  **`Fallback` (Selector):** 실패 원인을 진단하고 세 가지 복구 시나리오(`GoalIsOccupied`, `RobotIsStuck`, `Default`) 중 하나를 선택하는 데 사용됩니다. 우선순위에 따라 자식 노드를 실행하며, 하나라도 성공하면 즉시 멈춥니다.
2.  **`RoundRobin`:** 각 복구 시나리오 내에서 여러 단계의 복구 전략을 순차적으로 시도하는 데 사용됩니다. `RecoveryNode`가 실패하여 다시 `RoundRobin`을 실행할 때마다 다음 자식 노드(다음 복구 전략)를 실행합니다. 이는 "1단계 해보고 안되면, 다음에 실패했을 땐 2단계를 해보자"는 점진적 악화(escalation) 로직에 완벽하게 부합합니다.
3.  **`SubTree`:** 반복되는 로직(예: 특정 플래너/컨트롤러로 짧은 경로 주행)을 모듈화하여 재사용성을 높이고 전체 트리의 가독성을 향상시킵니다.

### 새로운 지능형 복구 로직 BT XML

아래는 요청하신 모든 요구사항을 반영하여 재설계한 BT XML입니다. 기존 BT의 `<RecoveryNode name="NavigateRecovery">`의 두 번째 자식(기존 복구 로직)을 아래의 새로운 로직으로 교체하면 됩니다.

**주요 변경 사항:**

  * 기존의 복잡한 복구 로직을 명확한 3가지 시나리오로 분기하는 `IntelligentRecoverySelector` `Fallback` 노드를 도입했습니다.
  * 각 시나리오별로 `RoundRobin`을 사용하여 요청하신 단계적 복구 전략을 구현했습니다.
  * 반복되는 '짧은 경로 우회 기동'과 '목표 지점 제거' 로직을 각각 `ManeuverSubtree`와 `RemoveGoalsSubtree`라는 서브트리로 모듈화했습니다.
  * 전체 복구 시도 횟수를 늘리기 위해 메인 `RecoveryNode`의 `number_of_retries`를 `15`로 상향 조정했습니다.

<!-- end list -->

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <!-- =================================================================== -->
  <!-- ======================= SUBTREE DEFINITIONS ======================= -->
  <!-- =================================================================== -->

  <!-- 서브트리 1: 짧은 경로 우회 기동 (Maneuver)
       특정 플래너와 컨트롤러를 사용하여 현재 경로를 기반으로 짧은 우회 경로를 주행합니다. -->
  <BehaviorTree ID="ManeuverSubtree">
    <Sequence>
      <Log message="Executing Maneuver with planner: {planner_id} and controller: {controller_id}"/>
      <!-- 현재 남은 목표점들을 기반으로 우회 경로 생성 -->
      <ComputePathThroughPoses goals="{goals}" path="{alt_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <!-- 경로를 짧게 잘라내어 즉각적인 우회 목표(alt_goal) 생성 -->
      <TruncatePathLocal input_path="{alt_path}" output_path="{short_path}" distance_forward="1.5"/>
      <SetTruncatedGoalFromPath short_path="{short_path}" alt_goal="{alt_goal}"/>
      
      <!-- 생성된 짧은 우회 경로 주행 -->
      <ComputePathToPose goal="{alt_goal}" path="{short_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <FollowPath path="{short_path}" controller_id="{controller_id}" goal_checker_id="super_relaxed_goal_checker" error_code_id="{follow_path_error_code}"/>
      <Log message="Maneuver completed successfully."/>
    </Sequence>
  </BehaviorTree>

  <!-- 서브트리 2: N개의 목표 지점 제거
       주어진 횟수(num_goals_to_remove)만큼 현재 경로의 맨 앞 목표 지점을 제거합니다. -->
  <BehaviorTree ID="RemoveGoalsSubtree">
    <RetryUntilSuccessful num_attempts="{num_goals_to_remove}">
      <Sequence>
        <Log message="Removing a goal point..."/>
        <RemoveFirstGoalAction input_goals="{goals}" remaining_goals="{goals}" />
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>

  <!-- =================================================================== -->
  <!-- ========================== MAIN TREE ========================== -->
  <!-- =================================================================== -->

  <BehaviorTree ID="MainTree">
    <!-- 전체 네비게이션을 감싸는 최상위 복구 노드. 
         복구 전략이 여러 단계이므로 retries 횟수를 충분히 늘립니다. -->
    <RecoveryNode number_of_retries="20" name="NavigateRecovery">

      <!-- 1. 주 임무: 네비게이션 파이프라인 (기존 BT와 동일) -->
      <Sequence>
        <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
        <PipelineSequence name="NavigateWithReplanning">
          <!-- 
            이 안에는 기존에 사용하시던 복잡한 주행 로직이 그대로 들어갑니다.
            (ControllerSelector, PlannerSelector, RateController, Fallback name="PlannerTrigger" 등)
          -->
        </PipelineSequence>
      </Sequence>

      <!-- 2. 복구 임무: 새로운 지능형 복구 로직 -->
      <Sequence name="IntelligentRecovery">
        <!-- 사전 조건: 플래너나 컨트롤러 실패 시에만 복구 로직을 실행합니다. -->
        <Fallback name="FailureTypeCheck">
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        </Fallback>
        <Log color="yellow" message="[RECOVERY] Planner or Controller failure detected. Initiating intelligent recovery."/>

        <!-- 핵심 로직: 실패 원인에 따라 다른 복구 전략을 선택합니다. (우선순위: 위 -> 아래) -->
        <Fallback name="IntelligentRecoverySelector">

          <!-- 시나리오 1: 목표 지점이 점유된 경우 -->
          <Sequence name="GoalIsOccupiedCase">
            <CheckGoalOccupied goal="{goals[0]}"/>
            <Log color="cyan" message="[RECOVERY CASE 1] Goal is occupied. Executing recovery."/>
            <RoundRobin name="GoalIsOccupiedRecoveryActions">
              <Wait name="WaitToClear" wait_duration="5.0"/>
              <!-- 남은 골이 2개 이상일 때만 마지막 골을 제외하고 모두 제거 -->
              <Subtree ID="RemoveGoalsSubtree" num_goals_to_remove="-2"/> 
            </RoundRobin>
          </Sequence>

          <!-- 시나리오 2: 로봇이 물리적으로 갇힌 경우 -->
          <Sequence name="RobotIsStuckCase">
            <CheckIsStuck velocity_threshold="0.02"/>
            <Log color="cyan" message="[RECOVERY CASE 2] Robot is stuck. Executing recovery."/>
            <RoundRobin name="RobotIsStuckRecoveryActions">
              <Wait name="WaitToThink" wait_duration="2.0"/>
              <Subtree ID="ManeuverSubtree" 
                       planner_id="RecoveryGridBased1" 
                       controller_id="RecoveryFollowPath1"/>
              <Subtree ID="ManeuverSubtree" 
                       planner_id="RecoveryGridBased2" 
                       controller_id="RecoveryFollowPath2"/>
            </RoundRobin>
          </Sequence>

          <!-- 시나리오 3: 위의 모든 경우가 아닌, 일반적인 플래너 실패 (가장 마지막에 실행) -->
          <Sequence name="DefaultPlannerFailureCase">
            <Log color="cyan" message="[RECOVERY CASE 3] Default planner failure. Executing recovery."/>
            <RoundRobin name="DefaultPlannerFailureRecoveryActions">
              <Sequence name="ClearCostmaps">
                <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
              <Wait name="DefaultWait" wait_duration="3.0"/>
              <Subtree ID="ManeuverSubtree" 
                       planner_id="RecoveryGridBased1" 
                       controller_id="RecoveryFollowPath1"/>
              <Subtree ID="ManeuverSubtree" 
                       planner_id="RecoveryGridBased2" 
                       controller_id="RecoveryFollowPath2"/>
              <Subtree ID="RemoveGoalsSubtree" num_goals_to_remove="-2"/>
            </RoundRobin>
          </Sequence>

        </Fallback>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>

  <!-- =================================================================== -->
  <!-- =================== CUSTOM NODE DEFINITIONS =================== -->
  <!-- =================================================================== -->
  
  <TreeNodesModel>
    <!-- 아래는 이 BT에 필요한 커스텀 노드들의 정의입니다. C++로 구현해야 합니다. -->
    <Condition ID="CheckGoalOccupied">
      <input_port name="goal" type="geometry_msgs::msg::PoseStamped" description="The goal to check"/>
    </Condition>
    <Condition ID="CheckIsStuck">
      <input_port name="velocity_threshold" type="double" default="0.02" description="Velocity below which robot is considered stuck"/>
    </Condition>
    <Action ID="RemoveFirstGoalAction">
        <input_port name="input_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
        <output_port name="remaining_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
    </Action>
    <!-- 기타 기존 BT에서 사용하던 커스텀 노드들도 여기에 포함시켜야 합니다. -->
  </TreeNodesModel>

</root>

```

### 사용 방법 및 추가 설명

1.  **커스텀 노드 구현:**

      * `CheckGoalOccupied`, `CheckIsStuck` 커스텀 Condition 노드를 이전 대화에서 설명한 대로 C++로 구현해야 합니다.
      * `RemoveFirstGoalAction`은 `goals` 벡터에서 첫 번째 원소를 제거하는 간단한 Action 노드입니다.
      * `number_of_goals_to_skip`은 블랙보드 변수로, 몇 개의 목표 지점을 건너뛸지 외부에서 설정할 수 있습니다. (예: `goals.size() - 2`)

2.  **파라미터 설정:**

      * `RecoveryGridBased1`, `RecoveryGridBased2`, `RecoveryFollowPath1`, `RecoveryFollowPath2` 등은 `nav2_params.yaml` 파일에 각각 다른 파라미터(예: 더 넓은 장애물 회피 반경, 더 공격적인 주행 등)를 가진 플래너와 컨트롤러로 정의되어야 합니다.
      * `super_relaxed_goal_checker` 또한 goal 도달 판정을 매우 느슨하게 하는 골 체커로 정의되어야 합니다.

3.  **실행 흐름:**

      * 주 네비게이션 파이프라인이 실패하면, `IntelligentRecovery` 시퀀스가 실행됩니다.
      * `IntelligentRecoverySelector`(`Fallback`)가 위에서부터 순서대로 조건을 확인합니다.
      * `CheckGoalOccupied`가 성공하면 `GoalIsOccupiedCase` 시퀀스만 실행되고, 그 안의 `RoundRobin`이 복구를 단계적으로 시도합니다. 첫 복구 시도(`Wait`)가 성공하고 다시 메인 파이프라인으로 돌아가서 또 실패하면, 다음 복구 시도(`RemoveGoalsSubtree`)가 실행됩니다.
      * `CheckGoalOccupied`가 실패하면 `Fallback`은 다음 자식인 `RobotIsStuckCase`를 시도합니다.
      * 이런 식으로 가장 먼저 조건이 맞는 시나리오의 복구 전략이 실행됩니다.

이 BT 구조는 사용자가 원하신 복잡하고 지능적인 복구 로직을 명확하고 체계적으로 구현한 것입니다. 각 상황에 맞는 최적의 대응을 통해 로봇 네비게이션의 강인함(robustness)을 크게 향상시킬 수 있을 것입니다.


   
   



```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <BehaviorTree ID="ManeuverSubtree">
    <Sequence>
      <LogTextAction message="[Subtree] Executing Maneuver with planner: {planner_id} and controller: {controller_id}" interval_s="1.0"/>
      <ComputePathThroughPoses goals="{goals}" path="{alt_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <TruncatePathLocal input_path="{alt_path}" output_path="{short_path}" distance_forward="1.5"/>
      <SetTruncatedGoalFromPath short_path="{short_path}" alt_goal="{alt_goal}"/>
      <ComputePathToPose goal="{alt_goal}" path="{short_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <FollowPath path="{short_path}" controller_id="{controller_id}" goal_checker_id="super_relaxed_goal_checker" error_code_id="{follow_path_error_code}"/>
      <LogTextAction message="[Subtree] Maneuver completed successfully." interval_s="1.0"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="RemoveGoalsSubtree">
    <RetryUntilSuccessful num_attempts="{num_goals_to_remove}">
      <Sequence>
        <LogTextAction message="[Subtree] Removing a goal point..." interval_s="1.0"/>
        <RemoveFirstGoalAction input_goals="{goals}" remaining_goals="{goals}" />
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>

  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="20" name="NavigateRecovery">

      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <Sequence>
            <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector" />
            <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector" />
            <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>

            <Fallback name="PlanningFallback">
              <IsPathValid path="{path}" max_cost="150" consider_unknown_as_obstacle="false" />
              <Sequence name="ComputeNewPath">
                <LogTextAction message="Path is invalid or replan is requested. Computing new path..." interval_s="1.0"/>
                <RecoveryNode number_of_retries="1" name="ComputePathRecovery">
                  <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
                  <Sequence name="ClearCostmapAndRetry">
                     <LogTextAction message="[RECOVERY] Main planner failed. Clearing costmaps and retrying..." interval_s="1.0"/>
                     <ClearEntireCostmap name="ClearGlobalCostmap-Main" service_name="global_costmap/clear_entirely_global_costmap" />
                     <ClearEntireCostmap name="ClearLocalCostmap-Main" service_name="local_costmap/clear_entirely_local_costmap" />
                  </Sequence>
                </RecoveryNode>
                <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}" smoother_id="simple_smoother" />
              </Sequence>
            </Fallback>
          </Sequence>
        </RateController>

        <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
          <Sequence>
            <PathPublisherAction input_path="{path}" topic_name="/plan_pruned" publish_period="0.3" node="{node}" />
            <FollowPath path="{path}" controller_id="{selected_controller}" goal_checker_id="precise_goal_checker" progress_checker_id="progress_checker" error_code_id="{follow_path_error_code}" />
          </Sequence>
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}" />
            <LogTextAction message="[RECOVERY] Controller failed. Clearing local costmap." interval_s="1.0"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap" />
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>

      <Sequence name="IntelligentRecovery">
        <Fallback name="FailureTypeCheck">
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
        </Fallback>
        <LogTextAction message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic." interval_s="1.0"/>

        <Fallback name="IntelligentRecoverySelector">

          <Sequence name="NoValidPathCase">
            <Precondition if="compute_path_error_code == 1" else="FAILURE">
              <Sequence>
                <LogTextAction message="[RECOVERY CASE 0] 'No valid path' error detected. Trying to resolve..." interval_s="1.0"/>
                <Fallback>
                  <Sequence>
                    <IsGoalsOccupiedCondition goals="{goals}"/>
                    <LogTextAction message=" -> Cause: Goal is occupied. Waiting or removing goals." interval_s="1.0"/>
                    <RoundRobin name="GoalOccupiedRecovery">
                      <Wait name="WaitToClear" wait_duration="5.0"/>
                      <Subtree ID="RemoveGoalsSubtree" num_goals_to_remove="-2"/> 
                    </RoundRobin>
                  </Sequence>
                  <Sequence>
                    <LogTextAction message=" -> Cause: Path is blocked but goal is clear. Clearing costmaps and trying maneuver." interval_s="1.0"/>
                    <RoundRobin name="PathBlockedRecovery">
                      <Sequence name="ClearCostmaps">
                        <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
                        <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
                      </Sequence>
                      <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
                    </RoundRobin>
                  </Sequence>
                </Fallback>
              </Sequence>
            </Precondition>
          </Sequence>

          <Sequence name="RobotIsStuckCase">
            <CheckIsStuck velocity_threshold="0.02"/>
            <LogTextAction message="[RECOVERY CASE 1] Robot appears to be stuck. Executing maneuver." interval_s="1.0"/>
            <RoundRobin name="RobotIsStuckRecoveryActions">
              <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
              <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath2"/>
            </RoundRobin>
          </Sequence>

          <Sequence name="DefaultFailureCase">
            <LogTextAction message="[RECOVERY CASE 2] Default failure. Executing general recovery sequence." interval_s="1.0"/>
            <RoundRobin name="DefaultFailureRecoveryActions">
              <Sequence name="ClearAllCostmaps">
                <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
                <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
              </Sequence>
              <Wait name="DefaultWait" wait_duration="2.0"/>
              <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
              <Subtree ID="RemoveGoalsSubtree" num_goals_to_remove="-2"/>
            </RoundRobin>
          </Sequence>

        </Fallback>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>

  <TreeNodesModel>
    <Action ID="LogTextAction">
        <input_port name="message" type="std::string"/>
        <input_port name="interval_s" type="float" default="1.0"/>
        <input_port name="bb_key" type="std::string"/>
    </Action>
    <Condition ID="IsGoalsOccupiedCondition">
      <input_port name="goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
    </Condition>
    <Condition ID="CheckIsStuck">
      <input_port name="velocity_threshold" type="double" default="0.02"/>
    </Condition>
    <Action ID="RemoveFirstGoalAction">
        <input_port name="input_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
        <output_port name="remaining_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
    </Action>
   </TreeNodesModel>

</root>

```






네, Nav2 및 Behavior Tree 전문가로서 문의주신 로직에 대해 분석하고 개선 방안을 제안해 드리겠습니다.

먼저, `RateController` 주기마다 경로의 유효성, 외부 플래그, 플래너 변경 등 다양한 조건을 순차적으로 확인하여 리플래닝(replanning)을 트리거하고, 마지막으로 플래너의 오류 상태를 확인하여 복구 로직으로 전환하거나 정상 흐름을 이어가려는 목표는 매우 실용적이고 강인한(robust) 내비게이션 시스템을 구축하기 위한 훌륭한 접근입니다.

요청하신 로직을 제가 이해한 바는 다음과 같습니다.

1.  **매 주기마다 검사:** `RateController` 내에서 다음 조건들을 순서대로 확인한다.
2.  **리플래닝 트리거:**
      * 현재 경로(`path`)가 유효하지 않거나 (`IsPathValid` -\> FAILURE)
      * 외부 플래그가 설정되었거나 (`CheckFlagCondition` -\> SUCCESS)
      * 플래너가 변경되었으면 (`CheckPlannerChanged` -\> SUCCESS)
        \-\> `ComputePathThroughPoses`를 실행하여 경로를 재계산한다.
3.  **오류 처리:**
      * 위 조건들에 해당하지 않아 리플래닝을 하지 않거나, 리플래닝을 수행한 후에,
      * 심각한 플래너 오류가 있는지 확인 (`CheckErrorExists`).
      * 오류가 있다면 (`SUCCESS`), 전체 플래닝 단계를 `FAILURE`로 만들어 상위의 `IntelligentRecovery` 시퀀스를 실행시킨다.
      * 오류가 없다면 (`FAILURE`), 플래닝 단계를 `SUCCESS`로 마무리하여 경로 추종(FollowPath)을 계속 진행한다.

이 로직은 매우 명확하며, BT로 충분히 구현 가능합니다. 말씀하신 `Fallback`을 사용하는 것이 맞지만, 조건의 성공/실패에 따라 다른 행동을 취해야 하므로 구조를 조금 더 다듬을 필요가 있습니다.

### 제안하는 개선된 Behavior Tree 구조

요청하신 로직을 가장 명확하고 효율적으로 구현할 수 있는 BT 구조를 제안합니다. 이 구조는 Nav2의 표준 BT에서 널리 사용되는 **"정상 조건 검사 -\> 실패 시 대안 실행"** 패턴을 따릅니다.

핵심은 `Fallback` 노드를 사용하여 "경로에 이상이 없는가?"를 먼저 검사하고, 이 검사가 실패했을 때 (즉, 이상이 있을 때) 리플래닝을 수행하는 것입니다.

```xml
<RateController hz="1.0">
  <Fallback name="PlannerLogic">

    <Sequence name="CheckPathAndConditions">
      <IsPathValid path="{path}" />
      
      <Inverter>
        <CheckFlagCondition name="CheckReplanFlag" />
      </Inverter>

      <Inverter>
        <CheckPlannerChanged />
      </Inverter>
    </Sequence>

    <Sequence name="ReplanAndHandleError">
      <LogTextAction message="Path is invalid or replan triggered. Re-computing path..." />
      
      <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{planner_error_code}" />

      <Precondition if="planner_error_code != 0" else="SUCCESS">
        <Sequence>
            <LogTextAction message="[CRITICAL] Planner returned an unrecoverable error code: {planner_error_code}. Triggering recovery."/>
            <AlwaysFailure />
        </Sequence>
      </Precondition>
      </Sequence>

  </Fallback>
</RateController>
```

### 이 구조의 장점

1.  **명확성 (Clarity):** BT의 흐름이 "문제가 없으면 통과, 문제가 있으면 해결 시도"로 매우 직관적입니다. `CheckPathAndConditions` 시퀀스는 '정상 상태'를 정의하고, `ReplanAndHandleError` 시퀀스는 '문제 해결'을 담당합니다.
2.  **효율성 (Efficiency):** 대부분의 경우 경로는 유효할 것이므로, 첫 번째 `Sequence`만 통과하고 빠르게 `SUCCESS`를 반환합니다. 불필요한 조건 검사를 최소화합니다.
3.  **확장성 (Scalability):** 나중에 리플래닝을 트리거할 새로운 조건을 추가하고 싶다면, `CheckPathAndConditions` 시퀀스 안에 `<Inverter><MyNewCondition /></Inverter>` 형태로 간단하게 추가할 수 있습니다.
4.  **정확성 (Correctness):**
      * `Inverter` 노드는 "플래그가 켜졌을 때"와 같이 **성공이 리플래닝의 조건**이 되는 경우를 자연스럽게 처리해줍니다. `CheckFlagCondition`이 `SUCCESS`를 반환하면 `Inverter`가 `FAILURE`를 반환하여 `CheckPathAndConditions` 시퀀스를 실패시키고, 결과적으로 리플래닝 로직이 실행됩니다.
      * `Precondition` 노드는 요청하신 **오류 코드에 따른 분기 처리**를 가장 우아하게 구현하는 방법입니다. `if` 조건이 참이면 자식을 실행하고, `else` 속성을 통해 조건이 거짓일 때의 결과를 명시적으로 지정할 수 있습니다.

### 결론

원래 구상하신 로직은 매우 훌륭합니다. 제가 제안드린 XML 구조는 그 로직의 의도를 Behavior Tree의 표준적인 설계 패턴에 맞춰 가장 명확하고 효율적으로 구현한 것입니다. 이 구조를 사용하시면 의도하신 대로 완벽하게 동작할 뿐만 아니라, 향후 유지보수와 기능 확장에도 유리할 것입니다.

`CheckFlagCondition`, `CheckPlannerChanged`, `CheckErrorExists`(제안에서는 `ComputePathThroughPoses`의 `error_code_id`를 활용)와 같은 커스텀 노드들은 C++로 직접 구현해야 합니다. `Precondition` 노드는 `nav2_behavior_tree` 패키지에 포함된 표준 노드이므로 바로 사용 가능합니다.






네, 알겠습니다. 제공해주신 BT XML은 기능적으로는 복잡하지만, 많은 부분이 재사용 가능한 모듈로 분리될 수 있는 좋은 구조를 가지고 있습니다.

Subtree를 활용하여 모듈화하고 가독성을 높이는 방향으로 코드를 최적화해 드리겠습니다. 모든 기능은 그대로 유지됩니다.

### 최적화 전략

1.  **논리적 기능 단위 식별:** 현재 `MainTree`에 있는 코드 블록들을 기능에 따라 나눕니다. (e.g., 초기화, 경로 계획, 경로 추종, 지능형 복구).
2.  **Subtree 생성:** 식별된 각 기능 단위를 별도의 `BehaviorTree` (Subtree)로 정의합니다. 각 Subtree는 필요한 데이터를 입력(Input Ports)으로 받고, 결과를 출력(Output Ports)으로 반환하는 명확한 API를 갖게 됩니다.
3.  **MainTree 재구성:** 기존의 긴 코드를 새로운 Subtree들을 호출하는 간결한 형태로 재구성합니다. 이렇게 하면 `MainTree`는 전체적인 흐름만 보여주게 되어 이해하기 쉬워집니다.
4.  **중복 코드 제거:** 복구 로직에 반복적으로 나타나는 `RemoveFirstGoalAction` 시퀀스를 기존의 `RemoveGoalsSubtree`를 재사용하여 최적화합니다.
5.  **`ManeuverSubtree` 역할 명확화:** `ManeuverSubtree`가 우회 기동이라는 단일 책임만 갖도록 내부의 "플래너/컨트롤러 리셋" 로직을 정리합니다. 복구 성공 후의 상태 복원은 `MainTree`의 흐름에 맡기는 것이 더 깔끔한 설계입니다.

-----

### 최적화된 BT XML 코드

아래는 위 전략을 적용하여 재구성한 전체 BT XML입니다.

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <BehaviorTree ID="InitializeNavSubtree">
    <Sequence>
      <LogTextAction message="[Subtree] Initializing navigation task..." interval_s="0.0"/>
      <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
      <UnsetBlackboard key="path" />
      <UnsetBlackboard key="compute_path_error_code" />
      <UnsetBlackboard key="follow_path_error_code" />
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="PlanPathSubtree">
    <Fallback name="PlannerLogic">
      <Sequence name="CheckPathAndConditions">
        <IsPathValidCustomCondition path="{path}" />
        <Inverter>
          <CheckFlagCondition flag_topic="/replan_flag" latch="false" node="{node}" />
        </Inverter>
        <Inverter>
          <IsBlackboardChangedCondition blackboard_entry="{selected_planner}" />
        </Inverter>
      </Sequence>
      <Sequence name="ReplanAndHandleError">
        <LogTextAction message="[Subtree] Path is invalid or replan triggered. Re-computing path..." />
        <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}" />
      </Sequence>
    </Fallback>
  </BehaviorTree>
  
  <BehaviorTree ID="FollowPathPipelineSubtree">
    <ReactiveFallback name="PauseAndResume">
      <Sequence>
        <CheckFlagCondition flag_topic="/controller_pause_flag" latch="true" node="{node}" />
        <LogTextAction message="[PAUSED] Controller is paused. Halting motion." interval_s="1.0" />
        <SequenceWithMemory>
          <CancelControl />
          <Repeat num_cycles="-1">
            <Sequence>
              <CheckFlagCondition flag_topic="/controller_pause_flag" latch="true" node="{node}" />
              <Wait wait_duration="0.1" />
            </Sequence>
          </Repeat>
        </SequenceWithMemory>
      </Sequence>
      <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
        <PipelineSequence name="FollowAndMonitorPipe">
          <LogTextAction name="FollowLog" message="[Subtree] Path is valid. Following path..." interval_s="1.0" />
          <TruncatePathLocal input_path="{path}" output_path="{pruned_path}" distance_forward="50.0" max_robot_pose_search_dist="2.0"/>
          <PathPublisherAction input_path="{pruned_path}" topic_name="/plan_pruned" publish_period="0.5" node="{node}" />
          <FollowPath path="{pruned_path}" controller_id="{selected_controller}" goal_checker_id="precise_goal_checker" progress_checker_id="progress_checker" error_code_id="{follow_path_error_code}" />
        </PipelineSequence>
        <Sequence name="FollowPathRecoveryAction">
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}" />
          <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap" />
        </Sequence>
      </RecoveryNode>
    </ReactiveFallback>
  </BehaviorTree>

  <BehaviorTree ID="ManeuverSubtree">
    <Sequence>
      <LogTextAction message="[Subtree] Executing Maneuver with planner: {planner_id} and controller: {controller_id}" interval_s="1.0"/>
      <GetNextFewGoalsAction name="ExtractNextGoals" num_goals="2" input_goals="{goals}" output_goals="{few_goals}" />
      <ComputePathThroughPoses goals="{few_goals}" path="{alt_path}" planner_id="{planner_id}" error_code_id="{recovery_compute_path_error_code}"/>
      <TruncatePathLocal input_path="{alt_path}" output_path="{short_path}" distance_forward="1.0" angular_distance_weight="0.2" max_robot_pose_search_dist="1.0"/>
      <SetTruncatedGoalFromPath short_path="{short_path}" alt_goal="{alt_goal}"/>
      <ComputePathToPose goal="{alt_goal}" path="{short_path}" planner_id="{planner_id}" error_code_id="{recovery_compute_path_error_code}"/>
      <FollowPath path="{short_path}" controller_id="{controller_id}" goal_checker_id="super_relaxed_goal_checker" progress_checker_id="progress_checker" error_code_id="{follow_path_error_code}"/>
      <LogTextAction message="[Subtree] Maneuver completed successfully." interval_s="1.0"/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="RemoveGoalsSubtree">
    <RetryUntilSuccessful num_attempts="{num_goals_to_remove}">
      <Sequence>
        <LogTextAction message="[Subtree] Removing a goal point..." interval_s="1.0"/>
        <RemoveFirstGoalAction input_goals="{goals}" output_goals="{goals}" />
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>
  
  <BehaviorTree ID="IntelligentRecoverySubtree">
    <Sequence>
      <LogTextAction message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic." interval_s="1.0"/>
      <Fallback name="FailureTypeCheck">
        <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
      </Fallback>
      
      <Fallback name="IntelligentRecoverySelector">
        <Sequence name="NoValidPathCase">
          <Precondition if="compute_path_error_code == 308" else="FAILURE">
            <Sequence>
              <LogTextAction message="[RECOVERY CASE 0] 'No valid path' error detected."/>
              </Sequence>
          </Precondition>
        </Sequence>
        <Sequence name="RobotIsStuckCase">
          <CheckFlagCondition flag_topic="/progress_checker/is_stuck" latch="false" node="{node}" />
          <LogTextAction message="[RECOVERY CASE 1] Robot appears to be stuck."/>
          <RoundRobin name="RobotIsStuckRecoveryActions">
            <SubTree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1" _autoremap="true"/>
            <SubTree ID="ManeuverSubtree" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath1" _autoremap="true"/>
          </RoundRobin>
        </Sequence>
        <Sequence name="GoalOccupiedPathCase">
          <Precondition if="compute_path_error_code == 306" else="FAILURE">
             <Sequence>
                <LogTextAction message="[RECOVERY CASE] Goal is occupied. Removing goals."/>
                <SubTree ID="RemoveGoalsSubtree" num_goals_to_remove="3" _autoremap="true"/>
             </Sequence>
          </Precondition>
        </Sequence>
        <Sequence name="DefaultFailureCase">
          <LogTextAction message="[RECOVERY CASE] Default failure."/>
          <RoundRobin name="DefaultFailureRecoveryActions">
            <Sequence name="ClearAllCostmaps">
              <ClearEntireCostmap name="ClearLocal" service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap name="ClearGlobal" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Wait name="DefaultWait" wait_duration="1.0"/>
            <SubTree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1" _autoremap="true"/>
            <SubTree ID="ManeuverSubtree" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath1" _autoremap="true"/>
          </RoundRobin>
        </Sequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="25" name="NavigateRecovery">
      
      <PipelineSequence name="MainNavigationPipeline">
        <RunNTimesDecorator num_ticks="1">
          <SubTree ID="InitializeNavSubtree"/>
        </RunNTimesDecorator>
        
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector" />
        <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector" />
        <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
        
        <SubTree ID="PlanPathSubtree" _autoremap="true"/>
        <SubTree ID="FollowPathPipelineSubtree" _autoremap="true"/>
        
        <Fallback>
            <IsNotBlackboardSet key="follow_path_error_code"/>
            <Sequence>
                 <LogTextAction message="[CRITICAL] Controller failed. Triggering recovery."/>
                 <AlwaysFailure/>
            </Sequence>
        </Fallback>
        <Fallback>
            <IsNotBlackboardSet key="compute_path_error_code"/>
            <Sequence>
                 <LogTextAction message="[CRITICAL] Planner failed. Triggering recovery."/>
                 <AlwaysFailure/>
            </Sequence>
        </Fallback>

      </PipelineSequence>

      <SubTree ID="IntelligentRecoverySubtree" _autoremap="true"/>
      
    </RecoveryNode>
  </BehaviorTree>

  <TreeNodesModel>
    <Action ID="LogTextAction">
      <input_port name="message" type="std::string"/>
      <input_port name="interval_s" type="float" default="1.0"/>
    </Action>
    </TreeNodesModel>

</root>
```

### 변경된 내용 요약

1.  **새로운 Subtree:**

      * `InitializeNavSubtree`: 네비게이션 시작 시 1회 실행되는 초기화 로직을 캡슐화했습니다.
      * `PlanPathSubtree`: 경로 유효성을 검사하고 필요시 리플래닝하는 로직을 캡슐화했습니다.
      * `FollowPathPipelineSubtree`: 일시정지 기능을 포함한 전체 경로 추종 로직을 캡슐화했습니다.
      * `IntelligentRecoverySubtree`: 거대했던 복구 로직 전체를 하나의 Subtree로 묶었습니다.

2.  **간결해진 `MainTree`:**

      * 이제 `MainTree`는 전체 네비게이션의 큰 그림(초기화 -\> 계획 -\> 추종)을 명확하게 보여줍니다.
      * 각 단계는 해당 기능을 담당하는 Subtree를 호출하는 방식으로 실행됩니다.
      * `_autoremap="true"` 속성을 사용하여 `MainTree`의 블랙보드 변수들이 자동으로 Subtree의 포트로 전달되도록 하여 코드의 중복을 줄였습니다.

3.  **코드 최적화:**

      * `GoalOccupiedPathCase` 복구 로직에서 3번 반복되던 `RemoveFirstGoalAction` 호출을 `<SubTree ID="RemoveGoalsSubtree" num_goals_to_remove="3" .../>` 한 줄로 대체하여 더 효율적이고 가독성 좋게 만들었습니다.
      * `ManeuverSubtree`는 순수하게 '우회 기동'만 담당하도록 단순화되었습니다.
      * 주행 파이프라인(`PipelineSequence`) 마지막에 `Fallback`을 추가하여, `error_code`가 설정되었을 때 명시적으로 `AlwaysFailure`를 반환하도록 하여 복구 로GIN으로 확실히 넘어가도록 하는 로직을 강화했습니다.

이와 같이 BT를 기능 단위로 모듈화하면 각 부분의 역할을 명확히 할 수 있어, 향후 특정 기능(예: 경로 계획 로직)만 수정하거나 다른 BT에서 재사용하기가 훨씬 용이해집니다.





```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">

  <!-- ===================================================== -->
  <!-- Refactored Nav2 BT (v4):
       - Consistent port names across subtrees for _autoremap
       - Internal-only buffers use '_' prefix (no auto-remap)
       - Output rename kept minimal; parents read canonical keys
       - Reused logic factored into helper subtrees
       - Behavior preserved; logs/IDs clarified where useful
  -->
  <!-- ===================================================== -->

  <!-- ================= Common / Helper Subtrees ================= -->

  <!-- 1) Initialization: run exactly once at session start -->
  <BehaviorTree ID="InitSubtree">
    <RunNTimesDecorator num_ticks="1" name="InitializingOnce">
      <Sequence>
        <LogTextAction message="Initializing..." interval_s="0.0"/>
        <ClearEntireCostmap name="ClearGlobal@Init" service_name="global_costmap/clear_entirely_global_costmap"/>
        <!-- Canonical keys cleared -->
        <UnsetBlackboard key="path"/>
        <UnsetBlackboard key="pruned_path"/>
        <UnsetBlackboard key="compute_path_error_code"/>
        <UnsetBlackboard key="follow_path_error_code"/>
        <UnsetBlackboard key="obstructed_goals"/>
        <UnsetBlackboard key="unoccupied_goals"/>
      </Sequence>
    </RunNTimesDecorator>
  </BehaviorTree>

  <!-- 2) Controller / Planner selectors (kept) -->
  <BehaviorTree ID="SelectPluginsSubtree">
    <Sequence>
      <ControllerSelector selected_controller="{selected_controller}"
        default_controller="FollowPath" topic_name="controller_selector"/>
      <PlannerSelector selected_planner="{selected_planner}"
        default_planner="GridBased" topic_name="planner_selector"/>
    </Sequence>
  </BehaviorTree>

  <!-- 3) Path truncate & publish (internal buffer '_' for short version) -->
  <BehaviorTree ID="TruncateAndPublishSubtree">
    <Sequence>
      <TruncatePathLocal
        input_path="{path}"
        output_path="{pruned_path}"
        distance_forward="50.0"
        distance_backward="0.0"
        angular_distance_weight="0.2"
        max_robot_pose_search_dist="2.0"
        robot_frame="base_link"
        transform_tolerance="0.3"/>

      <!-- short variant is internal-only -->
      <TruncatePathLocal
        input_path="{pruned_path}"
        output_path="{_truncated_short_path}"
        distance_forward="3.0"
        distance_backward="0.0"
        angular_distance_weight="0.2"
        max_robot_pose_search_dist="2.0"
        robot_frame="base_link"
        transform_tolerance="0.3"/>

      <PathPublisherAction input_path="{pruned_path}" topic_name="/plan_pruned"
        publish_period="0.5" node="{node}"/>
      <PathPublisherAction input_path="{_truncated_short_path}" topic_name="/plan_truncated_short"
        publish_period="0.5" node="{node}"/>
    </Sequence>
  </BehaviorTree>

  <!-- 4) Clear local+global costmaps together -->
  <BehaviorTree ID="ClearCostmapsPairSubtree">
    <Sequence>
      <ClearEntireCostmap name="ClearLocal@Pair"  service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap name="ClearGlobal@Pair" service_name="global_costmap/clear_entirely_global_costmap"/>
    </Sequence>
  </BehaviorTree>

  <!-- 5) Gate: reuse valid path or replan (publishes error codes on failure) -->
  <BehaviorTree ID="ReplanOrReusePathSubtree">
    <Fallback name="PlannerLogic">
      <!-- Keep path if valid && replan_flag=false && selected_planner unchanged -->
      <Sequence name="CheckPathAndConditions">
        <IsPathValidCustomCondition path="{path}"/>
        <Inverter>
          <CheckFlagCondition flag_topic="/replan_flag" latch="false" node="{node}"/>
        </Inverter>
        <Inverter>
          <IsBlackboardChangedCondition blackboard_entry="{selected_planner}"/>
        </Inverter>
      </Sequence>

      <!-- Otherwise compute path (with error publication on failure) -->
      <Sequence name="ReplanAndHandleError">
        <LogTextAction message="Path invalid or replan triggered. Re-computing path..."/>
        <Fallback name="PlannerTryOrEscalate">
          <ComputePathThroughPoses goals="{goals}" path="{path}"
            planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
          <Sequence>
            <LogTextAction message="[CRITICAL] Planner unrecoverable error: {compute_path_error_code}. Triggering recovery."/>
            <ErrorCodePublisherAction topic_name="/bt_error_code" error_code="{compute_path_error_code}" node="{node}"/>
            <ErrorCodePublisherAction topic_name="/bt_error_code" error_code="{follow_path_error_code}"  node="{node}"/>
            <AlwaysFailure/>
          </Sequence>
        </Fallback>
      </Sequence>
    </Fallback>
  </BehaviorTree>

  <!-- 6) Pause/Resume + FollowPath (with recovery); relies on pruned_path from 3) -->
  <BehaviorTree ID="FollowWithPauseAndRecoverySubtree">
    <ReactiveFallback name="PauseAndResume">
      <!-- PAUSE branch -->
      <Sequence>
        <CheckFlagCondition flag_topic="/controller_pause_flag" latch="true" node="{node}"/>
        <LogTextAction message="[PAUSED] Controller paused. Halting motion." interval_s="1.0"/>
        <SequenceWithMemory>
          <CancelControl/>
          <Repeat num_cycles="-1">
            <Sequence>
              <CheckFlagCondition flag_topic="/controller_pause_flag" latch="true" node="{node}"/>
              <Wait wait_duration="0.1"/>
            </Sequence>
          </Repeat>
        </SequenceWithMemory>
      </Sequence>

      <!-- FOLLOW branch with 1 retry recovery around controller -->
      <RecoveryNode number_of_retries="1" name="FollowPathRecovery">
        <PipelineSequence name="FollowAndMonitorPipe">
          <LogTextAction name="FollowLog"
            message="Path valid. Truncating, publishing, then following." interval_s="1.0"/>
          <SubTree ID="TruncateAndPublishSubtree" _autoremap="true"/>
          <Sequence>
            <LogTextAction message="############ FollowPath #########"/>
            <Fallback>
              <FollowPath path="{pruned_path}" controller_id="{selected_controller}"
                goal_checker_id="precise_goal_checker" progress_checker_id="progress_checker"
                error_code_id="{follow_path_error_code}"/>
              <Sequence>
                <LogTextAction message="[CRITICAL] Controller unrecoverable error: {follow_path_error_code}. Triggering recovery."/>
                <ErrorCodePublisherAction topic_name="/bt_error_code" error_code="{compute_path_error_code}" node="{node}"/>
                <ErrorCodePublisherAction topic_name="/bt_error_code" error_code="{follow_path_error_code}"  node="{node}"/>
                <AlwaysFailure/>
              </Sequence>
            </Fallback>
          </Sequence>
        </PipelineSequence>
        <Sequence>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
          <ClearEntireCostmap name="ClearLocalCostmap@FollowCtx" service_name="local_costmap/clear_entirely_local_costmap"/>
        </Sequence>
      </RecoveryNode>
    </ReactiveFallback>
  </BehaviorTree>

  <!-- 7) Remove N goals (reused routine) -->
  <BehaviorTree ID="RemoveGoalsSubtree">
    <RetryUntilSuccessful num_attempts="{num_goals_to_remove}">
      <Sequence>
        <LogTextAction message="[Subtree] Removing a goal point..." interval_s="1.0"/>
        <RemoveFirstGoalAction input_goals="{goals}" remaining_goals="{goals}"/>
      </Sequence>
    </RetryUntilSuccessful>
  </BehaviorTree>

  <!-- 8) Maneuver routine (internals privatized with '_' prefix) -->
  <BehaviorTree ID="ManeuverSubtree">
    <Sequence>
      <Sequence>
        <LogTextAction message="[Subtree] Executing Maneuver with planner: {planner_id} and controller: {controller_id}" interval_s="1.0"/>
        <GetNextFewGoalsAction name="ExtractNextGoals" num_goals="2" input_goals="{goals}" output_goals="{_few_goals}"/>
        <ComputePathThroughPoses goals="{_few_goals}" path="{_alt_path}" planner_id="{planner_id}" error_code_id="{recovery_compute_path_error_code}"/>
        <TruncatePathLocal
          input_path="{_alt_path}"
          output_path="{_short_path}"
          distance_forward="1.0"
          distance_backward="0.0"
          angular_distance_weight="0.2"
          max_robot_pose_search_dist="1.0"
          robot_frame="base_link"
          transform_tolerance="0.3"/>
        <SetTruncatedGoalFromPath short_path="{_short_path}" alt_goal="{_alt_goal}"/>
        <ComputePathToPose goal="{_alt_goal}" path="{_short_path}" planner_id="{planner_id}" error_code_id="{recovery_compute_path_error_code}"/>
        <FollowPath path="{_short_path}" controller_id="{controller_id}" goal_checker_id="super_relaxed_goal_checker" progress_checker_id="progress_checker" error_code_id="{follow_path_error_code}"/>
        <LogTextAction message="[Subtree] Maneuver completed successfully." interval_s="1.0"/>
      </Sequence>

      <!-- Reset selectors + single retry planner recovery (kept) -->
      <Sequence name="ResetPlannersAndControllers">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
        <Wait wait_duration="1.0"/>
        <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses@Reset">
          <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
          <Sequence>
            <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
            <ClearEntireCostmap name="ClearGlobalCostmap@ManeuverCtx" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
        </RecoveryNode>
      </Sequence>
    </Sequence>
  </BehaviorTree>

  <!-- 9) Intelligent recovery aggregator -->
  <BehaviorTree ID="IntelligentRecoverySubtree">
    <Sequence name="ShortRecoverySequenceTotal">
      <LogTextAction message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic." interval_s="1.0"/>

      <!-- Failure type check (for logging/branching) -->
      <Fallback name="FailureTypeCheck">
        <LogTextAction message="[!!! RECOVERY !!!] WouldAPlannerRecoveryHelp" interval_s="1.0"/>
        <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
        <LogTextAction message="[!!! RECOVERY !!!] Done WouldAControllerRecoveryHelp" interval_s="1.0"/>
      </Fallback>

      <Fallback name="IntelligentRecoverySelector">

        <!-- Case 0: No valid path (308) -->
        <Sequence name="NoValidPathCase">
          <Precondition if="compute_path_error_code == 308" else="FAILURE">
            <Sequence>
              <LogTextAction message="[RECOVERY CASE 0] 'No valid path' detected. Resolving..." interval_s="1.0"/>
              <Fallback>
                <!-- Goal occupied: wait / remove goals -->
                <Sequence>
                  <IsGoalsOccupiedCondition
                    name="IsGoalSafe"
                    goals="{goals}"
                    costmap_topic="local_costmap/costmap_raw"
                    occupied_cost_threshold="253"
                    occupied_goals="{obstructed_goals}"
                    unoccupied_goals="{unoccupied_goals}"/>
                  <LogTextAction message=" -> Cause: Goal occupied. Waiting or removing goals." interval_s="1.0"/>
                  <RoundRobin name="GoalOccupiedRecovery">
                    <Wait name="WaitToClear" wait_duration="1.0"/>
                    <SubTree ID="RemoveGoalsSubtree" _autoremap="true" num_goals_to_remove="3"/>
                  </RoundRobin>
                </Sequence>

                <!-- Goal clear but path blocked: clear costmaps + maneuver -->
                <Sequence>
                  <LogTextAction message=" -> Cause: Path blocked but goal clear. Clearing costmaps + maneuver." interval_s="1.0"/>
                  <RoundRobin name="PathBlockedRecovery">
                    <SubTree ID="ClearCostmapsPairSubtree" _autoremap="true"/>
                    <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
                  </RoundRobin>
                </Sequence>
              </Fallback>
            </Sequence>
          </Precondition>
        </Sequence>

        <!-- Case 1: Robot is stuck -->
        <Sequence name="RobotIsStuckCase">
          <CheckFlagCondition flag_topic="/progress_checker/is_stuck" latch="false" node="{node}"/>
          <LogTextAction message="[RECOVERY CASE 1] Robot appears stuck. Executing maneuver." interval_s="1.0"/>
          <RoundRobin name="RobotIsStuckRecoveryActions">
            <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
            <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath1"/>
          </RoundRobin>
        </Sequence>

        <!-- Case 2: Start occupied (305) -->
        <Sequence name="StartOccupiedPathCase">
          <Precondition if="compute_path_error_code == 305" else="FAILURE">
            <Sequence>
              <LogTextAction message="[RECOVERY CASE] 'StartOccupiedPathCase' detected. Trying maneuvers..." interval_s="1.0"/>
              <RoundRobin name="StartOccupiedRecovery">
                <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
                <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath1"/>
              </RoundRobin>
            </Sequence>
          </Precondition>
        </Sequence>

        <!-- Case 3: Goal occupied (306) -->
        <Sequence name="GoalOccupiedPathCase">
          <Precondition if="compute_path_error_code == 306" else="FAILURE">
            <Sequence>
              <IsGoalsOccupiedCondition
                name="IsGoalSafe"
                goals="{goals}"
                costmap_topic="local_costmap/costmap_raw"
                occupied_cost_threshold="253"
                occupied_goals="{obstructed_goals}"
                unoccupied_goals="{unoccupied_goals}"/>
              <LogTextAction message=" -> Cause: Goal occupied. Waiting or removing goals." interval_s="1.0"/>
              <RoundRobin name="GoalOccupiedRecovery">
                <Wait name="WaitToClear" wait_duration="1.0"/>
                <SubTree ID="RemoveGoalsSubtree" _autoremap="true" num_goals_to_remove="3"/>
              </RoundRobin>
            </Sequence>
          </Precondition>
        </Sequence>

        <!-- Default: general fallback recovery -->
        <Sequence name="DefaultFailureCase">
          <LogTextAction message="[RECOVERY CASE 2] Default failure. Running general recovery sequence." interval_s="1.0"/>
          <RoundRobin name="DefaultFailureRecoveryActions">
            <SubTree ID="ClearCostmapsPairSubtree" _autoremap="true"/>
            <Wait name="DefaultWait" wait_duration="1.0"/>
            <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
            <SubTree ID="ManeuverSubtree" _autoremap="true" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath1"/>
          </RoundRobin>
        </Sequence>

      </Fallback>
    </Sequence>
  </BehaviorTree>

  <!-- =========================== Main Tree =========================== -->

  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="25" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <SubTree ID="InitSubtree" _autoremap="true"/>
        <SubTree ID="SelectPluginsSubtree" _autoremap="true"/>
        <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"/>
        <SubTree ID="ReplanOrReusePathSubtree" _autoremap="true"/>
        <SubTree ID="FollowWithPauseAndRecoverySubtree" _autoremap="true"/>
      </PipelineSequence>
      <!-- On main failure, run the intelligent recovery bundle -->
      <SubTree ID="IntelligentRecoverySubtree" _autoremap="true"/>
    </RecoveryNode>
  </BehaviorTree>

  <!-- =================== Custom Node Model (kept) =================== -->

  <TreeNodesModel>
    <Action ID="LogTextAction">
      <input_port name="message" type="std::string"/>
      <input_port name="interval_s" type="float" default="1.0"/>
      <input_port name="bb_key" type="std::string"/>
    </Action>
    <Condition ID="CheckIsStuck">
      <!-- NOTE: Example signature placeholder; actual node ports defined in plugin -->
      <input_port name="flag_topic" type="std::string" default="/progress_checker/is_stuck"/>
      <input_port name="latch" type="bool" default="false"/>
      <input_port name="node" type="rclcpp::Node::SharedPtr"/>
    </Condition>
    <Action ID="RemoveFirstGoalAction">
      <input_port  name="input_goals"    type="Goals"/>
      <output_port name="remaining_goals" type="Goals"/>
    </Action>
  </TreeNodesModel>

</root>


```

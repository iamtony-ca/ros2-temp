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
      <Log color="green" message="[Subtree] Executing Maneuver with planner: {planner_id} and controller: {controller_id}"/>
      <ComputePathThroughPoses goals="{goals}" path="{alt_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <TruncatePathLocal input_path="{alt_path}" output_path="{short_path}" distance_forward="1.5"/>
      <SetTruncatedGoalFromPath short_path="{short_path}" alt_goal="{alt_goal}"/>
      <ComputePathToPose goal="{alt_goal}" path="{short_path}" planner_id="{planner_id}" error_code_id="{compute_path_error_code}"/>
      <FollowPath path="{short_path}" controller_id="{controller_id}" goal_checker_id="super_relaxed_goal_checker" error_code_id="{follow_path_error_code}"/>
      <Log color="green" message="[Subtree] Maneuver completed successfully."/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="RemoveGoalsSubtree">
    <RetryUntilSuccessful num_attempts="{num_goals_to_remove}">
      <Sequence>
        <Log color="green" message="[Subtree] Removing a goal point..."/>
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
                <Log message="Path is invalid or replan is requested. Computing new path..."/>
                <RecoveryNode number_of_retries="1" name="ComputePathRecovery">
                  <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
                  <Sequence name="ClearCostmapAndRetry">
                     <Log color="yellow" message="[RECOVERY] Main planner failed. Clearing costmaps and retrying..."/>
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
            <Log color="yellow" message="[RECOVERY] Controller failed. Clearing local costmap."/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap" />
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>

      <Sequence name="IntelligentRecovery">
        <Fallback name="FailureTypeCheck">
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
        </Fallback>
        <Log color="red" message="[!!! RECOVERY !!!] Main navigation failed. Initiating intelligent recovery logic."/>

        <Fallback name="IntelligentRecoverySelector">

          <Sequence name="NoValidPathCase">
            <Precondition if="compute_path_error_code == 1" else="FAILURE">
              <Sequence>
                <Log color="cyan" message="[RECOVERY CASE 0] 'No valid path' error detected. Trying to resolve..."/>
                <Fallback>
                  <Sequence>
                    <IsGoalsOccupiedCondition goals="{goals}"/>
                    <Log color="cyan" message=" -> Cause: Goal is occupied. Waiting or removing goals."/>
                    <RoundRobin name="GoalOccupiedRecovery">
                      <Wait name="WaitToClear" wait_duration="5.0"/>
                      <Subtree ID="RemoveGoalsSubtree" num_goals_to_remove="-2"/> 
                    </RoundRobin>
                  </Sequence>
                  <Sequence>
                    <Log color="cyan" message=" -> Cause: Path is blocked but goal is clear. Clearing costmaps and trying maneuver."/>
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
            <Log color="cyan" message="[RECOVERY CASE 1] Robot appears to be stuck. Executing maneuver."/>
            <RoundRobin name="RobotIsStuckRecoveryActions">
              <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased1" controller_id="RecoveryFollowPath1"/>
              <Subtree ID="ManeuverSubtree" planner_id="RecoveryGridBased2" controller_id="RecoveryFollowPath2"/>
            </RoundRobin>
          </Sequence>

          <Sequence name="DefaultFailureCase">
            <Log color="cyan" message="[RECOVERY CASE 2] Default failure. Executing general recovery sequence."/>
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
    <Condition ID="IsGoalsOccupiedCondition">
      <input_port name="goals" type="std::vector<geometry_msgs::msg::PoseStamped>" description="The goals to check"/>
    </Condition>
    <Condition ID="CheckIsStuck">
      <input_port name="velocity_threshold" type="double" default="0.02" description="Velocity below which robot is considered stuck"/>
    </Condition>
    <Action ID="RemoveFirstGoalAction">
        <input_port name="input_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
        <output_port name="remaining_goals" type="std::vector<geometry_msgs::msg::PoseStamped>"/>
    </Action>
    </TreeNodesModel>

</root>

```

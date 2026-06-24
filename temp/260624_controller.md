<thinking>Let me carefully analyze what changes are needed. The user wants:
1. Three AND conditions for goal_reached: remaining_goals==1, euclidean <= tol, path_distance <= 2.8m
2. Both controller and goal_checker apply these
3. Don't look at loop detection
4. Minimal code changes, no bugs

Let me figure out exactly what to add to each file.

Files:
- graceful_controller.hpp (controller header)
- graceful_controller.cpp (controller impl)
- stable_stopped_goal_checker.hpp (goal checker header)
- stable_stopped_goal_checker.cpp (goal checker impl)

I need to:
1. Both need to subscribe to /remaining_goals
2. Both apply the 3-condition AND gate

For the goal_checker: it already subscribes to /plan_truncated_short for path length. The user wants the gate to be: remaining_goals==1 AND euclidean<=tol AND path_distance<=2.8. The current gate is just `total_distance > 2.0*x_goal_tolerance_ -> return false`. 

Wait - the goal_checker already has goal_pose passed in (the euclidean distance can be computed). It already has current_path_ (path distance). It needs remaining_goals subscription added.

For the controller: it computes euclidean_dist and dist_to_goal already. It needs remaining_goals subscription. Then the [6] mode decision needs the 3-condition gate.

Let me think about message type for /remaining_goals. PublishRemainingGoalsAction - I don't have its source. The user hasn't provided it. nav2's Goals type in Jazzy is `nav2_msgs/msg/Goals` (introduced) or it could be nav_msgs/Path or geometry_msgs/PoseArray. I need to flag this uncertainty and not guess. Actually, I should ask or make it configurable. But the user wants code. Let me be honest: I don't know the type, so I'll note it and pick the most likely (nav_msgs::msg::Path, since the goal_checker already uses Path and it's consistent, OR geometry_msgs::msg::PoseArray). Actually "remaining_goals" - in their BT, goals is `{goals}` which for navigate_through_poses is `nav2_msgs/Goals` or vector of PoseStamped. PublishRemainingGoalsAction is custom.

I need to be honest about this uncertainty rather than fabricate. Let me note that the message type must be confirmed and provide code assuming a type but flag it clearly. I'll use the count - really I only need `.poses.size()` or equivalent count.

Let me keep it minimal. Let me think about what to actually write.

Let me be disciplined and concise per the response format, but this is a complex code task so it needs to be thorough. The user is an expert. Let me give precise diffs.

Key concern: minimal changes, no bugs. Let me identify exactly the edits.

### Controller (graceful_controller.hpp)
Add:
- subscription member for remaining_goals
- callback
- a count member + mutex
- need rclcpp node? The controller has access to node in configure via parent.lock(). Need to create subscription there.

Wait - the controller is a plugin. In configure() it has `parent` weakptr. It can create a subscription on that node. But the controller_server node spins - will the subscription callback be served? The controller plugin runs in controller_server which has an executor. Subscriptions created on the node will be served by the node's executor. Yes this works (the goal_checker already does it with path_sub_).

So I'll mirror the goal_checker's pattern.

Need a header for the message. If it's nav_msgs::msg::Path... but remaining_goals is goals not a path. Hmm. Let me think - most likely it's `nav2_msgs::msg::Goals` (Jazzy) or `geometry_msgs::msg::PoseArray`. I genuinely don't know. I'll write it to be easily swappable and clearly flag.

Actually, the cleanest approach given uncertainty: subscribe and just store the count. I'll need to pick a type to compile. Let me flag clearly that they must confirm and adjust the type + size accessor.

Let me provide the code assuming `nav_msgs::msg::Path` (poses.size()) but with a big clear note that if PublishRemainingGoalsAction publishes PoseArray or nav2_msgs/Goals, change the type and the size accessor accordingly. Actually for nav2_msgs/Goals, the field is `goals` (a vector of PoseStamped) — accessor would be `msg->goals.size()`. For PoseArray it's `msg->poses.size()`. For Path it's `msg->poses.size()`.

I'll make the count extraction a single line they can adjust.

### The 3-condition gate

For controller [6]:
```cpp
const double xy_tol = custom_checker ? custom_checker->getXGoalTolerance() : goal_dist_tolerance_;
const double PATH_DIST_MAX = 2.8;

size_t remaining_count;
{
  std::lock_guard<std::mutex> lock(remaining_goals_mutex_);
  remaining_count = remaining_goals_count_;
}
bool is_final_goal = (remaining_count <= 1);  // includes 0/unknown -> treat as final? 

bool gate_ok = is_final_goal
             && (euclidean_dist <= xy_tol)
             && (dist_to_goal <= PATH_DIST_MAX);
```

Wait, but what about when remaining_goals topic hasn't arrived (count unknown)? For navigate_through_poses it's always published. But initial nullptr. I need a default. Let me use a flag `remaining_goals_received_`. If not received, what? Conservative would be "not final" -> never reach goal until topic arrives. But that could deadlock if topic never comes (single goal navigate_to_pose). Hmm.

But this controller is used in navigate_through_poses (FollowPath_Main). And also in recovery (with relaxed checker). Recovery uses cfg_relaxed_checker = super_relaxed_goal_checker (SimpleGoalChecker, not custom). So custom_checker path only in main.

Let me think about the default carefully. The user said RemovePassedGoals 0.5m is fine. /remaining_goals is published at the start of MainTree before FollowPath. So by the time FollowPath runs, remaining_goals should have been published at least once (PublishRemainingGoalsAction runs before the Follow subtree in MainTree). So in practice it arrives. But to be safe, I'll default to: if not received, treat the count condition as... I'll make it configurable but lean toward "if never received, fall back to not gating on count" so single-goal still works. Actually the safest for THEIR setup (always multi-goal navigate_through_poses with remaining_goals published): if not received, don't allow goal reached yet (return false / don't enter rotation). But that risks deadlock in edge cases.

Hmm, let me reconsider. The cleanest: default count to a large number (not final) until received. Then once received use it. For navigate_through_poses this is fine since it's always published. I'll note this tradeoff.

Actually wait — there's a subtlety. The controller's [6] currently has TWO sub-paths: custom_checker and else (non-custom). For the else path (recovery with SimpleGoalChecker), there's no remaining_goals concept needed and the gate would break recovery if applied. So I should ONLY apply the new gate in the custom_checker branch. Good — that limits scope and avoids breaking recovery.

So:
```cpp
if (custom_checker) {
    // 3-condition gate
    bool gate_ok = is_final_goal
                 && (euclidean_dist <= custom_checker->getXGoalTolerance())
                 && (dist_to_goal <= PATH_DIST_MAX);
    if (gate_ok) {
        if (custom_checker->isXYLatched()) {
            enter_rotation_mode = true;
        } else {
            is_xy_stabilizing_phase = true;
        }
    }
    // gate_ok false -> neither mode -> falls to path following
} else {
    // unchanged
    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
        enter_rotation_mode = true;
    }
}
```

Wait, but the original logic was:
```cpp
if (custom_checker) {
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;
    } else if (euclidean_dist <= custom_checker->getXGoalTolerance()) {
        is_xy_stabilizing_phase = true;
    }
}
```

The isXYLatched check — if XY is already latched (goal_checker passed XY phase, now checking yaw), the controller should rotate. But with the new gate, if isXYLatched is true, that means goal_checker already accepted XY... but goal_checker now also has the 3-condition gate, so it wouldn't have latched unless gate passed. So isXYLatched being true implies the gate already passed in the goal_checker. So I can keep isXYLatched as a strong signal. But to be consistent let me structure:

Actually the cleanest minimal change preserving intent:

```cpp
if (custom_checker) {
    bool gate_ok = is_final_goal
                 && (euclidean_dist <= custom_checker->getXGoalTolerance())
                 && (dist_to_goal <= PATH_DIST_MAX);
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;   // already past XY in checker -> rotate
    } else if (gate_ok) {
        is_xy_stabilizing_phase = true;
    }
}
```

Hmm but if isXYLatched, should we still require gate? If checker latched XY, it means checker's own gate passed (with the same 3 conditions). So trusting isXYLatched is fine. Keep it.

But wait, there's a concern: the existing condition only checked euclidean. Now I add is_final_goal and path_dist. That's exactly what the user wants. Good.

Now `dist_to_goal` in the controller is `calculate_path_length(transformed_plan)` which is path length of the (pruned/transformed) plan. That's the controller's view of remaining path. With search_dist reduced, transformed_plan starts near robot. Good — this is the right "remaining path distance".

### goal_checker gate

Current:
```cpp
if (current_path_) {
    double total_distance = calculate_path_length(*current_path_);
    if (total_distance > 2.0 * x_goal_tolerance_) {
        return false;
    }
}
```

The user wants: remaining_goals==1 AND euclidean<=tol AND path_distance<=2.8. So replace the gate with:
```cpp
// path distance condition
double total_distance = current_path_ ? calculate_path_length(*current_path_) : 0.0;
double straight_dist = hypot(query_pose.x - goal_pose.x, query_pose.y - goal_pose.y);
size_t remaining_count = ...;

bool gate_ok = (remaining_count <= 1)
             && (straight_dist <= x_goal_tolerance_)
             && (total_distance <= PATH_DIST_MAX);
if (!gate_ok) {
    return false;
}
```

Wait — but the goal_checker's normal logic ALSO checks dx/dy <= x_goal_tolerance. The euclidean (straight_dist) vs separate dx,dy. The user wants euclidean <= 0.1 (or tol). But the existing logic uses dx<=tol AND dy<=tol (per-axis). euclidean (hypot) <= tol is slightly stricter than per-axis. That's fine — it's an additional gate before the detailed logic. Actually if straight_dist <= tol then both dx,dy <= tol automatically (since hypot >= each component). Wait no: hypot(dx,dy) <= tol implies dx<=tol and dy<=tol. So the gate (straight<=tol) is stricter; passing it guarantees the per-axis xy_ok. That's fine, no conflict. Actually it means once gate passes, xy_ok is guaranteed true in the stateful logic. Good, consistent.

Hmm but should I use x_goal_tolerance_ or a slightly larger value for the gate? The user said "euclidean <= 0.1m" but also agreed to use x_goal_tolerance_ (0.045). If I gate at straight_dist <= x_goal_tolerance_ (0.045), then the detailed dx/dy check (also 0.045 per axis) — passing the hypot gate is stricter, so the per-axis always passes. That's fine but means the gate is the binding constraint. Actually that's slightly TOO strict because the original allowed dx=0.045, dy=0.045 (corner = 0.063 hypot). With gate hypot<=0.045, that corner case (0.063) is rejected. Minor tightening. Probably acceptable, but to preserve original behavior maybe gate should be a bit looser, e.g. straight_dist <= x_goal_tolerance_ * 1.5. The user mentioned "비례한 값". Let me use a small multiplier or just note it. To keep it minimal and avoid tightening the actual tolerance, I'll gate slightly looser so the real per-axis check remains the binding one. Let me use `x_goal_tolerance_ * 1.5` for the gate. Actually, hmm. Let me reconsider — the gate's purpose here is "don't even enter the detailed logic unless physically close". Making it exactly tol could reject the diagonal corner. Using 1.5x keeps the gate as a coarse pre-filter and lets the existing per-axis logic do the precise check. I'll go with a factor, and explain.

Actually, simpler and clean: gate on straight_dist but use the diagonal bound. Or just note it. Let me use `x_goal_tolerance_ * 1.5` and explain it's a coarse pre-gate so the precise per-axis check stays binding. Good.

Now the remaining_goals count in goal_checker — need a subscription too. Same uncertainty about message type.

### Critical bug concerns

1. **latch reset on goal count change** — I raised this. When remaining_goals goes 2->1, the checker's check_xy_/latch state may be stale. Need to reset on count change. I'll add this in the remaining_goals callback.

Actually wait — is this needed? When count is 2, the gate returns false always (remaining_count<=1 false). During that time the detailed stateful logic never runs (we return false before it). So check_xy_ stays at whatever reset() set it to. reset() is called by controller_server on each new goal/path? Actually reset() is called when a new navigation goal starts. During a single navigate_through_poses, does reset() get called between waypoints? Not necessarily — it's one FollowPath action following one path. Hmm.

Actually within the gate-blocked period (count>=2), we return false before touching check_xy_. So check_xy_ remains true (from reset or initial). in_xy_tolerance_ stays false. When count drops to 1 and gate passes, the stateful logic starts fresh with check_xy_=true. That's actually fine — no stale state because we never advanced the state machine while gated. 

Wait, is that right? Let me trace: count=2, every isGoalReached call returns false at the gate (before the stateful block). So check_xy_, in_xy_tolerance_ never change. They remain at reset values (check_xy_=true, in_xy_tolerance_=false). When count becomes 1 and robot is at final goal, gate passes, stateful block runs with check_xy_=true → starts XY stability timing fresh. 

But there's a subtle issue: what if at an EARLIER point (count=2, but the gate's other conditions...) no — gate requires count<=1 AND straight<=tol AND path<=2.8 ALL true. If count=2, gate fails regardless. So stateful logic never runs while count>=2. So no stale latch. 

So actually the latch reset I worried about earlier may NOT be needed, because the gate blocks the stateful logic entirely until count==1. Let me double check: the only way check_xy_ becomes false is in the stateful block ("If XY is stable for duration, switch to Yaw phase: check_xy_ = false"). That block only runs after passing the gate. Gate requires count<=1. So check_xy_ can only go false when count<=1. Once count<=1 it stays (RemoveFirstGoals making it 0? then count=0<=1 still). So no transition from a higher count corrupting state. 

Hence latch reset NOT needed. I over-worried earlier. Let me verify there's no path where stateful state advances at the wrong goal. The risk would be: robot passes near an intermediate waypoint with count==2 → gate blocks (count condition) → good. So no false latch. I'll note that the gate ordering (count check before stateful logic) inherently prevents stale latch, so no extra reset needed. This is cleaner. Good — minimal change.

But wait, one more: what about the FIRST waypoint when there are e.g. 3 goals? count=3, gate blocks. Robot drives. RemovePassedGoals reduces to 2, then 1. When 1, gate can pass at final. Good. Intermediate waypoints are never "reached" by the checker — they're just driven past. That's the intended behavior (the user said 0.5m RemovePassedGoals handles passing). Good.

2. **nullptr / not-received default for count.** If remaining_goals not received yet, count unknown. I'll store count with a "received" flag. If not received → treat as count = large (not final) → gate blocks. For navigate_through_poses this is fine (published before follow). But is there risk of permanent block if topic never arrives? In their BT, PublishRemainingGoalsAction runs in MainTree before the follow subtree every pipeline cycle, so it's published. I'll default to "not final until received" but note the tradeoff and that for navigate_to_pose (single goal, no remaining_goals topic) this checker shouldn't be used, or they should publish a count=1. Actually — safer default given their setup: Let me think which is less dangerous. If default "final" (count=1 when unknown): risk = early goal reached before topic arrives (the very bug we're fixing). If default "not final": risk = never reaches goal if topic never published. Given the bug we're fixing is exactly premature reach, defaulting to "not final" (block) is safer for correctness, and their BT always publishes so no deadlock. I'll go with block-until-received and clearly note it.

Hmm, but actually there's a real risk: what if there's latency and the topic arrives a bit late — robot reaches final goal, but remaining_goals still shows old count? It's published at publish_interval 0.2 (5Hz) and also republished in MainTree each cycle. Should be current enough. The count changes via RemovePassedGoals which happens in MainTree. So by the time robot is at final goal, count should be 1. Fine.

3. **Message type** — genuinely unknown. I'll flag prominently and make it a one-line change point.

4. **Thread safety** — subscription callback runs in node executor thread; isGoalReached/computeVelocityCommands run in controller_server thread. Need mutex. I'll add mutex for the count. Actually a simple atomic<size_t> + atomic<bool> avoids mutex. Cleaner: use std::atomic. But count from msg in callback — store size into atomic. Use std::atomic<int> remaining_goals_count_{-1} (-1 = not received). That's clean and lock-free. 

5. **Controller: where to create subscription** — in configure(), using node from parent.lock(). Mirror goal_checker.

6. **dist_to_goal in controller when reduced** — fine.

Let me also reconsider: the controller's gate uses `dist_to_goal` = calculate_path_length(transformed_plan). With the PATH_DIST_MAX=2.8 condition. But note the existing [5] slowdown already uses dist_to_goal. No conflict.

Also: the else branch (non-custom checker, recovery) — leave unchanged. Good. But should recovery also get remaining_goals? Recovery uses super_relaxed (SimpleGoalChecker). The user only wants the 3 conditions in advanced_graceful_controller + stable_stopped_goal_checker. Recovery's relaxed checker is a stock SimpleGoalChecker — can't add to it without subclassing. Leave it. The controller's else-branch is used by recovery's FollowShort? No — recovery uses RecoveryFollowPath1 (MPPI) or RotationGraceful, controller_id = cfg_recov_controller = "RecoveryFollowPath1". That's MPPI, not advanced_graceful_controller. So advanced_graceful_controller (FollowPath) only runs in main with custom_checker. The else-branch in computeVelocityCommands is essentially dead for this controller? Not necessarily — RotationGraceful uses nav2_graceful_controller not advanced. So advanced_graceful_controller = "FollowPath" only, always with precise_goal_checker (custom). So else-branch rarely hit. Still leave it safe.

OK. Now also need to make sure I subscribe with the right callback group so it actually gets called. controller_server uses a node; creating a subscription on it — will its executor call the callback? The goal_checker already does exactly this (path_sub_) and presumably works (user uses it). So same pattern works. Good.

Now let me also handle: the controller already includes the goal checker header. Need to add subscription. Need `#include "nav_msgs/msg/path.hpp"` or whatever type. Controller already pulls in lots via nav2_core.

Let me also reconsider whether to add a tunable param for PATH_DIST_MAX (2.8) and the tol factor. The controller uses hardcoded tuning constants at top of computeVelocityCommands already (their style). So I'll add it there as a local const, matching their style. For goal_checker, add as a member with a param declaration (matching their param style) or hardcode. To keep minimal but configurable, I'll add a param `path_distance_max` to goal_checker. Actually minimal = hardcode with clear constant. But goal_checker already has param infrastructure. Hmm, "minimal changes" — I'll add a param for path_distance_max in goal_checker (since it's the cleaner spot and they parametrize everything) but keep it simple. Actually let me hardcode as a constant in both with a clear name and a comment to tune, to truly minimize. The user can promote to param later. I'll mention param option.

Wait, but the gate value should match between controller and checker (2.8). Hardcoding in both is a duplication risk. I'll note they must keep them in sync, or better — only the checker's gate actually gates "goal reached"; the controller's gate gates "rotation mode". They serve different purposes so slight divergence is tolerable, but conceptually should match. I'll use the same constant value in both and note it.

Let me now write the actual minimal edits.

Let me reconsider the message type one more time. The BT node is `PublishRemainingGoalsAction`. In nav2 Jazzy, `nav2_msgs/msg/Goals` exists (it's `geometry_msgs/PoseStamped[] goals` with header). Many custom impls publish `geometry_msgs/PoseArray` for viz or `nav_msgs/Path`. I really can't know. I'll write with a typedef/using and a single accessor function, and flag it as THE thing to confirm. I'll default to `geometry_msgs::msg::PoseArray` because "remaining_goals" as a viz/count topic is commonly PoseArray... but honestly nav2_msgs/Goals is more likely for a goals list in Jazzy. Ugh.

I'll be honest: I'll provide the code with a clearly-marked type and accessor, defaulting to nav_msgs::msg::Path (since the checker already depends on nav_msgs/Path and it minimizes new includes), but with a prominent note: "confirm the actual type published by PublishRemainingGoalsAction; if it's PoseArray or nav2_msgs/Goals, change the type and the .poses.size()/.goals.size() accessor." This is the honest approach.

Actually — I should just tell them I need the publisher type and provide the code parameterized on it, making the two change-points obvious. Given the whole conversation they've been providing sources; I'll note that I still need the PublishRemainingGoalsAction publisher declaration to lock the type, and provide code that's trivially adjustable.

Let me write concise, precise edits now. I'll keep explanation tight given length.

Let me structure:
1. Quick summary of the 4 edit locations.
2. goal_checker.hpp edits
3. goal_checker.cpp edits
4. controller.hpp edits
5. controller.cpp edits
6. The message-type caveat + sync note + search_dist + kappa guard reminder (briefly, since those are separate and already discussed — but user said "수정이 필요한 부분을 알려주고" for these 3-condition gate; the kappa/search_dist are separate. I'll mention they're separate and not include unless asked, to keep minimal per request. Actually user earlier agreed kappa guard + search_dist needed. But this message specifically asks for the 3-condition AND gate implementation. I'll focus on that and briefly remind the other two are separate edits not included here.)

Let me write the code.

For atomic count: `std::atomic<int> remaining_goals_count_{-1};` Need `#include <atomic>`.

Controller header additions:
```cpp
#include <atomic>
// in protected:
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;
std::atomic<int> remaining_goals_count_{-1};
void remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);
```
Wait controller header already includes path_handler etc. nav_msgs/path is available via nav2 includes. Add explicit include to be safe.

Controller configure():
```cpp
remaining_goals_sub_ = node->create_subscription<nav_msgs::msg::Path>(
  "/remaining_goals", rclcpp::QoS(10),
  std::bind(&GracefulController::remainingGoalsCallback, this, std::placeholders::_1));
```
callback:
```cpp
void GracefulController::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  remaining_goals_count_.store(static_cast<int>(msg->poses.size()));
}
```

Controller [6] edit:
```cpp
// 도착 게이트 상수
const double PATH_DIST_MAX = 2.8;
const int rgc = remaining_goals_count_.load();
const bool is_final_goal = (rgc == 1);  // 미수신(-1)이면 false -> 게이트 차단

if (custom_checker) {
    const double xy_tol = custom_checker->getXGoalTolerance();
    const bool gate_ok = is_final_goal
                       && (euclidean_dist <= xy_tol)
                       && (dist_to_goal <= PATH_DIST_MAX);
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;
    } else if (gate_ok) {
        is_xy_stabilizing_phase = true;
    }
} else {
    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {
        enter_rotation_mode = true;
    }
}
```

Wait — is_final_goal == (rgc==1). What if rgc==0? That could happen if all goals removed (e.g., RemoveFirstGoals took the last). Then is_final_goal false → never reach. Edge: after reaching final goal, count might drop to 0. But goal reached should fire before that. Hmm, if rgc becomes 0 while still trying to finish yaw... Let me use `rgc == 1` per user's exact spec ("개수는 1일때"). But to be safe against count==0 (last goal, maybe already culled), maybe `rgc <= 1 && rgc >= 0`? The user explicitly said "1일때". But count==0 meaning no goals left — arguably done already. I think `rgc == 1` is what user wants; if there's a count==0 race it'd block. Let me use `(rgc == 1)` but note the count==0 edge. Actually safer: `is_final_goal = (rgc == 1)`. Honor user spec, note edge. Hmm, but if RemovePassedGoals removes the final goal when robot gets within 0.5m, count→0 before yaw alignment completes → gate blocks → can't finish. That's a real risk!

Let me check: in MainTree, RemovePassedGoals radius=0.5 runs every pipeline cycle. When robot is within 0.5m of final goal (doing yaw alignment, not moving in xy), RemovePassedGoals might remove the final goal → count 1→0. Then is_final_goal(rgc==1) → false → gate blocks → controller stops rotating, goal never reached. DEADLOCK.

So `rgc == 1` is dangerous. Should be `rgc <= 1` (i.e., 1 or 0). When count is 0, we've passed/removed all goals — treat as final. But rgc==-1 (not received) should still block. So: `is_final_goal = (rgc == 0 || rgc == 1)` = `(rgc >= 0 && rgc <= 1)`. That handles the cull-to-zero race. Let me use `(rgc >= 0 && rgc <= 1)`. This is more robust. I'll explain why (RemovePassedGoals can cull final goal to 0).

Hmm wait, but does RemovePassedGoals actually remove the final goal during yaw alignment? RemovePassedGoals removes goals the robot has passed (within radius). If robot is AT the final goal doing yaw, it's within 0.5m, so yes it'd be removed → count 0. So `rgc <= 1` (including 0) is essential. Good catch. Let me use `(rgc >= 0 && rgc <= 1)`.

Actually, also consider: with multi-goal, when does count first hit 1? When second-to-last is passed. Then robot heads to final. Approaching final, within 0.5m → count 0. So during final approach/alignment count is likely 0, not 1! So `rgc==1` would almost never be the state during final goal alignment. `rgc<=1` (0 or 1) is correct. This strongly confirms `rgc <= 1`.

But then what distinguishes "final goal" from "intermediate"? Intermediate goals: count >= 2 (there's the intermediate + final + ...). Wait, "remaining goals" — if 3 total goals [A,B,C], at start count=3. Pass A → 2. Pass B → 1. Now heading to C (final), count=1. Approach C within 0.5m → 0. So count==1 means "C is the only one left = final, en route". count==0 means "even C culled". Both are final-goal territory. count>=2 means intermediate still pending. So `rgc <= 1` (with rgc>=0 to exclude -1 unknown) correctly = "final goal phase". 

So `is_final_goal = (rgc == 0 || rgc == 1)`.

Now goal_checker gate similarly uses count. Same `rgc <= 1 && rgc >= 0`.

For goal_checker, message type same uncertainty. Add atomic count + sub.

goal_checker.hpp additions:
```cpp
#include <atomic>
// members:
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;
std::atomic<int> remaining_goals_count_{-1};
void remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);
```

goal_checker.cpp initialize(): add subscription. callback. And the gate replacement.

goal_checker gate:
```cpp
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  const double total_distance = current_path_
      ? nav2_util::geometry_utils::calculate_path_length(*current_path_) : 0.0;
  const double straight_dist = std::hypot(
      query_pose.position.x - goal_pose.position.x,
      query_pose.position.y - goal_pose.position.y);
  const int rgc = remaining_goals_count_.load();
  const bool is_final_goal = (rgc >= 0 && rgc <= 1);

  const double PATH_DIST_MAX = 2.8;
  const bool gate_ok = is_final_goal
                     && (straight_dist <= x_goal_tolerance_ * 1.5)
                     && (total_distance <= PATH_DIST_MAX);
  if (!gate_ok) {
    return false;
  }
}
```

Wait — current behavior: if current_path_ is null, the original SKIPS the gate (passes through). My new code: if null, total_distance=0.0 → that condition passes, but is_final_goal still needs count. If count not received (-1) → gate fails → return false. So with both topics unreceived, it blocks. For their setup both are published. Note it.

Also note: putting `return false` inside the mutex scope — fine, lock_guard releases on return. Good.

One concern: straight_dist <= x_goal_tolerance_*1.5. The detailed stateful logic later checks dx,dy <= x_goal_tolerance_ (per axis, 0.045). The gate at hypot<=0.0675. So gate passes a bit wider, then per-axis does precise. Good — gate is coarse prefilter, per-axis binding. Consistent with my earlier reasoning.

Now, the path_mutex_ scope — original gate locked path_mutex_ for current_path_. I keep that. remaining_goals_count_ is atomic, no lock needed. Good.

Now there's a question: does goal_checker even need the path_distance (current_path_) condition anymore, given it also has count? The user wants all 3. Keep total_distance condition. Fine. It uses current_path_ from /plan_truncated_short (4m). With count==final and straight<=tol, total_distance<=2.8 — for the loop case the truncated_short might be ~1.9m <=2.8 ✓. Good, loop passes now (previously blocked at 0.09). 

Now also: should I keep the OLD `total_distance > 2.0*x_goal_tolerance_` gate? No — replace it entirely with the new 3-condition gate. The new path condition (<=2.8) replaces it. Good.

Let me make sure removing the old gate doesn't break anything. Old gate blocked when path long. New gate blocks when NOT(final && close && path<=2.8). For a far-away goal mid-journey: count likely >=2 → blocked. ✓. For final goal far: count<=1 but straight_dist large → blocked. ✓. Good, equivalent-or-better.

Now let me also double check the controller's existing `is_xy_stabilizing_phase` path: when gate_ok and not latched → is_xy_stabilizing_phase=true. Then Phase 2 runs setSpeedLimit(0.01,0.3,MAX_IN_PLACE_VEL) and falls to path following with reduced speed. That's existing behavior. With the gate now requiring path<=2.8 too, the loop (path 1.9) at final goal would enter is_xy_stabilizing_phase only when path<=2.8. For a fresh loop of 1.9m, path<=2.8 ✓ immediately, so it'd enter stabilizing right away → and then the kappa issue... but that's the kappa guard's job (separate). For "A" behavior (drive the loop), hmm — wait, if path=1.9<=2.8 and euclidean<=tol and count<=1, the gate passes IMMEDIATELY at loop start, so controller enters is_xy_stabilizing_phase → tries to stabilize XY → but euclidean already < tol so... Phase 2: `if euclidean_dist < 0.007 cmd_found=true else setSpeedLimit low and path-follow`. So it'd path-follow the loop slowly. Eventually completes. Actually this isn't quite "drive the full loop at normal speed" — it's "slow stabilize". 

Hmm, the user's earlier "A" conclusion was: loop with path>2.8 → gate fails → normal path follow (drive loop). loop with path<=2.8 → gate passes → stabilize/rotate. For a 1.9m loop, path<=2.8 from the start, so it goes straight to stabilize mode. Is that OK? In stabilize mode it still path-follows (Phase 2 else branch) but at low speed (0.3). So it drives the loop slowly then aligns. That's acceptable per "A" (it does traverse, just slowly). And kappa guard handles the spike. Fine. The user accepted this.

Actually wait, re-examine Phase 2 stabilizing: it sets speed limit low then falls through to path-following `if (!cmd_found)`. So robot follows the loop path at <=0.3 m/s. Drives the 1.9m loop slowly, reaches end, euclidean<0.007 → done, then... actually rotation? Phase 2 doesn't rotate to goal yaw. Hmm. Let me not over-engineer; this is existing Phase2 behavior and the user is iterating. The gate change is what they asked. I'll implement the gate and note Phase2/rotation interplay is unchanged.

OK I think I've analyzed enough. Let me also remember to keep changes minimal and flag the 2 separate items (kappa guard, search_dist) as NOT included here since user scoped this to the 3-condition gate.

Also the message-type: I'll prominently flag and provide a single using-alias or note. Let me provide code with nav_msgs::msg::Path and a clear ⚠️ to confirm/adjust.

Let me write it concisely now. Given response length guidance, but this is a code task requiring precision. I'll be thorough but not bloated. Korean response.

Let me make sure about includes and the count default semantics one more time, then write.

Also: I should double-check — in controller, `dist_to_goal` is computed before [6]. Yes, at [3]. Good. `euclidean_dist` too. Good.

One more bug check: controller's gate uses `dist_to_goal <= PATH_DIST_MAX` where dist_to_goal = path_length(transformed_plan). With search_dist reduced to ~2.0, transformed_plan is at most ~5m (costmap bound) but starts near robot. For a loop, transformed_plan would contain the loop from robot's closest point. Fine.

Write it.

I'll keep the "verify message type" as the one open item and offer to finalize once they confirm. Also remind sync of 2.8 constant and that kappa/search_dist are separate edits.루프 조건 빼고 3-조건 AND 게이트만 — 깔끔합니다. 수정 위치는 **4개 파일에 각각 한 곳씩**입니다. 최소 변경으로 정리하되, 분석 중 발견한 **버그 두 개**(count==0 race, 메시지 타입)를 같이 짚겠습니다.

## ⚠️ 먼저 확정해야 할 것: `/remaining_goals` 메시지 타입

`PublishRemainingGoalsAction`의 publisher 타입을 제가 모릅니다. 아래 코드는 **`nav_msgs::msg::Path`로 가정**했는데(checker가 이미 Path 의존이라 추가 include 최소), 실제로 `geometry_msgs::msg::PoseArray`나 `nav2_msgs::msg::Goals`면 **타입과 `.poses.size()` 접근자만** 바꾸면 됩니다. 이 한 줄만 확인해 주세요. 나머지 로직은 타입과 무관합니다.

## 핵심 설계 결정: `count == 1`이 아니라 `count <= 1`

당신은 "goal 1개일 때"라고 했지만, 코드로는 **`count == 0 || count == 1`**이어야 합니다. 이유가 중요합니다.

MainTree에서 `RemovePassedGoals radius="0.5"`가 매 사이클 돕니다. 로봇이 **최종 goal에 0.5m 이내로 접근해 yaw 정렬 중**이면, 이 노드가 최종 goal을 "지났다"고 제거해서 **count가 1→0**이 됩니다. 그러면 `count==1` 조건은 거짓이 되어 → 게이트 차단 → 회전 못 끝냄 → **deadlock**. 실제로 최종 goal 정렬 구간에서는 count가 1이 아니라 **0일 가능성이 더 큽니다.**

그래서 `is_final_goal = (rgc >= 0 && rgc <= 1)`. `rgc == -1`(미수신)은 차단, `0/1`은 최종 phase로 통과. count≥2는 중간 goal 남음 → 차단.

---

## ① goal_checker .hpp

```cpp
#include <atomic>                       // [추가]
// ... 기존 include 들 ...

// protected: 멤버에 추가
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;  // [추가] ⚠️타입 확인
  std::atomic<int> remaining_goals_count_{-1};                               // [추가] -1=미수신
  void remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);     // [추가]
```

## ② goal_checker .cpp

initialize()에 구독 추가 (기존 path_sub_ 옆):

```cpp
  remaining_goals_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/remaining_goals", rclcpp::QoS(10),
    std::bind(&StableStoppedGoalChecker::remainingGoalsCallback, this, _1));
```

콜백 추가:

```cpp
void StableStoppedGoalChecker::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  remaining_goals_count_.store(static_cast<int>(msg->poses.size()));  // ⚠️ PoseArray/Goals면 접근자 변경
}
```

`isGoalReached` 맨 앞의 **기존 path-length 게이트를 통째로 교체**:

```cpp
  // [교체] 3-조건 AND 게이트 (remaining_goals==최종 && 직선거리 근접 && 경로 짧음)
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    const double total_distance = current_path_
      ? nav2_util::geometry_utils::calculate_path_length(*current_path_) : 0.0;
    const double straight_dist = std::hypot(
      query_pose.position.x - goal_pose.position.x,
      query_pose.position.y - goal_pose.position.y);

    const int  rgc = remaining_goals_count_.load();
    const bool is_final_goal = (rgc >= 0 && rgc <= 1);   // -1(미수신)은 차단

    constexpr double PATH_DIST_MAX = 2.8;                // 컨트롤러와 동일값 유지
    const bool gate_ok = is_final_goal
                       && (straight_dist <= x_goal_tolerance_ * 1.5)  // 코스 프리게이트
                       && (total_distance <= PATH_DIST_MAX);
    if (!gate_ok) {
      return false;
    }
  }
  // 이하 기존 dx/dy/yaw/velocity stateful 로직 그대로
```

여기서 두 가지 짚을 점:

- `straight_dist <= x_goal_tolerance_ * 1.5`로 **약간 느슨하게** 잡았습니다. 게이트를 정확히 `x_goal_tolerance_`(0.045)로 하면 대각선 코너(dx=dy=0.045 → hypot 0.063)가 잘립니다. 1.5배는 코스 프리게이트 역할만 하고, **정밀 판정은 뒤의 per-axis `dx/dy <= x_goal_tolerance_`가** 그대로 담당합니다.
- **latch reset 불필요**(이전에 제가 걱정했던 것 정정). 게이트가 stateful 블록 *앞에서* 막으므로, count≥2인 동안 `check_xy_`가 절대 전진하지 않습니다. count가 1/0이 되어 게이트를 처음 통과할 때 `check_xy_`는 초기값(true) 그대로라 깨끗하게 시작합니다. 그래서 별도 reset 코드가 없어도 stale latch가 안 생깁니다.

---

## ③ controller .hpp

```cpp
#include <atomic>   // [추가] (이미 다수 include 있음)

// protected: 멤버에 추가
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr remaining_goals_sub_;   // [추가] ⚠️타입 확인
  std::atomic<int> remaining_goals_count_{-1};                                // [추가]
  void remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg);      // [추가]
```

## ④ controller .cpp

configure()에 구독 추가 (publisher 생성 옆):

```cpp
  remaining_goals_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/remaining_goals", rclcpp::QoS(10),
    std::bind(&GracefulController::remainingGoalsCallback, this, std::placeholders::_1));
```

콜백 추가:

```cpp
void GracefulController::remainingGoalsCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  remaining_goals_count_.store(static_cast<int>(msg->poses.size()));  // ⚠️ 접근자 타입 맞춤
}
```

`[6]` 모드 결정에서 **custom_checker 분기만** 교체 (else 분기는 recovery용이라 건드리지 않음):

```cpp
  bool enter_rotation_mode = false;
  bool is_xy_stabilizing_phase = false;

  constexpr double PATH_DIST_MAX = 2.8;                 // goal_checker와 동일값
  const int  rgc = remaining_goals_count_.load();
  const bool is_final_goal = (rgc >= 0 && rgc <= 1);    // -1 차단

  if (custom_checker) {
    const double xy_tol = custom_checker->getXGoalTolerance();
    const bool gate_ok = is_final_goal
                       && (euclidean_dist <= xy_tol)
                       && (dist_to_goal <= PATH_DIST_MAX);
    if (custom_checker->isXYLatched()) {
        enter_rotation_mode = true;          // checker가 이미 XY 통과(=게이트 통과)했으므로 신뢰
    } else if (gate_ok) {
        is_xy_stabilizing_phase = true;
    }
    // gate_ok 거짓 → 어떤 모드도 안 켬 → 경로 추종으로 떨어짐 (루프/교차/중간 = 그냥 주행)
  } else {
    if (dist_to_goal < goal_dist_tolerance_ || goal_reached_) {   // 기존 그대로
        enter_rotation_mode = true;
    }
  }
```

`isXYLatched()`를 게이트 없이 신뢰하는 이유: checker가 XY를 latch했다는 건 checker의 동일한 3-조건 게이트를 이미 통과했다는 뜻이라, 컨트롤러에서 다시 검사할 필요가 없습니다.

---

## 빠뜨리면 안 되는 것 (이 패치 범위 밖, 별도 수정)

이 3-조건 게이트는 "언제 회전/도착하나"만 정합니다. 앞서 합의한 **두 가지는 이 패치에 안 들어있고 따로 필요**합니다:

후처리 **kappa 특이점 가드 + context 클램프** — 게이트가 회전을 켜든, 루프를 주행하든 종단 `target_v→0`에서 1.8 스파이크는 그대로 납니다. 후처리 블록 수정은 별개입니다.

**`max_robot_pose_search_dist` 5.0→1.5~2.0** (nav2_params) — 교차 snap으로 `transformed_plan`이 잘리는 원인은 게이트가 못 막습니다. PathHandler 레벨이라 파라미터로 따로.

이 둘은 원하시면 이어서 정리하겠습니다(이번 요청은 3-조건 게이트로 한정해서 위만 적용).

## 적용 전 확인 2가지

**메시지 타입** — `PublishRemainingGoalsAction`의 publisher 선언 한 줄. Path면 위 코드 그대로, PoseArray/Goals면 타입 + `.poses.size()`(→`.goals.size()`)만 변경.

**2.8 동기화** — 컨트롤러·checker 두 곳의 `PATH_DIST_MAX`가 같은 값이어야 동작이 일관됩니다. 나중에 따로 갈리지 않게, 가능하면 둘 다 nav2_params의 동일 파라미터로 빼는 걸 권합니다(지금은 최소 변경 위해 상수로 둠).

타입만 알려주시면 이 패치는 그대로 빌드됩니다. 빌드 후 멀티 goal 주행에서 (a) 중간 goal 조기 종료가 사라지는지, (b) 최종 goal에서 count가 0으로 떨어져도 정상 도착하는지 두 가지를 확인해 보세요 — count==0 race를 `rgc<=1`로 막은 게 실제로 듣는지 검증 포인트입니다.

네, Nav2 전문가로서 이 요구사항을 처리하는 가장 좋은 방법을 제안해 드립니다.

요청하신 기능("주행 시작 후 3초간 0.3m/s 제한, 각속도 비율 유지")을 구현하려면, 로봇의 \*\*상태 전이(state transition)\*\*를 감지하고 **타임스탬프**를 기록하는 로직이 필요합니다.

가장 좋은 방법은 `/robot_status` 토픽을 구독하는 콜백(현재 `recoveryModeCallback`)을 확장하여, 로봇이 `IDLE` 같은 비주행 상태에서 `DRIVING` 또는 `FOLLOWING_WAYPOINTS` 같은 주행 상태로 **전환되는 순간**의 시간을 기록하는 것입니다.

그런 다음, `cmdVelCallback`에서 현재 시간과 이 시작 시간의 차이를 계산하여 3초가 지나지 않았는지 확인하고, 3초 이내라면 0.3m/s 속도 제한 로직을 적용합니다.

아래와 같이 코드를 수정하는 것을 권장합니다. 수정이 필요한 부분의 **완벽한 코드**를 제공합니다.

-----

## 1\. `velocity_modifier_node.hpp` 수정

새로운 상태 변수(마지막 상태, 주행 시작 시간)와 기능 관련 파라미터 변수를 헤더 파일에 추가하고, 콜백 함수 이름을 더 명확하게 변경합니다.

```cpp
#ifndef VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
#define VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_interfaces/msg/modifier_control.hpp>

#include <mutex>
#include <limits>
#include <memory>
#include <cmath>

namespace velocity_modifier
{

class VelocityModifierNode : public rclcpp::Node
{
public:
  using ModifierControl = robot_interfaces::msg::ModifierControl;
  using String = std_msgs::msg::String;

  explicit VelocityModifierNode(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlCallback(const ModifierControl::SharedPtr msg);
  // 콜백 이름 변경: recovery_mode 뿐만 아니라 전반적인 로봇 상태를 처리
  void robotStatusCallback(const String::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr cb_group_cmd_vel_;
  rclcpp::CallbackGroup::SharedPtr cb_group_control_;
  // 콜백 그룹 이름 변경
  rclcpp::CallbackGroup::SharedPtr cb_group_status_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ModifierControl>::SharedPtr control_sub_;
  // 구독 이름 변경
  rclcpp::Subscription<String>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adjusted_cmd_vel_pub_;

  // 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;

  // (기존 파라미터 변수들...)
  double speed_limit_linear_;
  double speed_limit_angular_;
  double speed_scale_;
  double min_abs_linear_vel_;
  double min_abs_angular_vel_;
  double ratio_scaling_max_linear_vel_;
  double ratio_scaling_max_angular_vel_;

  // 상태 변수
  bool recovery_mode_ = false;
  
  // === [새로 추가된 변수] ===
  // 마지막 로봇 상태 저장을 위한 변수
  std::string last_robot_status_;
  // 주행 시작 시간을 저장하기 위한 변수
  rclcpp::Time driving_start_time_;

  // 초기 속도 제한 기능 파라미터
  bool initial_speed_limit_enabled_;
  double initial_speed_limit_linear_;
  double initial_speed_limit_duration_;
  // === [여기까지 추가] ===
};

}  // namespace velocity_modifier

#endif  // VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
```

-----

## 2\. `velocity_modifier_node.cpp` 수정

### 2.1. 생성자 (`VelocityModifierNode::VelocityModifierNode`)

새로운 파라미터를 선언 및 초기화하고, `driving_start_time_`을 초기화합니다. 또한 구독 콜백과 그룹 이름을 변경합니다.

```cpp
VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options),
  speed_limit_linear_(std::numeric_limits<double>::max()),
  speed_limit_angular_(std::numeric_limits<double>::max()),
  speed_scale_(1.0),
  last_robot_status_("") // last_robot_status_ 초기화
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  // (기존 파라미터 선언...)
  this->declare_parameter<double>("min_abs_linear_vel", 0.03);
  this->declare_parameter<double>("min_abs_angular_vel", 0.03);
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.30);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.20);
  
  // === [새로 추가된 파라미터 선언] ===
  this->declare_parameter<bool>("initial_speed_limit.enabled", true);
  this->declare_parameter<double>("initial_speed_limit.linear", 0.3);
  this->declare_parameter<double>("initial_speed_limit.duration", 3.0);
  // === [여기까지 추가] ===

  // (기존 파라미터 GET...)
  this->get_parameter("min_abs_linear_vel", min_abs_linear_vel_);
  this->get_parameter("min_abs_angular_vel", min_abs_angular_vel_);
  this->get_parameter("ratio_scaling_max_linear_vel", ratio_scaling_max_linear_vel_);
  this->get_parameter("ratio_scaling_max_angular_vel", ratio_scaling_max_angular_vel_);

  // === [새로 추가된 파라미터 GET] ===
  this->get_parameter("initial_speed_limit.enabled", initial_speed_limit_enabled_);
  this->get_parameter("initial_speed_limit.linear", initial_speed_limit_linear_);
  this->get_parameter("initial_speed_limit.duration", initial_speed_limit_duration_);
  // === [여기까지 추가] ===

  // driving_start_time_ 초기화 (0초로 설정)
  driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  cb_group_cmd_vel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_control_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 콜백 그룹 이름 변경
  cb_group_status_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  adjusted_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // (기존 cmd_vel_sub_, control_sub_ 생성 코드...)

  // /robot_status 토픽 구독 로직 (변수 및 콜백 이름 변경)
  auto sub_status_opt = rclcpp::SubscriptionOptions();
  sub_status_opt.callback_group = cb_group_status_;
  robot_status_sub_ = this->create_subscription<String>(
    "/robot_status", 10,
    // 콜백 함수 이름 변경
    std::bind(&VelocityModifierNode::robotStatusCallback, this, std::placeholders::_1),
    sub_status_opt);

  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}
```

### 2.2. `robotStatusCallback` 함수 (기존 `recoveryModeCallback`)

함수 이름을 변경하고, **주행 시작을 감지**하는 로직을 추가합니다.

```cpp
// 함수 이름 변경
void VelocityModifierNode::robotStatusCallback(const String::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  const std::string new_status = msg->data;

  // 1. Recovery Mode 로직 (기존과 동일)
  if (new_status == "RECOVERY_RUNNING") {
    if (!recovery_mode_) {
      recovery_mode_ = true;
      RCLCPP_INFO(this->get_logger(), "Recovery mode ENABLED. Low-speed correction is active.");
    }
  } else if (new_status == "RECOVERY_SUCCESS" || new_status == "RECOVERY_FAILURE") {
    if (recovery_mode_) {
      recovery_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery mode DISABLED. Low-speed correction is inactive.");
    }
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "Received robot status: '%s'", new_status.c_str());
  }

  // === [새로 추가된 로직: 주행 시작 감지] ===
  // 주행 상태 정의 (RobotStatus enum 참고)
  bool is_driving = (new_status == "DRIVING" || new_status == "FOLLOWING_WAYPOINTS");
  // 이전 상태가 주행 상태였는지 정의
  bool was_driving = (last_robot_status_ == "DRIVING" || last_robot_status_ == "FOLLOWING_WAYPOINTS");

  // "주행 중" 상태가 아니었다가 "주행 중" 상태로 변경된 순간을 감지
  if (is_driving && !was_driving) {
    driving_start_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), 
      "Robot started driving. Applying initial speed limit (%.1fm/s for %.1fs).",
      initial_speed_limit_linear_, initial_speed_limit_duration_);
  }
  // 주행 상태가 아니게 되면 타이머 리셋
  else if (!is_driving) {
    driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }
  // === [여기까지 추가] ===

  // 마지막 상태 업데이트
  last_robot_status_ = new_status;
}
```

### 2.3. `cmdVelCallback` 함수

기존의 1, 2번 로직(스케일, 일반 제한)과 3번 로직(저속 보정) 사이에, **새로운 초기 속도 제한 로직**을 삽입합니다.

```cpp
void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto adjusted_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);

  // 1. apply speed scale
  adjusted_vel->linear.x *= speed_scale_;
  adjusted_vel->angular.z *= speed_scale_;

  // 2. speed limit
  adjusted_vel->linear.x = std::clamp(
    adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
  adjusted_vel->angular.z = std::clamp(
    adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);

  // === [새로 추가된 로직: 3. 초기 속도 제한 적용] ===
  if (initial_speed_limit_enabled_ && driving_start_time_.seconds() > 0.0) {
    double time_since_driving_start = (this->get_clock()->now() - driving_start_time_).seconds();

    // 주행 시작 후 설정된 시간(duration) 이내인지 확인
    if (time_since_driving_start >= 0.0 && time_since_driving_start < initial_speed_limit_duration_) {
      
      const double abs_vx = std::abs(adjusted_vel->linear.x);

      // 현재 선속도가 초기 제한 속도(0.3 m/s)를 초과하는지 확인
      if (abs_vx > initial_speed_limit_linear_) {
        // 속도를 줄여야 할 비율 계산
        double ratio = initial_speed_limit_linear_ / abs_vx;
        
        RCLCPP_DEBUG(this->get_logger(), 
          "Initial speed limit active (%.1f/%.1f s). Scaling velocity by %.3f.",
          time_since_driving_start, initial_speed_limit_duration_, ratio);

        // 선속도와 각속도 모두에 동일한 비율을 적용하여 속도 제한
        adjusted_vel->linear.x *= ratio;
        adjusted_vel->angular.z *= ratio;
      }
    }
  }
  // === [여기까지 추가] ===


  // 4. minimum speed threshold (기존 3번 로직)
  RCLCPP_INFO(this->get_logger(), "1▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->linear.z);

    
   // 4. 저속 보정 로직 ( numerically stable version )
  if (recovery_mode_) {
    // (기존 저속 보정 로직은 변경 없음...)
    const double vx = adjusted_vel->linear.x;
    const double wz = adjusted_vel->angular.z;
    const double abs_vx = std::abs(vx);
    const double abs_wz = std::abs(wz);
    const double epsilon = 1e-9;

    bool is_nonzero_and_too_slow = (abs_vx > epsilon || abs_wz > epsilon) &&
                                  ((abs_vx > epsilon && abs_vx < min_abs_linear_vel_) ||
                                    (abs_wz > epsilon && abs_wz < min_abs_angular_vel_));

    if (is_nonzero_and_too_slow) {
      RCLCPP_DEBUG(this->get_logger(), "Command is too slow, applying unified scaling.");
      
      double s_linear = 1.0;
      if (abs_vx > epsilon) { s_linear = min_abs_linear_vel_ / abs_vx; }
      double s_angular = 1.0;
      if (abs_wz > epsilon) { s_angular = min_abs_angular_vel_ / abs_wz; }

      double scale = std::max(s_linear, s_angular);
      double new_vx = vx * scale;
      double new_wz = wz * scale;
      RCLCPP_INFO(this->get_logger(), " scale: %.5lf, new_vx: %.5lf, new_wz: %.5lf", scale, new_vx, new_wz);

      adjusted_vel->linear.x = std::clamp(
        new_vx, -ratio_scaling_max_linear_vel_, ratio_scaling_max_linear_vel_);
      adjusted_vel->angular.z = std::clamp(
        new_wz, -ratio_scaling_max_angular_vel_, ratio_scaling_max_angular_vel_);
    }
  } // End of if (recovery_mode_)
  
  RCLCPP_INFO(this->get_logger(), "2▶ vx: %.11lf, wz: %.11lf", adjusted_vel->linear.x, adjusted_vel->angular.z);

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}
```
#####################  
######################  
######################  
네, 코드를 검토해 본 결과, **몇 가지 심각한 버그**와 의도하신 대로 동작하지 않을 수 있는 **치명적인 로직적 문제**들이 있습니다.

특히 \*\*컴파일 오류, 메모리 손상, 스레드 충돌(Data Race)\*\*을 일으킬 수 있는 부분들이 포함되어 있습니다.

-----

## 1\. 🚨 가장 심각한 버그 (컴파일 오류 / 스레드 충돌)

### 가. `auto adjusted_vel_;` (헤더 파일)

  * **문제**: C++ 클래스 멤버 변수는 `auto` 키워드로 선언할 수 없습니다. 이 코드는 컴파일되지 않습니다.
  * **분석**: `cmdVelCallback`의 로컬 변수인 `adjusted_vel`을 다른 콜백(`controlCallback`)에서 사용하기 위해 멤버 변수로 선언하려 하신 것 같습니다. 이는 매우 위험한 접근 방식입니다.

### 나. `controlCallback`에서의 `adjusted_vel_` 접근 (치명적)

  * **문제**: `controlCallback`의 `case ModifierControl::TYPE_SPEED_LIMIT_SCALE:` 내부에서 `adjusted_vel_`에 접근하는 모든 코드(`current_speed_limit_scale_linear_ = adjusted_vel_->linear.x;`)는 심각한 버그입니다.
  * **이유**:
    1.  **데이터 경합 (Data Race)**: `cmdVelCallback`와 `controlCallback`은 **서로 다른 Reentrant 콜백 그룹**에서 실행됩니다. 즉, 두 함수는 **동시에 다른 스레드에서 실행**될 수 있습니다. `data_mutex_`는 `adjusted_vel_` 멤버를 보호하지 않으므로, 한 스레드가 `adjusted_vel_`에 쓰고 다른 스레드가 읽으려 하면 100% 스레드 충돌이 발생합니다.
    2.  **댕글링 포인터 (Dangling Pointer)**: `cmdVelCallback`의 `adjusted_vel`은 `std::make_unique`로 생성된 로컬 `unique_ptr`입니다. `cmdVelCallback` 함수가 종료되면 이 포인터는 **즉시 메모리에서 해제됩니다.** `controlCallback`이 이 해제된 메모리 주소(`adjusted_vel_`)에 접근하면 프로그램은 \*\*즉시 비정상 종료(Crash)\*\*합니다.
    3.  **잘못된 할당**: `adjusted_vel_ = adjusted_vel;` 이 라인은 `std::unique_ptr`를 복사하려 시도하므로 컴파일 오류입니다. (만약 `adjusted_vel_ = adjusted_vel.get()`으로 하셨더라도 위 1, 2번 문제로 인해 여전히 치명적입니다.)

-----

## 2\. 📝 로직 및 설계 문제

### 가. `controlCallback`의 잘못된 로직

  * **문제**: `controlCallback`은 `/velocity_modifier/control` 토픽이 수신될 때만 실행됩니다. `cmdVelCallback`은 `/cmd_vel_adjusted` 토픽이 수신될 때 실행됩니다.
  * **분석**: `controlCallback`이 실행되는 시점에 로봇의 "현재 속도"(`adjusted_vel_`)를 가져오려는 설계 자체가 잘못되었습니다. `controlCallback`이 실행될 때 `cmdVelCallback`은 아예 실행 중이 아닐 수도 있습니다. "속도 제한"과 같은 *규칙*을 설정하는 콜백이, 특정 시점의 *데이터*에 의존해서는 안 됩니다.

### 나. `flag_speed_limit_scale_` (One-Shot 로직)

  * **문제**: `flag_speed_limit_scale_ = true;`로 플래그를 설정하고, `cmdVelCallback`에서 이 플래그를 확인하여 **단 한 번** 속도를 덮어쓴 뒤 `flag_speed_limit_scale_ = false;`로 리셋합니다.
  * **분석**: 이 로직은 "지속적인 속도 제한"이나 "지속적인 스케일링"이 아닙니다. 이것은 "다음에 들어오는 `cmd_vel` 메시지를 무시하고, `controlCallback`에서 계산한 값으로 **단 한 번** 강제 발행"하는 기능입니다.
  * **의도 확인**: 만약 이것이 의도한 기능이라면 위 1번 버그만 수정하면 되지만, `TYPE_SPEED_LIMIT_SCALE`라는 이름으로 볼 때, 아마도 \*\*"비율을 유지하는 지속적인 상한선"\*\*을 의도하신 것 같습니다.

### 다. `controlCallback`의 불필요한 계산

  * **문제**: `TYPE_SPEED_LIMIT_SCALE` 케이스에서 `msg->linear_value`와 `msg->angular_value`를 받아 변수에 저장한 뒤, `if/else if` 문을 통해 \*\*"현재 속도"\*\*를 기준으로 이 값들을 **다시 덮어쓰고 있습니다.**
  * **분석**: 위 1, 2번 문제로 인해 이 로직은 동작하지 않으며, 설령 동작하더라도 매우 혼란스러운 로직입니다. "속도 제한" 메시지는 "새로운 제한 값은 이것이다"라고 명시적으로 값을 전달해야 합니다.

-----

## 3\. 💡 버그 수정 및 올바른 로직 제안

아마도 의도하신 기능은 **"새로운 선속/각속 상한선을 설정하되, 이 상한선을 초과할 경우 비율을 유지하며 속도를 낮추는"** 기능으로 보입니다.

이 기능을 올바르게 구현하려면 **`controlCallback`에서는 규칙(제한값)만 저장**하고, **`cmdVelCallback`에서 모든 계산을 수행**해야 합니다.

### 제안 1: 헤더 파일 (`.hpp`) 수정

`auto adjusted_vel_`와 `flag_speed_limit_scale_`를 제거하고, 이 모드가 활성화되었는지 알려주는 `bool` 플래그를 추가합니다.

```cpp
// ...
  // 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;

  // 모드별 상태 변수
  double speed_limit_linear_;
  double speed_limit_angular_;
  double speed_scale_;

  // 새로운 모드를 위한 변수
  double ratio_limit_linear_;
  double ratio_limit_angular_;

  // 어떤 모드가 활성화되었는지 나타내는 Enum
  enum class SpeedMode {
    STANDARD_LIMIT,
    STANDARD_SCALE,
    RATIO_LIMIT_SCALE
  };
  SpeedMode current_mode_ = SpeedMode::STANDARD_LIMIT;

  // (min_abs_linear_vel_ 등 다른 변수들...)
  
  bool recovery_mode_ = false;

  // auto adjusted_vel_;  <-- [제거]
  // bool flag_speed_limit_scale_ = false; <-- [제거]
};
```

### 제안 2: `controlCallback` (`.cpp`) 수정

각 `case`가 자신의 모드를 활성화하고 다른 모드의 설정을 초기화하도록 변경합니다. **`adjusted_vel_` 관련 코드를 모두 제거합니다.**

```cpp
void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);

  switch (msg->command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      current_mode_ = SpeedMode::STANDARD_LIMIT;
      speed_limit_linear_ = msg->linear_value;
      speed_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0;
      RCLCPP_INFO(
        this->get_logger(), "Set Mode: STANDARD_LIMIT. Linear: %.2f, Angular: %.2f",
        speed_limit_linear_, speed_limit_angular_);
      break;

    case ModifierControl::TYPE_SPEED_SCALE:
      current_mode_ = SpeedMode::STANDARD_SCALE;
      speed_scale_ = msg->linear_value;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      RCLCPP_INFO(this->get_logger(), "Set Mode: STANDARD_SCALE. Scale: %.2f", speed_scale_);
      break;

    case ModifierControl::TYPE_SPEED_LIMIT_SCALE:
      current_mode_ = SpeedMode::RATIO_LIMIT_SCALE;
      ratio_limit_linear_ = msg->linear_value;
      ratio_limit_angular_ = msg->angular_value;
      // 다른 모드 설정 초기화
      speed_scale_ = 1.0; 
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();

      RCLCPP_INFO(
        this->get_logger(), "Set Mode: RATIO_LIMIT_SCALE. Linear: %.2f, Angular: %.2f",
        ratio_limit_linear_, ratio_limit_angular_);
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(), "Received control command with unknown type: %d", msg->command_type);
      break;
  }
}
```

### 제안 3: `cmdVelCallback` (`.cpp`) 수정

`current_mode_`에 따라 다른 속도 제한 로직을 적용합니다.

```cpp
void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto adjusted_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);
  // adjusted_vel_ = adjusted_vel;  <-- [제거]

  // [수정된 로직 시작]
  if (current_mode_ == SpeedMode::STANDARD_LIMIT) {
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  } 
  else if (current_mode_ == SpeedMode::STANDARD_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;
    // (limit은 max일 것)
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  }
  else if (current_mode_ == SpeedMode::RATIO_LIMIT_SCALE) {
    // 1.0 스케일 적용 (다른 모드와 일관성)
    adjusted_vel->linear.x *= speed_scale_; // (speed_scale_은 1.0일 것)
    adjusted_vel->angular.z *= speed_scale_;

    double abs_vx = std::abs(adjusted_vel->linear.x);
    double abs_wz = std::abs(adjusted_vel->angular.z);
    double scale = 1.0;
    
    // 비율 계산 (0으로 나누기 방지)
    double linear_ratio = 1.0;
    if (abs_vx > 1e-6) {
      linear_ratio = ratio_limit_linear_ / abs_vx;
    }
    double angular_ratio = 1.0;
    if (abs_wz > 1e-6) {
      angular_ratio = ratio_limit_angular_ / abs_wz;
    }

    // 두 제한을 모두 만족해야 하므로, 더 작은 비율(더 많이 줄여야 하는)을 선택
    if (abs_vx > ratio_limit_linear_ || abs_wz > ratio_limit_angular_) {
      scale = std::min(linear_ratio, angular_ratio);
    }
    
    // 1.0보다 큰 값으로 스케일링되지 않도록 (즉, 속도를 증가시키지 않도록)
    scale = std::min(scale, 1.0); 

    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }
  // [수정된 로직 끝]


  // 3. 저속 보정 로직 (이 로직은 모든 모드에 공통으로 적용됨)
  if (recovery_mode_) {
    // ... (기존의 안정적인 저속 보정 로C logic) ...
  } 

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}
```

**요약**:

1.  **버그 수정**: `auto adjusted_vel_` 멤버를 제거하여 컴파일 오류, 메모리 손상, 스레드 충돌을 모두 해결합니다.
2.  **로직 수정**: "One-shot 플래그" 대신 `SpeedMode`라는 상태(enum)를 도입하여, `controlCallback`이 모드를 설정하고 `cmdVelCallback`이 해당 모드의 로직을 수행하도록 책임을 명확히 분리합니다.
3.  **새 기능 구현**: `RATIO_LIMIT_SCALE` 모드는 `cmdVelCallback` 내에서 비율을 계산하여 적용하도록 수정합니다.

새로운 `TYPE_SPEED_LIMIT_SCALE`의 정확한 의도가 제가 추측한 것(비율 유지 상한선)과 다른지 확인해 주시면, 그에 맞게 코드를 다시 조정해 드릴 수 있습니다.
#############################  
#################################  
##################################  
네, 요청하신 대로 **로봇 상태가 `driving`으로 변하는 시점(Edge Detection)**을 감지하여 1구간(0~1초), 2구간(1~2.5초)에 걸쳐 비율을 유지하며 속도를 제한하는 기능을 추가하겠습니다.

기존 코드를 최대한 보존하면서, `Time-based Clamping` 기능을 심플하고 강건하게 구현했습니다.

### 핵심 변경 사항

1. **상태 감지**: `/robot_status` 콜백에서 이전 상태와 현재 상태를 비교하여 `driving`이 시작된 순간(`driving_start_time_`)을 기록합니다.
2. **시간 기반 제한**: `cmdVelCallback`에서 현재 시간과 시작 시간의 차이(`elapsed`)를 계산하여 구간별 제한 속도(`current_limit`)를 설정합니다.
3. **비율 유지 클램핑**: 선속도가 제한을 초과할 경우, `scale = limit / abs(vx)`를 계산하여 선속도와 각속도에 동일하게 곱해줍니다.

---

### 1. 헤더 파일 (`velocity_modifier_node.hpp`)

시간 기록을 위한 변수와 파라미터를 추가했습니다.

```cpp
#ifndef VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
#define VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_interfaces/msg/modifier_control.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <limits>
#include <memory>
#include <cmath>
#include <string> // string 추가

namespace velocity_modifier
{

class VelocityModifierNode : public rclcpp::Node
{
public:
  using ModifierControl = robot_interfaces::msg::ModifierControl;
  using String = std_msgs::msg::String;

  explicit VelocityModifierNode(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlCallback(const ModifierControl::SharedPtr msg);
  void recoveryModeCallback(const String::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr cb_group_cmd_vel_;
  rclcpp::CallbackGroup::SharedPtr cb_group_control_;
  rclcpp::CallbackGroup::SharedPtr cb_group_recovery_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ModifierControl>::SharedPtr control_sub_;
  rclcpp::Subscription<String>::SharedPtr recovery_mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adjusted_cmd_vel_pub_;

  // 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;

  // 일반 멤버 변수
  double speed_limit_linear_ = std::numeric_limits<double>::max();
  double speed_limit_angular_ = std::numeric_limits<double>::max();
  double speed_scale_ = 1.0;

  // 새로운 모드를 위한 변수
  double ratio_limit_linear_ = std::numeric_limits<double>::max();
  double ratio_limit_angular_ = std::numeric_limits<double>::max();

  // 어떤 모드가 활성화되었는지 나타내는 Enum
  enum class SpeedMode {
    STANDARD_LIMIT,
    STANDARD_SCALE,
    RATIO_LIMIT_SCALE
  };
  SpeedMode current_mode_ = SpeedMode::STANDARD_LIMIT;  

  double min_abs_linear_vel_ = 0.05;
  double min_abs_angular_vel_ = 0.05;
  
  // 비율 보정 시 적용될 상한선 
  double ratio_scaling_max_linear_vel_ = 0.35;
  double ratio_scaling_max_angular_vel_ = 0.25;
  
  bool recovery_mode_ = false;

  // === [Added] 주행 시작 속도 제한을 위한 변수 ===
  rclcpp::Time driving_start_time_;      // 주행 시작 시각 저장
  std::string last_robot_status_ = "";   // 상태 변화 감지용
  double startup_phase2_limit_ = 0.3;    // 1초 ~ 2.5초 사이의 속도 제한 값 (파라미터화)

};

}  // namespace velocity_modifier

#endif  // VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_

```

---

### 2. 소스 파일 (`velocity_modifier_node.cpp`)

`cmdVelCallback` 상단에 시간 체크 로직을 추가하여 가장 먼저 제한이 걸리도록 배치했습니다.

```cpp
#include "velocity_modifier/velocity_modifier_node.hpp"

#include <algorithm> // For std::clamp

namespace velocity_modifier
{

VelocityModifierNode::VelocityModifierNode(const rclcpp::NodeOptions & options)
: Node("velocity_modifier_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Velocity Modifier Node is initializing...");

  // 기존 파라미터들
  this->declare_parameter<double>("min_abs_linear_vel", 0.05);
  this->declare_parameter<double>("min_abs_angular_vel", 0.05);
  this->declare_parameter<double>("ratio_scaling_max_linear_vel", 0.35);
  this->declare_parameter<double>("ratio_scaling_max_angular_vel", 0.25);

  // [Added] Phase 2 (1.0s ~ 2.5s) 속도 제한 파라미터 (기본값 0.3)
  this->declare_parameter<double>("startup_phase2_limit", 0.3);

  this->get_parameter("min_abs_linear_vel", min_abs_linear_vel_);
  this->get_parameter("min_abs_angular_vel", min_abs_angular_vel_);
  this->get_parameter("ratio_scaling_max_linear_vel", ratio_scaling_max_linear_vel_);
  this->get_parameter("ratio_scaling_max_angular_vel", ratio_scaling_max_angular_vel_);
  
  // [Added] 파라미터 읽기
  this->get_parameter("startup_phase2_limit", startup_phase2_limit_);
  
  // [Added] 시간 초기화 (0초로 설정)
  driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  RCLCPP_INFO(this->get_logger(), "Startup Phase2 Limit: %.3f m/s", startup_phase2_limit_);

  // (이하 기존 코드와 동일)
  cb_group_cmd_vel_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_control_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_group_recovery_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  adjusted_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  auto sub_cmd_vel_opt = rclcpp::SubscriptionOptions();
  sub_cmd_vel_opt.callback_group = cb_group_cmd_vel_;
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_adjusted", 10,
    std::bind(&VelocityModifierNode::cmdVelCallback, this, std::placeholders::_1),
    sub_cmd_vel_opt);

  auto sub_control_opt = rclcpp::SubscriptionOptions();
  sub_control_opt.callback_group = cb_group_control_;
  rclcpp::QoS qos_control(10);
  qos_control.transient_local();
  control_sub_ = this->create_subscription<ModifierControl>(
    "velocity_modifier/control", qos_control,
    std::bind(&VelocityModifierNode::controlCallback, this, std::placeholders::_1),
    sub_control_opt);
  
  auto sub_recovery_opt = rclcpp::SubscriptionOptions();
  sub_recovery_opt.callback_group = cb_group_recovery_;

  recovery_mode_sub_ = this->create_subscription<String>(
    "/robot_status", 10,
    std::bind(&VelocityModifierNode::recoveryModeCallback, this, std::placeholders::_1),
    sub_recovery_opt);

  RCLCPP_INFO(this->get_logger(), "Node has been started successfully.");
}

void VelocityModifierNode::recoveryModeCallback(const String::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  std::string current_status = msg->data;

  // [Added] 주행 시작 감지 로직 (Edge Detection)
  // 이전 상태는 driving이 아니었는데, 현재 driving이 된 순간을 포착
  if (current_status == "driving" && last_robot_status_ != "driving") {
    driving_start_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Robot started DRIVING. Engaging startup speed limits.");
  }
  // 상태가 driving이 아니게 되면 타이머 리셋 (0으로 설정하여 로직 비활성화)
  else if (current_status != "driving") {
    driving_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  // 상태 업데이트
  last_robot_status_ = current_status;

  // 기존 Recovery Logic 유지
  if (current_status == "recovery_start") {
    if (!recovery_mode_) {
      recovery_mode_ = true;
      RCLCPP_INFO(this->get_logger(), "Recovery mode ENABLED.");
    }
  } else if (current_status == "recovery_finish") {
    if (recovery_mode_) {
      recovery_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "Recovery mode DISABLED.");
    }
  } else {
    // driving 등 다른 상태일 때는 recovery_mode_를 false로 두는 것이 안전할 수 있음
    // 하지만 기존 로직 유지를 위해 명시적 finish가 아닐 경우의 처리는 기존 코드 존중
    // (기존 코드의 else { recovery_mode_ = false; } 부분 유지)
    if (current_status != "recovery_start" && current_status != "driving") { 
        recovery_mode_ = false; 
    }
  }
}

void VelocityModifierNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto adjusted_vel = std::make_unique<geometry_msgs::msg::Twist>(*msg);

  // === [Added] 1. 주행 초기 속도 제한 (Startup Speed Limit) ===
  // driving_start_time_이 0이 아니라는 것은 현재 driving 상태라는 의미
  if (driving_start_time_.nanoseconds() > 0) {
    double elapsed_sec = (this->get_clock()->now() - driving_start_time_).seconds();
    double startup_limit = -1.0; // 음수는 제한 없음을 의미

    // Phase 1: 0.0s ~ 1.0s -> 0.1 m/s 제한
    if (elapsed_sec >= 0.0 && elapsed_sec < 1.0) {
      startup_limit = 0.1;
    }
    // Phase 2: 1.0s ~ 2.5s -> 설정된 속도(예: 0.3) 제한
    else if (elapsed_sec >= 1.0 && elapsed_sec < 2.5) {
      startup_limit = startup_phase2_limit_;
    }

    // 제한 값이 설정되었고, 현재 선속도가 그보다 크다면 Clamping 수행
    if (startup_limit > 0.0) {
      double abs_vx = std::abs(adjusted_vel->linear.x);
      if (abs_vx > startup_limit) {
        // 비율 유지 Clamping: scale = limit / current
        double scale = startup_limit / abs_vx;
        
        adjusted_vel->linear.x *= scale;
        adjusted_vel->angular.z *= scale; // 각속도도 동일 비율로 줄임

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "Startup Limit Active (t=%.2fs): Limit=%.2f, OrigVx=%.2f -> NewVx=%.2f", 
          elapsed_sec, startup_limit, abs_vx, adjusted_vel->linear.x);
      }
    }
  }
  // ==========================================================

  // [기존 로직: 2. 모드별 처리]
  // 주행 초기 제한이 걸렸더라도, 사용자가 설정한 Global Limit(STANDARD_LIMIT 등)이
  // 더 작다면 그 값으로 덮어씌워지므로 안전함 (std::clamp 사용 덕분)
  if (current_mode_ == SpeedMode::STANDARD_LIMIT) {
    adjusted_vel->linear.x *= speed_scale_; 
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  } 
  else if (current_mode_ == SpeedMode::STANDARD_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;
    adjusted_vel->linear.x = std::clamp(
      adjusted_vel->linear.x, -speed_limit_linear_, speed_limit_linear_);
    adjusted_vel->angular.z = std::clamp(
      adjusted_vel->angular.z, -speed_limit_angular_, speed_limit_angular_);
  }
  else if (current_mode_ == SpeedMode::RATIO_LIMIT_SCALE) {
    adjusted_vel->linear.x *= speed_scale_;
    adjusted_vel->angular.z *= speed_scale_;

    double abs_vx = std::abs(adjusted_vel->linear.x);
    double abs_wz = std::abs(adjusted_vel->angular.z);
    double scale = 1.0;
    
    double linear_ratio = 1.0;
    if (abs_vx > 1e-6) {
      linear_ratio = ratio_limit_linear_ / abs_vx;
    }
    double angular_ratio = 1.0;
    if (abs_wz > 1e-6) {
      angular_ratio = ratio_limit_angular_ / abs_wz;
    }

    if (abs_vx > ratio_limit_linear_ || abs_wz > ratio_limit_angular_) {
      scale = std::min(linear_ratio, angular_ratio);
    }
    
    scale = std::min(scale, 1.0); 

    adjusted_vel->linear.x *= scale;
    adjusted_vel->angular.z *= scale;
  }

  // [기존 로직: 3. 저속 보정 로직]
  if (recovery_mode_ == true) {
    // ... (기존 저속 보정 코드 내용 그대로 유지) ...
    const double vx = adjusted_vel->linear.x;
    const double wz = adjusted_vel->angular.z;
    const double abs_vx = std::abs(vx);
    const double abs_wz = std::abs(wz);
    const double epsilon = 1e-9;

    bool is_nonzero_and_too_slow = (abs_vx > epsilon || abs_wz > epsilon) &&
                                  ((abs_vx > epsilon && abs_vx < min_abs_linear_vel_) ||
                                    (abs_wz > epsilon && abs_wz < min_abs_angular_vel_));

    if (is_nonzero_and_too_slow) {
      double s_linear = 1.0;
      if (abs_vx > epsilon) {
        s_linear = min_abs_linear_vel_ / abs_vx;
      }

      double s_angular = 1.0;
      if (abs_wz > epsilon) {
        s_angular = min_abs_angular_vel_ / abs_wz;
      }

      double scale = std::max(s_linear, s_angular);
      double new_vx = vx * scale;
      double new_wz = wz * scale;

      adjusted_vel->linear.x = std::clamp(
        new_vx, -ratio_scaling_max_linear_vel_, ratio_scaling_max_linear_vel_);
      adjusted_vel->angular.z = std::clamp(
        new_wz, -ratio_scaling_max_angular_vel_, ratio_scaling_max_angular_vel_);
    }
  } 

  adjusted_cmd_vel_pub_->publish(std::move(adjusted_vel));
}

// (controlCallback은 기존과 동일하므로 생략하거나 그대로 둡니다)
void VelocityModifierNode::controlCallback(const ModifierControl::SharedPtr msg)
{
    const std::lock_guard<std::mutex> lock(data_mutex_);
    // ... (기존 switch case 로직 동일) ...
    // 편의상 생략, 기존 코드 사용
    switch (msg->command_type) {
    case ModifierControl::TYPE_SPEED_LIMIT:
      current_mode_ = SpeedMode::STANDARD_LIMIT;
      speed_limit_linear_ = msg->linear_value;
      speed_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0;
      break;
    case ModifierControl::TYPE_SPEED_SCALE:
      current_mode_ = SpeedMode::STANDARD_SCALE;
      speed_scale_ = msg->linear_value;
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      break;
    case ModifierControl::TYPE_SPEED_LIMIT_SCALE:
      current_mode_ = SpeedMode::RATIO_LIMIT_SCALE;
      ratio_limit_linear_ = msg->linear_value;
      ratio_limit_angular_ = msg->angular_value;
      speed_scale_ = 1.0; 
      speed_limit_linear_ = std::numeric_limits<double>::max();
      speed_limit_angular_ = std::numeric_limits<double>::max();
      break;
    default:
      break;
    }
}

}  // namespace velocity_modifier

```

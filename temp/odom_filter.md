```
#include <cmath>

void Roboteq::odom_publish()
{
  // ... (위치 odom_x, odom_y 계산까지는 동일) ...

  const float ALPHA_V = 0.5f; // 튀는 값이 여전하다면 필터 계수를 0.1 수준으로 낮춰보세요.
  const float ALPHA_W = 0.5f; 
  static float filtered_vx = 0.0f;
  static float filtered_vyaw = 0.0f;

  float vy = 0.0;

  if (dt > 0.001f) // dt 기준을 1ms 이상으로 좀 더 엄격하게 잡습니다.
  {
    float raw_vx = linear / dt; 
    float raw_vyaw = angular / dt; 

    // =========================================================================
    // [추가된 부분] 1단계: 스파이크 제거 (Outlier Rejection / Hard Limit)
    // 로봇이 물리적으로 낼 수 있는 최대 속도를 정의합니다. (현재 로봇 스펙에 맞게 수정)
    // =========================================================================
    const float MAX_SPEED_LIMIT = 1.2f;     // 예: 절대 1.2 m/s 이상 달릴 수 없음
    const float MAX_ANGULAR_LIMIT = 2.0f;  // 예: 절대 초당 반바퀴(pi, 3.14) 이상 회전할 수 없음

    // 현재 계산된 속도가 물리적 한계를 초과하면 통신 오류나 dt 지연으로 간주하고,
    // 이번 원시 데이터(raw)를 버린 뒤 이전 필터값으로 대체합니다.
    if (std::abs(raw_vx) > MAX_SPEED_LIMIT) {
        raw_vx = filtered_vx; 
        // RCLCPP_WARN(this->get_logger(), "Spike detected in linear velocity!"); // 필요시 로그 추가
    }
    if (std::abs(raw_vyaw) > MAX_ANGULAR_LIMIT) {
        raw_vyaw = filtered_vyaw;
    }

    // =========================================================================
    // 2단계: 가속도 한계치 제한 (Slew Rate Limiter) - 옵션 사항
    // 순간적으로 속도가 너무 급격히 변하는 것도 막아줍니다.
    // =========================================================================
    const float MAX_ACCEL = 2.5f; // 예: 최대 가속도 3.0 m/s^2
    const float MAX_ANGULAR_ACCEL = 3.2f;

    float max_delta_v = MAX_ACCEL * dt;
    float max_delta_w = MAX_ANGULAR_ACCEL * dt;
    
    if (raw_vx - filtered_vx > max_delta_v) raw_vx = filtered_vx + max_delta_v;
    else if (raw_vx - filtered_vx < -max_delta_v) raw_vx = filtered_vx - max_delta_v;

    if (raw_vyaw - filtered_vyaw > max_delta_w) raw_vyaw = filtered_vyaw + max_delta_w;
    else if (raw_vyaw - filtered_vyaw < -max_delta_w) raw_vyaw = filtered_vyaw - max_delta_w;

    // =========================================================================
    // 3단계: 기존 EMA 필터 적용
    // =========================================================================
    filtered_vx = (ALPHA_V * raw_vx) + ((1.0f - ALPHA_V) * filtered_vx);
    filtered_vyaw = (ALPHA_W * raw_vyaw) + ((1.0f - ALPHA_W) * filtered_vyaw);
  }

  // 이중 방어벽
  if (std::isnan(filtered_vx)) filtered_vx = 0.0f;
  if (std::isnan(filtered_vyaw)) filtered_vyaw = 0.0f;

  // 4. 메시지 대입
  odom_msg.twist.twist.linear.x = filtered_vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = filtered_vyaw;
  
  // ... (이후 퍼블리시 코드는 동일) ...
}
```

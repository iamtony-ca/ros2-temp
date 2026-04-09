```
#include <cmath>

void Roboteq::odom_publish()
{
  // ... (위치 odom_x, odom_y 계산까지는 동일) ...

  const float ALPHA_V = 0.1f; // 튀는 값이 여전하다면 필터 계수를 0.1 수준으로 낮춰보세요.
  const float ALPHA_W = 0.1f; 
  static float filtered_vx = 0.0f;
  static float filtered_vyaw = 0.0f;

  if (dt > 0.001f) // dt 기준을 1ms 이상으로 좀 더 엄격하게 잡습니다.
  {
    float raw_vx = linear / dt; 
    float raw_vyaw = angular / dt; 

    // =========================================================================
    // [추가된 부분] 1단계: 스파이크 제거 (Outlier Rejection / Hard Limit)
    // 로봇이 물리적으로 낼 수 있는 최대 속도를 정의합니다. (현재 로봇 스펙에 맞게 수정)
    // =========================================================================
    const float MAX_SPEED_LIMIT = 2.0f;     // 예: 절대 2.0 m/s 이상 달릴 수 없음
    const float MAX_ANGULAR_LIMIT = 3.14f;  // 예: 절대 초당 반바퀴(pi) 이상 회전할 수 없음

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
    const float MAX_ACCEL = 3.0f; // 예: 최대 가속도 3.0 m/s^2
    float max_delta_v = MAX_ACCEL * dt;
    
    if (raw_vx - filtered_vx > max_delta_v) raw_vx = filtered_vx + max_delta_v;
    else if (raw_vx - filtered_vx < -max_delta_v) raw_vx = filtered_vx - max_delta_v;

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


```
// =========================================================================
// 3단계: 비대칭 EMA 필터 & Zero-Snap (급정거 반응성 극대화)
// =========================================================================

// 3-1. Zero-Snap (데드밴드): 원시 데이터가 사실상 0이면 필터 무시하고 즉시 0 출력
if (std::abs(raw_vx) < 0.001f) 
{
    filtered_vx = 0.0f;
}
// 3-2. 감속 중 (급정거 포함): 현재 속도보다 원시 속도가 0에 더 가까울 때
else if (std::abs(raw_vx) < std::abs(filtered_vx)) 
{
    // 감속 시에는 반응성을 위해 아주 큰 Alpha 값(예: 0.6 ~ 0.8)을 사용합니다.
    const float ALPHA_DECEL = 0.7f; 
    filtered_vx = (ALPHA_DECEL * raw_vx) + ((1.0f - ALPHA_DECEL) * filtered_vx);
}
// 3-3. 가속 및 정속 주행 중
else 
{
    // 정속 주행 노이즈를 잡기 위해 기존의 부드러운 Alpha 값(예: 0.1 ~ 0.2)을 유지합니다.
    filtered_vx = (ALPHA_V * raw_vx) + ((1.0f - ALPHA_V) * filtered_vx);
}


```

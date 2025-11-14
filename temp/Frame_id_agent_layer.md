`rolling_window`를 `true`로 바꾸면 `ROI outside map bounds` 경고가 사라지는 것이 맞습니다. 그 경고는 \*\*"맵의 경계 밖에 로봇이 있다"\*\*는 뜻이었는데, `rolling_window`는 맵 자체가 로봇을 따라다니므로 로봇이 맵의 경계를 벗어날 일이 없어지기 때문입니다.

하지만 그 결과로 다른 로봇이 보이지 않게 된 것은 **좌표계(Frame) 불일치 문제** 때문이며, 이는 `agent_layer.cpp` 코드의 현재 로직이 `rolling_window`와 호환되지 않기 때문에 발생하는 **심각한 문제**입니다.

-----

## 1\. 롤링 윈도우 문제점 (좌표계 불일치)

`rolling_window`가 꺼져있을 때(False)와 켜져있을 때(True)의 동작 방식은 근본적으로 다릅니다.

  * **`rolling_window: false` (이전 방식)**

      * Costmap의 `global_frame` = **"map"**
      * Costmap의 좌표계는 **고정**되어 있습니다.
      * `/multi_agent_infos` 토픽의 좌표계 = **"map"**
      * **결과:** "map" 좌표를 "map" 그리드에 그리므로 완벽하게 동작합니다.

  * **`rolling_window: true` (현재 방식)**

      * Costmap의 `global_frame` = **"odom"** (일반적으로 Nav2의 기본 설정)
      * Costmap의 좌표계(grid)가 로봇을 따라 **움직입니다.**
      * `/multi_agent_infos` 토픽의 좌표계 = **"map"** (여전히 고정)
      * **문제 발생:** `agent_layer`는 "map" 프레임의 좌표를 가진 로봇 정보를 받아서, "odom" 프레임(움직이는 그리드)에 그리려고 시도합니다. 두 좌표계가 일치하지 않아 로봇이 완전히 엉뚱한 위치(아마도 맵 바깥)에 그려지게 되어 RViz2에서 보이지 않게 됩니다.

-----

## 2\. 문제의 코드 위치 (`agent_layer.cpp`)

문제는 `fillFootprintAt` 함수에서 발생합니다.

```cpp
void AgentLayer::fillFootprintAt(...)
{
    // ...
    // 'pose'는 "/multi_agent_infos"에서 온 "map" 프레임 좌표
    const double yaw = tf2::getYaw(pose.orientation); 
    // ...
    for (auto & p : poly) {
        // ...
        // 'pose.position.x' (map 좌표)를 사용해 월드 좌표 계산
        p.x = pose.position.x + c * x - s * y;
        p.y = pose.position.y + s * x + c * y;
    }
    // ...
    // "map" 좌표(p.x, p.y)를 "odom" 그리드에 그리려고 시도 (실패)
    grid->worldToMapEnforceBounds(minx, miny, min_i, min_j); 
    // ...
}
```

`grid` (Costmap)는 "odom" 좌표계를 기대하는데, `pose.position.x`는 "map" 좌표계이므로 좌표 변환이 완전히 틀어집니다.

-----

## 3\. 해결 방안 (TF 변환 추가)

이 문제를 **올바르게 해결**하려면, `agent_layer` 플러그인이 `obstacle_layer`처럼 TF(Transform)를 인지해야 합니다. 즉, `map` -\> `odom` 변환을 스스로 수행해야 합니다.

`agent_layer.hpp`와 `agent_layer.cpp`에 `tf_buffer_`와 `tf_listener_`를 추가하고, `fillFootprintAt`을 호출하기 전에 수신한 로봇의 `pose`를 `map` 프레임에서 Costmap의 `global_frame`("odom")으로 변환하는 로직이 필요합니다.

이는 코드 변경이 꽤 필요하지만, `rolling_window`를 사용하기 위한 필수적인 수정입니다.

혹시 이 문제를 해결하기 위해 `agent_layer`에 **TF 변환 기능을 추가**하는 구체적인 코드 수정을 원하시나요?

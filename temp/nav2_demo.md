네\! 이제야 완벽하게 이해했습니다. 여러 번 질문하게 해드려 정말 죄송합니다.

**`MapsThroughPoses` 액션 명령어 자체를 2개 만들고, 이 두 명령어를 순서대로 실행하는 전체 사이클을 10회 반복**하고 싶으시다는 말씀이시군요.

네, 당연히 가능합니다. 아래 스크립트가 바로 그 기능을 수행합니다.

-----

### 📝 `MapsThroughPoses` 2개 순차 실행 및 10회 반복 스크립트

이 스크립트는 첫 번째 경유지 세트(`WAYPOINTS_SET_1`)로의 주행이 완전히 끝나면, 이어서 두 번째 경유지 세트(`WAYPOINTS_SET_2`)로의 주행을 시작합니다. 그리고 이 전체 과정을 10번 반복합니다.

```bash
#!/bin/bash

# 총 반복 횟수
ITERATIONS=10

# 명령어 1: 첫 번째 경유지 세트 (예: A 지점 -> B 지점)
WAYPOINTS_SET_1="
{
  poses: [
    { header: {frame_id: 'map'}, pose: { position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0} } },
    { header: {frame_id: 'map'}, pose: { position: {x: 2.0, y: -1.0, z: 0.0}, orientation: {w: 1.0} } }
  ]
}"

# 명령어 2: 두 번째 경유지 세트 (예: C 지점 -> D 지점)
WAYPOINTS_SET_2="
{
  poses: [
    { header: {frame_id: 'map'}, pose: { position: {x: -1.0, y: -1.0, z: 0.0}, orientation: {w: 1.0} } },
    { header: {frame_id: 'map'}, pose: { position: {x: -2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0} } }
  ]
}"


echo "두 개의 'NavigateThroughPoses' 명령을 순차적으로 10회 반복합니다."

# 10회 반복 루프 시작
for i in $(seq 1 $ITERATIONS)
do
  echo ""
  echo "======================================="
  echo "--- [ 사이클 $i / $ITERATIONS ] ---"
  echo "======================================="

  # 1. 첫 번째 'NavigateThroughPoses' 명령어 실행 (완료될 때까지 대기)
  echo "[사이클 $i] 첫 번째 경유지 세트 주행을 시작합니다..."
  ros2 action send_goal --feedback /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "$WAYPOINTS_SET_1"

  # 첫 번째 명령어 성공 여부 확인
  if [ $? -eq 0 ]; then
    echo "✅ [사이클 $i] 첫 번째 경유지 세트 주행 완료."
    echo "---"

    # 2. 두 번째 'NavigateThroughPoses' 명령어 실행 (완료될 때까지 대기)
    echo "[사이클 $i] 두 번째 경유지 세트 주행을 시작합니다..."
    ros2 action send_goal --feedback /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "$WAYPOINTS_SET_2"

    if [ $? -eq 0 ]; then
      echo "✅ [사이클 $i] 두 번째 경유지 세트 주행 완료."
    else
      echo "❌ [사이클 $i] 두 번째 경유지 세트 주행에 실패했습니다!"
    fi
  else
    echo "❌ [사이클 $i] 첫 번째 경유지 세트 주행에 실패했습니다!"
  fi

  echo "사이클 $i 종료. 2초 후 다음 사이클을 시작합니다."
  sleep 2
done

echo ""
echo "🎉 모든 사이클이 완료되었습니다."

```

### 🔑 스크립트 핵심 설명

  * **`WAYPOINTS_SET_1`**: 첫 번째로 실행할 `MapsThroughPoses` 명령어에 들어갈 경유지 목록입니다. **이 부분을 첫 번째 경로로 수정하세요.**
  * **`WAYPOINTS_SET_2`**: 첫 번째 명령이 끝난 후, 이어서 실행할 `MapsThroughPoses` 명령어에 들어갈 경유지 목록입니다. **이 부분을 두 번째 경로로 수정하세요.**
  * **`for i in $(seq 1 $ITERATIONS)`**: 이 `for` 루프가 전체 로직을 10번 반복시켜 줍니다.
  * **순차적 실행**: 루프 안의 첫 번째 `ros2 action send_goal ... "$WAYPOINTS_SET_1"` 명령어가 `--feedback` 플래그 때문에 완전히 종료될 때까지 스크립트는 대기합니다. 성공적으로 끝나면(`if [ $? -eq 0 ]`), 비로소 두 번째 `ros2 action send_goal ... "$WAYPOINTS_SET_2"` 명령어를 실행합니다.

### 💻 실행 방법

1.  위 코드를 `run_sequential_waypoint_sets.sh` 와 같은 이름으로 저장합니다.
2.  터미널에서 **실행 권한**을 부여합니다.
    ```bash
    chmod +x run_sequential_waypoint_sets.sh
    ```
3.  스크립트를 실행합니다.
    ```bash
    ./run_sequential_waypoint_sets.sh
    ```

이제 원하시는 대로, 로봇이 첫 번째 경로 주행을 마친 후 두 번째 경로 주행을 시작하는 전체 과정을 10회 반복 실행할 것입니다. 제 의도 파악이 늦었던 점 다시 한번 사과드립니다.

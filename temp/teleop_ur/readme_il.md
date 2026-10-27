rollout 은 두 가지로 할 수 있습니다. 먼저 A 로 동작을 확인하고, 사람들에게 보여 줄 때는 B 로 Isaac 화면을 띄우는 순서를 권합니다. 모든 터미널에서 아래를 먼저 실행합니다. 그 PC 의 `ROS_DOMAIN_ID` 는 같은 호스트의 다른 컨테이너와 겹치지 않는 0 이 아닌 값이면 됩니다.

```bash
cd /isaac-sim/volume/ur_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42 GRASP_MODE=physical
CK=$PWD/outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model
```

## 0. 빌드 (pull 후 아직 안 했다면 한 번)
`git pull` 로 새 launch, config, 스크립트가 들어왔으니 `install/` 에 반영합니다.
```bash
colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## A. 동작 확인 (화면 없음, 1 trial, 약 2~5분)
```bash
bash src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 1 70 "$CK"
```
- 마지막 줄이 `== 1/1 rollouts succeeded` 면 정상입니다.
- 이 PC 의 첫 실행은 셰이더 컴파일 때문에 3~4분 더 걸립니다. 이렇게 한 번 돌려 두면 B 도 빨리 뜹니다.
- 10 trial 에 영상까지 남기려면 다음처럼 실행합니다(약 15분). 영상은 `outputs/rollout_videos/demo_s131/` 에 저장됩니다.
```bash
SEED=131 RAND= GRASP_LOG=$PWD/outputs/harness_logs/demo_s131.steps.csv RECORD_DIR=$PWD/outputs/rollout_videos/demo_s131 \
  bash src/ur_bringup/scripts/harness/rollout_gui_then_n.sh 10 70 "$CK" > outputs/harness_logs/demo_s131.run.log 2>&1
grep -E '^== [0-9]+/[0-9]+ rollouts' outputs/harness_logs/demo_s131.run.log
```

## B. Isaac 화면을 띄워 보여 주기 (터미널 3개)
A 가 끝났으면 먼저 `bash src/ur_bringup/scripts/harness/shutdown_rollout.sh` 로 정리합니다. 마지막 줄이 `CLEAN` 이어야 합니다. A 의 하네스는 Isaac 을 headless 로 띄우기 때문에, B 와 동시에 쓰면 안 됩니다.

**터미널 1: Isaac (GUI)**
```bash
/isaac-sim/python.sh $PWD/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \
  --asset-path $PWD/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed 131 \
  --camera-res 320x240 --grasp-mode physical --with-camera --with-static-cam \
  2>&1 | tee outputs/harness_logs/gui_isaac.log
```
- 넓은 범위로 보여 주려면 `--seed 211 --randomize-radius-x 0.10 --randomize-radius-y 0.06` 로 바꿉니다.
- 시작 배너의 `grasp mode` 줄이 `physical` 인지 확인합니다.

**터미널 2: 제어 스택과 스트리밍 컨트롤러** (Isaac 창이 뜬 뒤)
```bash
timeout 5 ros2 topic echo --once /clock                         # 값이 나와야 함
bash src/ur_bringup/scripts/harness/bringup.sh                  # 마지막 줄 "== ready" (RViz 도 뜸)
ros2 launch ur_bringup policy_inference.launch.py use_sim:=true  # "Loaded forward_position_controller"
```

**터미널 3: rollout**
```bash
RECORD_DIR=$PWD/outputs/rollout_videos/gui_s131 \
  bash src/ur_bringup/scripts/harness/rollout_n.sh 3 90 "$CK"
```
- 인자는 trial 수, trial 당 초(벽시계), 체크포인트 순서입니다. **체크포인트를 빼면** 그 PC 에 없는 기본 경로를 찾다가 `FAIL: no checkpoint` 로 끝납니다.
- GUI 실행 속도는 아직 재 본 적이 없어서, 여유 있게 90초로 잡았습니다. 시간이 남으면 70 으로 줄여도 됩니다.
- 화면에서는 trial 마다 다음 순서로 보입니다.
  1. 큐브가 새 위치로 옮겨집니다.
  2. 팔이 시작 자세로 갑니다.
  3. policy 가 큐브를 집어 왼쪽 marker 에 놓습니다.
  4. 결과가 `start=[x, y]  SUCCESS d=..mm` 로 찍힙니다.

## 끝나면
```bash
bash src/ur_bringup/scripts/harness/shutdown_rollout.sh    # 마지막 줄 CLEAN
```

**문제가 생기면**
- **팔이 전혀 안 움직이고 전부 FAIL:** 터미널 2 의 `policy_inference.launch.py` 가 빠진 경우입니다.
- **controller 나 `/robot_description` 오류:** `ROS_DOMAIN_ID` 가 터미널마다 다른지 확인합니다.
- **Isaac 이 바로 종료:** 로그에 `Could not find Isaac Sim assets folder` 가 있으면 인터넷 연결 문제입니다.

새 컨테이너에서 GUI 경로를 직접 확인한 적은 아직 없습니다. 시연 전에 B 를 한 번 미리 해 보시고, 막히면 출력을 보여 주세요.

# ACT physical contact grasp 개선 보고 (fixed-joint attach 대비)

Oct 21, 2026 · @tony

## 요약

Isaac Sim 의 UR16e + Robotiq 2F-85 pick-and-place 에서, 큐브를 finger 에 fixed joint 로 붙이던 sim 보조 grasp(attach)를 없애고 finger 마찰만으로 쥐는 physical contact grasp(physical)로 ACT 를 다시 수집·학습했다. 같은 학습량에서는 attach 와 대등했고, 200k step 까지 resume 학습한 checkpoint 는 20 trial 중 19회 성공, 성공 시 평균 placement error 6.2 mm 다.

- **같은 학습량(60k), 같은 시작 위치 10곳(seed 131)**: physical 8/10 · 12.1 mm, attach 9/10 · 13.2 mm. 대등하다.
- **Resume 학습(200k)**: seed 131 에서 9/10 · 5.9 mm, 새 seed 171 에서 10/10 · 6.5 mm. error 가 줄어든 것은 학습량 덕이다. attach 모델은 60k 넘게 학습하지 않았다.
- **현실성**: 이제 grip force 와 slip 이 성공을 가른다. attach checkpoint 를 같은 physical scene 에 넣으면 0/10 이다.
- **Wrist kick 제거**: 원래 gripper 설정을 physical scene 에서 쓰면 scripted demo 10회 중 6회에서 wrist 가 1.0–2.5 rad/s 로 튀었다. 새 soft profile 의 20회는 모두 0.64 rad/s 이하였다. attach scene 에서는 원래 설정도 튀지 않았다.

Success criterion 은 이전 보고와 같다. 70 s 가 끝난 시점에 큐브가 left marker 에서 수평 35 mm 이내에 있고 gripper 가 열려 있으면 성공이며, Isaac 의 ground-truth(GT) object pose 로 자동 채점한다. attach 의 13.2 mm 는 Isaac Sim 6.0.1 에서 학습한 checkpoint 를 이 container(6.1.0)에서 다시 돌린 값이다. 6.0.1 원래 결과는 9/10 · 7.6 mm 였다.

## 전체 파이프라인

사람 없이 한 번에 돈다. demo 100개 수집부터 conversion, 60k training, rollout 10회까지 무인으로 약 2시간 10분 걸렸다(2026-10-20 00:39–02:51). 그 뒤 200k 까지 resume 학습했다.

&#91;embedded content: ACT 물리 접촉 파지 파이프라인 · 6단계, 코드·설정 파일 기준 (2026-10-21)\]

1. **Isaac scene**: table 위에 35 mm 큐브 2개(빨강·파랑)와 marker 2개가 있다. episode 마다 두 큐브의 위치(±60 mm)와 yaw 를 다시 뽑는다. random seed 를 고정해 같은 배치를 다시 만들 수 있다(collection 91, rollout 131·171). camera 는 table 을 내려다보는 exterior camera 와 wrist 의 D405 두 대다.
2. **Scripted expert**: state machine 이 Isaac 의 GT object pose 를 읽어 pick-and-place 를 수행한다. 큰 이동은 MoveIt(기본 planner cuMotion)으로, approach·retreat 는 5 mm 간격의 Cartesian 직선 경로로 움직인다. 실패한 demo 는 저장하지 않는다. physical 데이터는 100/100 성공이었고, demo 자체의 placement error 는 평균 1 mm 다.
3. **Recorder (il\_recorder)**: sim time 기준 30 Hz 마다 joint 7개(arm 6 + gripper)와 camera 2대의 최신 값을 저장한다(zero-order hold). arm action 은 다음 frame 의 joint state(next\_state)이고, gripper action 은 controller command 다. 실물 leader(OMY L100) teleop 도 같은 recorder 를 쓰도록 만들었지만, 아직 fake follower 로만 시험했다.
4. **Conversion (raw\_to\_lerobot)**: ROS 쪽에는 LeRobot·torch 를 넣지 않는다. 그래서 기록은 JPEG + JSON 으로 하고, 분리된 ML venv 에서 LeRobot dataset(state·action 각 7-D, video 2개)으로 바꾼다. episode 100개, 49,862 frame, 1.4 GB 가 202 MB 가 된다.
5. **ACT training**: camera 2대 image 와 현재 joint state 를 보고 앞으로 100 frame(약 3.3 초)의 joint target 을 한 번에 예측한다. RTX 5090 에서 약 15 step/s 로, 60k 까지 1시간 8분, 200k 까지 총 약 3시간 46분이 걸렸다. GPU memory 는 allocated 기준 약 4 GiB 다.
6. **Rollout · scoring**: LeRobot policy server 가 action chunk 를 만들고, 우리 robot adapter 가 joint target 을 ROS controller 로 30 Hz 로 보낸다. chunk 100 step 중 50 step 만 실행하고 다음 chunk 를 받는다. inference 는 CPU 에서 chunk 당 약 60 ms 다. trial 마다 start pose 와 큐브 배치를 reset 하고, wall clock 70 초(sim 약 79 초) 뒤 GT 로 자동 판정한다. recording 을 켜면 trial 마다 video 도 남긴다.

실물에서도 같은 topic 과 controller 를 쓰도록 만들었다. 다만 exterior camera 는 실물에 아직 없고, GT 기반 scoring 은 sim 에서만 된다.

## 아키텍처

모델은 LeRobot 의 ACT 를 그대로 쓰고, 우리가 만든 것은 로봇과 정책을 잇는 ROS 쪽 구성이다.

### ACT model

&#91;embedded content: ACT model architecture · LeRobot 0.6.1, act\_red\_left\_100\_phys\_long 200k config.json\]

카메라 영상 두 장은 같은 ResNet18 을 거쳐 각각 80개 token 이 되고, joint state 1 token·latent 1 token 과 함께 Transformer encoder 에 들어간다. decoder 는 학습된 query 100개로 앞으로 100 frame 의 action 을 한 번에 낸다. CVAE encoder 는 training 때만 쓰이고, 이번 학습에서는 KL loss 가 약 18k step 부터 로그상 0.000 이라 latent 는 사실상 쓰이지 않았다.

### Rollout system

&#91;embedded content: rollout system architecture (sim) · rollout\_gui\_then\_n.sh / rollout\_n.sh, lerobot\_robot\_ur16e\_ros\]

정책은 로봇에 ROS topic 으로만 닿는다. 실물로 옮길 때는 파란 부분(Isaac Sim 과 topic\_based hardware)을 ur\_robot\_driver·Robotiq driver·RealSense 로 바꾸고, controller 이름과 topic 은 그대로 쓴다. 단 exterior camera 는 실물에 아직 없고, ground truth 를 쓰는 harness(reset·judge)는 sim 전용이다.

## 핵심 기술과 기법

같은 ACT 설정에서 결과를 가른 것은 data 와 simulation 설계였다. ACT 는 LeRobot 0.6.1 의 기본 구현을 고치지 않고 썼다. 구조는 위 아키텍처 그림에 있다.

### ACT model 설정

- **Input**: camera 2대 image(320×240, resize 없음)와 현재 joint state 7개. language input 은 없다.
- **Structure**: shared ResNet18 backbone(ImageNet init, fine-tuned)이 두 camera image 를 각각 8×10 feature map 으로 바꾸고, Transformer(encoder 4 layers, decoder 1 layer, d\_model 512)가 이를 받는다. parameter 는 모두 trainable 이며 5,160만 개다.
- **Action chunking**: 한 번에 앞으로 100 frame(약 3.3 초)의 absolute joint target 을 낸다. rollout 에서는 앞 50 frame(약 1.7 초)을 실행하고, 다 쓰면 다음 chunk 를 받는다. 겹친 구간을 섞는 aggregation(새 chunk 0.7·이전 0.3)이 있지만, 200k rollout 에서는 inference 399번 중 390번이 action queue 가 빈 뒤에 일어나 거의 섞이지 않았다.
- **CVAE**: latent 32-D, KL weight 10. KL loss 가 약 18k step 부터 로그상 0.000 이라, 실제로는 latent 를 거의 쓰지 않는다. inference 때는 0 으로 둔다.
- **Training**: L1 loss, AdamW, learning rate 1e-5 고정(scheduler 없음), batch 32. state·action 은 dataset 의 mean·std 로, image 는 ImageNet 통계로 normalization 한다. image augmentation 은 켜지 않았다. GPU memory 는 allocated 기준 약 4 GiB 다.

### 이번 결과를 만든 기법

중요하다고 판단한 순서다. 하나씩 빼 보는 ablation 은 하지 않았으므로 순서는 추정이다. 괄호 안은 HISTORY 절 번호다.

| 기법 | 해결한 문제 | 효과 |
| --- | --- | --- |
| Soft gripper profile (§53) | 원래 drive(26 N·m, 2.56 rad/s)가 큐브를 튕겨 내고 wrist 를 걷어참 | scripted demo grasp 0/10 → 20/20, 든 채 slip 1.6 mm 이하 |
| Gripper action = command 로 기록 (§53.7) | 측정값(next\_state)을 배우면 contact point(0.506 rad)까지만 명령해 거의 누르지 못할 것으로 예상 | 닫힌 구간 action 1.0(command) 대 state 약 0.63. 잘못된 설정이면 recorder 가 시작을 거부 |
| Position-plateau close detection, 한 번에 0.8 rad (§53.4) | PhysX 가 contact 중 finger velocity 를 상한값으로 잘못 보고 → 매번 15 s 대기가 idle frame 으로 기록될 뻔함 | close 0.79–0.86 s 로 일정 |
| Idle frame 을 collection 단계에서 줄임 (§31–32, §36.3) | 초기 데이터의 65.8 % 가 idle 이라 ACT 가 "멈춤"을 배움. loss 는 잘 되는 policy 와 같아 loss 로는 못 잡음 | 8.4 %, physical 데이터 10.0 %. 사후 thinning 불필요 |
| Single-task ACT + 넓은 randomization (§30) | language input 이 없어, task 를 섞으면 같은 image 에 다른 정답 | 위치 ±60 mm·yaw randomization 으로 trajectory 암기를 막음 |
| Data 50 → 100 episodes (§36, §38) | 50개에서는 randomization 범위 가장자리에서 실패 | 7/10 → 9/10, error 13.4 → 7.6 mm (attach 때) |
| Resume training 60k → 200k (§54.6) | 60k 에서도 loss 가 계속 내려감 | error 12 → 6 mm. optimizer state 와 data order 까지 이어서 |
| Seed-paired evaluation + GT auto-scoring (§52, §54.5) | 배치가 매번 달라지면 checkpoint 비교가 흔들림 | 같은 seed 를 다시 돌리면 배치와 실패 trial 이 그대로 재현됨 |
| 320×240 capture (§32.4) | ACT 는 resize 하지 않아 해상도가 곧 input. 640×480 은 /dev/shm 이 64 MiB 인 container 에서 DataLoader 가 죽음(이 container 는 32 GiB) | training 약 15 step/s, 어느 container 에서나 같은 설정 |
| Unattended-run safeguards (§52, §54.1) | Isaac 첫 기동의 shader compile(약 3.5 분) 중 controller spawn 실패, wait loop 가 자기 shell 을 잡아(pgrep self-match) 무한 대기, grasp mode 가 다른 데이터가 섞일 위험 | `/clock` 대기, launcher file 실행, grasp-mode label 검사. collection → training → rollout 무인 완주 |

## 무엇이 바뀌었나

grasp 방식을 바꾸면서 gripper physics 설정, 기록하는 gripper action, data, 학습 길이가 함께 바뀌었다. task(빨강 큐브 35 mm · 0.2 kg 를 left marker 로), camera 2대(exterior + wrist), 큐브 시작 위치 randomization 범위는 그대로다.

| 항목 | fixed-joint attach (이전) | physical contact grasp (현재) |
| --- | --- | --- |
| 코드·HISTORY 의 이름 | attach | physical (`GRASP_MODE` 기본값) |
| grasp 방식 | 조건을 만족하면 큐브를 finger 에 fixed joint 로 붙임 | fixed joint 없음. pad 마찰로만 쥔다 |
| gripper drive | stiffness 1146 N·m/rad, max force 26 N·m, max velocity 2.56 rad/s, physics 60 Hz | stiffness 28.6 N·m/rad, 3 N·m, 1 rad/s, physics 120 Hz, solver iterations 증가, pad friction 1.2 |
| scripted demo 의 close | 두 단계, 0.52 rad 까지 | 한 번에 0.8 rad command (큐브에 막혀 0.506 rad 에서 stall) |
| 기록하는 gripper action | 다음 frame 의 측정값(next\_state) | controller 가 보낸 command |
| start pose | `ready_v1` | `ready` (실물 leader OMY L100 과 맞춘 자세) |
| data | 100 episodes, 43,222 frames, median 14.3 s | 100 episodes, 49,862 frames, median 16.6 s (collection 실패 0) |
| training | 60k steps (Isaac Sim 6.0.1) | 60k steps 후 200k 까지 resume (loss 0.021 → 0.012, 6.1.0) |
| gripper guard | 없음 | linkage 가 한계 밖으로 벌어지면 되돌리는 guard (rollout 중 trip 0회) |

attach 의 rollout 수치는 6.0.1 에서 학습한 checkpoint 를 이 container(6.1.0)에서 다시 돌린 값이다. 6.0.1 에서는 같은 9/10 에 평균 error 7.6 mm 였다. 6.1.0 은 화면이 약 3 % 밝고 real-time factor 도 달라, 이 baseline 은 학습 환경과 조금 다르다.

## 결과

같은 학습량(60k)에서는 두 방식이 대등하다. physical 을 200k 까지 resume 학습하자 placement error 가 절반으로 줄었다.

| policy | rollout | 성공 | 성공 시 평균 placement error |
| --- | --- | --- | --- |
| attach, 60k | seed 131 × 10 | 9/10 | 13.2 mm |
| 같은 attach checkpoint 를 physical scene 에서 | seed 131 × 10 | 0/10 | — |
| physical, 60k | seed 131·171 각 10 | 17/20 | 12.1 mm |
| physical, 100k | seed 131·171 각 10 | 19/20 | 7.1 mm |
| physical, 140k | seed 131·171 각 10 | 16/20 | 7.8 mm |
| **physical, 200k (권장)** | seed 131·171 각 10 | **19/20** | **6.2 mm** |

학습을 60k 에서 200k 로 늘리자 평균 error 가 12.1 mm 에서 6.2 mm 로 줄었다. 200k 는 네 checkpoint 중 같은 20 trial 에서 가장 좋은 것을 고른 것이고, 100k 도 19/20(7.1 mm)으로 성공률은 같다. 140k 의 16/20 은 seed 당 10 trial 의 noise 범위로 본다. 같은 배치가 checkpoint 마다 성공과 실패를 오갔다. 60k 의 seed 131 결과는 recording 을 위해 다시 돌린 값이다. 처음 실행도 8/10 이었다.

아래는 seed 131 의 10 trial 을 trial 별로 놓은 것이다. seed 가 큐브 시작 위치를 정하므로 두 policy 는 같은 배치를 받았다.

&#91;embedded content: 롤아웃 로그 outputs/harness\_logs · 시드 131, 70 s × 10회, Isaac Sim 6.1.0 · 2026-10-19–20\]

두 policy 는 학습량이 다르다(60k 대 200k). 그래서 이 차이는 grasp 방식보다 학습량의 효과로 봐야 한다. 10 mm 를 넘는 placement 가 attach 는 4회, 200k 는 1회다. 실패한 배치도 서로 다르다. attach 는 5번, 200k 는 7번 trial 에서 실패했다.

## 왜 좋아졌나

physical contact grasp 가 동작하게 된 것은 아래 변경들이 함께 들어갔기 때문이다. gripper 가 wrist 를 걷어차지 않게 했고, demo 가 더 깊게 닫게 했고, 그 close command 를 그대로 기록하게 했다. ACT 에서 하나씩 빼 보는 ablation 은 하지 않았다.

### 1. Gripper 가 wrist 를 걷어차지 않게 (soft profile)

원래 gripper drive(stiffness 1146 N·m/rad, 26 N·m, 2.56 rad/s)는 physical scene 에서 큐브를 짓눌러 튕겨 내고, 그 반력을 같은 articulation 의 wrist 로 전했다. stiffness 를 28.6 N·m/rad, max force 를 3 N·m, max velocity 를 1 rad/s 로 낮추고, physics 를 120 Hz 로 올리고 solver iterations 를 늘렸다. robot asset(USD)은 다시 bake 하지 않았다.

&#91;embedded content: 그리퍼 실험 로그 gp\_asset\_main · gp\_soft\_main (스크립트 시연, ACT 아님) · 그리퍼가 닫힌 구간마다 wrist\_1·2 위치 차분의 최대값 · 2026-10-19\]

같은 scripted demo(ACT 아님)의 grasp 성공은 0/10 에서 20/20 으로 바뀌었다. wrist kick 은 close 직후가 아니라 약 5 s 뒤, 큐브가 빠지며 finger 가 끝까지 닫힐 때 나왔다. teleop 중 "닿으면 5번 축(wrist\_2)이 돈다" 는 현상의 기전으로 보지만, 사람 teleop 으로는 아직 확인하지 않았다. ACT rollout 에서도 arm joint 가 최대 2–2.9 rad/s 로 움직였지만, policy command 와 reset 이동의 속도이고 runaway rotation 은 없었다.

### 2. Close command 를 0.8 rad 로, 그리고 command 를 기록

attach 시대의 demo 는 0.52 rad 까지만 닫았다. 큐브가 0.25 rad 에서 fixed joint 로 붙었기에 그걸로 충분했다. 그 data 로 배운 policy 는 0.52 rad 만 command 한다. physical scene 에서는 finger 가 큐브에 막혀 0.506 rad 에서 stall 하므로 거의 누르지 못해, grip force 가 0.3–0.9 N·m 에 그쳤고 0/10 이었다. 그래서 physical demo 는 한 번에 0.8 rad 를 command 한다.

&#91;embedded content: lerobot\_ds\_red\_left\_100 / lerobot\_ds\_red\_left\_100\_phys 의 에피소드 0, 0.1 s 간격 · 쥐는 힘 = 28.6 N·m/rad × (명령 − 0.506 rad), 상한 3 N·m (soft 설정)\]

기록 방식도 바꿔야 했다. 이전 recorder 는 gripper action 을 다음 frame 의 측정값(next\_state)으로 남겼다. attach 에서는 command 와 측정값이 같아 문제가 없었지만(왼쪽), physical 에서는 측정값이 0.506 rad 에서 멈춘다(오른쪽 회색 선). 그걸 배우면 policy 는 contact point 까지만 command 해 grip force 가 0 이 된다. 이제 gripper action 만 controller command(파란 선)에서 가져오고, arm action 은 그대로다. 이렇게 수집·학습한 policy 는 같은 seed 131 에서 8/10(60k), 9/10(200k)이다.

## Rollout 장면

physical 200k policy 는 큐브를 쥐고, 들고, left marker 위에 놓는다. 아래는 seed 131 의 trial 4(error 2 mm)다. 각 칸의 왼쪽은 exterior camera, 오른쪽은 wrist camera 다.

&#91;image: 물리 접촉 파지 200k, 시드 131 시행 4: 쥐기, 들기, 운반, 마커 위에 놓기\]

같은 trial 의 grasp 부터 place 까지를 animation 으로 보면 이렇다(sim 16 초 구간, 3배속).

&#91;image: 물리 접촉 파지 200k 성공 장면 애니메이션\]

attach 로 학습한 checkpoint 를 physical scene 에 넣으면 10 trial 모두 실패한다. 아래 trial 2 처럼 큐브를 쥐고 들다가 놓치는 경우가 많다.

&#91;image: 고정 조인트 파지 체크포인트를 물리 씬에서: 접근, 닫기, 큐브 낙하, 빈 그리퍼\]

큐브 시작 위치는 기준점 (0.60, −0.10) m 에서 x·y 각 ±60 mm 안의 uniform random 값이고, yaw 도 random 이다. demo collection 은 seed 91, rollout 은 131·171 을 써서 rollout 배치는 학습에 없던 것이다. 같은 seed 는 같은 10개 배치를 만들므로, checkpoint 끼리 trial 번호가 같으면 시작 위치도 같다.

전체 video(trial 당 mp4, 3배속, 파일 이름에 SUCCESS/FAIL)는 workspace `outputs/rollout_videos/` 아래에 있다.

- physical 200k: `act_phys_long_200k_s131/`, `act_phys_long_200k_s171/`
- physical 100k·140k: `act_phys_long_100k_s*/`, `act_phys_long_140k_s*/`
- physical 60k: `act_red_left_100_phys_s131/`, `act_red_left_100_phys_s171/`
- attach checkpoint 를 physical scene 에서: `act_red_left_100_physical/`
- attach (attach scene): `act_red_left_100/`(trial 3–10)

## 한계와 다음 단계

남은 실패는 aiming 보다 grip force 와 grasp 위치의 문제로 보인다. 로그가 있는 실패 9회는 모두 큐브를 쥐고 2–19 cm 들어 올린 뒤 놓쳤다.

- **항상 실패하는 배치 하나**: seed 131 의 trial 7(큐브 시작 x 0.556, y −0.084 m)은 60k·100k·140k·200k 에서 모두 실패했다. attach 에서는 같은 배치가 성공했다(27 mm). 학습량으로는 안 풀렸다.
- **큐브 yaw 를 절반쯤만 따라감**: demo 는 wrist 를 큐브 yaw(90° 대칭이라 ±45° 로 fold)만큼 돌린다. 200k policy 는 같은 방향으로 돌리지만 필요한 각도의 약 0.4–0.6배만 돌려, grasp 직전 misalignment 가 평균 9–13°, 최대 33–36° 다. 작은 misalignment 는 닫힐 때 pad 가 큐브를 돌려 맞춘다. seed 131 의 trial 7 은 큐브가 −44° 로 fold boundary(±45°)에 있어 demo 의 grasp 방향이 90° 갈리는 자리고, misalignment 도 33° 였다. 이것이 그 배치가 항상 실패하는 원인으로 보이지만, 확인하지는 않았다.
- **다음 조정 후보**: 그 근처 demo 보강, fold boundary 근처 yaw 의 demo 보강, 또는 grasp robustness(pad friction, close force, grasp 높이) 조정.
- **진행 중**: x 범위를 ±100 mm 로 넓힌 바깥 띠 90 episode(그중 절반은 fold boundary 근처 yaw)를 더해 190 episode 로 재학습 중이다(HISTORY §58).
- **비교의 범위**: attach 와는 seed 131 의 10 trial 로만 비교했다. start pose(`ready_v1` 대 `ready`)와 학습 환경(6.0.1 대 6.1.0)도 다르다. seed 당 10 trial 이라 성공 개수 1–2개 차이는 noise 범위다.
- **Ablation 은 안 함**: gripper 설정, close command, 기록 방식을 한꺼번에 바꿔 ACT 를 다시 만들었다. 각각의 기여는 ACT rollout 으로 따로 재지 않았다.
- **검증 범위**: sim, 물체 1종(35 mm 큐브)만 했다. 다른 크기·질량의 물체와 실물은 아직이다.
- **아직 재수집하지 않은 것**: wrist camera 1대 ACT 와 GR00T 3-task 는 여전히 attach data 다. physical 로 바꾸는 harness 는 준비돼 있다.
- **사람 teleop 확인**: leader 로 teleop 할 때 wrist 회전이 다시 생기는지는 사용자 실측이 필요하다. Isaac 을 `--grasp-mode physical` 로 띄우면 된다.

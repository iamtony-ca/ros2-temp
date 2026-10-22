# ACT physical contact grasp 최종 보고

Oct 22, 2026 · @tony

## 요약

Isaac Sim 6.1.0 의 UR16e + Robotiq 2F-85 가 physical contact grasp(fixed joint 없이 pad 마찰로만 쥐기)로 빨간 큐브를 집어 왼쪽 marker 에 놓는 task. demo 190개로 학습한 ACT policy 는 큐브를 x ±100 mm 까지 넓힌 배치에서 20/20, 기본 범위(±60 mm)에서 19/20 성공.

- **성능**: 성공한 trial 의 placement error 평균 4–6 mm. 140k checkpoint 는 기본 범위 20/20.
- **권장 checkpoint**: `outputs/act_red_left_190_phys/checkpoints/200000` (200k step, loss 0.013).
- **데이터 범위 = policy 범위**: demo 100개(±60 mm)만으로 학습한 policy 는 넓은 범위에서 13/20. 실패 7회 모두 학습 범위 밖에서 큐브를 빗나간 경우였음. 바깥 띠 demo 90개를 더해 해결.
- **남은 문제**: 남은 실패는 쥐고 들다가 미끄러지는 경우. policy 가 큐브 yaw 만큼 wrist 를 돌리지 못해, 쥐는 순간 20° 넘게 어긋나면 떨어뜨리기 쉬움.

성공 기준은 Isaac 의 ground truth(GT)로 재는 두 조건: 큐브가 marker 중심에서 35 mm 안에 있고, gripper 가 큐브를 놓았을 것(finger\_joint ≤ 0.4 rad).

## 전체 파이프라인

collection 부터 rollout 채점까지 사람 없이 이어서 실행. 최종 모델 기준 소요 시간은 바깥 띠 demo 90개 collection 34분, 190개 conversion 16분, 200k training 약 4시간 23분, 평가 12 run(120 trial) 약 3시간.

&#91;embedded content: ACT physical contact grasp 파이프라인 · 6단계, 코드·설정 파일 기준 (2026-10-22)\]

1. **Isaac scene**: table 위에 35 mm 큐브 2개(빨강·파랑)와 marker 2개. episode 마다 두 큐브의 위치와 yaw 를 다시 뽑음. random seed 를 고정하면 같은 배치를 다시 만들 수 있음(collection 91·301, rollout 131·171·211·251). camera 는 table 을 내려다보는 exterior camera 와 wrist 의 D405 두 대.
2. **Scripted expert**: state machine 이 Isaac 의 GT object pose 를 읽어 pick-and-place 를 수행. 큰 이동은 MoveIt(기본 planner cuMotion), approach·retreat 는 5 mm 간격의 Cartesian 직선 경로. 실패한 demo 는 저장하지 않으며, 190개 모두 성공.
3. **Recorder (il\_recorder)**: sim time 기준 30 Hz 마다 joint 7개(arm 6 + gripper)와 camera 2대의 최신 값을 저장(zero-order hold). arm action 은 다음 frame 의 joint state(next\_state), gripper action 은 controller command. 실물 leader(OMY L100) teleop 도 같은 recorder 를 쓰도록 만들었으나, 아직 fake follower 로만 시험함.
4. **Conversion (raw\_to\_lerobot)**: ROS 쪽에는 LeRobot·torch 를 넣지 않음. 그래서 기록은 JPEG + JSON 으로 하고, 분리된 ML venv 에서 LeRobot dataset(state·action 각 7-D, video 2개)으로 변환. 다른 grasp mode 로 모은 옛 데이터가 섞이면 거부.
5. **ACT training**: camera 2대 image 와 현재 joint state 를 보고 앞으로 100 frame(약 3.3 초)의 joint target 을 한 번에 예측. RTX 5090 에서 약 13 step/s, GPU memory 는 allocated 기준 약 4 GiB. 100k 에서 멈추고 PC 재부팅 뒤 checkpoint 에서 이어 200k 완료.
6. **Rollout · scoring**: LeRobot policy server 가 action chunk 를 만들고, robot adapter 가 joint target 을 ROS controller 로 30 Hz 로 전송. chunk 100 step 중 50 step 만 실행하고 다음 chunk 를 받음. inference 는 CPU 에서 chunk 당 약 60 ms. trial 마다 start pose 와 큐브 배치를 reset 하고, wall clock 70 초(sim 약 79 초) 뒤 GT 로 자동 판정. trial 마다 video 와 step 로그도 저장.

실물에서도 같은 topic 과 controller 를 쓰도록 구성함. 다만 exterior camera 는 실물에 아직 없고, GT 기반 scoring 은 sim 에서만 가능.

## 아키텍처

모델은 LeRobot 의 ACT 를 그대로 사용. 직접 만든 것은 로봇과 정책을 잇는 ROS 쪽 구성.

### ACT model

&#91;embedded content: ACT model architecture · LeRobot 0.6.1, act\_red\_left\_190\_phys 200k config.json\]

카메라 영상 두 장은 같은 ResNet18 을 거쳐 각각 80개 token 이 되고, joint state 1 token·latent 1 token 과 함께 Transformer encoder 에 입력됨. decoder 는 학습된 query 100개로 앞으로 100 frame 의 action 을 한 번에 출력. CVAE encoder 는 training 때만 쓰이며, KL loss 가 약 18k step 부터 로그상 0.000 이라 latent 는 사실상 쓰이지 않았음.

### Rollout system

&#91;embedded content: rollout system architecture (sim) · rollout\_gui\_then\_n.sh / rollout\_n.sh, lerobot\_robot\_ur16e\_ros\]

정책은 로봇에 ROS topic 으로만 연결됨. 실물로 옮길 때는 파란 부분(Isaac Sim 과 topic\_based hardware)을 ur\_robot\_driver·Robotiq driver·RealSense 로 바꾸고, controller 이름과 topic 은 그대로 사용. 단 exterior camera 는 실물에 아직 없고, ground truth 를 쓰는 harness(reset·judge)는 sim 전용.

## 핵심 기술과 기법

ACT 모델 자체는 LeRobot 기본 설정을 거의 그대로 사용. 성능을 만든 것은 gripper 물리 설정, 데이터 기록 방식, 데이터 범위.

### ACT model 설정

- **Input**: camera 2대 image(320×240, resize 없음)와 현재 joint state 7개(arm 6 + gripper). language input 없음.
- **Structure**: shared ResNet18 backbone(ImageNet init, fine-tuned)이 두 image 를 각각 8×10 feature map 으로 변환. Transformer(encoder 4 layers, decoder 1 layer, d\_model 512)가 이를 입력으로 받음. parameter 는 모두 trainable, 5,160만 개.
- **Action chunking**: 한 번에 앞으로 100 frame(약 3.3 초)의 absolute joint target 을 출력. rollout 에서는 앞 50 frame(약 1.7 초)을 실행하고 다음 chunk 를 받음.
- **CVAE**: latent 32-D, KL weight 10. KL loss 가 약 18k step 부터 로그상 0.000 이라 latent 는 거의 쓰이지 않음. inference 때는 0 으로 둠.
- **Training**: L1 loss, AdamW, learning rate 1e-5 고정(scheduler 없음), batch 32, 200k step. state·action 은 dataset 의 mean·std 로, image 는 ImageNet 통계로 normalization. image augmentation 은 사용하지 않음.

### 이 결과를 만든 기법

괄호 안 번호는 `src/HISTORY.md` 의 절 번호.

| 기법 | 해결한 문제 | 효과 |
| --- | --- | --- |
| Soft gripper profile (§53) | 원래 drive(26 N·m, 2.56 rad/s)는 닫히는 순간 큐브를 튕겨 내고 wrist 를 2.2–2.8 rad/s 로 걷어참 | scripted grasp test 20/20 성공, 든 채 slip 1.6 mm 이하, wrist 속도 0.64 rad/s 이하 |
| Gripper action = command 로 기록 (§53.7) | 측정값(next\_state)을 배우면 큐브에 막힌 0.506 rad 까지만 명령해 쥐는 힘이 0 이 됨 | 닫힌 구간 action 1.0(command) 대 state 약 0.63. 설정이 틀리면 recorder 가 시작을 거부 |
| Position-plateau close detection, 한 번에 0.8 rad (§53.4) | PhysX 가 contact 중 finger velocity 를 상한값으로 잘못 보고해, 속도로는 닫힘 판정 불가 | close 0.79–0.86 s 로 일정 |
| Idle frame 을 collection 단계에서 줄임 (§31–32, §36.3) | idle frame 이 많으면 ACT 가 "멈춤"을 학습. loss 는 잘 되는 policy 와 같아 loss 로는 못 잡음 | idle 10.0 %(기본 100) · 11.8 %(바깥 띠 90). 사후 thinning 불필요 |
| Single-task ACT + randomization (§30) | language input 이 없어, task 를 섞으면 같은 image 에 다른 정답이 생김 | 위치·yaw randomization 으로 trajectory 암기 방지 |
| 바깥 띠 demo 추가 (§58) | demo 범위(±60 mm) 밖 배치에서 policy 가 학습 범위 쪽으로 끌려가 큐브를 빗나감 | 넓은 범위 13/20 → 20/20 |
| Resume training (§54.6, §58.6) | 200k 학습은 4시간이 넘어 중간에 멈춰야 할 때가 있음 | optimizer state 와 data order 까지 이어서 학습. PC 재부팅 뒤에도 100k 에서 이어 200k 완료 |
| Seed-paired evaluation + GT auto-scoring (§52, §54.5) | 배치가 매번 달라지면 checkpoint 비교가 흔들림 | 같은 seed 는 모델이 달라도 trial 마다 같은 배치 |
| 320×240 capture (§32.4) | ACT 는 resize 하지 않아 해상도가 곧 input. 640×480 은 /dev/shm 이 64 MiB 인 container 에서 DataLoader 가 죽음 | training 약 13 step/s, 어느 container 에서나 같은 설정 |
| Unattended-run safeguards (§52, §54.1, §58) | Isaac 첫 기동의 shader compile(약 3.5 분) 중 controller spawn 실패, wait loop 가 자기 shell 을 잡아(pgrep self-match) 무한 대기, grasp mode 가 다른 데이터가 섞일 위험 | `/clock` 대기, launcher file 실행, grasp-mode label 검사. collection → training → rollout 무인 완주 |
| Gripper guard | soft drive 에서 linkage 가 한계 밖으로 벌어질 수 있음 | 한계를 넘으면 되돌림. rollout 중 trip 0회 |

## 데이터

학습 데이터는 scripted expert 가 만든 demo 190개. 기본 범위 100개에 큐브를 더 멀리 놓은 바깥 띠 90개를 추가. 두 묶음 모두 collection 실패 0.

| 묶음 | episode | 빨간 큐브 위치 (기준점 0.60, −0.10 m) | 큐브 yaw | frame | collection seed | 수집 시간 |
| --- | --- | --- | --- | --- | --- | --- |
| 기본 범위 | 100 | x·y 각각 ±60 mm | 균일 | 49,862 | 91 | 약 40 분 |
| 바깥 띠 | 90 | x 쪽 60–100 mm(앞뒤 양쪽), y ±60 mm | 30 % 는 fold boundary(±45°) ±10° 에서 뽑고 나머지는 균일(결과 46/90 이 ±10° 안) | 44,712 | 301 | 34 분 |
| 합계 | 190 |  |  | 94,574 |  |  |

- **episode 하나**: 시작 자세 `ready` 에서 빨간 큐브를 집어 왼쪽 marker 에 놓고 종료. 중앙값 16.6 s, 30 Hz.
- **y 범위를 넓히지 않은 이유**: 파란 큐브(y +0.10 m)와 왼쪽 marker(y +0.215 m) 때문에 y 쪽에는 더 놓을 자리가 없음.
- **fold boundary 를 몰아 넣은 이유**: 큐브는 90° 대칭이라 yaw 가 ±45° 근처면 demo 의 grasp 방향이 90° 갈림. 이 구간 데이터를 늘리면 wrist 각도를 더 잘 배울 것으로 기대함. 결과는 "실패 분석과 wrist yaw" 참고.
- **크기**: raw(JPEG + JSON) 1.4 GB + 1.3 GB → LeRobot dataset(h264) 382 MB. 경로 `outputs/lerobot_ds_red_left_190_phys`.

## 결과

최종 200k checkpoint 는 기본 범위 19/20, 넓은 범위 20/20. 성공한 trial 의 placement error 평균 4–6 mm.

| checkpoint | 기본 범위 (seed 131+171) | 넓은 범위 (seed 211+251) | 성공 시 placement error 평균 |
| --- | --- | --- | --- |
| **demo 190개 · 200k (최종)** | 19/20 | **20/20** | 4.0–6.0 mm |
| demo 190개 · 140k | **20/20** | 10/10 (seed 211만) | 7.1–8.0 mm |
| demo 190개 · 100k | 17/20 | 10/10 (seed 211만) | 11.0–11.8 mm |
| demo 100개(기본 범위만) · 200k | 19/20 | 13/20 | 기본 5.9–6.5 · 넓은 10.5–11.4 mm |

- **평가 방법**: seed 하나에 10 trial, trial 마다 wall clock 70 초(sim 약 79 초) 동안 실행하고 GT 로 자동 판정. 넓은 범위는 큐브를 x ±100 mm, y ±60 mm 안에서 균일하게 뽑음.
- **같은 seed = 같은 배치**: checkpoint 가 달라도 trial 마다 같은 큐브 위치·yaw(로그로 확인).
- **checkpoint 고르기**: 140k 가 30/30 으로 가장 많이 성공, 200k 는 placement error 가 가장 작음. seed 당 10 trial 이라 1회 차이는 noise 범위. 마지막 행의 demo 100개 policy 는 데이터 범위의 효과를 보기 위한 비교군.

&#91;embedded content: outputs/harness\_logs/e190\_{old,new}200k\_w{211,251}.run.log · seed 당 10 trial × 70 s, Isaac Sim 6.1.0 · 2026-10-22\]

demo 100개 policy 의 실패(빨간 ×)는 모두 회색 띠 바깥. demo 190개 policy 는 띠 안팎 20개 모두 성공.

## 실패 분석과 wrist yaw

최종 policy 의 남은 실패는 큐브를 쥐고 들다가 미끄러지는 경우. policy 는 큐브 yaw 만큼 wrist 를 돌리지 못하고, 쥐는 순간 20° 넘게 어긋나면 떨어뜨리기 쉬움. 아래 수치는 rollout 의 step 로그(매 step 의 joint, TCP, 큐브 pose)에서 구했고, 별도 분석으로 다시 검증함.

### demo 범위 밖에서는 aiming 이 무너진다

- demo 100개 policy 의 넓은 범위 실패 7회는 모두 grasp 전에 끝남. 6회는 grasp 가 한 번도 걸리지 않았고, 1회는 0.3 s 만 모서리를 집음.
- 처음 닿는 순간 TCP 가 큐브에서 33–53 mm 떨어져 있었음(성공한 grasp 는 중앙값 6 mm). 열린 finger 가 큐브를 밀거나 넘어뜨렸고, 2회는 큐브 없이 jaw 만 닫힌 채 종료.
- 7회 중 6회는 x 오차가 기준점 쪽을 향함. policy 가 학습한 범위 안쪽으로 끌려간다는 뜻. 같은 배치에서 최종 policy 는 3–9 mm 안에서 쥐고 들어 올림.

### demo 190개 policy 의 실패

- **seed 131 trial 9**(큐브 yaw −26.6°): 100k·200k 는 실패, 140k 는 성공. 세 checkpoint 의 접근 높이·위치·wrist 각도는 같았고, jaw 는 큐브 면에서 약 30° 어긋나 있었음. 차이는 쥔 뒤 큐브가 pad 사이에서 면으로 돌아앉았는지 여부. 200k 는 들어 올린 뒤 운반을 시작한 직후 놓침.
- **100k 의 나머지 두 번**: seed 171 trial 10 은 닫으면서 큐브를 23 mm 밀고 들다가 빠짐. trial 8 은 placement error 35.2 mm 로 판정 한계(35 mm)를 겨우 넘음.
- **fold boundary 배치**: seed 131 trial 7(큐브 yaw −44.2°)은 demo 100개 policy 가 모든 checkpoint 에서 떨어뜨렸고, 최종 policy 는 100k·140k·200k 모두 성공. 둘 다 대각선으로 모서리를 쥐. 다만 이런 배치는 하나뿐이라 결론으로 보기는 어려움.

### wrist yaw 추종

- **demo**: wrist\_3 = −(큐브 yaw) − 83° 로, 큐브 yaw 를 그대로 따라감(기울기 −1.00). 큐브는 90° 대칭이라 yaw 는 ±45° 로 fold 해서 봄.
- **policy**: 쥐는 순간 wrist\_3 의 기울기가 모든 run 에서 −0.62 \~ +0.05. wrist 가 큐브를 따라가지 않고 −63 \~ −84° 의 평균 각도로 끌림.
- **fold boundary oversampling 의 효과는 확인되지 않음**: 기본 범위의 같은 배치 20쌍에서 쥐는 순간 어긋난 각도(misalignment)는 최종 15.8°, demo 100개 policy 11.2° 로 오히려 큼(차이 95 % CI +1.9 \~ +7.3°). 넓은 범위에서는 15.0° 대 19.6° 로 작았지만 유의하지 않음.
- **pad 가 일부 맞춤**: 닫히면서 pad 가 큐브를 jaw 쪽으로 약 20 % 돌려 앉힘. 1 s 뒤 misalignment 평균이 15.1° 에서 10.5° 로 감소.

쥐는 순간의 misalignment 가 크면 실패 비율이 올라감. 아래는 demo 190개 policy 의 평가 run 12개와 demo 100개 policy 의 run 6개(seed 131·171, 100k·140k·200k)에서 grasp 가 걸린 174회를 나눈 결과. 실패에는 낙하 외에 placement 오차 1회와 모서리만 집었다 빠진 1회가 포함됨.

| misalignment | 실패 / grasp |
| --- | --- |
| 10° 이하 | 0 / 71 |
| 10–20° | 1 / 64 |
| 20–30° | 3 / 15 |
| 30–45° | 7 / 24 |

실패가 몇 개 배치에 몰려 있어(같은 배치를 여러 checkpoint 가 되풀이) 독립 표본은 아님. 그래도 방향은 분명해, wrist yaw 추종이 다음으로 고칠 부분.

## Rollout 장면

최종 policy 가 demo 100개의 범위 밖(기준점에서 x +99 mm)에 놓인 큐브를 집어 놓는 장면(넓은 범위 seed 211, trial 4). 각 칸의 왼쪽은 exterior camera, 오른쪽은 wrist camera.

&#91;image: 최종 policy, 넓은 범위 seed 211 trial 4: 접근, 닫기, 운반, marker 위에 놓기\]

같은 trial 의 움직임(sim 9–22.5 초 구간).

&#91;image: 최종 policy 성공 장면 애니메이션\]

같은 큐브 배치에서 demo 100개 policy 는 큐브에서 43 mm 빗나간 곳에서 jaw 를 닫고, 빈 gripper 로 marker 까지 이동(위 줄). 최종 policy 는 큐브를 쥐고 9 mm 오차로 놓음(아래 줄).

&#91;image: 같은 배치: demo 100개 policy 는 빗나가고 최종 policy 는 성공\]

전체 video(trial 당 하나) 위치: `outputs/rollout_videos/`

- `e190_new200k_s131`, `e190_new200k_s171`: 최종 policy, 기본 범위
- `e190_new200k_w211`, `e190_new200k_w251`: 최종 policy, 넓은 범위
- `e190_old200k_w211`, `e190_old200k_w251`: demo 100개 policy, 넓은 범위(같은 배치)
- `e190_new100k_*`, `e190_new140k_*`: checkpoint 비교

파일 이름 끝의 SUCCESS / FAIL 이 그 trial 의 판정.

## 한계와 다음 단계

다음으로 고칠 부분은 wrist yaw 추종. 나머지는 검증 범위의 한계.

- **wrist yaw 추종**: 후보를 아래 순서로 검토.
  1. 320×240 wrist 영상에서 큐브 각도 단서가 약한지 확인(해상도, crop).
  2. approach 구간의 closed-loop 비중 늘리기. 현재는 50 step(약 1.7 초)을 피드백 없이 실행.
  3. grasp robustness: pad friction, close force, grasp 높이.
- **y 범위**: 큐브 y 는 ±60 mm 까지만 학습·평가. 파란 큐브와 marker 배치를 바꾸지 않으면 더 넓힐 수 없음.
- **평가 규모**: seed 당 10 trial 이라 성공 1–2개 차이는 noise 범위. 넓은 범위의 20/20 대 13/20 은 같은 배치의 쌍 비교라 신뢰할 만하지만, 실패 원인 통계는 몇 개 배치에 몰려 있음.
- **Ablation 은 안 함**: gripper 설정, close command, 기록 방식을 한꺼번에 정함. 각각의 기여는 ACT rollout 으로 따로 측정하지 않음.
- **검증 범위**: sim, 물체 1종(35 mm 큐브), task 1개만 검증. 다른 크기·질량의 물체와 실물은 미검증.
- **아직 physical grasp 로 재수집하지 않은 것**: wrist camera 1대 ACT 와 GR00T 3-task. 바꾸는 harness 는 준비됨.
- **사람 teleop 확인**: 실물 leader(OMY L100)로 teleop 할 때 wrist 가 튀지 않는지는 사용자 실측 필요. Isaac 을 `--grasp-mode physical` 로 띄우면 됨.

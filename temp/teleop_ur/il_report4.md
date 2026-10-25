# ACT physical contact grasp 최종 보고 — 구조 개요판

Oct 25, 2026 · @tony

## 요약

Isaac Sim 속 로봇팔이 physical contact grasp(fixed joint 없이 pad 마찰로만 쥐기)로 큐브를 집어 marker 에 놓는 task. demo 190개로 학습한 ACT policy 는 넓은 배치 범위에서 20/20, 기본 범위에서 19/20 성공.

- **성능**: 성공한 trial 의 placement error 평균 4–6 mm. 140k checkpoint 는 기본 범위 20/20.
- **데이터 범위 = policy 범위**: 기본 범위 demo 만으로 학습한 policy 는 넓은 범위에서 13/20. 실패는 모두 학습 범위 밖에서 큐브를 빗나간 경우. 바깥 범위 demo 를 더해 해결.
- **남은 문제**: 쥐고 들다가 미끄러지는 실패. policy 가 큐브 yaw 만큼 wrist 를 돌리지 못해, 쥐는 순간 20° 넘게 어긋나면 떨어뜨리기 쉬움.

성공 판정은 시뮬레이터의 ground truth(GT) 기준 두 조건: 큐브가 marker 중심 35 mm 안에 있고, gripper 가 큐브를 놓았을 것.

## 전체 파이프라인

collection 부터 rollout 채점까지 사람 개입 없이 자동 실행. 최종 모델 기준 소요 시간은 추가 demo 90개 collection 34분, conversion 16분, training 약 4시간 23분, rollout 120 trial 약 3시간.

&#91;embedded content: 파이프라인 · 6단계, 역할 기준\]

1. **Isaac scene**: 로봇팔·gripper·camera 2대(exterior camera, wrist camera)와 큐브 2개·marker 2개. episode 마다 큐브 위치와 yaw 를 무작위로 다시 뽑고, seed 를 고정하면 같은 배치 재현. 물체의 GT pose 는 demo 와 scoring 에만 쓰고 policy 입력에는 넣지 않음.
2. **Scripted expert**: state machine 이 GT pose 로 pick-and-place 를 수행. 큰 이동은 MoveIt(기본 planner cuMotion), approach·retreat 는 Cartesian 직선 경로. 실패한 demo 는 버리며 190개 모두 성공.
3. **Recorder**: 30 Hz 마다 joint 7개(arm 6 + gripper)와 camera image 2장의 최신값 저장. arm action 은 다음 시점의 joint state, gripper action 은 controller 에 보낸 command. 실물 leader 장치 teleop 도 같은 recorder 를 쓰도록 설계했으나 아직 fake follower 로만 시험함.
4. **Conversion**: 로봇 미들웨어 환경과 ML 환경을 분리. raw 기록을 ML 환경에서 LeRobot dataset(state·action 각 7-D, video 2개)으로 변환. 다른 grasp mode 의 데이터가 섞이면 거부.
5. **ACT training**: camera 2대 image 와 현재 joint state 로 앞으로 약 3.3초의 joint target 을 한 번에 예측. 중간에 멈춰도 checkpoint 에서 이어 학습.
6. **Rollout · scoring**: 학습된 policy 가 같은 scene 에서 로봇을 직접 구동. trial 마다 배치를 reset 하고 일정 시간 뒤 GT 로 자동 판정, video 와 step 로그 저장.

실물로 옮길 때는 policy·recorder·제어 인터페이스는 그대로 두고 시뮬레이터와 하드웨어 연결부만 실물 driver 로 교체하는 구조. 단, exterior camera 는 실물에 아직 없고, GT pose 가 필요한 scripted expert 와 자동 scoring 은 sim 전용.

## 아키텍처

모델은 공개 구현(LeRobot ACT)을 기본 설정을 거의 그대로 사용. 직접 만든 것은 policy 와 로봇을 잇는 연결 계층과 자동화 harness.

### ACT model

&#91;embedded content: ACT model architecture · LeRobot ACT 기본 설정\]

camera image 2장은 같은 CNN backbone 을 거쳐 token 이 되고, joint state·latent 와 함께 Transformer encoder 에 입력됨. decoder 가 앞으로 100 step 의 action 을 한 번에 출력. 학습 약 18k step 부터 KL loss 가 로그상 0.000 이 되어 latent 는 사실상 미사용.

### Rollout system

&#91;embedded content: Rollout system · 5계층 + rollout harness\]

policy 는 control layer 의 표준 인터페이스(joint state·joint command·camera image)로만 로봇과 연결. 실물 전환 시 파란 두 계층만 교체하고 위쪽은 그대로 사용. 단, exterior camera 는 실물에 아직 없음. rollout harness 는 GT pose 를 쓰므로 sim 전용.

## 핵심 기술과 기법

ACT 모델은 기본 설정을 거의 그대로 사용. 성능을 만든 것은 gripper 물리 설정, 데이터 기록 방식, 데이터 범위.

### ACT model 설정

- **Input**: camera 2대 image(320×240, resize 없음)와 joint state 7개(arm 6 + gripper). language input 없음.
- **Action chunking**: 앞으로 약 3.3초의 absolute joint target 을 한 번에 출력. rollout 때는 앞 절반만 실행하고 다음 chunk 를 받음.
- **Training**: L1 loss, 고정 learning rate, batch 32, 200k step. image augmentation 없음.

### 이 결과를 만든 기법

| 기법 | 해결한 문제 | 효과 |
| --- | --- | --- |
| Soft gripper profile | 원래 drive 는 닫히는 순간 큐브를 튕겨 내고 wrist 를 2.2–2.8 rad/s 로 흔듦 | scripted grasp test 20/20, 든 채 slip 1.6 mm 이하, wrist 속도 0.64 rad/s 이하 |
| Gripper action = command 로 기록 | 측정값(state)을 배우면 큐브에 막힌 위치까지만 명령해 쥐는 힘이 0 | 닫힌 구간의 action 은 완전 닫힘 command. 설정이 틀리면 recorder 가 시작 거부 |
| Position-plateau close detection, 한 번에 닫기 | 물리 엔진이 contact 중 finger velocity 를 잘못 보고해 속도로는 판정 불가 | close 0.79–0.86 s 로 일정 |
| Idle frame 을 collection 단계에서 줄임 | idle frame 이 많으면 policy 가 "멈춤"을 학습. loss 로는 구분 불가 | idle 10.0 %(기본) · 11.8 %(바깥). 사후 thinning 불필요 |
| Single-task ACT + randomization | language input 이 없어 task 를 섞으면 같은 image 에 다른 정답이 생김 | 위치·yaw randomization 으로 trajectory 암기 방지 |
| 바깥 범위 demo 추가 | demo 범위 밖 배치에서 policy 가 학습 범위 쪽으로 끌려가 큐브를 빗나감 | 넓은 범위 13/20 → 20/20 |
| Resume training | 4시간이 넘는 학습을 중간에 멈춰야 할 때가 있음 | optimizer state·data order 까지 이어 학습. 재부팅 뒤에도 200k 완료 |
| Seed-paired evaluation + GT auto-scoring | 배치가 매번 달라지면 checkpoint 비교가 흔들림 | 같은 seed 는 모델이 달라도 trial 마다 같은 배치 |
| 320×240 capture | ACT 는 resize 하지 않아 해상도가 곧 input. 큰 image 는 공유 메모리가 작은 container 에서 학습 중단 | training 약 13 step/s, 어느 환경에서나 같은 설정 |
| Unattended-run safeguards | 시뮬레이터 첫 기동 지연, wait loop 오작동, 다른 grasp mode 데이터 혼입 위험 | 시뮬레이터 준비 대기, wait loop 수정, grasp mode label 검사. collection → training → rollout 무인 완주 |
| Gripper guard | soft 설정에서 linkage 가 한계 밖으로 벌어질 수 있음 | 한계를 넘으면 되돌림. rollout 중 trip 0회 |

## 데이터

학습 데이터는 scripted expert 가 만든 demo 190개. 기본 범위 100개에 큐브를 더 멀리 놓은 바깥 범위 90개를 추가. 두 묶음 모두 collection 실패 0.

| 묶음 | episode | 큐브 위치 (기준점 = 기본 범위 중심) | 큐브 yaw | frame | 수집 시간 |
| --- | --- | --- | --- | --- | --- |
| 기본 범위 | 100 | 앞뒤·좌우 각각 ±60 mm | 균일 | 49,862 | 약 40분 |
| 바깥 범위 | 90 | 앞뒤 60–100 mm(양쪽), 좌우 ±60 mm | 30 % 는 fold boundary(±45°) ±10° 에서, 나머지는 균일 | 44,712 | 34분 |
| 합계 | 190 |  |  | 94,574 |  |

- **episode 하나**: start pose 에서 큐브를 집어 marker 에 놓고 종료. 중앙값 16.6초, 30 Hz.
- **좌우 범위를 넓히지 않은 이유**: 다른 큐브와 marker 가 옆에 있어 더 놓을 자리가 없음.
- **fold boundary 를 몰아 넣은 이유**: 큐브는 90° 대칭이라 yaw 가 ±45° 근처면 demo 의 grasp 방향이 90° 갈림. 이 구간 데이터를 늘리면 wrist 각도를 더 잘 배울 것으로 기대. 결과는 "실패 분석과 wrist yaw" 참고.
- **크기**: raw 기록 2.7 GB → LeRobot dataset 382 MB.

## 결과

최종 policy(demo 190개, 200k step)는 기본 범위 19/20, 넓은 범위 20/20. 성공한 trial 의 placement error 평균 4–6 mm.

| checkpoint | 기본 범위 (rollout 2회 × 10 trial) | 넓은 범위 (rollout 2회 × 10 trial) | 성공 시 placement error 평균 |
| --- | --- | --- | --- |
| **demo 190개 · 200k (최종)** | 19/20 | **20/20** | 4.0–6.0 mm |
| demo 190개 · 140k | **20/20** | 10/10 (rollout 1회만) | 7.1–8.0 mm |
| demo 190개 · 100k | 17/20 | 10/10 (rollout 1회만) | 11.0–11.8 mm |
| demo 100개(기본 범위만) · 200k | 19/20 | 13/20 | 기본 5.9–6.5 · 넓은 10.5–11.4 mm |

- **평가 방법**: 고정 seed 하나에 10 trial, trial 마다 일정 시간(sim 약 79초) 실행 후 GT 로 자동 판정. 넓은 범위는 큐브를 앞뒤 ±100 mm, 좌우 ±60 mm 안에서 균일하게 뽑음.
- **같은 seed = 같은 배치**: checkpoint 가 달라도 trial 마다 같은 큐브 위치·yaw 라 짝 비교 가능.
- **checkpoint 고르기**: 140k 가 가장 많이 성공(30/30), 200k 는 placement error 가 가장 작음. seed 당 10 trial 이라 1회 차이는 noise 범위. 마지막 행의 demo 100개 policy 는 데이터 범위의 효과를 보기 위한 비교군.

&#91;embedded content: 넓은 범위 rollout 2회 · 회당 10 trial, 두 policy 에 같은 배치 · 2026-10-22\]

demo 100개 policy 의 실패(빨간 ×)는 모두 회색 띠 바깥. demo 190개 policy 는 띠 안팎 20개 모두 성공.

## 실패 분석과 wrist yaw

최종 policy 의 남은 실패는 큐브를 쥐고 들다가 미끄러지는 경우. policy 는 큐브 yaw 만큼 wrist 를 돌리지 못하고(wrist yaw 추종 부족), 쥐는 순간 20° 넘게 어긋나면 떨어뜨리기 쉬움. 아래 수치는 rollout 때 남긴 step 로그(매 step 의 joint, TCP, 큐브 pose)에서 구하고 별도 분석으로 재검증함.

### demo 범위 밖에서는 aiming 이 무너짐

- demo 100개 policy 의 넓은 범위 실패 7회는 모두 grasp 전에 끝남. 6회는 grasp 가 한 번도 걸리지 않았고, 1회는 0.3초만 모서리를 집음.
- 처음 닿는 순간 TCP 가 큐브에서 33–53 mm 떨어져 있었음(성공한 grasp 는 중앙값 6 mm). 열린 finger 가 큐브를 밀거나 넘어뜨렸고, 2회는 큐브 없이 jaw 만 닫힌 채 종료.
- 7회 중 6회는 앞뒤 오차가 기준점 쪽을 향함. policy 가 학습한 범위 안쪽으로 끌려간다는 뜻. 같은 배치에서 최종 policy 는 3–9 mm 안에서 쥐고 들어 올림.

### demo 190개 policy 의 실패

- **큐브 yaw −26.6° 배치**: 100k·200k 는 실패, 140k 는 성공. 세 checkpoint 의 접근 높이·위치·wrist 각도는 같았고, jaw 는 큐브 면에서 약 30° 어긋나 있었음. 차이는 쥔 뒤 큐브가 pad 사이에서 면으로 돌아앉았는지 여부. 200k 는 들어 올린 뒤 운반을 시작한 직후 놓침.
- **100k 의 나머지 두 번**: 한 번은 닫으면서 큐브를 23 mm 밀고 들다가 빠짐. 다른 한 번은 placement error 35.2 mm 로 판정 한계(35 mm)를 겨우 넘음.
- **fold boundary 배치**: 큐브 yaw −44.2° 배치는 demo 100개 policy 가 모든 checkpoint 에서 떨어뜨렸고, 최종 policy 는 100k·140k·200k 모두 성공. 둘 다 대각선으로 모서리를 쥔. 다만 이런 배치는 하나뿐이라 결론으로 보기는 어려움.

### wrist yaw 추종

- **demo**: wrist 회전이 큐브 yaw 를 그대로 따라감(큐브 yaw 대비 wrist 각도의 기울기 −1.00, 크기 1 = 완전 추종). 큐브는 90° 대칭이라 yaw 는 ±45° 로 fold 해서 봄.
- **policy**: 쥐는 순간 wrist 회전의 기울기가 모든 rollout 에서 −0.62 \~ +0.05. 큐브를 따라가지 않고 평균 각도 쪽으로 끌림.
- **fold boundary oversampling 의 효과는 확인되지 않음**: 기본 범위의 같은 배치 20쌍에서 쥐는 순간 어긋난 각도(misalignment)는 최종 15.8°, demo 100개 policy 11.2° 로 오히려 큼(차이 95 % CI +1.9 \~ +7.3°). 넓은 범위에서는 15.0° 대 19.6° 로 작았지만 유의하지 않음.
- **pad 가 일부 맞춤**: 닫히면서 pad 가 큐브를 jaw 쪽으로 약 20 % 돌려 앉힘. 1초 뒤 misalignment 평균이 15.1° 에서 10.5° 로 감소.

쥐는 순간의 misalignment 가 크면 실패 비율이 올라감. 아래는 두 policy 의 rollout 18회에서 grasp 가 걸린 174회를 나눈 결과. 실패에는 낙하 외에 placement error 1회와 모서리만 집었다 빠진 1회가 포함됨.

| misalignment | 실패 / grasp |
| --- | --- |
| 10° 이하 | 0 / 71 |
| 10–20° | 1 / 64 |
| 20–30° | 3 / 15 |
| 30–45° | 7 / 24 |

실패가 몇 개 배치에 몰려 있어(같은 배치를 여러 checkpoint 가 되풀이) 독립 표본은 아님. 그래도 방향은 분명해, wrist yaw 추종이 다음으로 고칠 부분.

## Rollout 장면

최종 policy 가 demo 100개의 범위 밖(기준점에서 앞뒤 +99 mm)에 놓인 큐브를 집어 놓는 장면. 각 칸의 왼쪽은 exterior camera, 오른쪽은 wrist camera.

&#91;image: 최종 policy: approach, close, 운반, marker 위에 놓기\]

같은 trial 의 움직임(sim 9–22.5초 구간).

&#91;image: 최종 policy 성공 장면 애니메이션\]

같은 큐브 배치에서 demo 100개 policy 는 큐브에서 43 mm 빗나간 곳에서 jaw 를 닫고, 빈 gripper 로 marker 까지 이동(위 줄). 최종 policy 는 큐브를 쥐고 9 mm 오차로 놓음(아래 줄).

&#91;image: 같은 배치: demo 100개 policy 는 빗나가고 최종 policy 는 성공\]

모든 trial 의 전체 video 와 판정 결과는 rollout run 별로 보관.

## 한계와 다음 단계

다음으로 고칠 부분은 wrist yaw 추종. 나머지는 검증 범위의 한계.

- **wrist yaw 추종**: 후보를 아래 순서로 검토.
  1. wrist image 에서 큐브 각도 단서가 약한지 확인(해상도, crop).
  2. approach 구간의 closed-loop 비중 늘리기. 현재는 action chunk 절반(약 1.7초)을 피드백 없이 실행.
  3. grasp robustness: pad friction, close force, grasp 높이.
- **좌우 범위**: 큐브 좌우는 ±60 mm 까지만 학습·평가. 다른 큐브와 marker 배치를 바꾸지 않으면 더 넓힐 수 없음.
- **평가 규모**: seed 당 10 trial 이라 성공 1–2개 차이는 noise 범위. 넓은 범위의 20/20 대 13/20 은 같은 배치의 짝 비교라 신뢰할 만하지만, 실패 원인 통계는 몇 개 배치에 몰려 있음.
- **Ablation 은 안 함**: gripper 설정, close command, 기록 방식을 한꺼번에 정함. 각각의 기여는 ACT rollout 으로 따로 측정하지 않음.
- **검증 범위**: sim, 물체 1종(35 mm 큐브), task 1개만 검증. 다른 크기·질량의 물체와 실물은 미검증.
- **아직 physical grasp 로 재수집하지 않은 것**: wrist camera 1대 ACT 와 다중 task VLA(GR00T). 전환 준비는 됨.
- **사람 teleop 확인**: 실물 leader 장치로 teleop 할 때 physical grasp 에서 wrist 가 튀지 않는지는 사용자 실측 필요.

ACT physical contact grasp 최종 보고 문서의 rollout 장면 (2026-10-25 저장)

원본은 outputs/rollout_videos/<run>/trial_NN_*.mp4 (960x404, 10 frame = sim 1 s).
GIF 는 2 frame 마다 1장, 640x269, 150 ms/장. 위쪽 라벨의 checkpoint 이름을
final (190 demos) / baseline (100 demos) 로 바꿈.
  final    = outputs/act_red_left_190_phys/checkpoints/200000
  baseline = outputs/act_red_left_100_phys_long/checkpoints/200000

파일                          원본 run / trial               frame    내용
success_final.gif             e190_new200k_w211 trial 04     90-225   성공: 기본 범위 밖(+99 mm) 큐브를 집어 marker 위에 놓음(오차 9 mm)
success_final_steps.png       e190_new200k_w211 trial 04     4 장면   위 trial 의 approach / close / 운반 / 놓기 정지 장면
fail_baseline_miss.gif        e190_old200k_w211 trial 04     90-225   실패: 같은 배치에서 큐브 43 mm 옆에서 jaw 를 닫고 빈 gripper 로 이동
compare_same_placement.gif    위 두 trial (위 baseline, 아래 final)  90-225   같은 배치 비교, 시간 맞춤
compare_same_placement.png    위 두 trial                    정지 장면 같은 배치 비교 정지 장면
fail_baseline_push.gif        e190_old200k_w251 trial 05     120-236  실패: 넓은 범위(+93 mm), 41 mm 앞에 내려와 큐브를 약 15 cm 밀어내고 넘어뜨림
fail_final_drop.gif           e190_new200k_s131 trial 09     120-232  실패: 큐브 yaw -26.6 deg, 약 18 cm 들어 올린 뒤 운반 시작 직후 떨어뜨림

문서에 들어간 것: success_final.gif, success_final_steps.png, compare_same_placement.png (두 문서 모두),
fail_final_drop.gif, fail_baseline_push.gif (구조 개요판; 기존 문서엔 *_raw 버전).
fail_baseline_miss.gif, compare_same_placement.gif 는 파일로만 저장(문서에는 같은 장면의 정지 이미지).

*_raw.gif = 같은 장면, 라벨에 원래 checkpoint 이름 그대로(기존 최종 보고 문서에 들어간 버전).
  success_final_raw.gif, fail_final_drop_raw.gif, fail_baseline_push_raw.gif
2026-10-25: 실패 GIF 2개를 기존 최종 보고 문서에도 추가(raw 버전).

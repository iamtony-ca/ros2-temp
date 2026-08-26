
1단계 — 프롬프트 스윕 (분할만, 눈으로 먼저)

cd /isaac-sim/volume/spatial_manipulation_ws/src/vision
source envs/env.sh

# 왼쪽 이미지만 모은다
mkdir -p assets/real_imgs_zedx
for f in runs/real01/frame_*/left.png; do cp $f assets/real_imgs_zedx/$(basename $(dirname $f)).png; done
# (선택) 몸체 외관 라벨 — {"frame_0000":"black", …}  black|orange|clear|twotone
#   $EDITOR assets/real_imgs_zedx/appearance.json

envs/seg_sam3/bin/python tools/sam3_prompt_sweep.py \
    --imgs assets/real_imgs_zedx --out runs/psweep_zedx \
    --prompts-json assets/prompts/real_finalists.json --target full,flange --confidence 0.05

real_finalists.json 이 두 시트와 정확히 일치함을 확인했다 — full 11 / flange 12(같은 문장 중복 하나 제거), 누락·여분 0.

볼 순서: sheets/perfect__{full,flange}.png → sheets/perfect/<target>__NN__*.png → 전수는 matrix__*.png

2단계 — 전 파이프라인 한 번에

envs/pose/bin/python tools/run_group_a.py \
    --in runs/real01 --out runs/real01_A \
    --mode all --preset n30black \
    --text-prompt "<1단계 full 최선>" --text-conf 0.10 \
    --text-prompt-flange "<1단계 flange 최선>" \
    --note "1차" --true-distance-mm 280

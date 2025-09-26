좋아요. 현상 정리부터 할게요.

* **Depth Anything V2의 기본 출력은 ‘상대(상대적) 깊이’**예요. 즉 장면 내에서 가깝/멀다의 순위는 잘 맞추지만, 절대 거리(미터 단위)가 아니고, 도메인/텍스처/조명에 따라 흔히 오판(특히 유리·반사·하늘·텍스처 적은 벽)이 납니다. 이건 모델 구조적 한계 + 입력 전처리/리사이즈/후처리 영향이 섞여 있어요. 리포는 기본 입력 크기 518을 쓰고 더 키우면 디테일이 좋아진다고 명시합니다. 또 metric depth용 파인튜닝 체크포인트도 따로 제공해요. ([GitHub][1])
* V2는 다양한 스케일(viTs/viTb/viTl/viTg)을 제공하고, 큰 모델과 큰 입력이 보통 더 안정적입니다. ([GitHub][1])
* 논문/프로젝트 페이지에도 “상대깊이→metric깊이”로 파인튜닝한 모델이 따로 있다고 적혀있습니다. 로보틱스처럼 절대스케일이 필요한 경우엔 반드시 metric 버전을 고려하세요. ([arXiv][2])

---

# 개선 체크리스트 (현 코드 기준 빠르게 적용)

1. **입력 해상도 올리기(가장 큰 개선/쉽다)**
   `--input-size`를 672~896 정도로 키워보세요(메모리 허용 범위). V2 리드미가 입력크기 증가 시 결과가 더 세밀해진다고 안내합니다. ([GitHub][1])

2. **더 큰 인코더 써보기**
   `--encoder vitl`→`vitg`(가능하면). 장면 복잡/원거리 디테일에서 안정도가 올라갑니다. ([GitHub][1])

3. **출력 정규화 방식을 ‘퍼센타일’로 변경**
   현재 코드는 `min/max`로 0~255 스케일링을 하는데, 한두 개 이상치가 다이내믹레인지를 뭉개는 경우가 있습니다. 1~99%(또는 2~98%) 퍼센타일 클리핑 후 정규화하면 계조가 더 안정적입니다. (코드 스니펫 아래 참고)

4. **TTA(Test-Time Augmentation) 간단 적용**
   수평 flip 추론 1회 추가 후 뒤집어 되돌려 평균하면 경계/박스형 구조에서 오판이 줄 때가 많습니다. 비용 대비 효과가 좋아요.

5. **가이드 필터 기반 경계 보정(후처리)**
   깊이맵을 컬러 이미지로 가이드하는 fast guided filter / joint bilateral filter를 1~2회 돌리면 경계가 날카로워지고 얇은 구조가 복원됩니다(특히 텍스처 적은 벽). OpenCV의 `ximgproc.guidedFilter`가 있으면 활용하세요.

6. **16-bit depth로 저장**
   시각화용 컬러맵 PNG만 저장하면 미세 차이가 날아갑니다. 8-bit 컬러 대신 **그레이스케일 16-bit PNG**(또는 float16 npy)를 함께 저장해 후단 알고리즘(예: nvblox 프리프로세싱, 점점이 매칭 스케일링)에 쓰세요. 커뮤니티 구현 예시도 있습니다. ([GitHub][3])

7. **metric depth가 필요하면 ‘metric 체크포인트’ 사용**
   실외/실내 전용 metric 모델을 공개해두었습니다(Transformers/HF 버전 포함). 로봇 적용 시 스케일 안정성이 훨씬 낫습니다. 다만 카메라/도메인 차이가 크면 소량 파인튜닝이나 간단한 스케일 보정(바닥 평면/카메라 높이 이용)도 고려하세요. ([GitHub][1])

8. **장면별 스케일 보정(옵션)**
   단안의 고질적 스케일 불확실성을 줄이려면

* 바닥 평면(추정) + 카메라 높이로 선형 스케일 맞춤,
* sparse LiDAR/ SfM 포인트로 리스케일(최근 제안들도 있어요),
  방법이 유효합니다. ([arXiv][4])

---

# 당신 코드(run.py)용 패치 제안

아래는 **퍼센타일 정규화 + flip TTA + 16-bit 저장(옵션)**을 넣은 최소 패치예요. 성능/속도 밸런스용으로 가볍게 작성했습니다.

```python
# --- (중략) 기존 import 그대로 사용 ---
import argparse
import cv2
import glob
import matplotlib
import numpy as np
import os
import torch

from depth_anything_v2.dpt import DepthAnythingV2

def percentile_normalize(depth, lo=2, hi=98):
    lo_v, hi_v = np.percentile(depth, [lo, hi])
    depth = np.clip(depth, lo_v, hi_v)
    depth = (depth - lo_v) / max(hi_v - lo_v, 1e-8)
    return depth

def infer_with_tta(model, img_bgr, input_size):
    # 원본
    d0 = model.infer_image(img_bgr, input_size)
    # 수평 플립 TTA
    img_flip = cv2.flip(img_bgr, 1)
    d1 = model.infer_image(img_flip, input_size)
    d1 = cv2.flip(d1, 1)
    # 평균
    return 0.5 * (d0 + d1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Depth Anything V2')
    parser.add_argument('--img-path', type=str)
    parser.add_argument('--input-size', type=int, default=672)  # ↑ 기본 518 → 672 권장
    parser.add_argument('--outdir', type=str, default='./vis_depth')
    parser.add_argument('--encoder', type=str, default='vitl', choices=['vits', 'vitb', 'vitl', 'vitg'])
    parser.add_argument('--pred-only', dest='pred_only', action='store_true')
    parser.add_argument('--grayscale', dest='grayscale', action='store_true')
    parser.add_argument('--tta', action='store_true', help='use horizontal flip TTA')
    parser.add_argument('--save-16bit', action='store_true', help='also save 16-bit depth PNG (normalized)')
    args = parser.parse_args()

    DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

    model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
    }

    depth_anything = DepthAnythingV2(**model_configs[args.encoder])
    depth_anything.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{args.encoder}.pth', map_location='cpu'))
    depth_anything = depth_anything.to(DEVICE).eval()

    if os.path.isfile(args.img_path):
        filenames = open(args.img_path).read().splitlines() if args.img_path.endswith('txt') else [args.img_path]
    else:
        filenames = glob.glob(os.path.join(args.img_path, '**/*'), recursive=True)

    os.makedirs(args.outdir, exist_ok=True)
    cmap = matplotlib.colormaps.get_cmap('Spectral_r')

    for k, filename in enumerate(filenames):
        if not os.path.isfile(filename):
            continue
        print(f'Progress {k+1}/{len(filenames)}: {filename}')
        raw_image = cv2.imread(filename)  # BGR
        if raw_image is None:
            continue

        depth = infer_with_tta(depth_anything, raw_image, args.input_size) if args.tta \
                else depth_anything.infer_image(raw_image, args.input_size)

        # 퍼센타일 정규화로 안정화
        depth01 = percentile_normalize(depth, lo=2, hi=98)

        # 시각화용 8-bit
        vis8 = (depth01 * 255.0).astype(np.uint8)
        if args.grayscale:
            vis3 = np.repeat(vis8[..., None], 3, axis=-1)
        else:
            vis3 = (cmap(vis8)[:, :, :3] * 255).astype(np.uint8)[:, :, ::-1]  # RGB→BGR

        # 병합/저장
        out_png = os.path.join(args.outdir, os.path.splitext(os.path.basename(filename))[0] + '.png')
        if args.pred_only:
            cv2.imwrite(out_png, vis3)
        else:
            split_region = np.ones((raw_image.shape[0], 50, 3), dtype=np.uint8) * 255
            combined = cv2.hconcat([raw_image, split_region, vis3])
            cv2.imwrite(out_png, combined)

        # 16-bit 저장(옵션, 후처리/후단 알고리즘용)
        if args.save_16bit:
            out_png16 = os.path.join(args.outdir, os.path.splitext(os.path.basename(filename))[0] + '_16bit.png')
            cv2.imwrite(out_png16, (depth01 * 65535.0).astype(np.uint16))
```

> 메모: 리드미 예시처럼 OpenCV 경로를 그대로 쓰는 걸 권장합니다(업샘플러 차이로 HF-Pillow 경로와 약간 다를 수 있음). ([GitHub][1])

---

# Metric depth로 바로 가고 싶다면

* **체크포인트 교체**: 실외면 *Depth-Anything-V2-Metric-Outdoor*, 실내면 대응 모델을 사용하세요(HF/Transformers에도 등록). 절대스케일 품질이 훨씬 낫습니다. ([Hugging Face][5])
* 그래도 카메라/도메인 갭이 크면, **바닥 평면 + 카메라 높이**로 스케일 1D 보정(선형)하거나, **저해상도 LiDAR/SfM 포인트**로 스케일만 리스케일하는 접근도 실전에서 효과적입니다. ([arXiv][4])

---

# 로보틱스 파이프라인 팁(ROS/SLAM/nvblox와 함께 쓸 때)

* 컬러맵 PNG 대신 **16-bit depth** 또는 **float npy**를 publish/저장해 후단에서 쓸 것.
* 프레임간 일관성은 V2 자체가 이미지 단발 추론이라 한계가 있으니, **시각오도메트리(ORB-SLAM/DSO 등)로 스케일과 스무딩을 결합**하거나, Video-Depth 전용 모델을 참고하세요(공식도 비디오 데모 목적 언급). ([GitHub][1])

---

원인이 될 수 있는 **특정 케이스(유리, 하늘, 강한 반사, 텍스처리스 벽, 극저조도)**가 있다면 샘플 몇 장을 공유해주면, 위 옵션 조합 중 어떤 게 가장 효과적일지 바로 찍어서 튜닝 조합을 제안해줄게요.

[1]: https://github.com/DepthAnything/Depth-Anything-V2 "GitHub - DepthAnything/Depth-Anything-V2: [NeurIPS 2024] Depth Anything V2. A More Capable Foundation Model for Monocular Depth Estimation"
[2]: https://arxiv.org/abs/2406.09414?utm_source=chatgpt.com "Depth Anything V2"
[3]: https://github.com/petermg/Depth-Anything-V2_16bitPNG_Touchly1?utm_source=chatgpt.com "petermg/Depth-Anything-V2_16bitPNG_Touchly1"
[4]: https://arxiv.org/abs/2412.14103?utm_source=chatgpt.com "A Simple yet Effective Test-Time Adaptation for Zero-Shot ..."
[5]: https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Outdoor-Large-hf?utm_source=chatgpt.com "depth-anything/Depth-Anything-V2-Metric-Outdoor-Large-hf"

######################  

좋습니다. 아래는 **완전한 실행 스크립트**예요.
요구하신 대로 입력 이미지를 **세로축 기준으로 절반(가로를 1/2) 잘라, 왼쪽 절반만**을 모델 입력으로 사용합니다.
또한 퍼센타일 정규화, 반사(유리/디스플레이) 마스크, 평면 보간, 가이드 필터, TTA, 16-bit 저장 옵션을 포함했습니다. (PEP 8 스타일)

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Depth-Anything-V2 inference script with:
- Left-half crop (use only the left side of the image)
- Percentile normalization
- Specular/reflective candidate mask (glass/displays)
- Planar fill for low-confidence regions
- Guided/joint-bilateral smoothing
- Optional horizontal flip TTA
- Optional 16-bit depth saving

Tested with: Python 3.10+, OpenCV 4.x, torch 2.x
"""

import argparse
import glob
import os
from typing import List, Tuple

import cv2
import matplotlib
import numpy as np
import torch

from depth_anything_v2.dpt import DepthAnythingV2


# ----------------------------- Utils ----------------------------- #

def list_image_files(path: str) -> List[str]:
    """Return a list of image file paths (or from .txt file)."""
    if os.path.isfile(path):
        if path.lower().endswith(".txt"):
            with open(path, "r") as f:
                names = [ln.strip() for ln in f.readlines()]
            return [p for p in names if os.path.isfile(p)]
        return [path]
    # directory (recursive)
    exts = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp")
    files = glob.glob(os.path.join(path, "**/*"), recursive=True)
    return [f for f in files if os.path.splitext(f)[1].lower() in exts]


def percentile_normalize(depth: np.ndarray, lo: float = 2, hi: float = 98) -> np.ndarray:
    """Percentile-based normalization to [0,1] with outlier suppression."""
    d = depth.astype(np.float32)
    lo_v, hi_v = np.percentile(d, [lo, hi])
    d = np.clip(d, lo_v, hi_v)
    scale = max(hi_v - lo_v, 1e-8)
    return (d - lo_v) / scale


def specular_mask_bgr(img_bgr: np.ndarray) -> np.ndarray:
    """
    Build a specular/reflective candidate mask (0/255) using
    brightness, saturation and edge cues.
    """
    img_u8 = img_bgr if img_bgr.dtype == np.uint8 else np.clip(img_bgr, 0, 255).astype(np.uint8)
    hsv = cv2.cvtColor(img_u8, cv2.COLOR_BGR2HSV)
    s = hsv[..., 1].astype(np.float32) / 255.0
    v = hsv[..., 2].astype(np.float32) / 255.0

    # bright + low saturation → candidate highlights
    bright = (v > 0.92) & (s < 0.25)

    # strengthen with edge information
    gray = cv2.cvtColor(img_u8, cv2.COLOR_BGR2GRAY)
    edges = cv2.Laplacian(gray, ddepth=cv2.CV_32F)
    edges = np.abs(edges)
    if edges.max() > 0:
        edges /= edges.max()
    edgey = edges > 0.25

    mask = ((bright | edgey).astype(np.uint8)) * 255
    # open → dilate to reduce spurious small holes and connect highlight blobs
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return mask


def guided_smooth(depth01: np.ndarray, guide_bgr: np.ndarray,
                  radius: int = 4, eps: float = 1e-3) -> np.ndarray:
    """
    Edge-aware smoothing of depth using the RGB as a guide.
    Tries ximgproc.guidedFilter, falls back to bilateral filter.
    """
    d = depth01.astype(np.float32)
    try:
        import cv2.ximgproc as xip  # type: ignore
        gf = xip.guidedFilter(guide=guide_bgr, src=d, radius=radius, eps=eps)
        return gf.astype(np.float32)
    except Exception:
        # joint bilateral (approx) fallback
        d8 = np.clip(d * 255.0, 0, 255).astype(np.uint8)
        bf = cv2.bilateralFilter(d8, d=7, sigmaColor=25, sigmaSpace=7)
        return (bf.astype(np.float32) / 255.0)


def fill_planar(depth01: np.ndarray, invalid_mask_u8: np.ndarray, iters: int = 2) -> np.ndarray:
    """
    Simple neighborhood diffusion + median-based fill for invalid regions.
    Enough for reflective/transparent patches to avoid holes.
    """
    d = depth01.astype(np.float32).copy()
    inv = invalid_mask_u8 > 0
    if not np.any(inv):
        return d

    kernel = np.ones((3, 3), np.uint8)
    for _ in range(iters):
        # 1) dilate numeric values (simple outward diffusion)
        dil = cv2.dilate(d, kernel)
        # 2) median smoothing for robustness
        med = cv2.medianBlur(np.clip(d * 255.0, 0, 255).astype(np.uint8), 5).astype(np.float32) / 255.0
        neigh = 0.5 * dil + 0.5 * med
        d[inv] = neigh[inv]
    return d


def infer_with_tta(model, img_bgr: np.ndarray, input_size: int) -> np.ndarray:
    """Single-frame inference with optional horizontal flip TTA."""
    d0 = model.infer_image(img_bgr, input_size)  # (H, W) float32
    img_flip = cv2.flip(img_bgr, 1)
    d1 = model.infer_image(img_flip, input_size)
    d1 = cv2.flip(d1, 1)
    return 0.5 * (d0 + d1)


def crop_left_half(img: np.ndarray) -> np.ndarray:
    """Cut image vertically in half and return the left half."""
    h, w = img.shape[:2]
    return img[:, :w // 2].copy()


# ----------------------------- Main ----------------------------- #

def build_model(encoder: str, device: str) -> DepthAnythingV2:
    model_cfgs = {
        "vits": {"encoder": "vits", "features": 64, "out_channels": [48, 96, 192, 384]},
        "vitb": {"encoder": "vitb", "features": 128, "out_channels": [96, 192, 384, 768]},
        "vitl": {"encoder": "vitl", "features": 256, "out_channels": [256, 512, 1024, 1024]},
        "vitg": {"encoder": "vitg", "features": 384, "out_channels": [1536, 1536, 1536, 1536]},
    }
    model = DepthAnythingV2(**model_cfgs[encoder])
    ckpt = f"checkpoints/depth_anything_v2_{encoder}.pth"
    state = torch.load(ckpt, map_location="cpu")
    model.load_state_dict(state)
    model = model.to(device).eval()
    return model


def visualize_depth(depth01: np.ndarray, grayscale: bool) -> np.ndarray:
    """Return BGR visualization image from [0,1] depth."""
    vis8 = np.clip(depth01 * 255.0, 0, 255).astype(np.uint8)
    if grayscale:
        return cv2.cvtColor(vis8, cv2.COLOR_GRAY2BGR)
    # consistent colormap (matplotlib Spectral_r), converted to BGR
    cmap = matplotlib.colormaps.get_cmap("Spectral_r")
    vis3 = (cmap(vis8)[:, :, :3] * 255.0).astype(np.uint8)[:, :, ::-1]
    return vis3


def process_one(
    model: DepthAnythingV2,
    filename: str,
    outdir: str,
    input_size: int,
    use_tta: bool,
    grayscale: bool,
    pred_only: bool,
    save_16bit: bool,
    save_mask: bool,
    crop_left: bool,
) -> None:
    base = os.path.splitext(os.path.basename(filename))[0]
    raw = cv2.imread(filename, cv2.IMREAD_COLOR)
    if raw is None:
        print(f"[WARN] Failed to read: {filename}")
        return

    # --- crop left half as requested ---
    if crop_left:
        raw = crop_left_half(raw)

    # --- inference ---
    depth = infer_with_tta(model, raw, input_size) if use_tta else model.infer_image(raw, input_size)

    # --- percentile normalization to [0,1] ---
    depth01 = percentile_normalize(depth, lo=2, hi=98)

    # --- specular candidate mask (glass / display) ---
    spec_mask = specular_mask_bgr(raw)  # 0/255
    spec_bool = spec_mask > 0

    # --- planar fill for masked (low-confidence) regions ---
    depth_filled = fill_planar(depth01, spec_mask, iters=2)

    # --- guided smoothing (edge-aware) ---
    depth_for_smooth = depth_filled.copy()
    # prevent over-smoothing inside masked areas (keep them conservative)
    depth_for_smooth[spec_bool] = depth01[spec_bool]
    depth_smooth = guided_smooth(depth_for_smooth, raw, radius=4, eps=1e-3)

    # --- final composition (conservative inside mask) ---
    final01 = depth_smooth
    final01[spec_bool] = np.minimum(final01[spec_bool], depth_filled[spec_bool])

    # --- save outputs ---
    os.makedirs(outdir, exist_ok=True)
    vis = visualize_depth(final01, grayscale=grayscale)

    if pred_only:
        cv2.imwrite(os.path.join(outdir, f"{base}_depth.png"), vis)
    else:
        split = np.ones((raw.shape[0], 50, 3), dtype=np.uint8) * 255
        concat = cv2.hconcat([raw, split, vis])
        cv2.imwrite(os.path.join(outdir, f"{base}_depth.png"), concat)

    if save_mask:
        cv2.imwrite(os.path.join(outdir, f"{base}_mask.png"), spec_mask)

    if save_16bit:
        cv2.imwrite(
            os.path.join(outdir, f"{base}_depth16.png"),
            np.clip(final01 * 65535.0, 0, 65535).astype(np.uint16),
        )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Depth Anything V2 with robust post-processing")
    p.add_argument("--img-path", type=str, required=True,
                   help="Image path, directory, or a .txt file listing image paths")
    p.add_argument("--input-size", type=int, default=672,
                   help="Model input size (longer side). Try 672~896 for better details")
    p.add_argument("--outdir", type=str, default="./vis_depth")

    p.add_argument("--encoder", type=str, default="vitl",
                   choices=["vits", "vitb", "vitl", "vitg"])

    p.add_argument("--pred-only", dest="pred_only", action="store_true",
                   help="Save only prediction (no side-by-side concat)")
    p.add_argument("--grayscale", dest="grayscale", action="store_true",
                   help="Use grayscale visualization instead of color map")

    p.add_argument("--tta", action="store_true", help="Enable horizontal flip TTA")
    p.add_argument("--save-16bit", action="store_true",
                   help="Also save 16-bit depth PNG (normalized)")
    p.add_argument("--save-mask", action="store_true",
                   help="Save specular/reflective candidate mask as PNG")
    p.add_argument("--no-crop-left", action="store_true",
                   help="Disable left-half cropping (default: use left half)")

    return p.parse_args()


def main() -> None:
    args = parse_args()

    device = (
        "cuda" if torch.cuda.is_available()
        else "mps" if torch.backends.mps.is_available()
        else "cpu"
    )
    print(f"[INFO] Using device: {device}")

    model = build_model(args.encoder, device)
    files = list_image_files(args.img_path)

    if not files:
        print("[ERROR] No input images found.")
        return

    os.makedirs(args.outdir, exist_ok=True)

    for idx, f in enumerate(files, 1):
        print(f"[{idx}/{len(files)}] {f}")
        try:
            process_one(
                model=model,
                filename=f,
                outdir=args.outdir,
                input_size=args.input_size,
                use_tta=args.tta,
                grayscale=args.grayscale,
                pred_only=args.pred_only,
                save_16bit=args.save_16bit,
                save_mask=args.save_mask,
                crop_left=not args.no_crop_left,  # default: crop left half
            )
        except Exception as e:
            print(f"[WARN] Failed on {f}: {e}")


if __name__ == "__main__":
    main()
```

### 사용 예

```bash
python3 run_depth_v2_left_half.py \
  --img-path /path/to/images \
  --encoder vitl \
  --input-size 672 \
  --tta \
  --save-16bit \
  --save-mask
```

* 기본은 **왼쪽 절반 크롭 사용**입니다. 원본 전체로 돌리고 싶으면 `--no-crop-left`를 주세요.
* 결과물:

  * `*_depth.png` : 시각화 결과(기본은 raw|bar|depth 합쳐서 저장, `--pred-only` 시 depth만 저장)
  * `*_depth16.png` : 16-bit 정규화 깊이(옵션)
  * `*_mask.png` : 반사/투명 후보 마스크(옵션)

원하시면, 이걸 **ROS 2 노드**로 감싸서 `sensor_msgs/Image`(mono16 + mask) 퍼블리시하는 버전도 만들어 드릴게요.

##########################  


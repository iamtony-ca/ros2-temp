#!/usr/bin/env python3
"""ZED X 의 cx/cy 가 **코너 원점**인지 **OpenCV 픽셀 중심**인지 실측으로 판정한다.

    # Jetson 에서 (pyzed 필요)
    python3 tools/zedx_check_pp_convention.py --mode rectify     # ① 오프라인, 카메라만 연결
    python3 tools/zedx_check_pp_convention.py --mode cloud       # ② 점군 자기일관성

왜 0.5px 이 문제인가
    코너 원점은 픽셀 k 가 [k, k+1] 을 덮고 중심이 k+0.5, OpenCV 는 픽셀 k 의 중심이 정수 k 다.
    같은 광학중심이 두 규약에서 **정확히 0.5px 다르게** 적힌다. 0.35m 에서 0.481mm/px 이므로
    **0.24mm** 의 계통 편향이다.
    ⚠️ 단, `cx_left == cx_right` 이므로 **시차(depth)에는 영향이 없다.** 절대 pose 의
       횡방향 편향만 생기고, hand-eye 를 같은 파이프라인으로 잡으면 상당 부분 상쇄된다.
       → **급하지 않다.** 다만 한 번 확정해 두면 영원히 안 헷갈린다.

판정
    ① rectify : ZED 의 **raw** 캘리브레이션(K,D,R,T)을 `cv2.stereoRectify` 에 그대로 넣어
                OpenCV 가 내놓는 새 cx(P1[0,2])와 ZED 의 rectified cx 를 비교한다.
                OpenCV 의 출력은 **정의상 OpenCV 규약**이다.
                차이 ≈0.00 → ZED 도 OpenCV | 차이 ≈0.50 → ZED 는 코너 원점
                ⚠️ ZED 가 자체 정류 알고리즘을 쓰면 3px 이상 벌어질 수 있다 → 그러면 ②로.
    ② cloud   : SDK 점군은 SDK 자신의 규약으로 역투영된 값이다.
                `u_pred = fx·X/Z + cx` 를 **픽셀 인덱스**와 비교한다.
                잔차 중앙값 ≈0.0 → OpenCV | ≈+0.5 → 코너 원점
                ⚠️ depth 원점이 광학중심에서 상수 d 만큼 밀려 있으면 X/Z 가 왜곡된다 →
                   d 를 함께 스캔해 |d| 가 유의하면 경고한다.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np


def verdict(delta: float, what: str) -> str:
    if abs(delta) < 0.15:
        return f"  → {what}: **OpenCV(픽셀 중심)** 규약 (Δ={delta:+.4f})"
    if abs(delta - 0.5) < 0.15:
        return f"  → {what}: **코너 원점** 규약 (Δ={delta:+.4f})"
    if abs(delta + 0.5) < 0.15:
        return f"  → {what}: 코너 원점의 **반대 부호** (Δ={delta:+.4f}) — 부호 규약 확인 필요"
    return f"  ⚠️ {what}: 판정 불가 (Δ={delta:+.4f}) — 0.0/±0.5 어디에도 안 붙는다"


def mode_rectify(res_key: str) -> int:
    import cv2
    import pyzed.sl as sl

    z = sl.Camera()
    p = sl.InitParameters()
    p.camera_resolution = getattr(sl.RESOLUTION, res_key)
    p.depth_mode = sl.DEPTH_MODE.NONE
    if z.open(p) != sl.ERROR_CODE.SUCCESS:
        print("❌ 카메라 open 실패", file=sys.stderr)
        return 2
    cc = z.get_camera_information().camera_configuration
    W, H = cc.resolution.width, cc.resolution.height
    raw, rect = cc.calibration_parameters_raw, cc.calibration_parameters

    def K(c):
        return np.array([[c.fx, 0, c.cx], [0, c.fy, c.cy], [0, 0, 1]], float)

    # ZED 의 disto 는 (k1 k2 p1 p2 k3 k4 k5 k6 ...) — OpenCV rational 순서와 같다
    def D(c):
        d = np.asarray(c.disto, float).ravel()
        return d[:8].reshape(1, 8)

    T = np.asarray(raw.stereo_transform.get_translation().get(), float).ravel()[:3]
    R = np.asarray(raw.stereo_transform.get_rotation_matrix().r, float).reshape(3, 3)
    z.close()

    R1, R2, P1, P2, Q, *_ = cv2.stereoRectify(
        K(raw.left_cam), D(raw.left_cam), K(raw.right_cam), D(raw.right_cam),
        (W, H), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0, newImageSize=(W, H))

    print(f"해상도            {W}x{H}")
    print(f"ZED  rectified    fx {rect.left_cam.fx:.5f}  cx {rect.left_cam.cx:.5f}  cy {rect.left_cam.cy:.5f}")
    print(f"cv2.stereoRectify fx {P1[0,0]:.5f}  cx {P1[0,2]:.5f}  cy {P1[1,2]:.5f}")
    if abs(P1[0, 0] - rect.left_cam.fx) > 2.0:
        print(f"⚠️ fx 가 {abs(P1[0,0]-rect.left_cam.fx):.2f}px 다르다 — ZED 가 자체 정류를 쓴다."
              f" 이 모드로는 판정 불가. --mode cloud 를 쓸 것.")
        return 1
    print(verdict(rect.left_cam.cx - P1[0, 2], "cx"))
    print(verdict(rect.left_cam.cy - P1[1, 2], "cy"))
    return 0


def mode_cloud(res_key: str, n_frames: int, zmin: float, zmax: float) -> int:
    import pyzed.sl as sl

    z = sl.Camera()
    p = sl.InitParameters()
    p.camera_resolution = getattr(sl.RESOLUTION, res_key)
    p.depth_mode = sl.DEPTH_MODE.NEURAL
    p.coordinate_units = sl.UNIT.MILLIMETER
    p.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE      # X→우, Y→하, Z→전방 (OpenCV/BOP 와 동일)
    if z.open(p) != sl.ERROR_CODE.SUCCESS:
        print("❌ 카메라 open 실패", file=sys.stderr)
        return 2
    cc = z.get_camera_information().camera_configuration
    c = cc.calibration_parameters.left_cam
    W, H = cc.resolution.width, cc.resolution.height
    uu, vv = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))

    ru, rv, Xs, Ys, Zs = [], [], [], [], []
    m = sl.Mat()
    got = 0
    for _ in range(n_frames * 10):
        if z.grab() != sl.ERROR_CODE.SUCCESS:
            continue
        z.retrieve_measure(m, sl.MEASURE.XYZ)
        a = m.get_data()[:, :, :3].astype(np.float64)
        X, Y, Z = a[:, :, 0], a[:, :, 1], a[:, :, 2]
        ok = np.isfinite(X) & np.isfinite(Z) & (Z > zmin) & (Z < zmax)
        # 화면 가장자리는 정류 보간 오차가 커서 제외
        ok[: H // 8, :] = ok[-H // 8:, :] = ok[:, : W // 8] = ok[:, -W // 8:] = False
        if ok.sum() < 5000:
            continue
        idx = np.flatnonzero(ok.ravel())
        idx = np.random.default_rng(0).choice(idx, size=min(40000, idx.size), replace=False)
        Xf, Yf, Zf = X.ravel()[idx], Y.ravel()[idx], Z.ravel()[idx]
        ru.append(c.fx * Xf / Zf + c.cx - uu.ravel()[idx])
        rv.append(c.fy * Yf / Zf + c.cy - vv.ravel()[idx])
        Xs.append(Xf); Ys.append(Yf); Zs.append(Zf)
        got += 1
        if got >= n_frames:
            break
    z.close()
    if not got:
        print("❌ 유효 depth 프레임을 못 얻었다 — 텍스처 있는 장면을 0.5~2m 에 두고 다시.", file=sys.stderr)
        return 2

    ru, rv = np.concatenate(ru), np.concatenate(rv)
    X, Y, Z = np.concatenate(Xs), np.concatenate(Ys), np.concatenate(Zs)
    print(f"프레임 {got} · 표본 {ru.size:,} · Z {Z.min():.0f}~{Z.max():.0f}mm")
    print(f"잔차 u  중앙 {np.median(ru):+.4f}  IQR {np.subtract(*np.percentile(ru,[75,25])):.4f}")
    print(f"잔차 v  중앙 {np.median(rv):+.4f}  IQR {np.subtract(*np.percentile(rv,[75,25])):.4f}")

    # depth 원점 오프셋 d 스캔 — |d| 가 크면 위 잔차를 믿으면 안 된다
    best = min(((float(np.subtract(*np.percentile(c.fx * X / (Z - d) + c.cx - (c.fx * X / Z + c.cx - ru),
                                                 [75, 25]))), d) for d in np.arange(-60, 61, 2.0)))
    if abs(best[1]) > 6.0:
        print(f"⚠️ depth 원점 오프셋 d≈{best[1]:+.0f}mm 로 추정된다 — 잔차가 오염됐을 수 있다."
              f" --mode rectify 결과와 대조할 것.")
    print(verdict(float(np.median(ru)), "cx"))
    print(verdict(float(np.median(rv)), "cy"))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="ZED cx/cy 반픽셀 규약 판정")
    ap.add_argument("--mode", default="rectify", choices=["rectify", "cloud"])
    ap.add_argument("--resolution", default="HD1200")
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--zmin", type=float, default=300.0)
    ap.add_argument("--zmax", type=float, default=3000.0)
    a = ap.parse_args(argv)
    return mode_rectify(a.resolution) if a.mode == "rectify" else \
        mode_cloud(a.resolution, a.frames, a.zmin, a.zmax)


if __name__ == "__main__":
    raise SystemExit(main())

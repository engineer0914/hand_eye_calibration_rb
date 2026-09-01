#!/usr/bin/env python3
# coding: utf-8

"""
RB5-850 Eye-in-Hand calibration - checkerboard 180deg ambiguity fix

핵심:
- 7x5 inner-corner checkerboard는 180도 회전 대칭 문제가 생길 수 있음.
- 각 이미지마다 corner order를 normal / reversed 두 후보로 solvePnP.
- Hand-eye AX=XB에서 상대 회전 A와 B는 conjugate 관계이므로
  rotation angle magnitude가 같아야 함.
- 이 조건으로 2^(N-1) binary label을 전수 탐색하여 각 이미지의
  checkerboard orientation을 자동 정렬.
- 이후 OpenCV 5개 hand-eye solver 실행 + 고정 board consistency 검증.

출력:
    ^TCP T_CAM
즉 Camera frame -> TCP frame 변환.

실행:
python compute_in_hand_rb850_flipfix.py \
    --data-dir eye_hand_data/data20260901193246
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


np.set_printoptions(precision=8, suppress=True)


# ============================================================
# Transform utilities
# ============================================================

def make_H(Rm, t):
    H = np.eye(4, dtype=np.float64)
    H[:3, :3] = np.asarray(Rm, dtype=np.float64).reshape(3, 3)
    H[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return H


def invert_H(H):
    Rm = H[:3, :3]
    t = H[:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rm.T
    out[:3, 3] = -Rm.T @ t
    return out


def rotation_angle_deg(Rm):
    c = (np.trace(Rm) - 1.0) * 0.5
    c = np.clip(c, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


# ============================================================
# RB5-850 pose
# ============================================================

def rb_euler_to_R(rx, ry, rz):
    """
    RB Cartesian pose: Z-Y'-X'' Euler
    pose array order: [Rx, Ry, Rz]
    matrix: Rz @ Ry @ Rx
    angles are radians here.
    """
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([
        [1, 0, 0],
        [0, cx, -sx],
        [0, sx, cx]
    ], dtype=np.float64)

    Ry = np.array([
        [cy, 0, sy],
        [0, 1, 0],
        [-sy, 0, cy]
    ], dtype=np.float64)

    Rz = np.array([
        [cz, -sz, 0],
        [sz, cz, 0],
        [0, 0, 1]
    ], dtype=np.float64)

    return Rz @ Ry @ Rx


def pose_to_H_base_tcp(pose):
    x, y, z, rx, ry, rz = pose
    return make_H(
        rb_euler_to_R(rx, ry, rz),
        [x, y, z]
    )


def load_poses(path):
    poses = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            vals = [float(v) for v in line.split(",")]
            if len(vals) != 6:
                raise ValueError(
                    f"poses.txt line {line_no}: expected 6 values, got {len(vals)}"
                )
            poses.append(np.asarray(vals, dtype=np.float64))
    return poses


# ============================================================
# Data / camera
# ============================================================

def numeric_images(folder):
    out = []
    for p in Path(folder).glob("*.jpg"):
        m = re.fullmatch(r"(\d+)\.jpg", p.name)
        if m:
            out.append((int(m.group(1)), p))
    return sorted(out)


def latest_data_dir(root):
    root = Path(root)
    ds = [p for p in root.glob("data*") if p.is_dir()]
    if not ds:
        raise FileNotFoundError(f"No data* folder under {root}")
    return max(ds, key=lambda p: p.stat().st_mtime)


def get_rs_intrinsic(width=640, height=480, fps=30):
    if rs is None:
        raise RuntimeError(
            "pyrealsense2 unavailable. Use --fx --fy --cx --cy."
        )

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(
        rs.stream.color, width, height, rs.format.bgr8, fps
    )

    try:
        prof = pipe.start(cfg)
        vsp = prof.get_stream(
            rs.stream.color
        ).as_video_stream_profile()
        intr = vsp.get_intrinsics()

        K = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dist = np.asarray(
            intr.coeffs, dtype=np.float64
        ).reshape(-1, 1)

        return K, dist, {
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.ppx,
            "cy": intr.ppy,
            "dist": list(intr.coeffs),
            "model": str(intr.model),
        }
    finally:
        try:
            pipe.stop()
        except Exception:
            pass


def make_object_points(xx, yy, square):
    objp = np.zeros((xx * yy, 3), np.float64)
    objp[:, :2] = np.mgrid[0:xx, 0:yy].T.reshape(-1, 2)
    objp *= square
    return objp


def solve_pnp_from_corners(objp, corners, K, dist):
    ok, rvec, tvec = cv2.solvePnP(
        objp,
        corners,
        K,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None

    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            objp, corners, K, dist, rvec, tvec
        )

    Rm, _ = cv2.Rodrigues(rvec)
    proj, _ = cv2.projectPoints(
        objp, rvec, tvec, K, dist
    )

    e = (
        proj.reshape(-1, 2)
        - corners.reshape(-1, 2)
    )
    rmse = float(
        np.sqrt(np.mean(np.sum(e * e, axis=1)))
    )

    return {
        "R": Rm,
        "t": tvec.reshape(3, 1),
        "H": make_H(Rm, tvec),
        "rmse": rmse,
    }


def detect_two_board_candidates(
    image, objp, pattern, K, dist
):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    found, corners = cv2.findChessboardCorners(
        gray, pattern, flags
    )
    if not found:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS
        + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4,
    )
    corners = cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1), criteria
    )

    # Candidate 0: OpenCV order
    c0 = solve_pnp_from_corners(
        objp, corners, K, dist
    )

    # Candidate 1: exact 180deg board indexing flip
    c1 = solve_pnp_from_corners(
        objp, corners[::-1].copy(), K, dist
    )

    if c0 is None or c1 is None:
        return None

    return {
        "corners": corners,
        "candidates": [c0, c1],
    }


# ============================================================
# Binary checkerboard orientation optimization
# ============================================================

def relative_rotation_angle(H_a, H_b):
    """
    rotation angle of inv(H_a) @ H_b.
    """
    H_rel = invert_H(H_a) @ H_b
    return rotation_angle_deg(H_rel[:3, :3])


def build_pair_costs(G_list, C_candidates):
    """
    G_i = ^Base T_TCP_i
    C_i = ^Cam_i T_Board

    From fixed board:
        G_i X C_i = G_j X C_j

    Therefore:
        inv(G_j) G_i X = X C_j inv(C_i)

    A and B are conjugate -> same rotation angle.

    pair cost = (angle(A) - angle(B))^2
    """
    n = len(G_list)
    pair_costs = []

    for i in range(n):
        for j in range(i + 1, n):
            A = invert_H(G_list[j]) @ G_list[i]
            a_deg = rotation_angle_deg(A[:3, :3])

            costs = np.zeros((2, 2), dtype=np.float64)

            for si in (0, 1):
                for sj in (0, 1):
                    Ci = C_candidates[i][si]["H"]
                    Cj = C_candidates[j][sj]["H"]

                    B = Cj @ invert_H(Ci)
                    b_deg = rotation_angle_deg(
                        B[:3, :3]
                    )

                    diff = abs(a_deg - b_deg)

                    # robust-ish cap: one bad pair should not dominate
                    diff = min(diff, 90.0)
                    costs[si, sj] = diff * diff

            pair_costs.append(
                (i, j, a_deg, costs)
            )

    return pair_costs


def optimize_binary_labels(n, pair_costs):
    """
    Global all-flip produces equivalent physical target convention,
    so fix sample 0 label = 0.

    Search 2^(n-1) states exactly.
    """
    if n > 24:
        raise RuntimeError(
            f"N={n} too large for exact binary search in this script."
        )

    nstates = 1 << (n - 1)
    ids = np.arange(nstates, dtype=np.uint32)

    labels = np.zeros(
        (nstates, n), dtype=np.uint8
    )

    for k in range(1, n):
        labels[:, k] = (
            (ids >> (k - 1)) & 1
        ).astype(np.uint8)

    energy = np.zeros(nstates, dtype=np.float64)

    for i, j, _, costs in pair_costs:
        energy += costs[
            labels[:, i],
            labels[:, j]
        ]

    best_idx = int(np.argmin(energy))
    best_labels = labels[best_idx].copy()

    return best_labels, float(energy[best_idx])


def pair_angle_rms(labels, pair_costs):
    diffs = []
    for i, j, a_deg, costs in pair_costs:
        # recover squared capped difference
        d = math.sqrt(
            float(costs[
                int(labels[i]),
                int(labels[j])
            ])
        )
        diffs.append(d)

    return float(
        np.sqrt(np.mean(np.square(diffs)))
    )


# ============================================================
# Hand-eye + validation
# ============================================================

def validate_board(G_list, C_list, X_tcp_cam):
    Ys = [
        G @ X_tcp_cam @ C
        for G, C in zip(G_list, C_list)
    ]

    ts = np.asarray(
        [Y[:3, 3] for Y in Ys],
        dtype=np.float64,
    )
    Rs = np.asarray(
        [Y[:3, :3] for Y in Ys],
        dtype=np.float64,
    )

    tmean = np.mean(ts, axis=0)
    d = ts - tmean
    t_rms_mm = float(
        np.sqrt(
            np.mean(
                np.sum(d * d, axis=1)
            )
        ) * 1000.0
    )
    t_max_mm = float(
        np.max(np.linalg.norm(d, axis=1))
        * 1000.0
    )
    t_std_mm = np.std(
        ts, axis=0
    ) * 1000.0

    rot = Rotation.from_matrix(Rs)
    mean_rot = rot.mean()
    e_deg = np.degrees(
        (mean_rot.inv() * rot).magnitude()
    )

    r_rms_deg = float(
        np.sqrt(np.mean(e_deg ** 2))
    )
    r_max_deg = float(np.max(e_deg))

    return {
        "t_mean_m": tmean,
        "t_std_mm": t_std_mm,
        "t_rms_mm": t_rms_mm,
        "t_max_mm": t_max_mm,
        "r_rms_deg": r_rms_deg,
        "r_max_deg": r_max_deg,
    }


def methods():
    return {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }


# ============================================================
# Main
# ============================================================

def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--data-dir", default=None
    )
    ap.add_argument(
        "--config", default="config.yaml"
    )
    ap.add_argument(
        "--max-axis-offset-m",
        type=float,
        default=0.10
    )

    ap.add_argument("--fx", type=float)
    ap.add_argument("--fy", type=float)
    ap.add_argument("--cx", type=float)
    ap.add_argument("--cy", type=float)
    ap.add_argument(
        "--dist",
        nargs=5,
        type=float
    )

    args = ap.parse_args()

    if args.data_dir:
        data_dir = Path(args.data_dir)
    else:
        data_dir = latest_data_dir(
            Path("eye_hand_data")
        )

    with open(
        args.config, "r", encoding="utf-8"
    ) as f:
        cfg = yaml.safe_load(f)

    cc = cfg["checkerboard_args"]
    XX = int(cc["XX"])
    YY = int(cc["YY"])
    L = float(cc["L"])

    pattern = (XX, YY)
    objp = make_object_points(
        XX, YY, L
    )

    imgs = numeric_images(data_dir)
    poses = load_poses(
        data_dir / "poses.txt"
    )

    print("=" * 74)
    print("RB5-850 Hand-Eye: checkerboard 180deg ambiguity fixer")
    print("=" * 74)
    print("data:", data_dir)
    print("images:", len(imgs))
    print("poses :", len(poses))
    print(
        f"checkerboard: {XX} x {YY} inner corners, "
        f"{L*1000:.1f} mm"
    )

    if len(imgs) != len(poses):
        print(
            "[WARNING] image count != pose count. "
            "Index-based pairing will be used."
        )

    # Camera K
    manual = [
        args.fx, args.fy,
        args.cx, args.cy
    ]

    if all(v is not None for v in manual):
        K = np.array([
            [args.fx, 0, args.cx],
            [0, args.fy, args.cy],
            [0, 0, 1]
        ], dtype=np.float64)

        dist = np.asarray(
            args.dist
            if args.dist is not None
            else [0, 0, 0, 0, 0],
            dtype=np.float64
        ).reshape(-1, 1)

        intr_info = {
            "source": "manual",
            "model": "manual"
        }
    elif any(v is not None for v in manual):
        raise ValueError(
            "Use all of --fx --fy --cx --cy together."
        )
    else:
        print("\nReading RealSense color intrinsic...")
        K, dist, intr_info = get_rs_intrinsic()

    print("K:")
    print(K)
    print("dist:", dist.reshape(-1))
    print("model:", intr_info.get("model"))

    if np.allclose(dist, 0):
        print(
            "distortion coefficients are all zero -> "
            "inverse/brown model distinction has no numerical effect here."
        )

    # Load samples and two board candidates
    G_list = []
    C_candidates = []
    valid_image_ids = []
    pnp_rmse = []

    print("\nDetecting checkerboard and generating two PnP candidates...")

    for image_id, image_path in imgs:
        row = image_id - 1

        if row >= len(poses):
            print(
                f"[SKIP] {image_path.name}: no pose row"
            )
            continue

        image = cv2.imread(
            str(image_path)
        )
        if image is None:
            print(
                f"[SKIP] {image_path.name}: read failed"
            )
            continue

        found = detect_two_board_candidates(
            image, objp, pattern, K, dist
        )

        if found is None:
            print(
                f"[SKIP] {image_path.name}: checker/PnP failed"
            )
            continue

        G = pose_to_H_base_tcp(
            poses[row]
        )

        G_list.append(G)
        C_candidates.append(
            found["candidates"]
        )
        valid_image_ids.append(
            image_id
        )

        rmse = found["candidates"][0]["rmse"]
        pnp_rmse.append(rmse)

        print(
            f"{image_path.name:>6s}: "
            f"PnP RMSE {rmse:.4f} px"
        )

    n = len(G_list)
    if n < 4:
        raise RuntimeError(
            "Too few valid samples."
        )

    print(
        f"\nValid N={n}, PnP mean/max="
        f"{np.mean(pnp_rmse):.4f}/"
        f"{np.max(pnp_rmse):.4f} px"
    )

    # Pairwise exact binary optimization
    print("\nBuilding AX=XB pairwise rotation-angle constraints...")
    pair_costs = build_pair_costs(
        G_list, C_candidates
    )

    all_normal = np.zeros(
        n, dtype=np.uint8
    )
    before_rms = pair_angle_rms(
        all_normal, pair_costs
    )

    labels, energy = optimize_binary_labels(
        n, pair_costs
    )
    after_rms = pair_angle_rms(
        labels, pair_costs
    )

    print(
        f"Pair rotation-angle RMS before: "
        f"{before_rms:.3f} deg"
    )
    print(
        f"Pair rotation-angle RMS after : "
        f"{after_rms:.3f} deg"
    )

    flipped = [
        valid_image_ids[i]
        for i in range(n)
        if int(labels[i]) == 1
    ]

    print("Orientation labels:")
    for i, image_id in enumerate(
        valid_image_ids
    ):
        s = (
            "REVERSED 180deg"
            if labels[i]
            else "NORMAL"
        )
        print(
            f"  {image_id:2d}.jpg : {s}"
        )

    print(
        "\nImages corrected by 180deg reversal:",
        flipped if flipped else "none"
    )

    # Selected board poses
    selected = [
        C_candidates[i][int(labels[i])]
        for i in range(n)
    ]

    R_g2b = [
        G[:3, :3].copy()
        for G in G_list
    ]
    t_g2b = [
        G[:3, 3].reshape(3, 1).copy()
        for G in G_list
    ]
    R_t2c = [
        c["R"].copy()
        for c in selected
    ]
    t_t2c = [
        c["t"].copy()
        for c in selected
    ]
    C_list = [
        c["H"].copy()
        for c in selected
    ]

    # Solve
    print("\nRunning hand-eye solvers after orientation correction...")

    results = {}

    for name, method in methods().items():
        print("\n" + "-" * 74)
        print(name)

        try:
            R_x, t_x = cv2.calibrateHandEye(
                R_g2b,
                t_g2b,
                R_t2c,
                t_t2c,
                method=method,
            )

            R_x = np.asarray(
                R_x, dtype=np.float64
            ).reshape(3, 3)
            t_x = np.asarray(
                t_x, dtype=np.float64
            ).reshape(3)

            if not (
                np.all(np.isfinite(R_x))
                and np.all(np.isfinite(t_x))
            ):
                raise ValueError(
                    "NaN/Inf solver output"
                )

            X = make_H(R_x, t_x)
            val = validate_board(
                G_list, C_list, X
            )

            phys = bool(
                np.all(
                    np.abs(t_x)
                    <= args.max_axis_offset_m
                )
            )

            print("^TCP T_CAM [m]")
            print(X)
            print(
                "translation [mm] =",
                np.round(
                    t_x * 1000, 3
                )
            )
            print(
                "|t| [mm] =",
                f"{np.linalg.norm(t_x)*1000:.3f}"
            )
            print(
                "physical axis check =",
                "PASS" if phys else "FAIL"
            )
            print(
                "fixed-board std xyz [mm] =",
                np.round(
                    val["t_std_mm"], 3
                )
            )
            print(
                "fixed-board RMS/MAX [mm] =",
                f"{val['t_rms_mm']:.3f} / "
                f"{val['t_max_mm']:.3f}"
            )
            print(
                "fixed-board rotation RMS/MAX [deg] =",
                f"{val['r_rms_deg']:.4f} / "
                f"{val['r_max_deg']:.4f}"
            )

            results[name] = {
                "ok": True,
                "physical": phys,
                "X": X,
                "validation": val,
            }

        except Exception as e:
            print("FAILED:", e)
            results[name] = {
                "ok": False,
                "error": str(e)
            }

    valid = [
        (name, r)
        for name, r in results.items()
        if r["ok"]
    ]

    plausible = [
        (name, r)
        for name, r in valid
        if r["physical"]
    ]

    pool = (
        plausible
        if plausible
        else valid
    )

    pool.sort(
        key=lambda x: (
            x[1]["validation"][
                "t_rms_mm"
            ],
            x[1]["validation"][
                "r_rms_deg"
            ],
        )
    )

    print("\n" + "=" * 74)
    print("SUMMARY")
    print("=" * 74)
    print(
        f"{'Method':12s} "
        f"{'Phys':>6s} "
        f"{'|t|mm':>9s} "
        f"{'BoardRMSmm':>12s} "
        f"{'RotRMSdeg':>11s}"
    )

    for name, r in valid:
        X = r["X"]
        v = r["validation"]
        print(
            f"{name:12s} "
            f"{('PASS' if r['physical'] else 'FAIL'):>6s} "
            f"{np.linalg.norm(X[:3,3])*1000:9.3f} "
            f"{v['t_rms_mm']:12.3f} "
            f"{v['r_rms_deg']:11.4f}"
        )

    if not pool:
        raise RuntimeError(
            "No valid hand-eye result."
        )

    best_name, best = pool[0]
    best_X = best["X"]

    print("\nRecommended:", best_name)
    if not plausible:
        print(
            "[WARNING] No solver passed the "
            "physical ±axis constraint."
        )

    print("\nBEST ^TCP T_CAM [m]")
    print(best_X)

    Xmm = best_X.copy()
    Xmm[:3, 3] *= 1000.0
    print(
        "\nBEST ^TCP T_CAM "
        "[translation displayed in mm]"
    )
    print(Xmm)

    print("\nInverse ^CAM T_TCP [m]")
    print(invert_H(best_X))

    # JSON
    out = {
        "data_dir": str(data_dir),
        "valid_image_ids": valid_image_ids,
        "checkerboard_labels": {
            str(valid_image_ids[i]): int(labels[i])
            for i in range(n)
        },
        "reversed_image_ids": flipped,
        "pair_rotation_rms_before_deg": before_rms,
        "pair_rotation_rms_after_deg": after_rms,
        "pnp_rmse_mean_px": float(
            np.mean(pnp_rmse)
        ),
        "pnp_rmse_max_px": float(
            np.max(pnp_rmse)
        ),
        "recommended_method": best_name,
        "best_H_tcp_cam": (
            best_X.tolist()
        ),
        "methods": {}
    }

    for name, r in results.items():
        if not r["ok"]:
            out["methods"][name] = {
                "ok": False,
                "error": r["error"],
            }
            continue

        v = r["validation"]
        out["methods"][name] = {
            "ok": True,
            "physical": r["physical"],
            "H_tcp_cam": r["X"].tolist(),
            "board_t_std_mm": (
                v["t_std_mm"].tolist()
            ),
            "board_t_rms_mm": v[
                "t_rms_mm"
            ],
            "board_t_max_mm": v[
                "t_max_mm"
            ],
            "board_r_rms_deg": v[
                "r_rms_deg"
            ],
            "board_r_max_deg": v[
                "r_max_deg"
            ],
        }

    outpath = (
        data_dir
        / "handeye_rb850_flipfix_result.json"
    )
    with open(
        outpath, "w", encoding="utf-8"
    ) as f:
        json.dump(
            out, f,
            indent=2,
            ensure_ascii=False
        )

    print("\nSaved:", outpath)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
    except Exception as e:
        print("\n[FATAL]", e)
        sys.exit(1)


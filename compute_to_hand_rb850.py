#!/usr/bin/env python3
# coding: utf-8

"""
RB5-850 Eye-to-Hand calibration
===============================

물리 구성
---------
- Camera: Robot Base/외부에 고정
- Checkerboard(Target): TCP/End-effector에 강체로 고정
- 로봇을 여러 자세로 이동하면서 image + TCP pose를 동시 수집

입력 pose 형식
--------------
poses.txt:
    x, y, z, rx, ry, rz
    xyz    : meter
    rxryrz : radian

RB5-850 자세 정의:
    R = Rz @ Ry @ Rx

OpenCV calibrateHandEye()를 Eye-to-Hand에 사용하는 방법
--------------------------------------------------------
실제 로봇 pose:
    G_i = ^Base T_TCP_i

Eye-to-Hand에서는 다음을 OpenCV 첫 번째 입력으로 사용:
    G'_i = inv(G_i) = ^TCP_i T_Base

PnP:
    C_i = ^Camera T_Target_i

그러면:
    G'_i @ X @ C_i = ^TCP T_Target = constant

여기서:
    X = ^Base T_Camera

즉 최종 출력:
    ^Base T_Camera

좌표변환:
    P_base = ^Base T_Camera @ P_camera

중요
----
일반 checkerboard는 180도 회전 대칭 때문에 이미지마다 corner order가
NORMAL / REVERSED로 뒤집힐 수 있다.
이 스크립트는 각 이미지에 두 후보를 만들고 AX=XB 상대회전 조건을 이용해
전수 탐색으로 자동 보정한다.

실행
----
python compute_to_hand_rb850.py \
    --data-dir eye_hand_data/dataXXXXXXXXXXXXXX

카메라 intrinsic 수동 입력:
python compute_to_hand_rb850.py \
    --data-dir eye_hand_data/dataXXXXXXXXXXXXXX \
    --fx 606.8 --fy 606.9 --cx 327.5 --cy 240.6 \
    --dist 0 0 0 0 0
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
# Homogeneous transform utilities
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
    c = (np.trace(Rm) - 1.0) / 2.0
    c = np.clip(c, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


# ============================================================
# RB5-850 pose
# ============================================================

def rb_euler_to_R(rx, ry, rz):
    """
    RB5-850 Z-Y'-X'' Euler:
        R = Rz @ Ry @ Rx

    입력 각도는 radian.
    """
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cx, -sx],
        [0.0, sx, cx],
    ], dtype=np.float64)

    Ry = np.array([
        [cy, 0.0, sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0, cy],
    ], dtype=np.float64)

    Rz = np.array([
        [cz, -sz, 0.0],
        [sz, cz, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)

    return Rz @ Ry @ Rx


def rb_pose_to_H_base_tcp(pose):
    """
    pose = [x,y,z,rx,ry,rz]
    xyz: m
    angle: rad

    return:
        ^Base T_TCP
    """
    x, y, z, rx, ry, rz = pose

    return make_H(
        rb_euler_to_R(rx, ry, rz),
        [x, y, z],
    )


def load_poses(path):
    poses = []

    with open(path, "r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()

            if not line:
                continue

            vals = [float(v) for v in line.split(",")]

            if len(vals) != 6:
                raise ValueError(
                    f"poses.txt line {line_no}: "
                    f"6 values expected, got {len(vals)}"
                )

            poses.append(
                np.asarray(vals, dtype=np.float64)
            )

    return poses


# ============================================================
# Files
# ============================================================

def numeric_images(folder):
    out = []

    for p in Path(folder).glob("*.jpg"):
        m = re.fullmatch(r"(\d+)\.jpg", p.name)

        if m:
            out.append((int(m.group(1)), p))

    return sorted(out, key=lambda x: x[0])


def latest_data_dir(root):
    root = Path(root)

    candidates = [
        p for p in root.glob("data*")
        if p.is_dir()
    ]

    if not candidates:
        raise FileNotFoundError(
            f"No data* directory under {root}"
        )

    return max(
        candidates,
        key=lambda p: p.stat().st_mtime,
    )


# ============================================================
# RealSense intrinsic
# ============================================================

def get_rs_color_intrinsic(
    width=640,
    height=480,
    fps=30,
):
    if rs is None:
        raise RuntimeError(
            "pyrealsense2 unavailable. "
            "Use --fx --fy --cx --cy."
        )

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        width,
        height,
        rs.format.bgr8,
        fps,
    )

    try:
        profile = pipeline.start(config)

        stream = (
            profile
            .get_stream(rs.stream.color)
            .as_video_stream_profile()
        )

        intr = stream.get_intrinsics()

        K = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dist = np.asarray(
            intr.coeffs,
            dtype=np.float64,
        ).reshape(-1, 1)

        info = {
            "width": int(intr.width),
            "height": int(intr.height),
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.ppx),
            "cy": float(intr.ppy),
            "dist": [
                float(v)
                for v in intr.coeffs
            ],
            "model": str(intr.model),
            "source": "RealSense factory intrinsic",
        }

        return K, dist, info

    finally:
        try:
            pipeline.stop()
        except Exception:
            pass


# ============================================================
# Checkerboard / PnP
# ============================================================

def make_object_points(xx, yy, square_size):
    objp = np.zeros(
        (xx * yy, 3),
        dtype=np.float64,
    )

    objp[:, :2] = (
        np.mgrid[0:xx, 0:yy]
        .T
        .reshape(-1, 2)
    )

    objp *= float(square_size)

    return objp


def solve_pnp(
    objp,
    corners,
    K,
    dist,
):
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
            objp,
            corners,
            K,
            dist,
            rvec,
            tvec,
        )

    R_cam_target, _ = cv2.Rodrigues(rvec)

    projected, _ = cv2.projectPoints(
        objp,
        rvec,
        tvec,
        K,
        dist,
    )

    err = (
        projected.reshape(-1, 2)
        - corners.reshape(-1, 2)
    )

    rmse = float(
        np.sqrt(
            np.mean(
                np.sum(err * err, axis=1)
            )
        )
    )

    return {
        "R": R_cam_target,
        "t": np.asarray(
            tvec,
            dtype=np.float64,
        ).reshape(3, 1),
        "H": make_H(
            R_cam_target,
            tvec,
        ),
        "rmse": rmse,
    }


def detect_two_candidates(
    image,
    objp,
    pattern,
    K,
    dist,
):
    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY,
    )

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    found, corners = cv2.findChessboardCorners(
        gray,
        pattern,
        flags,
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
        gray,
        corners,
        (5, 5),
        (-1, -1),
        criteria,
    )

    # OpenCV가 반환한 순서
    normal = solve_pnp(
        objp,
        corners,
        K,
        dist,
    )

    # 180° 뒤집힌 checkerboard index
    reversed_180 = solve_pnp(
        objp,
        corners[::-1].copy(),
        K,
        dist,
    )

    if (
        normal is None
        or reversed_180 is None
    ):
        return None

    return [
        normal,
        reversed_180,
    ]


# ============================================================
# Checkerboard 180° ambiguity optimizer
# ============================================================

def build_pair_costs(
    pseudo_G_list,
    C_candidates,
):
    """
    Eye-to-Hand 변환식을 OpenCV 형식으로 바꾸면:

        G'_i X C_i = Z

    G'_i = ^TCP_i T_Base
    X    = ^Base T_Camera       (찾고 싶은 값)
    C_i  = ^Camera T_Target_i
    Z    = ^TCP T_Target        (고정)

    따라서 두 sample i,j에 대해:

        inv(G'_j) G'_i X
        =
        X C_j inv(C_i)

    A X = X B 이므로
    A와 B의 회전각 magnitude는 동일해야 함.
    """

    n = len(pseudo_G_list)
    pair_costs = []

    for i in range(n):
        for j in range(i + 1, n):

            A = (
                invert_H(pseudo_G_list[j])
                @ pseudo_G_list[i]
            )

            a_deg = rotation_angle_deg(
                A[:3, :3]
            )

            costs = np.zeros(
                (2, 2),
                dtype=np.float64,
            )

            for si in (0, 1):
                for sj in (0, 1):

                    Ci = C_candidates[i][si]["H"]
                    Cj = C_candidates[j][sj]["H"]

                    B = Cj @ invert_H(Ci)

                    b_deg = rotation_angle_deg(
                        B[:3, :3]
                    )

                    diff = abs(a_deg - b_deg)

                    # 한 pair가 전체를 과도하게 지배하지 않도록 제한
                    diff = min(diff, 90.0)

                    costs[si, sj] = diff ** 2

            pair_costs.append(
                (i, j, costs)
            )

    return pair_costs


def exact_binary_search(
    n,
    pair_costs,
):
    """
    전체 checkerboard를 동시에 180도 뒤집는 것은
    target frame 정의만 바꾸므로 첫 sample label=0으로 고정.

    N=17이면 2^16 = 65536 경우.
    """

    if n > 24:
        raise RuntimeError(
            "Too many samples for exact ambiguity search. "
            f"N={n}"
        )

    nstates = 1 << (n - 1)

    ids = np.arange(
        nstates,
        dtype=np.uint32,
    )

    labels = np.zeros(
        (nstates, n),
        dtype=np.uint8,
    )

    for k in range(1, n):
        labels[:, k] = (
            (ids >> (k - 1)) & 1
        ).astype(np.uint8)

    energy = np.zeros(
        nstates,
        dtype=np.float64,
    )

    for i, j, costs in pair_costs:
        energy += costs[
            labels[:, i],
            labels[:, j],
        ]

    best_idx = int(
        np.argmin(energy)
    )

    return (
        labels[best_idx].copy(),
        float(energy[best_idx]),
    )


def pair_rotation_rms(
    labels,
    pair_costs,
):
    values = []

    for i, j, costs in pair_costs:
        sq = float(
            costs[
                int(labels[i]),
                int(labels[j]),
            ]
        )

        values.append(
            math.sqrt(sq)
        )

    return float(
        np.sqrt(
            np.mean(
                np.square(values)
            )
        )
    )


# ============================================================
# Eye-to-Hand validation
# ============================================================

def validate_eye_to_hand(
    H_base_tcp_list,
    H_cam_target_list,
    H_base_cam,
):
    """
    Eye-to-Hand에서 Target은 TCP에 고정되어 있으므로:

        ^TCP T_Target_i
        =
        inv(^Base T_TCP_i)
        @ ^Base T_Camera
        @ ^Camera T_Target_i

    이 값이 모든 sample에서 일정해야 한다.
    """

    H_tcp_target_list = []

    for H_base_tcp, H_cam_target in zip(
        H_base_tcp_list,
        H_cam_target_list,
    ):
        H_tcp_target = (
            invert_H(H_base_tcp)
            @ H_base_cam
            @ H_cam_target
        )

        H_tcp_target_list.append(
            H_tcp_target
        )

    translations = np.asarray(
        [
            H[:3, 3]
            for H in H_tcp_target_list
        ],
        dtype=np.float64,
    )

    rotations = np.asarray(
        [
            H[:3, :3]
            for H in H_tcp_target_list
        ],
        dtype=np.float64,
    )

    t_mean = np.mean(
        translations,
        axis=0,
    )

    delta = (
        translations
        - t_mean
    )

    t_std_mm = (
        np.std(
            translations,
            axis=0,
        )
        * 1000.0
    )

    norms = np.linalg.norm(
        delta,
        axis=1,
    )

    t_rms_mm = float(
        np.sqrt(
            np.mean(norms ** 2)
        )
        * 1000.0
    )

    t_max_mm = float(
        np.max(norms)
        * 1000.0
    )

    rot_obj = Rotation.from_matrix(
        rotations
    )

    mean_rot = rot_obj.mean()

    rot_errors_deg = np.degrees(
        (
            mean_rot.inv()
            * rot_obj
        ).magnitude()
    )

    r_rms_deg = float(
        np.sqrt(
            np.mean(
                rot_errors_deg ** 2
            )
        )
    )

    r_max_deg = float(
        np.max(
            rot_errors_deg
        )
    )

    return {
        "target_translation_mean_m": t_mean,
        "target_translation_std_mm": t_std_mm,
        "target_translation_rms_mm": t_rms_mm,
        "target_translation_max_mm": t_max_mm,
        "target_rotation_rms_deg": r_rms_deg,
        "target_rotation_max_deg": r_max_deg,
    }


def handeye_methods():
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

    parser = argparse.ArgumentParser(
        description=(
            "RB5-850 Eye-to-Hand calibration "
            "with checkerboard 180deg ambiguity correction"
        )
    )

    parser.add_argument(
        "--data-dir",
        default=None,
        help=(
            "Eye-to-Hand dataset folder. "
            "If omitted, newest eye_hand_data/data* is used."
        ),
    )

    parser.add_argument(
        "--config",
        default="config.yaml",
    )

    parser.add_argument(
        "--fx",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--fy",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--cx",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--cy",
        type=float,
        default=None,
    )

    parser.add_argument(
        "--dist",
        nargs=5,
        type=float,
        default=None,
        metavar=(
            "K1",
            "K2",
            "P1",
            "P2",
            "K3",
        ),
    )

    args = parser.parse_args()

    # --------------------------------------------------------
    # Dataset
    # --------------------------------------------------------

    if args.data_dir:
        data_dir = Path(
            args.data_dir
        )
    else:
        data_dir = latest_data_dir(
            Path("eye_hand_data")
        )

    pose_file = (
        data_dir
        / "poses.txt"
    )

    if not pose_file.exists():
        raise FileNotFoundError(
            f"poses.txt not found: {pose_file}"
        )

    images = numeric_images(
        data_dir
    )

    poses = load_poses(
        pose_file
    )

    # --------------------------------------------------------
    # Checkerboard config
    # --------------------------------------------------------

    with open(
        args.config,
        "r",
        encoding="utf-8",
    ) as f:
        cfg = yaml.safe_load(f)

    cb = cfg[
        "checkerboard_args"
    ]

    XX = int(cb["XX"])
    YY = int(cb["YY"])
    L = float(cb["L"])

    pattern = (
        XX,
        YY,
    )

    objp = make_object_points(
        XX,
        YY,
        L,
    )

    print("=" * 76)
    print(
        "RB5-850 Eye-to-Hand calibration "
        "+ checkerboard 180deg ambiguity correction"
    )
    print("=" * 76)

    print(
        f"Data folder      : {data_dir}"
    )

    print(
        f"Images           : {len(images)}"
    )

    print(
        f"Pose rows        : {len(poses)}"
    )

    print(
        f"Checkerboard     : {XX} x {YY} inner corners"
    )

    print(
        f"Square size      : {L*1000:.3f} mm"
    )

    print()
    print(
        "[IMPORTANT] Eye-to-Hand dataset must have:"
    )
    print(
        "  Camera       : fixed to Base/world"
    )
    print(
        "  Checkerboard : rigidly fixed to TCP/end-effector"
    )
    print(
        "Do NOT use an Eye-in-Hand dataset here."
    )

    # --------------------------------------------------------
    # Camera intrinsic
    # --------------------------------------------------------

    manual = [
        args.fx,
        args.fy,
        args.cx,
        args.cy,
    ]

    if all(
        v is not None
        for v in manual
    ):

        K = np.array([
            [
                args.fx,
                0.0,
                args.cx,
            ],
            [
                0.0,
                args.fy,
                args.cy,
            ],
            [
                0.0,
                0.0,
                1.0,
            ],
        ], dtype=np.float64)

        dist = np.asarray(
            (
                args.dist
                if args.dist is not None
                else [0, 0, 0, 0, 0]
            ),
            dtype=np.float64,
        ).reshape(-1, 1)

        intr_info = {
            "source": "manual",
            "model": "manual/OpenCV",
            "fx": args.fx,
            "fy": args.fy,
            "cx": args.cx,
            "cy": args.cy,
            "dist": dist.reshape(-1).tolist(),
        }

    elif any(
        v is not None
        for v in manual
    ):
        raise ValueError(
            "--fx --fy --cx --cy must all be supplied together."
        )

    else:
        print()
        print(
            "Reading RealSense D435 color intrinsic..."
        )

        K, dist, intr_info = (
            get_rs_color_intrinsic()
        )

    print()
    print("Camera K:")
    print(K)

    print(
        "dist:",
        dist.reshape(-1),
    )

    print(
        "model:",
        intr_info.get(
            "model"
        ),
    )

    if np.allclose(
        dist,
        0.0,
    ):
        print(
            "distortion coefficients are zero."
        )

    # --------------------------------------------------------
    # Read image + exact pose row + PnP candidates
    # --------------------------------------------------------

    H_base_tcp_list = []

    # OpenCV Eye-to-Hand pseudo first input:
    # ^TCP T_Base
    H_tcp_base_list = []

    C_candidates = []

    valid_ids = []

    pnp_rmse = []

    print()
    print(
        "Detecting checkerboard "
        "and making NORMAL/REVERSED PnP candidates..."
    )

    for image_id, image_path in images:

        pose_index = (
            image_id - 1
        )

        if (
            pose_index < 0
            or pose_index >= len(poses)
        ):
            print(
                f"[SKIP] {image_path.name}: "
                "matching pose row not found"
            )
            continue

        image = cv2.imread(
            str(image_path)
        )

        if image is None:
            print(
                f"[SKIP] {image_path.name}: "
                "image read failed"
            )
            continue

        candidates = detect_two_candidates(
            image,
            objp,
            pattern,
            K,
            dist,
        )

        if candidates is None:
            print(
                f"[SKIP] {image_path.name}: "
                "checkerboard/PnP failed"
            )
            continue

        H_base_tcp = (
            rb_pose_to_H_base_tcp(
                poses[pose_index]
            )
        )

        H_tcp_base = invert_H(
            H_base_tcp
        )

        H_base_tcp_list.append(
            H_base_tcp
        )

        H_tcp_base_list.append(
            H_tcp_base
        )

        C_candidates.append(
            candidates
        )

        valid_ids.append(
            image_id
        )

        pnp_rmse.append(
            candidates[0]["rmse"]
        )

        print(
            f"{image_path.name:>6s} -> pose row {image_id:2d}, "
            f"PnP RMSE = {candidates[0]['rmse']:.4f} px"
        )

    n = len(valid_ids)

    if n < 4:
        raise RuntimeError(
            f"Only {n} valid samples."
        )

    print()
    print(
        f"Valid N = {n}"
    )

    print(
        "PnP RMSE mean/max = "
        f"{np.mean(pnp_rmse):.4f} / "
        f"{np.max(pnp_rmse):.4f} px"
    )

    # --------------------------------------------------------
    # 180° ambiguity correction
    # --------------------------------------------------------

    print()
    print(
        "Optimizing checkerboard 180deg orientation..."
    )

    pair_costs = build_pair_costs(
        H_tcp_base_list,
        C_candidates,
    )

    all_normal = np.zeros(
        n,
        dtype=np.uint8,
    )

    before_rms = pair_rotation_rms(
        all_normal,
        pair_costs,
    )

    labels, _ = exact_binary_search(
        n,
        pair_costs,
    )

    after_rms = pair_rotation_rms(
        labels,
        pair_costs,
    )

    print(
        f"Pair rotation-angle RMS before: "
        f"{before_rms:.3f} deg"
    )

    print(
        f"Pair rotation-angle RMS after : "
        f"{after_rms:.3f} deg"
    )

    reversed_ids = []

    print()
    print(
        "Orientation labels:"
    )

    for idx, image_id in enumerate(
        valid_ids
    ):
        if labels[idx] == 1:
            status = (
                "REVERSED 180deg"
            )
            reversed_ids.append(
                image_id
            )
        else:
            status = "NORMAL"

        print(
            f"  {image_id:2d}.jpg : {status}"
        )

    print()
    print(
        "Images corrected by 180deg reversal:",
        (
            reversed_ids
            if reversed_ids
            else "none"
        ),
    )

    selected = [
        C_candidates[i][
            int(labels[i])
        ]
        for i in range(n)
    ]

    # --------------------------------------------------------
    # OpenCV inputs
    # --------------------------------------------------------

    # Eye-to-Hand trick:
    # feed ^TCP T_Base as pseudo "gripper2base".
    R_pseudo = [
        H[:3, :3].copy()
        for H in H_tcp_base_list
    ]

    t_pseudo = [
        H[:3, 3]
        .reshape(3, 1)
        .copy()
        for H in H_tcp_base_list
    ]

    R_target2cam = [
        c["R"].copy()
        for c in selected
    ]

    t_target2cam = [
        c["t"].copy()
        for c in selected
    ]

    H_cam_target_list = [
        c["H"].copy()
        for c in selected
    ]

    # --------------------------------------------------------
    # Hand-eye solvers
    # --------------------------------------------------------

    print()
    print(
        "Running Eye-to-Hand solvers..."
    )

    results = {}

    for name, method in handeye_methods().items():

        print()
        print("-" * 76)
        print(name)

        try:
            R_base_cam, t_base_cam = (
                cv2.calibrateHandEye(
                    R_pseudo,
                    t_pseudo,
                    R_target2cam,
                    t_target2cam,
                    method=method,
                )
            )

            R_base_cam = np.asarray(
                R_base_cam,
                dtype=np.float64,
            ).reshape(3, 3)

            t_base_cam = np.asarray(
                t_base_cam,
                dtype=np.float64,
            ).reshape(3)

            if (
                not np.all(
                    np.isfinite(
                        R_base_cam
                    )
                )
                or not np.all(
                    np.isfinite(
                        t_base_cam
                    )
                )
            ):
                raise ValueError(
                    "NaN/Inf result"
                )

            H_base_cam = make_H(
                R_base_cam,
                t_base_cam,
            )

            validation = (
                validate_eye_to_hand(
                    H_base_tcp_list,
                    H_cam_target_list,
                    H_base_cam,
                )
            )

            print(
                "^BASE T_CAM [m]"
            )
            print(
                H_base_cam
            )

            H_mm = (
                H_base_cam.copy()
            )

            H_mm[:3, 3] *= 1000.0

            print()
            print(
                "^BASE T_CAM "
                "[translation displayed in mm]"
            )
            print(
                H_mm
            )

            print()
            print(
                "Camera origin in Base [mm] = "
                f"[{t_base_cam[0]*1000:.3f}, "
                f"{t_base_cam[1]*1000:.3f}, "
                f"{t_base_cam[2]*1000:.3f}]"
            )

            print(
                "Fixed TCP->Target std xyz [mm] =",
                np.round(
                    validation[
                        "target_translation_std_mm"
                    ],
                    3,
                ),
            )

            print(
                "Fixed TCP->Target "
                "translation RMS/MAX [mm] = "
                f"{validation['target_translation_rms_mm']:.3f} / "
                f"{validation['target_translation_max_mm']:.3f}"
            )

            print(
                "Fixed TCP->Target "
                "rotation RMS/MAX [deg] = "
                f"{validation['target_rotation_rms_deg']:.4f} / "
                f"{validation['target_rotation_max_deg']:.4f}"
            )

            results[name] = {
                "ok": True,
                "H_base_cam": H_base_cam,
                "H_cam_base": invert_H(
                    H_base_cam
                ),
                "validation": validation,
            }

        except Exception as e:

            print(
                f"FAILED: {e}"
            )

            results[name] = {
                "ok": False,
                "error": str(e),
            }

    # --------------------------------------------------------
    # Ranking
    # --------------------------------------------------------

    valid_results = [
        (name, result)
        for name, result
        in results.items()
        if result.get("ok")
    ]

    if not valid_results:
        raise RuntimeError(
            "All hand-eye solvers failed."
        )

    valid_results.sort(
        key=lambda item: (
            item[1]["validation"][
                "target_translation_rms_mm"
            ],
            item[1]["validation"][
                "target_rotation_rms_deg"
            ],
        )
    )

    best_name, best = (
        valid_results[0]
    )

    print()
    print("=" * 76)
    print(
        "SUMMARY"
    )
    print("=" * 76)

    print(
        f"{'Method':12s} "
        f"{'TargetRMSmm':>13s} "
        f"{'TargetMAXmm':>13s} "
        f"{'RotRMSdeg':>11s}"
    )

    for name, result in valid_results:

        v = result[
            "validation"
        ]

        print(
            f"{name:12s} "
            f"{v['target_translation_rms_mm']:13.3f} "
            f"{v['target_translation_max_mm']:13.3f} "
            f"{v['target_rotation_rms_deg']:11.4f}"
        )

    print()
    print(
        f"Recommended: {best_name}"
    )

    print()
    print(
        "BEST ^BASE T_CAM [m]"
    )

    print(
        best["H_base_cam"]
    )

    best_mm = (
        best["H_base_cam"]
        .copy()
    )

    best_mm[:3, 3] *= 1000.0

    print()
    print(
        "BEST ^BASE T_CAM "
        "[translation displayed in mm]"
    )

    print(
        best_mm
    )

    print()
    print(
        "Inverse ^CAM T_BASE [m]"
    )

    print(
        best["H_cam_base"]
    )

    print()
    print(
        "Coordinate use:"
    )

    print(
        "  P_base = T_base_cam @ P_cam"
    )

    print(
        "  T_base_obj = T_base_cam @ T_cam_obj"
    )

    # --------------------------------------------------------
    # JSON
    # --------------------------------------------------------

    output = {
        "mode": "eye-to-hand",
        "meaning": (
            "Camera fixed to Base; "
            "Target fixed to TCP"
        ),
        "data_dir": str(
            data_dir
        ),
        "valid_image_ids": (
            valid_ids
        ),
        "reversed_image_ids": (
            reversed_ids
        ),
        "pair_rotation_rms_before_deg": (
            before_rms
        ),
        "pair_rotation_rms_after_deg": (
            after_rms
        ),
        "pnp_rmse_mean_px": float(
            np.mean(
                pnp_rmse
            )
        ),
        "pnp_rmse_max_px": float(
            np.max(
                pnp_rmse
            )
        ),
        "recommended_method": (
            best_name
        ),
        "best_H_base_cam": (
            best[
                "H_base_cam"
            ].tolist()
        ),
        "best_H_cam_base": (
            best[
                "H_cam_base"
            ].tolist()
        ),
        "methods": {},
    }

    for name, result in results.items():

        if not result.get("ok"):

            output["methods"][name] = {
                "ok": False,
                "error": result.get(
                    "error",
                    "",
                ),
            }

            continue

        v = result[
            "validation"
        ]

        output["methods"][name] = {
            "ok": True,
            "H_base_cam": result[
                "H_base_cam"
            ].tolist(),
            "H_cam_base": result[
                "H_cam_base"
            ].tolist(),
            "target_translation_std_mm": (
                v[
                    "target_translation_std_mm"
                ].tolist()
            ),
            "target_translation_rms_mm": (
                v[
                    "target_translation_rms_mm"
                ]
            ),
            "target_translation_max_mm": (
                v[
                    "target_translation_max_mm"
                ]
            ),
            "target_rotation_rms_deg": (
                v[
                    "target_rotation_rms_deg"
                ]
            ),
            "target_rotation_max_deg": (
                v[
                    "target_rotation_max_deg"
                ]
            ),
        }

    output_path = (
        data_dir
        / "handeye_to_hand_rb850_result.json"
    )

    with open(
        output_path,
        "w",
        encoding="utf-8",
    ) as f:

        json.dump(
            output,
            f,
            indent=2,
            ensure_ascii=False,
        )

    print()
    print(
        f"Saved: {output_path}"
    )

    print("=" * 76)


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:
        print(
            "\nInterrupted."
        )
        sys.exit(130)

    except Exception as e:
        print(
            f"\n[FATAL] {e}"
        )
        sys.exit(1)


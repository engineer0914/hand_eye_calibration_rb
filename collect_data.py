import os
import cv2
import yaml
import numpy as np
import pyrealsense2 as rs
import rbpodo as rb

from datetime import datetime


# ============================================================
# Robot
# ============================================================

ROBOT_IP = "192.168.0.100"


# ============================================================
# config.yaml에서 Checkerboard 정보 읽기
# ============================================================

with open("config.yaml", "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

XX = cfg["checkerboard_args"]["XX"]
YY = cfg["checkerboard_args"]["YY"]
L = cfg["checkerboard_args"]["L"]

CHECKERBOARD = (XX, YY)


print("============================================")
print(" Eye-in-Hand Data Collection")
print("============================================")
print(f"Robot IP            : {ROBOT_IP}")
print(f"Checkerboard corner : {XX} x {YY}")
print(f"Square size         : {L} m")
print("============================================")


# ============================================================
# 저장 폴더 생성
#
# 원래 repository의 compute_in_hand.py가
# eye_hand_data/dataXXXX... 중 가장 최신 폴더를 찾는 구조이므로
# 동일하게 저장
# ============================================================

base_dir = "eye_hand_data"

os.makedirs(base_dir, exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d%H%M%S")

save_dir = os.path.join(
    base_dir,
    f"data{timestamp}"
)

os.makedirs(save_dir, exist_ok=True)


pose_file = os.path.join(
    save_dir,
    "poses.txt"
)


print(f"\nSave directory:")
print(save_dir)


# ============================================================
# Rainbow Robotics 연결
# ============================================================

print("\nConnecting robot...")

robot = rb.Cobot(ROBOT_IP)
rc = rb.ResponseCollector()

print("Robot connection created.")


# ============================================================
# RealSense 설정
# ============================================================

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

profile = pipeline.start(config)


# ============================================================
# Camera Intrinsic 출력
# ============================================================

color_stream = (
    profile
    .get_stream(rs.stream.color)
    .as_video_stream_profile()
)

intr = color_stream.get_intrinsics()

print("\n========== Camera Intrinsic ==========")
print(f"width  : {intr.width}")
print(f"height : {intr.height}")
print(f"fx     : {intr.fx}")
print(f"fy     : {intr.fy}")
print(f"cx     : {intr.ppx}")
print(f"cy     : {intr.ppy}")
print(f"dist   : {intr.coeffs}")
print("======================================")



# ============================================================
# Sub-pixel refinement 설정
# ============================================================

criteria = (
    cv2.TERM_CRITERIA_EPS
    + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)


# ============================================================
# Capture number
# ============================================================

capture_count = 0


print()
print("Controls")
print("--------------------------------------")
print("s : save image + robot TCP pose")
print("q : quit")
print("--------------------------------------")


try:

    while True:

        # ====================================================
        # Camera frame
        # ====================================================

        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue


        image = np.asanyarray(
            color_frame.get_data()
        )

        display = image.copy()


        # ====================================================
        # Checkerboard detection
        # ====================================================

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        detected, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            None
        )


        # ====================================================
        # Detection successful
        # ====================================================

        if detected:

            corners_subpix = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                criteria
            )

            cv2.drawChessboardCorners(
                display,
                CHECKERBOARD,
                corners_subpix,
                detected
            )

            cv2.putText(
                display,
                f"DETECTED : {len(corners_subpix)} corners",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        else:

            cv2.putText(
                display,
                "CHECKERBOARD NOT DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )


        # ====================================================
        # 현재 저장 개수 표시
        # ====================================================

        cv2.putText(
            display,
            f"Samples : {capture_count}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )


        cv2.imshow(
            "Eye-in-Hand Data Collection",
            display
        )


        key = cv2.waitKey(1) & 0xFF


        # ====================================================
        # q : 종료
        # ====================================================

        if key == ord("q"):

            print("\nQuit.")
            break


        # ====================================================
        # s : 데이터 저장
        # ====================================================

        elif key == ord("s"):

            # ------------------------------------------------
            # Checkerboard가 안 잡혔으면 저장 금지
            # ------------------------------------------------

            if not detected:

                print(
                    "[SKIP] Checkerboard not detected."
                )

                continue


            try:

                # ============================================
                # 현재 RB TCP pose 획득
                #
                # [X, Y, Z, Rx, Ry, Rz]
                #
                # XYZ    : mm
                # RxRyRz : degree
                # ============================================

                response, tcp = robot.get_tcp_info(rc)

                # rbpodo 내부 error 확인
                rc.error().throw_if_not_empty()


                tcp = np.array(
                    tcp,
                    dtype=np.float64
                )


                # ============================================
                # 원본 RB 값
                # ============================================

                x_mm = tcp[0]
                y_mm = tcp[1]
                z_mm = tcp[2]

                rx_deg = tcp[3]
                ry_deg = tcp[4]
                rz_deg = tcp[5]


                # ============================================
                # Repository 형식으로 변환
                #
                # mm     -> meter
                # degree -> radian
                # ============================================

                x = x_mm / 1000.0
                y = y_mm / 1000.0
                z = z_mm / 1000.0

                rx = np.deg2rad(rx_deg)
                ry = np.deg2rad(ry_deg)
                rz = np.deg2rad(rz_deg)


                pose = np.array([
                    x,
                    y,
                    z,
                    rx,
                    ry,
                    rz
                ])


                # ============================================
                # 다음 sample 번호
                # ============================================

                next_index = capture_count + 1


                # ============================================
                # 이미지 저장
                # ============================================

                image_path = os.path.join(
                    save_dir,
                    f"{next_index}.jpg"
                )

                image_ok = cv2.imwrite(
                    image_path,
                    image
                )

                if not image_ok:

                    print(
                        "[ERROR] Failed to save image."
                    )

                    continue


                # ============================================
                # pose 저장
                #
                # 반드시 이미지와 같은 순서로 저장
                # ============================================

                with open(
                    pose_file,
                    "a",
                    encoding="utf-8"
                ) as f:

                    f.write(
                        ",".join(
                            f"{v:.10f}"
                            for v in pose
                        )
                        + "\n"
                    )


                # 여기까지 성공한 경우에만 증가
                capture_count = next_index


                # ============================================
                # 터미널 출력
                # ============================================

                print()
                print(
                    f"========== Sample {capture_count} =========="
                )

                print(
                    "RB raw [mm, deg] :"
                )

                print(
                    f"[{x_mm:.3f}, "
                    f"{y_mm:.3f}, "
                    f"{z_mm:.3f}, "
                    f"{rx_deg:.3f}, "
                    f"{ry_deg:.3f}, "
                    f"{rz_deg:.3f}]"
                )


                print(
                    "Saved [m, rad]    :"
                )

                print(
                    f"[{x:.6f}, "
                    f"{y:.6f}, "
                    f"{z:.6f}, "
                    f"{rx:.6f}, "
                    f"{ry:.6f}, "
                    f"{rz:.6f}]"
                )

                print(
                    f"Image : {image_path}"
                )

                print(
                    "=================================="
                )


            except Exception as e:

                print(
                    f"[ROBOT ERROR] {e}"
                )


finally:

    pipeline.stop()

    cv2.destroyAllWindows()

    print()
    print("============================================")
    print("Collection finished")
    print(f"Samples : {capture_count}")
    print(f"Folder  : {save_dir}")
    print("============================================")

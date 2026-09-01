import cv2
import numpy as np
import pyrealsense2 as rs


# ============================================================
# Checkerboard 설정
#
# 실제 square가 8 x 6이면
# inner corner는 7 x 5
# ============================================================

CHECKERBOARD = (7, 5)

# 한 칸 크기 [m]
SQUARE_SIZE = 0.03


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


# ============================================================
# 카메라 시작
# ============================================================

profile = pipeline.start(config)


# ============================================================
# 카메라 Intrinsic 확인
# 나중에 3D 좌표 변환할 때도 사용 가능
# ============================================================

color_stream = profile.get_stream(
    rs.stream.color
).as_video_stream_profile()

intr = color_stream.get_intrinsics()

print("========== Camera Intrinsic ==========")
print(f"width  : {intr.width}")
print(f"height : {intr.height}")
print(f"fx     : {intr.fx}")
print(f"fy     : {intr.fy}")
print(f"cx     : {intr.ppx}")
print(f"cy     : {intr.ppy}")
print(f"dist   : {intr.coeffs}")
print("======================================")

print()
print("Checkerboard inner corners :", CHECKERBOARD)
print("Square size                :", SQUARE_SIZE, "m")
print()
print("[q] quit")
print("[s] save current image")


# ============================================================
# Sub-pixel corner refinement 조건
# repo의 compute_in_hand.py와 같은 개념
# ============================================================

criteria = (
    cv2.TERM_CRITERIA_EPS
    + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)


save_count = 0


try:

    while True:

        # ----------------------------------------------------
        # RealSense frame 수신
        # ----------------------------------------------------

        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue


        # ----------------------------------------------------
        # RealSense frame -> NumPy
        # ----------------------------------------------------

        image = np.asanyarray(
            color_frame.get_data()
        )

        display = image.copy()


        # ----------------------------------------------------
        # grayscale 변환
        # ----------------------------------------------------

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )


        # ----------------------------------------------------
        # Checkerboard corner 검출
        #
        # 이 저장소 compute_in_hand.py에서도
        # 동일한 findChessboardCorners()를 사용
        # ----------------------------------------------------

        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            None
        )


        # ----------------------------------------------------
        # 검출 성공
        # ----------------------------------------------------

        if ret:

            # corner 위치를 sub-pixel 단위까지 refinement
            corners_subpix = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                criteria
            )

            # 검출된 corner 표시
            cv2.drawChessboardCorners(
                display,
                CHECKERBOARD,
                corners_subpix,
                ret
            )

            cv2.putText(
                display,
                "CHECKERBOARD DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f"corners = {len(corners_subpix)}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )


        # ----------------------------------------------------
        # 검출 실패
        # ----------------------------------------------------

        else:

            cv2.putText(
                display,
                "NOT DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )


        # ----------------------------------------------------
        # 화면 출력
        # ----------------------------------------------------

        cv2.imshow(
            "Checkerboard Detection Test",
            display
        )


        key = cv2.waitKey(1) & 0xFF


        # ----------------------------------------------------
        # q : 종료
        # ----------------------------------------------------

        if key == ord('q'):
            break


        # ----------------------------------------------------
        # s : 현재 이미지 저장
        # ----------------------------------------------------

        elif key == ord('s'):

            save_count += 1

            filename = (
                f"checkerboard_test_{save_count}.jpg"
            )

            cv2.imwrite(
                filename,
                image
            )

            print(
                f"[SAVE] {filename}, "
                f"detected={ret}"
            )


finally:

    pipeline.stop()
    cv2.destroyAllWindows()

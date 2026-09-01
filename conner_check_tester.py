import pyrealsense2 as rs
import numpy as np
import cv2

# ==========================================
# Configuration
# ==========================================
CHECKERBOARD = (7, 5)  # Number of INNER corners (width, height)
SQUARE_SIZE = 0.03     # Physical size of one square in meters (e.g., 30mm = 0.03m)

# Prepare 3D object points based on the real-world square size
# Z is 0 because we assume the checkerboard is completely flat
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Subpixel refinement criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ==========================================
# RealSense Pipeline Setup
# ==========================================
pipeline = rs.pipeline()
config = rs.config()

# Enable color stream
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Get camera intrinsics directly from RealSense
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# Build the camera matrix and distortion coefficients for OpenCV
camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                          [0, intrinsics.fy, intrinsics.ppy],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float32)

print("Started RealSense Pipeline. Press 'q' to quit.")

try:
    while True:
        # Wait for a coherent frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            # Refine the corner locations to subpixel accuracy
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Draw the corners
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

            # Calculate the 3D pose (rotation and translation vectors)
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)

            if success:
                # The Euclidean distance is the magnitude of the translation vector (tvec)
                distance = np.linalg.norm(tvec)

                # tvec contains [x, y, z] coordinates of the board's origin relative to the camera
                x, y, z = tvec.flatten()

                # Display text on the image
                cv2.putText(img, f"Distance: {distance:.3f} m", (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(img, f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}", (20, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # Draw 3D coordinate axes on the board (length = 3 * SQUARE_SIZE)
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, SQUARE_SIZE * 3)

        # Show the image
        cv2.imshow('D435 Checkerboard Distance', img)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming and close windows
    pipeline.stop()
    cv2.destroyAllWindows()

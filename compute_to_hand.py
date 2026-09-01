# coding=utf-8

"""
Eye-to-Hand (아이 투 핸드) 수집된 이미지 정보와 로봇 팔 포즈 정보를 사용하여 로봇 베이스 좌표계에 대한 카메라 좌표계의 회전 행렬 및 평행 이동 벡터를 계산[cite: 3]

"""

import os
import logging

import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog

from save_poses2 import poses2_main

np.set_printoptions(precision=8,suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"eye_hand_data")

images_path = os.path.join("eye_hand_data",find_latest_data_folder(current_path))

file_path = os.path.join(images_path,"poses.txt")  # 캘리브레이션 보드 이미지를 수집할 때 대응하는 로봇 팔 말단의 포즈. 첫 번째 줄부터 마지막 줄까지 수집된 캘리브레이션 보드 이미지의 순서와 일치해야 함[cite: 3]


with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") # 캘리브레이션 보드의 가로 길이에 해당하는 코너 포인트 개수[cite: 3]
YY = data.get("checkerboard_args").get("YY") # 캘리브레이션 보드의 세로 길이에 해당하는 코너 포인트 개수[cite: 3]
L = data.get("checkerboard_args").get("L")   # 캘리브레이션 보드 한 격자의 길이. 단위는 미터[cite: 3]

def func():

    path = os.path.dirname(__file__)
    print(path)

    # 서브픽셀 코너 포인트를 찾기 위한 매개변수 설정, 사용된 종료 기준은 최대 반복 횟수 30 및 최대 허용 오차 0.001[cite: 3]
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 캘리브레이션 보드 코너 포인트의 위치 획득[cite: 3]
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 월드 좌표계를 캘리브레이션 보드에 설정하므로 모든 포인트의 Z 좌표는 0이 되며 x와 y만 할당하면 됨[cite: 3]
    objp = L*objp

    obj_points = []     # 3D 포인트 저장[cite: 3]
    img_points = []     # 2D 포인트 저장[cite: 3]

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    for i in range(1, len(images_num) + 1):   # 캘리브레이션된 이미지는 images_path 경로에 있으며, 0.jpg부터 x.jpg까지 존재함[cite: 3]

        image_file = os.path.join(images_path,f"{i}.jpg")

        if os.path.exists(image_file):

            logger_.info(f'읽기 {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:

                obj_points.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 원래 코너 포인트를 기반으로 서브픽셀 코너 포인트 찾기[cite: 3]
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)

    N = len(img_points)

    # 캘리브레이션, 카메라 좌표계에서 패턴의 포즈 획득[cite: 3]
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

    # logger_.info(f"내부 파라미터 행렬:\n:{mtx}" ) # 내부 파라미터 행렬[cite: 3]
    # logger_.info(f"왜곡 계수:\n:{dist}")  # 왜곡 계수   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)[cite: 3]

    print("-----------------------------------------------------")

    poses2_main(file_path)
    # 베이스 좌표계 기준 로봇 말단의 포즈[cite: 3]

    csv_file = os.path.join(path,"RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file,delimiter=',')

    R_tool = []
    t_tool = []

    for i in range(int(N)):

        R_tool.append(tool_pose[0:3,4*i:4*i+3])
        t_tool.append(tool_pose[0:3,4*i+3])

    R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

    return R,t

if __name__ == '__main__':

    # 회전 행렬[cite: 3]
    rotation_matrix, translation_vector = func()

    # 회전 행렬을 쿼터니언(사원수)으로 변환[cite: 3]
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    x, y, z = translation_vector.flatten()

    logger_.info(f"회전 행렬:\n {            rotation_matrix}")

    logger_.info(f"평행 이동 벡터:\n {            translation_vector}")

    logger_.info(f"쿼터니언(사원수):\n {             quaternion}")

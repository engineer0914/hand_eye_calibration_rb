## 개요

핸드-아이 캘리브레이션(Hand-Eye Calibration, 손-눈 캘리브레이션)은 로봇 및 컴퓨터 비전 분야에서 주로 사용되며, 특히 로봇 팔이 환경과 정밀하게 상호작용해야 하는 환경에서 필수적입니다. 핸드-아이 캘리브레이션은 로봇 매니퓰레이터(손)와 카메라(눈)의 좌표계를 일치시켜 둘 사이의 좌표 변환 관계를 해결함으로써, 로봇 팔이 카메라가 위치를 특정한 대상 물체를 정확히 파지(Grasp)할 수 있도록 만듭니다.

핸드-아이 시스템은 손(로봇 팔)과 눈(카메라)의 관계를 다룹니다. 눈(카메라)이 물체를 감지했을 때 손(로봇 팔)에게 물체의 위치를 알려주어야 합니다. 물체의 카메라 기준 위치가 결정되었을 때 눈과 손 사이의 상대적 관계를 알고 있다면, 로봇 팔 기준에서의 물체 위치를 정확히 도출할 수 있습니다.

로봇 물체 파지, 동적 환경 상호작용, 정밀 측정 및 검사, 비주얼 서보잉(Visual Servoing) 제어 등 다양한 분야에서 핸드-아이 캘리브레이션이 요구됩니다.

캘리브레이션을 한 번 완료하면 카메라와 로봇 팔의 상대적 위치가 바뀌지 않는 한 재캘리브레이션을 수행할 필요가 없습니다. 아래의 방법은 **정방향 설치(정장), 측면 설치(측장), 역방향 천장 설치(도장)** 모두에 적용 가능합니다.

핸드-아이 캘리브레이션은 크게 두 가지 형태로 나뉩니다:

* **Eye-in-Hand (카메라가 로봇 팔에 고정된 형태):** 카메라가 로봇 팔 말단(End-Effector)에 장착되어 로봇 팔 이동 시 함께 움직입니다.

* **Eye-to-Hand (카메라가 로봇 팔 외부에 고정된 형태):** 카메라가 로봇 팔 외부 특정 위치에 고정되어 있습니다.

---

## 원리

### 1. Eye-in-Hand (아이 인 핸드)

**Eye-in-Hand 방식에서 핸드-아이 캘리브레이션의 목적은 카메라와 로봇 팔 말단(End-Effector) 사이의 좌표 변환 관계를 도출하는 것입니다:**

대상 포인트의 3차원 공간 좌표를 변환하는 과정에서 먼저 마주치는 문제는 로봇 팔 말단 좌표계와 카메라 좌표계 간의 위치 변환 관계(핸드-아이 위치 관계)입니다. 이것이 바로 핸드-아이 캘리브레이션의 최종 계산 목표이며, 기호 $X$로 표현됩니다. 이는 $AX=XB$ 방정식을 통해 구할 수 있습니다. 여기서 $A$는 연속된 두 동작 간 **로봇 팔 말단의 변환 관계**를, $B$는 연속된 두 동작 간 **카메라 좌표계의 상대적 변환 관계**를 나타냅니다.

그림 1은 Eye-in-Hand 구성의 예시입니다 (실제 실험 환경이 아닌 시각적 설명을 위한 예시입니다). 카메라는 로봇 팔 말단에 고정되어 로봇 팔의 움직임에 따라 함께 움직입니다.

* **A:** 로봇 팔 베이스(Base) 좌표계 기준 말단(End)의 포즈(Pose). 로봇 팔 API를 통해 획득 (알고 있는 값).

$${}^{base}_{end}M$$

* **B:** 로봇 팔 말단 좌표계 기준 카메라의 포즈. 이 변환은 고정되어 있으며, 이를 알면 언제든 카메라의 실제 위치를 계산할 수 있습니다 (구하고자 하는 미지의 값).

$${}^{end}_{camera}M$$

* **C:** 캘리브레이션 보드(Board) 좌표계 기준 카메라의 포즈. 카메라의 외부 파라미터(Extrinsic Parameters)에 해당 (카메라 캘리브레이션을 통해 계산).

$${}^{board}_{camera}M$$

* **D:** 로봇 베이스 좌표계 기준 캘리브레이션 보드의 포즈. 캘리브레이션 중 말단만 움직이고 보드와 로봇 베이스는 고정되어 있으므로 불변입니다.

$${}^{base}_{board}M$$

따라서 변환 $B$를 계산하면, 로봇 팔 좌표계 기준 캘리브레이션 보드의 포즈 $D$ 역시 자연스럽게 도출됩니다:

$${}^{base}_{board}M = {}^{base}_{end}M \cdot {}^{end}_{camera}M \cdot {}^{camera}_{board}M$$

그림 2와 같이 로봇 팔을 두 위치로 이동시켜 두 위치 모두에서 보드가 보이도록 한 뒤, 공간 변환 루프를 구성합니다:

$$A_1 \cdot B \cdot C_1^{-1} = A_2 \cdot B \cdot C_2^{-1}$$

$$\left( A_2^{-1} \cdot A_1 \right) \cdot B = B \cdot \left( C_2^{-1} \cdot C_1 \right)$$

이는 다음 수식과 동일합니다:

$${}^{base}_{end}M_1 \cdot {}^{end}_{camera}M_1 \cdot {}^{camera}_{board}M_1 = {}^{base}_{end}M_2 \cdot {}^{end}_{camera}M_2 \cdot {}^{camera}_{board}M_2$$

$$\left( {}^{base}_{end}M_2 \right)^{-1} \cdot {}^{base}_{end}M_1 \cdot {}^{end}_{camera}M_1 = {}^{end}_{camera}M_2 \cdot {}^{camera}_{board}M_2 \cdot \left( {}^{camera}_{board}M_1 \right)^{-1}$$

이는 전형적인 **$AX=XB$** 형태의 문제이며, 정의에 따라 $X$는 4x4 동차 변환 행렬(Homogeneous Transformation Matrix)입니다:

$$X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

핸드-아이 캘리브레이션의 목적은 바로 이 $X$를 계산하는 것입니다.

---

### 2. Eye-to-Hand (아이 투 핸드)

**Eye-to-Hand** 캘리브레이션에서는 **로봇 팔 베이스와 카메라를 고정**하고, **캘리브레이션 보드를 로봇 팔 말단에 고정**합니다. 따라서 캘리브레이션 과정 동안 **보드와 로봇 팔 말단 간의 관계 및 카메라와 로봇 베이스 간의 관계는 고정 불변**입니다.

* **캘리브레이션 목표:** 카메라 좌표계에서 로봇 팔 베이스 좌표계로의 변환 행렬 도출

$${}^{base}_{camera}M$$

* **구현 방법:**
1. 캘리브레이션 보드를 로봇 팔 말단에 견고히 고정합니다.
2. 로봇 팔 말단을 움직이며 다양한 자세에서 카메라로 보드 사진 $n$장(10~20장)을 촬영합니다.



매 이미지와 로봇 팔 포즈를 수집할 때마다 다음 방정식이 성립합니다:

$${}^{end}_{board}M = {}^{end}_{base}M \cdot {}^{base}_{camera}M \cdot {}^{camera}_{board}M$$

| 기호 | 설명 |
| --- | --- |
| $${}^{end}_{board}M$$

 | 캘리브레이션 보드에서 로봇 팔 말단으로의 변환 행렬 (말단에 고정되어 있으므로 불변) |
| $${}^{end}_{base}M$$

 | 로봇 팔 말단 포즈를 통해 계산 가능 |
| $${}^{base}_{camera}M$$

 | 핸드-아이 캘리브레이션을 통해 구하고자 하는 목표값 |
| $${}^{camera}_{board}M$$

 | 카메라 캘리브레이션을 통해 획득 |

이를 정리하면 다음 수식 체계를 얻을 수 있습니다:

$${}^{end}_{base}M_1 \cdot {}^{base}_{camera}M_1 \cdot {}^{camera}_{board}M_1 = {}^{end}_{base}M_2 \cdot {}^{base}_{camera}M_2 \cdot {}^{camera}_{board}M_2$$

$${}^{end}_{base}M_2^{-1} \cdot {}^{end}_{base}M_1 \cdot {}^{base}_{camera}M_1 = {}^{base}_{camera}M_2 \cdot {}^{camera}_{board}M_2 \cdot {}^{camera}_{board}M_1^{-1}$$

$$\vdots$$

$${}^{end}_{base}M_n^{-1} \cdot {}^{end}_{base}M_{n-1} \cdot {}^{base}_{camera}M_{n-1} = {}^{base}_{camera}M_n \cdot {}^{camera}_{board}M_n \cdot {}^{camera}_{board}M_{n-1}^{-1}$$

이 또한 전형적인 **$AX=XB$** 문제이며, $X$는 4x4 동차 변환 행렬(여기서 $R$은 카메라에서 로봇 베이스 좌표계로의 회전 행렬, $t$는 평행 이동 벡터)입니다:

$$X = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix}$$

핸드-아이 캘리브레이션의 목적은 이 $X$를 계산하는 것입니다.

---

## 핵심 코드 설명

### 디렉토리 구조

```
---eye_hand_data           Eye-in-Hand 캘리브레이션 시 수집된 데이터
---libs
      ---auxiliary.py      보조 패키지/유틸리티 함수
      ---log_settings.py   로그 설정 패키지
---robotic_arm_package     로봇 팔 제어 Python 패키지
---collect_data.py         캘리브레이션 데이터 수집 프로그램
---compute_in_hand.py      Eye-in-Hand 캘리브레이션 연산 프로그램
---compute_to_hand.py      Eye-to-Hand 캘리브레이션 연산 프로그램
---requirements.txt        환경 의존성 파일
---save_poses.py           연산 의존 파일
---save_poses2.py          연산 의존 파일

```

---

### compute_in_hand.py | compute_to_hand.py 핵심 코드

#### 1. 메인 함수 `func()`

```python
def func():
    
    path = os.path.dirname(__file__)

    # 서브픽셀 코너 검출 종료 기준
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 캘리브레이션 보드의 3D 좌표 준비
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
    objp = L * objp

    obj_points = []     # 3D 좌표 저장 리스트
    img_points = []     # 2D 픽셀 좌표 저장 리스트

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    for i in range(1, len(images_num) + 1):
        image_file = os.path.join(images_path, f"{i}.jpg")

        if os.path.exists(image_file):
            logger_.info(f'읽는 중: {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:
                obj_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)

```

수집된 보드 이미지를 순회하며 체스보드 코너(Corner) 포인트를 검출하여 배열에 저장합니다.

#### 2. 카메라 캘리브레이션

```python
# 카메라 캘리브레이션 수행: 보드의 카메라 좌표계 기준 포즈 계산
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

```

* OpenCV의 `calibrateCamera` 함수를 통해 내부 파라미터 및 왜곡 계수를 계산합니다.
* `rvecs`와 `tvecs`는 각 이미지별 회전/이동 벡터로, **카메라 좌표계 기준 캘리브레이션 보드의 포즈**를 나타냅니다.

#### 3. 로봇 팔 포즈 데이터 처리

* **compute_in_hand.py:**

```python
poses_main(file_path)

```

로봇 팔 말단 포즈 데이터를 **베이스 좌표계 기준 로봇 팔 말단 좌표계**의 회전 행렬 및 이동 벡터로 변환합니다.

* **compute_to_hand.py:**

```python
poses2_main(file_path)

```

로봇 팔 말단 포즈 데이터를 **말단 좌표계 기준 베이스 좌표계**의 회전 행렬 및 이동 벡터로 변환합니다.

#### 4. 핸드-아이 캘리브레이션 계산

```python
R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)
return R, t

```

* OpenCV의 `calibrateHandEye` 함수를 사용하여 연산을 수행합니다.
* `compute_in_hand.py`: **로봇 팔 말단** 기준 **카메라**의 회전 행렬 `R_cam2end`와 이동 벡터 `T_cam2end`를 도출합니다.
* `compute_to_hand.py`: **로봇 베이스** 기준 **카메라**의 회전 행렬 `R_cam2base`와 이동 벡터 `T_cam2base`를 도출합니다.


* 알고리즘 방식으로는 널리 사용되는 `CALIB_HAND_EYE_TSAI` 방식을 채택했습니다.

---

### save_poses.py 핵심 코드

#### 1. 오일러 각(Euler Angle) -> 회전 행렬 변환 함수

```python
def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 회전 행렬 계산
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    return R

```

Z-Y-X 순서로 각 축의 회전 행렬을 곱하여 최종 회전 행렬 $R$을 도출합니다.

#### 2. 포즈(Pose) -> 동차 변환 행렬(Homogeneous Matrix) 변환

```python
def pose_to_homogeneous_matrix(pose):
    x, y, z, rx, ry, rz = pose
    R = euler_angles_to_rotation_matrix(rx, ry, rz)
    t = np.array([x, y, z]).reshape(3, 1)

    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t[:, 0]

    return H

```

위치 $(x, y, z)$를 이동 벡터로, 자세 $(rx, ry, rz)$를 오일러 각 기반 회전 행렬로 묶어 4x4 동차 변환 행렬 $H$를 구성합니다.

#### 3. 다중 행렬의 CSV 파일 저장

```python
def save_matrices_to_csv(matrices, file_name):
    rows, cols = matrices[0].shape
    num_matrices = len(matrices)
    combined_matrix = np.zeros((rows, cols * num_matrices))

    for i, matrix in enumerate(matrices):
        combined_matrix[:, i * cols: (i + 1) * cols] = matrix

    with open(file_name, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        for row in combined_matrix:
            csv_writer.writerow(row)

```

복수의 행렬을 가로 방향으로 병합하여 단일 CSV 파일로 저장하며, 이후 읽어 들일 때 고정 간격으로 분할하여 로드합니다.

---

## 캘리브레이션 절차

### 1. 환경 구성 요구사항

**기본 환경**

| 항목 | 버전 |
| --- | --- |
| 운영체제 | Ubuntu / Windows |
| Python | 3.9 이상 |

**Python 패키지 환경**

| 패키지 | 버전 |
| --- | --- |
| numpy | 2.0.2 |
| opencv-python | 4.10.0.84 |
| pyrealsense2 | 2.55.1.6486 |
| scipy | 1.13.1 |

다음 명령어로 패키지를 설치합니다:

```cmd
pip install -r requirements.txt

```

**필요 장비**

* 로봇 팔: RM75, RM65, RM63, GEN72 등
* 카메라: Intel RealSense Depth Camera D435
* 카메라 데이터 케이블
* 이더넷 랜선
* 체스보드 캘리브레이션 보드 (종이 출력물 또는 기성품 보드)

---

### 2. 파라미터 설정

설정 파일(`config.yaml`)에서 체스보드 파라미터를 입력합니다:

* `XX`: 보드 가로 방향 코너 수 (긴 변의 격자 수 - 1, 기본값 11)
* `YY`: 보드 세로 방향 코너 수 (짧은 변의 격자 수 - 1, 기본값 8)
* `L`: 개별 격자의 실제 크기 (단위: 미터, 기본값 0.03)

---

### 3. Eye-in-Hand 절차

**데이터 수집**

1. 카메라를 PC에 연결하고, 로봇 팔과 PC를 랜선으로 연결합니다.
2. PC와 로봇 팔의 IP 대역을 동일하게 설정합니다. (예: 로봇 팔이 `192.168.1.18`인 경우 PC는 `192.168.1.x`로 설정)
3. 보드를 평면에 고정하고 카메라를 로봇 말단에 장착한 뒤 보드를 향하게 합니다. (캘리브레이션 중 보드가 절대 움직이면 안 됩니다.)
4. `collect_data.py`를 실행합니다.
5. 로봇 팔을 움직여 카메라 화면에 보드가 선명하고 왜곡 없이 들어오도록 맞춥니다. 이때 카메라 면과 보드가 평행하지 않고 **기울어진 각도**를 유지해야 합니다.
6. 키보드 `s`를 눌러 데이터를 저장합니다.
7. 로봇 팔 말단의 회전축 각도를 크게(30° 이상) 변경해가며 3축(X, Y, Z)에 대한 회전 변화를 충분히 주어 15~20장의 데이터를 수집합니다.

**결과 연산**

* `compute_in_hand.py`를 실행하여 **로봇 말단 기준 카메라 좌표계의 회전 행렬 및 평행 이동 벡터**를 도출합니다.

---

### 4. Eye-to-Hand 절차

**데이터 수집**

1. 네트워크 및 케이블을 연결합니다.
2. **소형 보드를 로봇 팔 말단에 고정**하고, **카메라는 외부 삼각대/벽면에 고정**합니다.
3. `collect_data.py`를 실행합니다.
4. 로봇 팔을 조작하여 카메라 시야 내에 보드가 들어오도록 자세를 잡습니다.
5. `s` 키를 눌러 데이터를 저장하고, 3축에 대해 30° 이상의 회전 변화를 주며 15~20회 반복 측정합니다.

**결과 연산**

* `compute_to_hand.py`를 실행하여 **로봇 베이스 기준 카메라 좌표계의 회전 행렬 및 평행 이동 벡터**를 도출합니다.

**오차 범위:** 수집 이미지 품질에 따라 평행 이동 벡터 오차는 통상 1cm 이내로 유지됩니다.

---

## 문제 해결 (FAQ)

연산 스크립트 실행 중 아래 에러가 발생하는 경우:

```
[ERROR:0@1.418] global calibration_handeye.cpp:335 calibrateHandEyeTsai Hand-eye calibration failed! Not enough informative motions--include larger rotations.

```

* **원인:** 데이터 수집 간 회전량(특히 각 축별 회전 각도)이 부족하여 $AX=XB$ 방정식을 풀기 위한 충분한 기하학적 정보가 확보되지 않은 상태입니다.
* **해결책:**
1. **회전량 증대:** 축별 회전 각도를 30도 이상 크게 변경하여 촬영합니다.
2. **자세 다양화:** 한 축으로만 기울이지 말고 X, Y, Z 세 축을 고르게 비틀어 포즈를 생성합니다.
3. **데이터 수량 확보:** 최소 10~15개 이상의 상이한 포즈 데이터를 확보합니다.



---

## 캘리브레이션 결과 활용법

### 1. Eye-in-Hand 방식 물체 파지 좌표 변환

물체의 베이스 좌표계 기준 포즈는 다음과 같은 연쇄 곱으로 획득됩니다:

$$H^{ROB}_{OBJ} = H^{ROB}_{EE} \cdot H^{EE}_{CAM} \cdot H^{CAM}_{OBJ}$$

* $H^{ROB}_{EE}$: 로봇 제어기/API에서 실시간 취득하는 말단 포즈
* $H^{EE}_{CAM}$: 핸드-아이 캘리브레이션으로 구한 변환 행렬
* $H^{CAM}_{OBJ}$: 비전/딥러닝 모델이 측정한 카메라 기준 물체 포즈

**포인트 변환 코드 (3D 좌표 x, y, z 변환):**

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

# 핸드-아이 캘리브레이션 결과 행렬
rotation_matrix = np.array([[-0.00235395,  0.99988123, -0.01523124],
                            [-0.99998543, -0.00227965,  0.0048937 ],
                            [ 0.00485839,  0.01524254,  0.99987202]])
translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])

def convert(x, y, z, x1, y1, z1, rx, ry, rz):
    # 카메라가 검출한 물체 좌표
    obj_camera_coordinates = np.array([x, y, z])

    # 로봇 말단 포즈 (라디안 단위)
    end_effector_pose = np.array([x1, y1, z1, rx, ry, rz])

    # T_cam2end 동차 변환 행렬
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector

    # T_end2base 동차 변환 행렬
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position

    # 베이스 기준 좌표 연산
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])
    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)

    return list(obj_base_coordinates_homo[:3])

```

**포즈 전체 변환 코드 (위치 및 회전 rx, ry, rz 변환):**

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

rotation_matrix = np.array([[-0.00235395,  0.99988123, -0.01523124],
                            [-0.99998543, -0.00227965,  0.0048937 ],
                            [ 0.00485839,  0.01524254,  0.99987202]])
translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])

def decompose_transform(matrix):
    translation = matrix[:3, 3]
    rotation = matrix[:3, :3]
    sy = np.sqrt(rotation[0, 0]**2 + rotation[1, 0]**2)

    if sy >= 1e-6:
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        ry = np.arctan2(-rotation[2, 0], sy)
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        ry = np.arctan2(-rotation[2, 0], sy)
        rz = 0

    return translation, rx, ry, rz

def convert(x, y, z, rx, ry, rz, x1, y1, z1, rx1, ry1, rz1):
    obj_camera_coordinates = np.array([x1, y1, z1, rx1, ry1, rz1])
    end_effector_pose = np.array([x, y, z, rx, ry, rz])

    T_camera_to_end = np.eye(4)
    T_camera_to_end[:3, :3] = rotation_matrix
    T_camera_to_end[:3, 3] = translation_vector

    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
    T_end_to_base = np.eye(4)
    T_end_to_base[:3, :3] = orientation
    T_end_to_base[:3, 3] = position

    position2 = obj_camera_coordinates[:3]
    orientation2 = R.from_euler('xyz', obj_camera_coordinates[3:], degrees=False).as_matrix()
    T_obj_to_camera = np.eye(4)
    T_obj_to_camera[:3, :3] = orientation2
    T_obj_to_camera[:3, 3] = position2

    obj_base = T_end_to_base @ T_camera_to_end @ T_obj_to_camera
    return decompose_transform(obj_base)

```

---

### 2. Eye-to-Hand 방식 물체 파지 좌표 변환

외부에 카메라가 고정되어 있으므로 변환식이 직접 계산됩니다:

$$H^{ROB}_{OBJ} = H^{ROB}_{CAM} \cdot H^{CAM}_{OBJ}$$

**포인트 변환 코드 (3D 좌표 변환):**

```python
import numpy as np

rotation_matrix = np.array([[-0.00235395,  0.99988123, -0.01523124],
                            [-0.99998543, -0.00227965,  0.0048937 ],
                            [ 0.00485839,  0.01524254,  0.99987202]])
translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])

def convert(x, y, z):
    obj_camera_coordinates = np.array([x, y, z])

    T_camera_to_base = np.eye(4)
    T_camera_to_base[:3, :3] = rotation_matrix
    T_camera_to_base[:3, 3] = translation_vector

    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])
    obj_base_coordinates_homo = T_camera_to_base.dot(obj_camera_coordinates_homo)

    return list(obj_base_coordinates_homo[:3])

```

**포즈 전체 변환 코드 (위치 및 회전 변환):**

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

rotation_matrix = np.array([[-0.00235395,  0.99988123, -0.01523124],
                            [-0.99998543, -0.00227965,  0.0048937 ],
                            [ 0.00485839,  0.01524254,  0.99987202]])
translation_vector = np.array([-0.09321419, 0.03625434, 0.02420657])

def decompose_transform(matrix):
    translation = matrix[:3, 3]
    rotation = matrix[:3, :3]
    sy = np.sqrt(rotation[0, 0]**2 + rotation[1, 0]**2)

    if sy >= 1e-6:
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        ry = np.arctan2(-rotation[2, 0], sy)
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        ry = np.arctan2(-rotation[2, 0], sy)
        rz = 0

    return translation, rx, ry, rz

def convert(x, y, z, rx, ry, rz):
    obj_camera_coordinates = np.array([x, y, z, rx, ry, rz])

    T_camera_to_base = np.eye(4)
    T_camera_to_base[:3, :3] = rotation_matrix
    T_camera_to_base[:3, 3] = translation_vector

    position = obj_camera_coordinates[:3]
    orientation = R.from_euler('xyz', obj_camera_coordinates[3:], degrees=False).as_matrix()
    T_obj_to_camera = np.eye(4)
    T_obj_to_camera[:3, :3] = orientation
    T_obj_to_camera[:3, 3] = position

    obj_base = T_camera_to_base.dot(T_obj_to_camera)
    return decompose_transform(obj_base)

```

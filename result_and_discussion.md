
(myenv) recl3090@recl3090-Precision-3680:~/Downloads/handhand/hand_eye_calibration_rb$ python compute_in_hand_rb850.py 
==========================================================================
RB5-850 Hand-Eye: checkerboard 180deg ambiguity fixer
==========================================================================
data: eye_hand_data/data20260902011854
images: 13
poses : 13
checkerboard: 7 x 5 inner corners, 30.0 mm

Reading RealSense color intrinsic...
K:
[[606.82922363   0.         327.46951294]
 [  0.         606.95635986 240.55847168]
 [  0.           0.           1.        ]]
dist: [0. 0. 0. 0. 0.]
model: distortion.inverse_brown_conrady
distortion coefficients are all zero -> inverse/brown model distinction has no numerical effect here.

Detecting checkerboard and generating two PnP candidates...
 1.jpg: PnP RMSE 0.2650 px
 2.jpg: PnP RMSE 0.2883 px
 3.jpg: PnP RMSE 0.4620 px
 4.jpg: PnP RMSE 0.3259 px
 5.jpg: PnP RMSE 0.3112 px
 6.jpg: PnP RMSE 0.4066 px
 7.jpg: PnP RMSE 0.3310 px
 8.jpg: PnP RMSE 0.3082 px
 9.jpg: PnP RMSE 0.4259 px
10.jpg: PnP RMSE 0.2335 px
11.jpg: PnP RMSE 0.2367 px
12.jpg: PnP RMSE 0.2883 px
13.jpg: PnP RMSE 0.1919 px

Valid N=13, PnP mean/max=0.3134/0.4620 px

Building AX=XB pairwise rotation-angle constraints...
Pair rotation-angle RMS before: 0.441 deg
Pair rotation-angle RMS after : 0.441 deg
Orientation labels:
   1.jpg : NORMAL
   2.jpg : NORMAL
   3.jpg : NORMAL
   4.jpg : NORMAL
   5.jpg : NORMAL
   6.jpg : NORMAL
   7.jpg : NORMAL
   8.jpg : NORMAL
   9.jpg : NORMAL
  10.jpg : NORMAL
  11.jpg : NORMAL
  12.jpg : NORMAL
  13.jpg : NORMAL

Images corrected by 180deg reversal: none

Running hand-eye solvers after orientation correction...

--------------------------------------------------------------------------
TSAI
^TCP T_CAM [m]
[[-0.99836273 -0.05329743 -0.02076623  0.03853532]
 [ 0.02114006 -0.00645982 -0.99975565 -0.02931987]
 [ 0.05315026 -0.99855779  0.00757596  0.0532259 ]
 [ 0.          0.          0.          1.        ]]
translation [mm] = [ 38.535 -29.32   53.226]
|t| [mm] = 71.956
physical axis check = PASS
fixed-board std xyz [mm] = [4.779 4.454 5.448]
fixed-board RMS/MAX [mm] = 8.507 / 13.761
fixed-board rotation RMS/MAX [deg] = 1.0713 / 1.9467

--------------------------------------------------------------------------
PARK
^TCP T_CAM [m]
[[-0.99965254 -0.02404076  0.01080959  0.02904012]
 [-0.01095049  0.00573686 -0.99992358 -0.03133096]
 [ 0.02397691 -0.99969452 -0.00599812  0.06008897]
 [ 0.          0.          0.          1.        ]]
translation [mm] = [ 29.04  -31.331  60.089]
|t| [mm] = 73.727
physical axis check = PASS
fixed-board std xyz [mm] = [2.514 3.338 3.088]
fixed-board RMS/MAX [mm] = 5.196 / 9.802
fixed-board rotation RMS/MAX [deg] = 0.5774 / 1.1655

--------------------------------------------------------------------------
HORAUD
^TCP T_CAM [m]
[[-0.99964892 -0.0241585   0.01088178  0.02902424]
 [-0.0110226   0.00570369 -0.99992298 -0.03131971]
 [ 0.02409457 -0.99969187 -0.00596798  0.06009641]
 [ 0.          0.          0.          1.        ]]
translation [mm] = [ 29.024 -31.32   60.096]
|t| [mm] = 73.722
physical axis check = PASS
fixed-board std xyz [mm] = [2.514 3.345 3.087]
fixed-board RMS/MAX [mm] = 5.200 / 9.805
fixed-board rotation RMS/MAX [deg] = 0.5774 / 1.1635

--------------------------------------------------------------------------
ANDREFF
^TCP T_CAM [m]
[[-0.99971223 -0.02162889  0.01037517  0.02914021]
 [-0.01050652  0.00596831 -0.99992699 -0.0311972 ]
 [ 0.02156539 -0.99974825 -0.00619383  0.06017359]
 [ 0.          0.          0.          1.        ]]
translation [mm] = [ 29.14  -31.197  60.174]
|t| [mm] = 73.779
physical axis check = PASS
fixed-board std xyz [mm] = [2.564 3.161 3.095]
fixed-board RMS/MAX [mm] = 5.114 / 9.843
fixed-board rotation RMS/MAX [deg] = 0.5798 / 1.2071

--------------------------------------------------------------------------
DANIILIDIS
^TCP T_CAM [m]
[[-0.9997542  -0.01969818  0.01017455  0.03265331]
 [-0.01037992  0.01035211 -0.99989254 -0.03395087]
 [ 0.01959073 -0.99975238 -0.01055403  0.05928039]
 [ 0.          0.          0.          1.        ]]
translation [mm] = [ 32.653 -33.951  59.28 ]
|t| [mm] = 75.717
physical axis check = PASS
fixed-board std xyz [mm] = [2.406 4.008 2.715]
fixed-board RMS/MAX [mm] = 5.406 / 9.498
fixed-board rotation RMS/MAX [deg] = 0.5941 / 1.2295

==========================================================================
SUMMARY
==========================================================================
Method         Phys     |t|mm   BoardRMSmm   RotRMSdeg
TSAI           PASS    71.956        8.507      1.0713
PARK           PASS    73.727        5.196      0.5774
HORAUD         PASS    73.722        5.200      0.5774
ANDREFF        PASS    73.779        5.114      0.5798
DANIILIDIS     PASS    75.717        5.406      0.5941

Recommended: ANDREFF

BEST ^TCP T_CAM [m]
[[-0.99971223 -0.02162889  0.01037517  0.02914021]
 [-0.01050652  0.00596831 -0.99992699 -0.0311972 ]
 [ 0.02156539 -0.99974825 -0.00619383  0.06017359]
 [ 0.          0.          0.          1.        ]]

BEST ^TCP T_CAM [translation displayed in mm]
[[ -0.99971223  -0.02162889   0.01037517  29.14021415]
 [ -0.01050652   0.00596831  -0.99992699 -31.19719545]
 [  0.02156539  -0.99974825  -0.00619383  60.17359429]
 [  0.           0.           0.           1.        ]]













맞아. `best_name`은 그냥 “PARK를 미리 정해둔 것”이 아니다. **각 알고리즘으로 구한 Hand–Eye 행렬을 다시 실제 데이터에 적용해 보고, 고정되어 있어야 하는 체스보드가 얼마나 덜 흔들리는지를 측정해서 가장 좋은 것을 고른 것**이다.

핵심은 이 식이다.

$$
\boxed{
{}^{B}T_{Board,i}
=
{}^{B}T_{TCP,i}
\;{}^{TCP}T_{CAM}
\;{}^{CAM}T_{Board,i}
}
$$

Eye-in-Hand 촬영 당시 체스보드는 책상에 고정되어 있었으니까, 사진이 17장이든 100장이든 이상적으로는

$$
{}^{B}T_{Board,1}
=
{}^{B}T_{Board,2}
=
\cdots
=
{}^{B}T_{Board,17}
$$

이어야 한다.

---

## 1. 각 알고리즘마다 서로 다른 \(T_{TCP,CAM}\)을 얻는다

코드가 먼저 이렇게 5개를 각각 계산한다.

```python
TSAI
PARK
HORAUD
ANDREFF
DANIILIDIS
```

예를 들어 PARK가 구한 결과를

$$
X_{PARK}={} ^{TCP}T_{CAM}
$$

라고 하자.

그러면 모든 17개 sample에 대해:

```python
Y = G @ X_tcp_cam @ C
```

를 계산했다.

코드의 정확한 부분이:

```python
Ys = [
    G @ X_tcp_cam @ C
    for G, C in zip(G_list, C_list)
]
```

이다.

여기서:

```text
G = Base → TCP
X = TCP → Camera  (Hand-Eye 결과)
C = Camera → Checkerboard
```

따라서:

```text
Base → TCP → Camera → Checkerboard
```

가 되고 결과 `Y`는:

$$
Y={}^BT_{Board}
$$

가 된다.

---

# 2. 좋은 calibration이라면 17개의 Board 위치가 거의 같아야 한다

예를 들어 좋은 결과라면:

```text
sample 1 : [300.1, -420.3, 15.2] mm
sample 2 : [301.0, -419.4, 14.8] mm
sample 3 : [299.7, -421.0, 15.6] mm
...
```

처럼 모여야 한다.

나쁜 Hand-Eye라면:

```text
sample 1 : [300, -420, 15]
sample 2 : [410, -310, 80]
sample 3 : [180, -600, 150]
...
```

처럼 체스보드가 움직이는 것처럼 나온다.

하지만 실제 체스보드는 **안 움직였다.**

따라서 이 흔들림이 바로 Hand-Eye calibration의 품질을 나타내는 좋은 지표가 된다.

---

# 3. Translation RMS를 이렇게 계산했다

각 sample에서 얻은 체스보드 위치를:

$$
t_i =
\begin{bmatrix}
x_i\\y_i\\z_i
\end{bmatrix}
$$

라고 한다.

먼저 17개의 평균 위치:

$$
\bar t
=
\frac{1}{N}
\sum_{i=1}^{N}t_i
$$

를 구한다.

코드에서는:

```python
tmean = np.mean(ts, axis=0)
```

이다.

그 다음 각 sample이 평균에서 얼마나 떨어졌는지:

$$
d_i=t_i-\bar t
$$

```python
d = ts - tmean
```

를 구한다.

그리고 각 sample의 3차원 거리:

$$
\|d_i\|
=
\sqrt{
(x_i-\bar x)^2+
(y_i-\bar y)^2+
(z_i-\bar z)^2
}
$$

를 가지고 최종 RMS:

$$
\boxed{
E_t
=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
\|t_i-\bar t\|^2
}
}
$$

를 계산했다.

코드가 정확히:

```python
t_rms_mm = float(
    np.sqrt(
        np.mean(
            np.sum(d * d, axis=1)
        )
    ) * 1000.0
)
```

이다.

그래서 출력의

```text
BoardRMSmm
```

가 바로 이 값이다.

**작을수록 좋다.**

---

# 4. Rotation도 똑같은 개념으로 평가했다

위치는 평균을 그냥 계산할 수 있지만 rotation matrix는 원소별 평균을 단순하게 내면 안 된다.

그래서:

```python
rot = Rotation.from_matrix(Rs)
mean_rot = rot.mean()
```

으로 평균 회전을 구한다.

각 sample의 회전이 평균 회전에서 얼마나 차이나는지를:

```python
e_deg = np.degrees(
    (mean_rot.inv() * rot).magnitude()
)
```

로 계산한다.

즉 각 sample에 대해:

$$
\Delta R_i
=
\bar R^{-1}R_i
$$

를 만들고, 그 상대회전의 angle:

$$
\theta_i
$$

를 얻는다.

그 다음:

$$
\boxed{
E_R
=
\sqrt{
\frac{1}{N}
\sum_i \theta_i^2
}
}
$$

를 계산한다.

이게 출력의:

```text
RotRMSdeg
```

이다.

역시 **작을수록 좋다.**

---

# 5. 그래서 네 결과를 실제로 배열하면

네 결과는:

| Method     |    Board RMS | Rotation RMS |
| ---------- | -----------: | -----------: |
| TSAI       |     9.709 mm |      2.0079° |
| **PARK**   | **3.600 mm** |      0.6710° |
| HORAUD     |     3.601 mm |  **0.6708°** |
| ANDREFF    |     4.405 mm |      0.6714° |
| DANIILIDIS |     3.685 mm |      0.6719° |

이었다.

위치 기준으로 보면:

```text
PARK        3.600 mm   ← 1등
HORAUD      3.601 mm   ← 2등
DANIILIDIS  3.685 mm
ANDREFF     4.405 mm
TSAI        9.709 mm
```

그래서 PARK가 선택됐다.

---

# 6. 실제 `best_name` 선택 코드는 이것이다

```python
valid = [
    (name, r)
    for name, r in results.items()
    if r["ok"]
]
```

먼저 계산 자체가 실패한 알고리즘을 제거한다.

그 다음:

```python
plausible = [
    (name, r)
    for name, r in valid
    if r["physical"]
]
```

우리가 알고 있는 물리 조건:

$$
|t_x|,\;|t_y|,\;|t_z| < 100\text{ mm}
$$

을 통과한 것만 남긴다.

네 경우는 5개 모두 PASS였다.

그 다음:

```python
pool.sort(
    key=lambda x: (
        x[1]["validation"]["t_rms_mm"],
        x[1]["validation"]["r_rms_deg"],
    )
)
```

이게 핵심이다.

Python tuple sorting이므로 우선순위는:

```text
1순위 : translation RMS가 작은 것
2순위 : translation RMS가 같으면 rotation RMS가 작은 것
```

이다.

그리고:

```python
best_name, best = pool[0]
```

즉 정렬한 결과의 **맨 앞에 있는 것**을 선택한다.

그래서:

```python
best_name = "PARK"
```

가 된 것이다.

---

# 7. HORAUD가 Rotation은 더 좋은데 왜 PARK인가?

좋은 질문이 바로 이 부분이다.

```text
PARK
translation = 3.600 mm
rotation    = 0.6710°

HORAUD
translation = 3.601 mm
rotation    = 0.6708°
```

HORAUD가 rotation은 아주 조금 더 좋다.

그런데 우리가 정렬을:

```python
(
    translation_RMS,
    rotation_RMS
)
```

순서로 했기 때문이다.

따라서:

```text
PARK   (3.600, 0.6710)
HORAUD (3.601, 0.6708)
```

이면 첫 번째 숫자부터 비교한다.

$$
3.600 < 3.601
$$

이므로 PARK가 선택되고 두 번째 값은 비교할 필요가 없다.

또 출력은 소수점 3자리로 잘라 보여준 것이기 때문에 실제 내부 값은 예를 들어:

```text
PARK   = 3.5996...
HORAUD = 3.6012...
```

처럼 더 자세한 값을 가지고 정렬된다.

---

# 8. 그런데 여기서 중요한 것: PARK가 HORAUD보다 "본질적으로 우수하다"는 뜻은 아니다

이건 꼭 구분해야 한다.

현재 데이터에서는:

$$
3.600\text{ mm}
$$

와

$$
3.601\text{ mm}
$$

차이다.

차이가:

$$
0.001\text{ mm}=1\,\mu m
$$

수준인데, D435와 30 mm checkerboard를 사용하는 현재 실험에서 **이 차이는 사실상 의미가 없다.**

따라서 과학적으로 해석하면:

> PARK와 HORAUD는 사실상 동등한 결과를 냈고, 코드의 정렬 규칙 때문에 PARK가 1위로 선택되었다.

가 정확하다.

오히려 우리가 신뢰할 수 있는 중요한 부분은:

```text
PARK       ≈ 3.60 mm / 0.67°
HORAUD     ≈ 3.60 mm / 0.67°
DANIILIDIS ≈ 3.69 mm / 0.67°
```

처럼 **서로 다른 세 알고리즘이 거의 같은 결과로 수렴했다는 것**이다.

---

# 9. 그리고 `Pair rotation RMS = 0.468°`는 best method 선택 기준이 아니다

이것도 코드상 분리되어 있다.

우리가 앞에서:

```text
before = 50.931°
after  = 0.468°
```

을 계산했지.

그건:

> 어떤 사진의 checkerboard corner 순서를 180° 뒤집어야 하는가?

를 찾기 위한 것이다.

즉:

```text
Pair rotation RMS
       ↓
Checkerboard 180° orientation 결정
```

에 사용한다.

그 이후:

```text
Board Translation RMS
Board Rotation RMS
       ↓
Hand-Eye solver 선택
```

이다.

서로 다른 단계다.

---

# 그래서 코드의 판단 구조 전체는 이렇다

```text
17 images + 17 robot poses
          │
          ↓
checkerboard PnP
          │
          ↓
NORMAL / REVERSED 후보
          │
          ↓
AX=XB 상대회전 검사
          │
          ↓
180° 뒤집힌 image 수정
          │
          ├───────────────┐
          ↓               ↓
       TSAI             PARK
       HORAUD           ...
          │
          ↓
각각 T_TCP_CAM 계산
          │
          ↓
각 solver마다
Base→Board를 17번 역산
          │
          ↓
실제로 Board는 고정되어 있으므로
얼마나 같은 위치가 나오는지 측정
          │
          ↓
┌─────────────────────────┐
│ physical 조건 PASS?     │
│ |tx|,|ty|,|tz| < 100mm │
└─────────────────────────┘
          │
          ↓
Translation RMS 작은 순
          │
          ↓
동률이면 Rotation RMS 작은 순
          │
          ↓
       best_name
```

### 따라서 현재 코드에서 "`good`"의 정확한 정의는

$$
\boxed{
\text{Good}
=
\text{물리적으로 가능한 해}
+
\text{고정 Board가 가장 덜 움직여 보이는 해}
}
$$

다.

그리고 네 데이터에서는 **PARK와 HORAUD가 사실상 공동 1등이고**, 코드 규칙상 PARK가 아주 근소하게 선택된 것이다.

개인적으로 지금처럼 calibration 품질을 논문이나 실험 결과로 다룬다면 `Recommended: PARK` 하나만 보여주기보다 **PARK/HORAUD/DANIILIDIS 세 방법의 결과가 서로 얼마나 가까운지도 같이 제시하는 것**이 더 설득력 있다.






이번 데이터셋도 **정상적으로 Hand–Eye calibration이 된 결과**로 보인다. 오히려 이번에는 처음부터 checkerboard 방향이 모두 일관되게 검출돼서,

```text
Pair rotation-angle RMS
before = 0.441°
after  = 0.441°
```

이고 `REVERSED 180deg = none`이다. 즉 지난 데이터셋에서 발생했던 180° checkerboard ambiguity가 이번 촬영에서는 발생하지 않았다.

그리고 이번에 `Recommended: ANDREFF`가 나온 이유도 이전에 설명한 `best_name` 선택 로직 때문이다.

네 결과를 보면:

| Method      | Translation RMS | Rotation RMS |
| ----------- | --------------: | -----------: |
| TSAI        |        8.507 mm |      1.0713° |
| PARK        |        5.196 mm |  **0.5774°** |
| HORAUD      |        5.200 mm |  **0.5774°** |
| **ANDREFF** |    **5.114 mm** |      0.5798° |
| DANIILIDIS  |        5.406 mm |      0.5941° |

현재 코드는 우선:

```python
pool.sort(
    key=lambda x: (
        x[1]["validation"]["t_rms_mm"],
        x[1]["validation"]["r_rms_deg"],
    )
)
```

즉,

```text
1순위: Translation RMS
2순위: Rotation RMS
```

로 정렬한다.

그래서:

$$
5.114 < 5.196
$$

이므로 ANDREFF가 선택된 것이다.

---

## 그런데 나는 ANDREFF가 PARK보다 "더 좋은 calibration"이라고 단정하지 않겠다

차이를 보면:

```text
ANDREFF
translation RMS = 5.114 mm
rotation RMS    = 0.5798°

PARK
translation RMS = 5.196 mm
rotation RMS    = 0.5774°
```

Translation 차이:

$$
5.196-5.114=0.082\text{ mm}
$$

밖에 안 된다.

반대로 rotation은 PARK가 조금 더 좋다.

$$
0.5774^\circ < 0.5798^\circ
$$

이 정도 차이는 D435 + checkerboard 실험 오차를 생각하면 사실상 **동급**이라고 보는 게 맞다.

그래서 이번 데이터의 정확한 해석은:

> ANDREFF가 압도적으로 가장 좋다.

가 아니라,

> **PARK / HORAUD / ANDREFF가 거의 동일한 해로 수렴했고, 코드의 translation RMS 우선 정렬 때문에 ANDREFF가 1위가 되었다.**

가 맞다.

---

## 더 중요한 건 세 알고리즘이 구한 행렬 자체가 굉장히 비슷하다는 것

Translation을 비교해보자.

### PARK

```text
[ 29.040
 -31.331
  60.089 ] mm
```

### HORAUD

```text
[ 29.024
 -31.320
  60.096 ] mm
```

### ANDREFF

```text
[ 29.140
 -31.197
  60.174 ] mm
```

거의 동일하다.

PARK와 ANDREFF 차이만 계산해도:

```text
dx ≈ 0.10 mm
dy ≈ 0.13 mm
dz ≈ 0.08 mm
```

정도다.

즉 세 solver가 사실상 동일한

$$
{}^{TCP}T_{CAM}
$$

을 찾고 있다.

이게 **Recommended: ANDREFF**라는 글자 자체보다 훨씬 중요한 결과다.

---

# 그리고 이전 calibration과도 비교해보자

지난번 PARK 결과는:

```text
[32.757, -32.568, 59.613] mm
```

이번 PARK 결과는:

```text
[29.040, -31.331, 60.089] mm
```

차이는:

$$
\Delta t
\approx
[-3.72,\;1.24,\;0.48]\text{ mm}
$$

정도다.

전체 translation 거리 차이는 대략 4 mm 정도다.

두 데이터셋을 완전히 새로 촬영했는데도 **약 4 mm 범위에서 같은 Hand–Eye 결과가 재현됐다**는 뜻이다.

그리고 전체 camera offset도:

지난번:

$$
75.4\text{ mm}
$$

이번:

$$
73.7\text{ mm}
$$

정도로 비슷하다.

이건 상당히 좋은 검증이다.

---

## Rotation도 거의 같은 형태다

이번 PARK:

$$
R=
\begin{bmatrix}
-0.99965 & -0.02404 & 0.01081\\
-0.01095 & 0.00574 & -0.99992\\
0.02398 & -0.99969 & -0.00600
\end{bmatrix}
$$

지난 PARK:

$$
R=
\begin{bmatrix}
-0.99990 & -0.01312 & 0.00489\\
-0.00500 & 0.00798 & -0.99996\\
0.01308 & -0.99988 & -0.00805
\end{bmatrix}
$$

큰 구조는 완전히 동일하다.

```text
TCP X ≈ -Camera X
TCP Y ≈ -Camera Z
TCP Z ≈ -Camera Y
```

형태가 계속 유지되고 있다.

그래서 기구적으로도 상당히 일관적이다.

---

# 그러면 실제 사용할 행렬은 PARK냐 ANDREFF냐?

내가 네 시스템이라면 **PARK를 계속 사용하겠다.**

이유는 이번 한 번의 ranking 때문이 아니라 반복 실험까지 봤을 때:

```text
첫 calibration
PARK → best

두 번째 calibration
PARK / HORAUD / ANDREFF → 사실상 동급
```

이고, PARK가 두 번 모두 매우 안정적으로 같은 영역에 수렴했기 때문이다.

이번 ANDREFF의 0.082 mm RMS 우위는 알고리즘 선택을 바꿀 만큼 의미 있는 차이가 아니다.

그래서 최종적으로는 이번 데이터의 PARK:

$$
\boxed{
{}^{TCP}T_{CAM}
=
\begin{bmatrix}
-0.99965254 & -0.02404076 & 0.01080959 & 0.02904012\\
-0.01095049 & 0.00573686 & -0.99992358 & -0.03133096\\
0.02397691 & -0.99969452 & -0.00599812 & 0.06008897\\
0 & 0 & 0 & 1
\end{bmatrix}
}
$$

를 한 후보로 쓰는 게 합리적이다.

다만 **더 좋은 방법은 첫 번째와 두 번째 calibration 중 하나를 눈감고 선택하는 게 아니다.**

이제 네가 실제로 원하는 manipulation 관점에서 확인하면 된다.

$$
{}^BT_{OBJ}
=
{}^BT_{TCP}
{}^{TCP}T_{CAM}
{}^{CAM}T_{OBJ}
$$

로 실제 checkerboard 또는 ArUco 같은 고정 target을 여러 robot pose에서 계산해서, **Base 좌표에서 실제 위치가 몇 mm 흔들리는지 직접 측정**하면 된다.

현재 calibration 내부 검증에서는 대략 **5 mm RMS** 수준이므로, 실제 외부 검증에서도 비슷한 수준이면 현재 Hand–Eye를 성공적으로 사용할 수 있다고 보면 된다.

그리고 한 가지는 코드도 바꾸는 걸 추천한다. 지금처럼 0.08 mm 차이로 `ANDREFF`가 자동으로 `best`가 되는 건 조금 과민하다. 예를 들어 **translation RMS 차이가 0.5 mm 이내면 동급으로 보고, 그 안에서는 rotation RMS 또는 반복 실험 안정성을 기준으로 고르는 방식**이 훨씬 합리적이다.


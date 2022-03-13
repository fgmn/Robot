

# def convert(cc):
    # 设 P 在相机坐标系下为 P_c = (x1, y1, z1) ^ T，在机械臂坐标系为 P_w = (x2, y2, z2) ^ T
    # P_w = T * P_c 其中 T 为相机坐标系 c 在机械臂坐标系 w 下的表示
    # R = mat()
    # T = mat()
    # wc = R * cc + T # (3*3) * (3*1) + (3*1)
    # return wc


import numpy as np

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]  # total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA), BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.matmul(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T, U.T)

    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
    return R, t


# 点在相机坐标系下的表示
# a = np.array([[-55.55, 153.22, 752.74],
#               [80.00, 138.92, 676.46],
#               [87.19, 230.55, 671.54],
#               [-63.65, 219.63, 665.57],
#               [-83.97, 209.36, 614.94],
#               [-104.13, 296.12, 825.91],
#               [138.61, 223.97, 621.39],
#               [-108.83, 218.18, 678.79],
              # [86.44, 229.41, 675.41],
              # [87.66, 229.25, 674.94],
              # [2.87, 189.82, 676.38],
              # [179.39, 260.17, 729.21]])

 #[24.32,151.57,588.11],
            #[11.79,135.90,671.19],
            #[32.38,181.93,519.94],
           # [-7.61,179.19,638.95],

a=np.array([[-133.73,239.70,631.85],
             [-175.98,160.74,676.34],
             [-233.39,181.91,617.19],
             [-215.87,167.96,640.10],
             [121.87,242.34,706.02],
             [62.21,194.85,692.37],
             [68.94,212.82,743.71],
             [114.77,243.47,699.77],
           ])

# [-181.70,-50.60,351.60],
# [-152.30,-54.40,374.50],
# [-232.60,-54.60,288.60],
# [-162.60,-38.50,318.30],
b=np.array([[-100.00,102.70,278.50],
            [-100.40,109.50,378.40],
            [-130.10,166.30,344.10],
            [-117.00,99.60,368.40],
            [-78.30,-164.40,330.20],
            [-108.20,-87.10,363.40],
            [-60.70,-111.80,369.40],
            [-78.40,-164.40,318.20],
            ])

# 在机械臂坐标系下的表示
# b = np.array([[-87.90, 80.00, 386.80],
#               [-162.70, -53.40, 360.10],
#               [-98.60, -58.00, 299.50],
#               [-100.50, 98.90, 300.00],
#               [-146.70, 98.90, 304.58],
#               [39.10, 106.40, 327.50],
#               [-152.70, -99.00, 299.20],
#               [-99.30, 118.70, 317.90],
              # [-106.60, 150.60, 213.70],
              # [-7.00, 184.40, 213.50],
              # [-118.20, 24.80, 326.50],
              # [-57.40, -137.70, 310.50]])



c = np.reshape(a[-2:], (2, 3))
test_a1 = np.reshape(c[0], (1, 3))
test_a2 = np.reshape(c[1], (1, 3))

c = np.reshape(b[-2:], (2, 3))
test_b1 = np.reshape(c[0], (1, 3))
test_b2 = np.reshape(c[1], (1, 3))

a = a[:-2]
b = b[:-2]
r, t = rigid_transform_3D(a, b)
print('r:', r)
print('t:', t)

# 数据筛查
bb = np.matmul(a, r.T) + t.reshape([1, 3])
print('b-bb:', b - bb)


c = np.matmul(test_a1, r.T) + t.reshape([1, 3])
print('c-test_b1:', c - test_b1)

c = np.matmul(test_a2, r.T) + t.reshape([1, 3])
print('c-test_b2:', c - test_b2)


# PCA










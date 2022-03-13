
from math import *


# def I(X, Y, Z, RX, RY, RZ):
#     px = X
#     py = Y
#     pz = Z
#
#     # 1.RX, RY, RZ -> n, a, p
#     # 2.变换使得 d6 = 0
#
#     nx = cos(RZ) * cos(RY)
#     ny = sin(RZ) * cos(RY)
#     nz = -sin(RY)
#     ox = cos(RZ) * sin(RY) * sin(RX) - sin(RZ) * cos(RX)
#     oy = sin(RZ) * sin(RY) * sin(RX) + cos(RZ) * cos(RX)
#     oz = cos(RY) * sin(RX)
#     ax = cos(RZ) * sin(RY) * cos(RX) + sin(RZ) * sin(RX)
#     ay = sin(RZ) * sin(RY) * cos(RX) - cos(RZ) * sin(RX)
#     az = cos(RY) * cos(RX)
#
#     # return [[nx, ny, nz], [ox, oy, oz], [ax, ay, az]]
#     return [nx, ny, nz, ox, oy, oz, ax, ay, az]
#
# # Res = I(1 , 2, 3, 180, 0, 0)
# Res = I(62, -63, 398, -75, 0, -90)
#
# print(Res)

# A = [1, 2, 3]
# B = [0, 0, 0, 0, 0, 0]
# C = [[1, 2, 3], []]
# B[3:6] = C[0] # : 左开右闭
# # B[4] = A[1]
# # B[5] = A[2]
# print(B)
# B[0] = -1
# B[1] = -2
# B[2] = -3
# print(B)


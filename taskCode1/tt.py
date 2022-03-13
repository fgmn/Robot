
from numpy import *


def trans(u, v):
    ## 相机参数区
    Z_c = 500.0  ## unit : mm
    f_x = 1728.4
    f_y = 1728.4
    u_0 = 640 / 2  ## 根据最大分辩率计算
    v_0 = 480 / 2

    # R： Singular matrix
    # 非方阵逆的计算
    R_T = mat([[-1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, -1, Z_c],
              [0, 0, 0, 1]])
    R = mat([[-1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, -1, Z_c],
              [0, 0, 0, 0]])
    P = mat([[f_x, 0, u_0, 0],
            [0, f_y, v_0, 0],
            [0, 0, 1, 0]])
'''
I 0
0 0
'''

    R_ = R_T.I
    P_ = P.I

    coords_px = mat([[u], [v], [1]])
    coords_px = Z_c * coords_px

    coords_world = R_ * P_ * coords_px           ## 4*4 4*3 3*1
    coords_world[2] += Z_c
    coords_world *= 5

    coords_world = coords_world.tolist()
    print(coords_world)
    print(type(coords_world))

    return coords_world

res = trans(320, 480)
print(round(res[2][0], 2))
# trans(640, 240)
# trans(640, 480)


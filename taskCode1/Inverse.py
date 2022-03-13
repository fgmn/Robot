
# 固定了末端执行器的姿态， 即目标位姿矩阵中的 R 部分固定
# 只计算了一组解
# 关于多重解问题未仔细考究

from numpy import *
from math import *
# UDP
from socket import socket, AF_INET, SOCK_DGRAM

# UDP通信参数
ADDRESS = "172.20.10.6"
PORT = 5555

s = socket(AF_INET, SOCK_DGRAM)

PI = pi

# 工具
Tlength = 0

# D_H parameter
alpha1 = 0.0
alpha2 = PI/2
alpha3 = 0.0
alpha4 = 0.0
alpha5 = PI/2
alpha6 = -PI/2

a1 = 0.0
a2 = 0.0
a3 = -110.4
a4 = -96.0
a5 = 0.0
a6 = 0.0

d1 = 131.56
d2 = 0.0
d3 = 0.0
d4 = 64.62
d5 = 73.18
d6 = 48.6 + Tlength

offset1 = 0.0
offset2 = -PI/2
offset3 = 0.0
offset4 = -PI/2
offset5 = PI/2
offset6 = 0.0

# variable
theta1 = -1.0
theta2 = -1.0
theta3 = -1.0
theta4 = -1.0
theta5 = -1.0
theta6 = -1.0

# input
# 位姿 matrix
# 设置夹爪固定姿态
nx = 1.0
ny = 0.0
nz = 0.0
ox = 0.0
oy = -1.0
oz = 0.0
ax = 0.0
ay = 0.0
az = -1.0


px = -1.0
py = -1.0
pz = -1.0

# 是否计算末端位姿
op = True

def I(X, Y, Z, RX, RY, RZ):
    px = X
    py = Y
    pz = Z

    # RX, RY, RZ -> n, a, p
    # p33
    if op == True:
        nx = cos(RZ) * cos(RY)
        ny = sin(RZ) * cos(RY)
        nz = -sin(RY)
        ox = cos(RZ) * sin(RY) * sin(RX) - sin(RZ) * cos(RX)
        oy = sin(RZ) * sin(RY) * sin(RX) + cos(RZ) * cos(RX)
        oz = cos(RY) * sin(RX)
        ax = cos(RZ) * sin(RY) * cos(RX) + sin(RZ) * sin(RX)
        ay = sin(RZ) * sin(RY) * cos(RX) - cos(RZ) * sin(RX)
        az = cos(RY) * cos(RX)

    # θ_1=atan2(p_y,p_x )-atan2(-d_4±√(p_x^2+p_y^2-d_4^2 ))
    theta1 = atan2(py, px) - atan2(-d4, (px * px + py * py - d4 * d4) ** 0.5)
    # 3.14 rad = 180 deg

    c1 = cos(theta1)
    s1 = sin(theta1)

    # s_5=±√((-s_1 n_x+c_1 n_y )^2+(-s_1 o_x+c_1 o_y )^2 )
    s5 = ((-s1 * nx + c1 * ny) ** 2 + (-s1 * ox + c1 * oy) ** 2) ** 0.5

    # θ_5=atan2(s_5,s_1 a_x-c_1 a_y)
    theta5 = atan2(s5, s1 * ax - c1 * ay)

    # θ_6=atan2((-s_1 o_x+c_1 o_y)/s_5 ,(s_1 n_x-c_1 n_y)/s_5 )
    theta6 = atan2((-s1 * ox + c1 * oy) / s5, (s1 * nx - c1 * ny) / s5)

    # θ_234=atan2(-a_z/s_5 ,-(c_1 a_x+s_1 a_y)/s_5 )
    theta234 = atan2(-az / s5, -(c1 * ax + s1 * ay) / s5)
    # print("theta234: ", theta234 * 180 / PI)
    s234 = sin(theta234)
    c234 = cos(theta234)

    # A=-2B_2 a_3
    # B=2B_1 a_3
    # C=B_1^2+B_2^2+a_3^2-a_4^2
    # B_1=c_1 p_x+s_1 p_y-d_5 s_234
    # B_2=p_z-d_1+d_5 c_234
    B1 = c1 * px + s1 * py - d5 * s234 # + d6 * c234 * s5
    B2 = pz - d1 + d5 * c234 # + d6 * s234 * s5
    C = B1 * B1 + B2 * B2 + a3 * a3 - a4 * a4 # *
    A = -2.0 * B2 * a3
    B = 2.0 * B1 * a3

    # θ_2=atan2(B,A)-atan2(C,±√(A^2+B^2-C^2 ))
    # no solutions
    try:
        theta2 = atan2(B, A) - atan2(C, (A * A + B * B - C * C) ** 0.5)
    except:
        return -1
    else:

        s2 = sin(theta2)
        c2 = cos(theta2)

        # s_23=(B_2-a_3 s_2)/a_4
        # c_23=(B_1-a_3 c_2)/a_4

        # θ_23=atan2((B_2-a_3 s_2)/a_4 ,(B_1-a_3 c_2)/a_4 )
        theta23 = atan2((B2 - a3 * s2) / a4, (B1 - a3 * c2) / a4)

        theta3 = theta23 - theta2
        theta4 = theta234 - theta23


        Res = [theta1 - offset1, theta2 - offset2, theta3 - offset3, theta4 - offset4, theta5 - offset5, theta6 - offset6]
        for i in range(6):
            Res[i] = Res[i] * 180 / PI
            if Res[i] > 180:
                Res[i] = Res[i] - 360
            if Res[i] < -180:
                Res[i] = Res[i] + 360

        # print("{} {} {} {} {} {}".format(Res[0], Res[1], Res[2], Res[3], Res[4], Res[5]))
        return Res


while True:
    msg = input("> ")
    if msg[0:3] == "wcs":
        p = msg.split(" ")
        Res = I(float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]))
        if Res == -1:
            print("can't reach!")
            continue
        # 构造was命令
        msg = "was"
        for i in range(6):
            msg = msg + " " + str(round(Res[i], 2))
    print(msg)
    s.sendto(msg.encode(), (ADDRESS, PORT))

s.close()

# test about pose
# 1
# wcs 159 0 250 0 0 0
# gcs 155 0 253 -87 66 -63
# 2
# wcs 160 100 250 180 0 0
# gcs 140 116 246 -90 -24 -7

# wcs 62 -63 398 -75 0 -90


# nx = cos(RY) * cos(RZ)
# ny = sin(RX) * sin(RY) * cos(RZ) + cos(RX) * sin(RZ)
# nz = -cos(RX) * sin(RY) * cos(RZ) + sin(RX) * sin(RZ)
#
# ox = -cos(RY) * sin(RZ)
# oy = -sin(RX) * sin(RY) * sin(RZ) + cos(RX) * cos(RZ)
# oz = cos(RX) * sin(RY) * sin(RZ) + sin(RX) * cos(RZ)
#
# ax = sin(RY)
# ay = -sin(RX) * cos(RY)
# az = cos(RX) * cos(RY)


## work ##

# coords
# cal res
# real res

# 1
# wcs 40 -80 200 -75 0 -90
# gas 1 -34 51 -2 -1 -1
# have a crash

# 2
# wcs 25 -63 227 -36 0 -90 no use
# was 0 -135 150 40 0 0


# 3 ***
# wcs 62 -63 398 -75 0 -90
# 1.gas 1 -35 51 -1 -1 0
# 2.gas 30 23 -54 48 -29 -8
# -28.64788975654116 58.969026734014044 63.02535746439057
# can't reach!
# was 88.63 34.42 -18.03 -1.38 1.33 -89.64
# gcs 61 -68 398 0 -75 -90 !!! !!!

# swap RX RY
# was 119.06 30.34 -20.22 18.75 57.6 -155.03
# gcs 64 -64 398 74 0 -90

# 31
# wcs 60 -90 390 -75 0 -90

# 32


# 4
# wcs 80 160 200 0 90 90
# gas 82 -140 106 34 8 90
# was -124.45 49.59 -6.46 136.87 55.55 -0.0
# gcs 77 159 197 90 0 90
'''
wcs -80 -160 200 0 90 0
gas -110 -111 40 70 109 90
-108.86198107485642 70.4281826366305 -91.67324722093171
can't reach!

wcs -100 -100 200 180 0 0
gas -108 23 -105 -8 0 161
was -107.81 22.47 -103.47 -9.0 0.0 162.19 !!!

wcs 100 -100 150 180 0 0
gas -18 -95 133 -127 0 -108
was -17.81 18.92 -131.85 22.92 0.0 -107.81 !!!

wcs 100 -150 250 180 90 0
1.was 84.86 111.76 -85.64 -26.12 -84.86 -90.0
2.was 116.91 80.44 -6.74 -73.71 63.09 90.0
gcs 90 -90 90
'''



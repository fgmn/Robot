

from numpy import *
from math import *
# UDP
from socket import socket, AF_INET, SOCK_DGRAM

#
# UDP通信参数
ADDRESS = "172.20.10.6"
PORT = 5555

s = socket(AF_INET, SOCK_DGRAM)



def I(X, Y, Z, RX, RY, RZ):
    PI = pi

    # 工具
    Tlength = 0

    # D_H parameter
    alpha1 = 0.0
    alpha2 = PI / 2
    alpha3 = 0.0
    alpha4 = 0.0
    alpha5 = PI / 2
    alpha6 = -PI / 2

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
    d6 = 48.6

    offset1 = 0.0
    offset2 = -PI / 2
    offset3 = 0.0
    offset4 = -PI / 2
    offset5 = PI / 2
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

    # RX, RY, RZ -> n, a, p
    # p33
    if op == True:

        # deg -> rad
        RX = RX / 180 * PI
        RY = RY / 180 * PI
        RZ = RZ / 180 * PI

        nx = cos(RZ) * cos(RY)
        ny = sin(RZ) * cos(RY)
        nz = -sin(RY)

        ox = cos(RZ) * sin(RY) * sin(RX) - sin(RZ) * cos(RX)
        oy = sin(RZ) * sin(RY) * sin(RX) + cos(RZ) * cos(RX)
        oz = cos(RY) * sin(RX)

        ax = cos(RZ) * sin(RY) * cos(RX) + sin(RZ) * sin(RX)
        ay = sin(RZ) * sin(RY) * cos(RX) - cos(RZ) * sin(RX)
        az = cos(RY) * cos(RX)

        px = X - ax * (d6 + Tlength)
        py = Y - ay * (d6 + Tlength)
        pz = Z - az * (d6 + Tlength)

    # print(nx, ny, nz, ox, oy, oz, ax, ay, az)

    theta1 = atan2(py, px) - atan2(-d4, (px * px + py * py - d4 * d4) ** 0.5)

    c1 = cos(theta1)
    s1 = sin(theta1)

    s5 = ((-s1 * nx + c1 * ny) ** 2 + (-s1 * ox + c1 * oy) ** 2) ** 0.5

    theta5 = atan2(s5, s1 * ax - c1 * ay)

    theta6 = atan2((-s1 * ox + c1 * oy) / s5, (s1 * nx - c1 * ny) / s5)

    theta234 = atan2(-az / s5, -(c1 * ax + s1 * ay) / s5)
    s234 = sin(theta234)
    c234 = cos(theta234)

    B1 = c1 * px + s1 * py - d5 * s234 # + d6 * c234 * s5
    B2 = pz - d1 + d5 * c234 # + d6 * s234 * s5
    C = B1 * B1 + B2 * B2 + a3 * a3 - a4 * a4 # *
    A = -2.0 * B2 * a3
    B = 2.0 * B1 * a3

    print((round(theta1, 1) - offset1) * 180 / PI, (round(theta5, 1) - offset5) * 180 / PI, (round(theta6, 1) - offset6) * 180 / PI)
    try:
        theta2 = atan2(B, A) - atan2(C, (A * A + B * B - C * C) ** 0.5)
    except:
        return -1
    else:

        s2 = sin(theta2)
        c2 = cos(theta2)

        theta23 = atan2((B2 - a3 * s2) / a4, (B1 - a3 * c2) / a4)

        theta3 = theta23 - theta2
        theta4 = theta234 - theta23

        Res = [theta1 - offset1, theta2 - offset2, theta3 - offset3, theta4 - offset4, theta5 - offset5, theta6 - offset6]
        # Res = [ rads ]
        for i in range(6):
            Res[i] = Res[i] * 180 / PI
            if Res[i] > 180:
                Res[i] = Res[i] - 360
            if Res[i] < -180:
                Res[i] = Res[i] + 360
        # Res = [ angles ]
        return Res

while True:
    msg = input("> ")
    if msg[0:3] == "wcs":
        p = msg.split(" ")
        Res = I(float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6]))
        if Res == -1:
            print("can't reach!")
            continue
        # 构造 was 命令
        msg = "was"
        for i in range(6):
            msg = msg + " " + str(round(Res[i], 2))
    print(msg)
    # s.sendto(msg.encode(), (ADDRESS, PORT))

s.close()

#






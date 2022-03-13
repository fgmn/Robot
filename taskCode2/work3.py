
from numpy import *
from math import *
from socket import socket, AF_INET, SOCK_DGRAM
import time

# UDP通信参数
ADDRESS = "192.168.8.103"
PORT = 5555

# 工作解选取
priority_joint = [4, 5, 0, 1, 2, 3, 6, 7]

s = socket(AF_INET, SOCK_DGRAM)

def ini():
    msg = "was 0 -130 150 40 0 0"
    s.sendto(msg.encode(), (ADDRESS, PORT))


def I(X, Y, Z, RX, RY, RZ):
    PI = pi

    # 工具
    Tlength = 200.0

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

    Res = [[], [], [], [], [], [], [], []]
    # 多重解
    sign = [[1, 1, 1],
            [1, 1, -1],
            [1, -1, 1],
            [1, -1, -1],
            [-1, 1, 1],
            [-1, 1, -1],
            [-1, -1, 1],
            [-1, -1, -1], ]

    for i in range(8):
        sign1 = sign[i][0]
        sign2 = sign[i][1]
        sign3 = sign[i][2]

        try:
            theta1 = atan2(py, px) - atan2(-d4, sign1 * (px * px + py * py - d4 * d4) ** 0.5)
            if isnan(theta1):
                Res[i] = -1
                continue
        except:
            Res[i] = -1
            continue
        else:
            c1 = cos(theta1)
            s1 = sin(theta1)

            s5 = sign2 * ((-s1 * nx + c1 * ny) ** 2 + (-s1 * ox + c1 * oy) ** 2) ** 0.5

            theta5 = atan2(s5, s1 * ax - c1 * ay)

            theta6 = atan2((-s1 * ox + c1 * oy) / s5, (s1 * nx - c1 * ny) / s5)

            theta234 = atan2(-az / s5, -(c1 * ax + s1 * ay) / s5)
            s234 = sin(theta234)
            c234 = cos(theta234)

            B1 = c1 * px + s1 * py - d5 * s234  # + d6 * c234 * s5
            B2 = pz - d1 + d5 * c234  # + d6 * s234 * s5
            C = B1 * B1 + B2 * B2 + a3 * a3 - a4 * a4  # *
            A = -2.0 * B2 * a3
            B = 2.0 * B1 * a3
            # print("A:",A,"B:",B,"C:",C,"B1:",B1,"B2:",B2)
            # print((round(theta1, 1) - offset1) * 180 / PI, (round(theta5, 1) - offset5) * 180 / PI, (round(theta6, 1) - offset6) * 180 / PI)
            try:
                theta2 = atan2(B, A) - atan2(C, sign3 * (A * A + B * B - C * C) ** 0.5)  # get NaN, no error
                if isnan(theta2):
                    Res[i] = -1
                    continue
            except:
                Res[i] = -1
                continue  # 跳过该解
            else:

                s2 = sin(theta2)
                c2 = cos(theta2)

                theta23 = atan2((B2 - a3 * s2) / a4, (B1 - a3 * c2) / a4)

                theta3 = theta23 - theta2
                theta4 = theta234 - theta23

                Res[i] = [theta1 - offset1, theta2 - offset2, theta3 - offset3, theta4 - offset4, theta5 - offset5,
                          theta6 - offset6]
                # Res = [ rads ]
                for j in range(6):
                    Res[i][j] = Res[i][j] * 180 / PI
                    if Res[i][j] > 180:
                        Res[i][j] = Res[i][j] - 360
                    if Res[i][j] < -180:
                        Res[i][j] = Res[i][j] + 360
                    Res[i][j] = round(Res[i][j], 2)
                # Res = [ angles ]
                print(i, "th solution:  ", Res[i])
    return Res


def work3(coords):
    # TEST DATA
    # 0 * 6 : 43 -63 412 -89 0 -89
    # 0 -15 30 30 0 0 : -18 -63 415 -44 0 -90
    # 5 号舵机 == 30 deg : -22 -40 411 -48 -20 -67
    # 45 deg : -27 -31 406 -54 -30 -54
    # 60 deg : -33 -24 400 -63 -37 -39


    # 理想位置的坐标参数 test
    x_dream = -1
    y_dream = -63
    z_dream = 412

    # z_o 为 5 号坐标系原点的 z 值 test
    # 根据姿态计算
    z_o = z_dream - 5

    # 计算目标位姿
    d6 = 48.6
    Tlength = 200.0

    len = d6 + Tlength

    D = coords[1] - y_dream
    len = Tlength + d6
    L = (len ** 2 - D ** 2) ** 0.5
    ang5 = tan(D / L)  ## + -

    h = coords[2] - z_o
    x_ = (L ** 2 - h ** 2) ** 0.5
    angy = tan(h / x_)

    # calculation for bias
    bias = -1

    # Target = (coords[0], coords[1], coords[2], ang5, angy, 0)
    coords[3] = ang5
    coords[4] = angy + bias
    coords[5] = 0.0
    Res = I(coords[0], coords[1], coords[2], coords[3], coords[4], coords[5])

    # 选解 test
    i = 4

    # 构造 was 命令
    msg = "was"
    for j in range(6):
        msg = msg + " " + str(round(Res[i][j], 2))
    print(msg)
    s.sendto(msg.encode(), (ADDRESS, PORT))
    time.sleep(2)

    # "摁"
    Res[i][3] = Res[i][3] - 15  # 4 号舵机发力
    msg = "wa"
    msg = msg + " 4 " + str(round(Res[i][3], 2))
    s.sendto(msg.encode(), (ADDRESS, PORT))
    time.sleep(10)

    return 1


# test
while True:
    msg = input("> ")
    if msg[0:4] == "work":
        p = msg.split(" ")
        coords = [float(p[1]), float(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])]
        work3(coords)
    elif msg == "ini":
        ini()
    else:
        s.sendto(msg.encode(), (ADDRESS, PORT))







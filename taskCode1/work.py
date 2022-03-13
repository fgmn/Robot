import cv2
import numpy as np
from numpy import *

## UDP通信配置区
from socket import socket, AF_INET, SOCK_DGRAM
ADDRESS = "172.20.10.6"  # M5Stack address
PORT = 5555
Socket = socket(AF_INET, SOCK_DGRAM)

import time


def draw_direction(img, lx, ly, nx, ny):
    dx = nx - lx
    dy = ny - ly
    if abs(dx) < 4 and abs(dy) < 4:
        dx = 0
        dy = 0
    else:
        r = (dx**2 + dy**2)**0.5
        dx = int(dx/r*40)
        dy = int(dy/r*40)       ## 缩小为 40分之1
    cv2.arrowedLine(img, (60, 100), (60+dx, 100+dy), (0, 255, 0), 2)        ## 绘制 ->

## 2D -> 3D
def trans(u, v):
    ## 相机参数区
    Z_c = 450  ## unit : mm
    f_x = 1728.4
    f_y = 1728.4
    u_0 = 640 / 2  ## 根据最大分辩率计算
    v_0 = 480 / 2

    # R： Singular matrix
    # 非方阵逆的计算
    R_T = mat([[1, 0, 0, 0],
               [0, -1, 0, 0],
               [0, 0, -1, Z_c],
               [0, 0, 0, 1]])
    R = mat([[-1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, -1, Z_c],
             [0, 0, 0, 0]])
    P = mat([[f_x, 0, u_0, 0],
             [0, f_y, v_0, 0],
             [0, 0, 1, 0]])

    R_ = R_T.I
    P_ = P.I

    coords_px = mat([[u], [v], [1]])
    coords_px = Z_c * coords_px

    coords_world = R_ * P_ * coords_px  ## 4*4 4*3 3*1
    coords_world[2] += Z_c
    coords_world *= 5

    coords_world = coords_world.tolist()
    # print(coords_world)

    return coords_world


# without test
def Call(coords_w):
    msg = "wcs"
    for i in range(3):
        msg += " " + str(round(coords_w[i][0], 2))
    msg += " -180.0 0 0"
    print(msg)
    Socket.sendto(msg.encode(), (ADDRESS, PORT))


## 全局参数区

## 摄像头设置
frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(1)  # 0对应笔记本自带摄像头 1对应USB摄像头
cap.set(3, frameWidth)  # set中，这里的3，下面的4和10是类似于功能号的东西，数字的值没有实际意义
cap.set(4, frameHeight)
cap.set(10, 80)        # 设置亮度
pulse_ms = 30

lower = np.array([4, 180, 156])     # 适用于橙色乒乓球4<=h<=32
upper = np.array([32, 255, 255])

targetPos_x = 0     # 颜色检测得到的x坐标
targetPos_y = 0     # 颜色检测得到的y坐标
lastPos_x = 0       # 上一帧图像颜色检测得到的x坐标
lastPos_y = 0       # 上一帧图像颜色检测得到的x坐标
ColorXs = []        # 这些是用来存储x，y坐标的列表，便于后期写入文件
ColorYs = []

# time control
sending = 0

def work():
    # UnboundLocalError: local variable 'targetPos_x' referenced before assignment
    global targetPos_x
    global targetPos_y
    global lastPos_x
    global lastPos_y
    global sending

    while True:
        time.sleep(3)
        # sending += 1
        _, img = cap.read()
        print(shape(img))

        imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        ## 使用遮罩进行颜色选择，再进行 findContours()
        imgMask = cv2.inRange(imgHsv, lower, upper)     # 获取遮罩
        imgOutput = cv2.bitwise_and(img, img, mask=imgMask)
        contours, hierarchy = cv2.findContours(imgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)   # 查找轮廓
        # https://blog.csdn.net/laobai1015/article/details/76400725
        # CV_RETR_EXTERNAL 只检测最外围轮廓
        # CV_CHAIN_APPROX_NONE 保存物体边界上所有连续的轮廓点到contours向量内
        imgMask = cv2.cvtColor(imgMask, cv2.COLOR_GRAY2BGR)     # 转换后，后期才能够与原画面拼接，否则与原图维数不同

        # 下面的代码查找包围框，并绘制
        x, y, w, h = 0, 0, 0, 0
        R_ball = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                lastPos_x = targetPos_x
                lastPos_y = targetPos_y
                targetPos_x = int(x+w/2)
                targetPos_y = int(y+h/2)
                # 计算图像中乒乓球的半径 R
                R_ball = (area / 3.1415929) ** 0.5
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(img, (targetPos_x, targetPos_y), 2, (0, 255, 0), 4)

                ## trans() 2D -> 3D -> angles
                res = trans(targetPos_x, targetPos_y)
                print("({}, {}, {})".format(res[0], res[1], res[2]))
                ##
                res[2][0] = 40.00
                ## Call() UDP发出指令
                # if sending == 5:
                    # sending = 0
                Call(res)



        # 坐标（图像内的）
        cv2.putText(img, "({:0<2d}, {:0<2d})".format(targetPos_x, targetPos_y), (20, 30),   ## 取2位左对齐
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)  # 文字
        draw_direction(img, lastPos_x, lastPos_y, targetPos_x, targetPos_y)     ## 指示球的方向变化

        cv2.imshow('status', img)       # 显示

        ColorXs.append(targetPos_x)     # 坐标存入列表
        ColorYs.append(targetPos_y)

        if cv2.waitKey(pulse_ms) & 0xFF == ord('q'):          # 按下“q”推出（英文输入法）
            print("Quit\n")
            break


work()

cap.release()
cv2.destroyAllWindows()
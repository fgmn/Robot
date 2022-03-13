import matplotlib.animation as animation
import matplotlib.pyplot as plt
import sys
import logging
import time
import numpy as np
import math
import cmath


def 坐标转关节角度(x0,y0):
    #只用了输入的二维坐标（假设物体就在桌面上），且只用了五个关节（最后一个关节由于是抓球调位姿也不需要了，234关节的转度根据骨长求定值，改变第一个关节和第五个关节转度进行调节，达到目的）
    骨长1=70.42
    骨长2=110.4
    骨长3=96.0
    骨长4=73.18
    小骨长=66.39
    末骨长=43.6
    抓手长=70  #需要修改（量了一下大约7cm）
    
    J=[0,0,0,0,0,0] #表示关节角度，J1,J2,J3,J4,J5,J6
    #初始化
    抓手坐标=(0,0)
    #mycobot=None
    当前坐标=[0,0,0,0,0,0]
    当前角度=[0,0,0,0,0]
    工作半径范围=(50,骨长2+骨长3-5)
    
    p0=(None,None)
    thi=0
    #球状物体，不用设置位姿，设零就可以了
    #最后一个关节J[5]即J6设零即可，因为在任何位姿状态下都可以抓取球形物体
    pJ5=(None,None)
    #自动校准
    
    #初始化
    R=0
    g0=小骨长
    g1=骨长1
    g2=骨长2
    g3=骨长3
    g4=骨长4
    g5=g4+抓手长
    g6=末骨长
    r=R
            
    lr=np.sqrt(r**2+(g5-g1)**2)
    Ji=np.arcsin(r/lr)/np.pi*180;
    i2=np.arccos((lr**2+g2**2-g3**2)/(2*lr*g2))/np.pi*180
    i3=np.arccos((g3**2+g2**2-lr**2)/(2*g3*g2))/np.pi*180
    i4=np.arccos((lr**2+g3**2-g2**2)/(2*lr*g3))/np.pi*180
    J2=Ji-i2
    J3=180-i3
    J4=180-i4-Ji
    #J2
    J[1]=J2
    #J3
    J[2]=J3
    #J4
    J[3]=J4  
    g0=小骨长
    (minr,maxr)=工作半径范围
    g6=末骨长
    pJ5=计算J5坐标(x0,y0,thi)
    if pJ5!=0:
                               
        (x5,y5)=pJ5
        #pJ5=pJ5
                #print("Pj5:",pJ5)
        cr=np.sqrt(x5**2+y5**2)
        i1=math.degrees(cmath.polar(complex(x5,y5))[1])                
        R=np.sqrt(cr**2-g0**2)
        i2=math.degrees(np.arcsin(g0/cr))
        J1=i1+i2
        #J1
        J[0]=J1
        p0=(x0,y0)#目标点中心坐标
        pm=计算抓手坐标(J1,R)#假想目标点中心坐标
        J5=jj(p0,pJ5,pm)
        #J5
        J[4]=J5
    print(J)
    return J

def 计算J5坐标(x0,y0,thi):
    骨长1=70.42
    骨长2=110.4
    骨长3=96.0
    骨长4=73.18
    小骨长=66.39
    末骨长=43.6
    抓手长=70  #需要修改
    
    J=[0,0,0,0,0,0] #表示关节角度，J1,J2,J3,J4,J5,J6
    #初始化
    抓手坐标=(0,0)
    mycobot=None
    当前坐标=[0,0,0,0,0,0]
    当前角度=[0,0,0,0,0]
    工作半径范围=(50,骨长2+骨长3-5)
    g0=小骨长
    g1=骨长1
    g2=骨长2
    g3=骨长3
    g4=骨长4
    g5=g4+抓手长
    g6=末骨长
    x5=x0+g6*np.cos(math.radians(thi))
    x5i=x0+g6*np.cos(math.radians(180+thi))
    y5=y0+g6*np.sin(math.radians(thi))
    y5i=y0+g6*np.sin(math.radians(180+thi))
    r=np.sqrt(x5**2+y5**2)
    ri=np.sqrt(x5i**2+y5i**2)
    if r<=ri and g0<r:
        return (x5,y5)
    elif r<=ri and g0<ri and g0>r:
        return (x5i,y5i)
    elif r>ri and g0<ri:
        return (x5i,y5i)
    elif r>ri and g0<r and g0>ri:
        return (x5,y5)
    else:
        self.p0=(None,None)
        logging.error("cannot 计算 J5，woring 目标值")
        return False
    
def 计算抓手坐标(J1,R): 
    (x,y)=(0,0)
    x=R*np.cos(J1/180*np.pi)
    y=R*np.sin(J1/180*np.pi)
    return (round(x, 2),round(y, 2))


#转坐标轴以及参数的转换
def jj(p1,p0,p2):
    try:
        骨长1=70.42
        骨长2=110.4
        骨长3=96.0
        骨长4=73.18
        小骨长=66.39
        末骨长=43.6
        抓手长=70  #需要修改
    
        J=[0,0,0,0,0,0] #表示关节角度，J1,J2,J3,J4,J5,J6
    #初始化
        抓手坐标=(0,0)
        mycobot=None
        当前坐标=[0,0,0,0,0,0]
        当前角度=[0,0,0,0,0]
        工作半径范围=(50,骨长2+骨长3-5)
        g0=小骨长
        g1=骨长1
        g2=骨长2
        g3=骨长3
        g4=骨长4
        g5=g4+抓手长
        g6=末骨长
        
        (x0,y0)=p0
        (x1,y1)=p1
        (x2,y2)=p2 
            
            #向量P0-->p1的极坐标转角
        x=x1-x0
        y=y1-y0
        alpha1=math.degrees(cmath.polar(complex(x,y))[1])
            #向量P0-->p2的极坐标转角
        x=x2-x0
        y=y2-y0
        alpha2=math.degrees(cmath.polar(complex(x,y))[1])    
            
        alpha=alpha2-alpha1
        return round(alpha, 2)
    except Exception as e:
        logging.error("计算计算J5坐标异常：",e)  

x1 = 200
y1 = 600
#接口，输入二维坐标返回一个六元数组
坐标转关节角度(x1,y1)
        

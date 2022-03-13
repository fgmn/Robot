from __future__ import print_function
import numpy as np
import pyzed.sl as sl
import cv2
import math
import numpy as np
import sys
import os
import cv2
import imageio
import PIL.Image
import numpy as np
import tensorflow as tf
from socket import socket, AF_INET, SOCK_DGRAM
import time
from button_detection import ButtonDetector
from character_recognition import CharacterRecognizer
import MCP


###################UDP通信配置区################################

# # ADDRESS = "172.20.10.6"  # M5Stack address
# ADDRESS = "192.168.43.246"  # M5Stack address
# PORT = 5555
# Socket = socket(AF_INET, SOCK_DGRAM)

###################UDP通信配置区################################


########################矩阵参数################################

#R=np.mat()#相机坐标系相对于机械臂的旋转矩阵 size:3x3
#T=np.mat()#相机坐标系相对于机械臂的平移矩阵 size:3x1

########################矩阵参数################################


########################相机参数##############################

input_type = sl.InputType() # Set configuration parameters
init = sl.InitParameters(input_t=input_type)  # 初始化
init.camera_resolution = sl.RESOLUTION.HD2K # 相机分辨率(默认-HD720)
init.depth_mode = sl.DEPTH_MODE.ULTRA         # 深度模式  (默认-PERFORMANCE)
init.coordinate_units = sl.UNIT.MILLIMETER    # 毫米级 (默认-MILLIMETER)

########################相机参数##############################


def button_candidates(boxes, scores, image):
    img_height = image.shape[0]
    img_width = image.shape[1]

    button_scores = []
    button_patches = []
    button_positions = []
    button_xy = []
    ratio=0.0
    
    for box, score in zip(boxes, scores):
        if score < 0.5: continue

        #print(box[0],box[1])
        y_min = int(box[0] * img_height)
        x_min = int(box[1] * img_width)
        y_max = int(box[2] * img_height)
        x_max = int(box[3] * img_width)

        button_patch = image[y_min: y_max, x_min: x_max]
        button_patch = cv2.resize(button_patch, (180, 180))
        ratio=(x_max-x_min)/180
        button_scores.append(score)
        button_patches.append(button_patch)
        button_positions.append([x_min, y_min, x_max, y_max])
        
        button_xy.append([(x_min+x_max)/2,(y_min+y_max)/2])
        
    return button_patches, button_positions, button_scores, button_xy,ratio

def get_image_name_list(target_path):
    assert os.path.exists(target_path)
    image_name_list = []
    file_set = os.walk(target_path)
    for root, dirs, files in file_set:
      for image_name in files:
        image_name_list.append(image_name.split('.')[0])
    return image_name_list


# 坐标转换
# def convert_cc_to_wc(joint_world):
#     """
#     相机坐标系 -> 世界坐标系: inv(R) * pt +T
#     joint_cam = np.dot(inv(R), joint_world.T)+T
#     :return:
#     """
#     joint_world = np.asarray(joint_world)
#     R = np.asarray(R)
#     T = np.asarray(T)
#     # 相机坐标系 -> 世界坐标系
#     joint_cam = np.dot(np.linalg.inv(R), joint_world.T).T + T
#     return joint_cam

def convert_to_arm(coords_camera):
    # 设 P 在相机坐标系下为 P_c = (x1, y1, z1) ^ T，在机械臂坐标系为 P_w = (x2, y2, z2) ^ T
    # P_w = T * P_c 其中 T 为相机坐标系 c 在机械臂坐标系 w 下的表示
    R = mat()
    T = mat()
    coords_arm = R * coords_camera + T # (3*3) * (3*1) + (3*1)
    return coords_arm

#检测电梯按钮是否亮起
def button_is_on(img_np,x_min,y_min,x_max,y_max):
    width=x_max-x_min
    height=y_max-y_min
    x=(x_min+x_max)/2
    y=(y_min+y_max)/2
   # print(x,y)
    img_gray = cv2.cvtColor(img_np,cv2.COLOR_RGB2GRAY)
    img_temp=img_gray[y_min: y_max, x_min: x_max]
    
    for i in range(int(height*0.2),int(height*0.7)):
          for j in range(int(width*0.3),int(width*0.7)):
                if img_temp[i,j]<128:
                      img_temp[i,j]=0
                if img_temp[i,j]<192:
                      img_temp[i,j]=128
                else:
                      img_temp[i,j]=255
    cv2.imshow("button",img_temp)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))

    closed1 = cv2.morphologyEx(img_temp, cv2.MORPH_CLOSE, kernel,iterations=1) # 高级形态学运算
    x, y = closed1.shape

    ind = 0

    for i in range(int(height*0.2),int(height*0.7)):
        for j in range(int(width*0.3),int(width*0.7)):
            if closed1[i,j]==255:
                ind += 1
            if closed1[i,j]==128:
                ind +=0.5

    light_rate = ind/(height*0.5*width*0.4)
    print(light_rate)

    if light_rate > 0.6:
        print("亮")
        return True
    else:
        print("暗")
        return False

def main():
    detector = ButtonDetector()
    recognizer = CharacterRecognizer(verbose=False)  
    zed = sl.Camera()           # Create a ZED camera object

   

    err = zed.open(init)  # Open the camera
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    
    image_size = zed.get_camera_information().camera_resolution  # retrieve half-resolution images
    while True:
        zed.grab()
        
        image_sl_left = sl.Mat()  # left_img
        zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
        image_cv_left = image_sl_left.get_data()
        
        image_sl_right = sl.Mat()  # right_img
        zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
        image_cv_right = image_sl_right.get_data()
        
        #左侧相机深度图
        depth_left_map = sl.Mat()
        zed.retrieve_measure(depth_left_map,sl.MEASURE.DEPTH, sl.MEM.CPU)
        
        #左侧相机点云
        point_cloud=sl.Mat()
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU) # Get the point cloud
        ################# BGRA 转成 BGR #################
        image_cv_left = cv2.cvtColor(image_cv_left, 1)
        image_cv_right = cv2.cvtColor(image_cv_right, 1)
        ################# BGRA 转成 BGR #################
        
        ####只使用左侧相机识别####
        img_np = image_cv_left
        t0 = cv2.getTickCount()
        boxes, scores, _ = detector.predict(img_np, True)      
        button_patches, button_positions, _, button_coordinates,button_ratio = button_candidates(boxes, scores, img_np)
        #print(button_coordinates)
        
        for button_img, button_pos,button_xy in zip(button_patches, button_positions,button_coordinates):
            button_text, button_score, button_draw =recognizer.predict(button_img, draw=True)
            x_min, y_min, x_max, y_max = button_pos
            
            #print(button_pos)
            
            if button_text=='2' or button_text=='(':   
            #point_depth = depth_left_map.get_value((x_max-x_min)/2, (y_max-y_min)/2,sl.MEM.CPU)
            #print((x_max-x_min)/2, (y_max-y_min)/2,point_depth)
            #center_depth = depth_left_map.get_value(button_xy[0]*image_size.width,button_xy[1]*image_size.height, sl.MEM.CPU) # each depth map pixel is a float value
                center_depth = depth_left_map.get_value(button_xy[0],button_xy[1], sl.MEM.CPU)
                # Read a point cloud value
                err, pc_value = point_cloud.get_value(button_xy[0], button_xy[1]) # each point cloud pixel contains 4 floats, so 
                #if pc_value[2]:
                print(button_text,"Point cloud coordinates : X=", pc_value[0], ", Y=", pc_value[1], ", Z=", pc_value[2])
                #button_is_on(img_np,x_min,y_min,x_max,y_max)
                ####################task############################################
                #1.得到目标按钮相对于摄像头的三维坐标，转换为相对于机械臂的三维坐标
                coords = [0, 0, 0, 0, 0, 0]
                Ret = convert_to_arm(pc_value)
                coords[0:3] = Ret

                # 基于经验选择的末端执行器姿态
                poseture = [[0, 30, -30],
                            [0, 35, 0]]
                if button_text=='2':
                    coords[3:6] = poseture[0]
                else:
                    coords[3:6] = poseture[1]

                #2.得到目标按钮相对于机械臂的三维坐标，发送给机械臂
                # MCP.ini()
                #4.按钮在机械臂可达范围之内，
                if MCP.work2(coords):
                     #5.检测按钮是否变亮，如果亮，给小车反馈；如果没亮，返回步骤4
                    while not button_is_on(img_np,x_min,y_min,x_max,y_max):
                        #调用机械臂控制程序
                        MCP.work2(coords)
                
                
                #3.按钮在机械臂可达范围之外，需要给小车反馈，指挥小车移动，小车移动后，返回步骤1
                
               
                    
            
                
                ####################task############################################
            
            button_rec = cv2.resize(button_draw, (x_max-x_min, y_max-y_min))
            detector.image_show[y_min+6:y_max-6, x_min+6:x_max-6] = button_rec[6:-6, 6:-6]
            image_cv_left = detector.image_show
        
        t1 = cv2.getTickCount()
        time = (t1-t0)/cv2.getTickFrequency()
        #print('Time elapsed: {}'.format(time))
        ####识别####
        
        
        img_sbs = np.concatenate((image_cv_left, image_cv_right), axis=1)
        img = cv2.resize(img_sbs, (int(image_size.width/1.2), int(image_size.height/2)))
        
        
        cv2.imshow("ZED-img", img)
        cv2.waitKey(10)
   

if __name__ == "__main__":
    main()
#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
from vehicle import Buguang

import RPi.GPIO as GPIO
import random
import statistics
import random


def generate_number():
    # 生成1.5到3之间的随机数
    number = random.uniform(1.5, 3)
    
    # 四舍五入到一位小数
    rounded_number = round(number, 1)
    
    # 检查是否需要将数字转换为整数形式以保持一位有效数字
    if rounded_number == int(rounded_number):
        return int(rounded_number)
    else:
        return rounded_number

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()

    my_car.STOP_PARAM = False
    light = Buguang(5)
    for i in range(3,7):
        light.set_light(i, 0, 0, 0)
    my_car.task.arm.set_grap_angle(0)
    time.sleep(1)
    my_car.task.arm.set(0, 0,speed = [0.3, 0.3])
    # my_car.task.reset()
                # for i in range(1, 5):
                # self.light.set_light(i, 255, 255, 255)
    # led = Buguang(5)
    # led.set_light(1,255,255,255)
    def AI_Read(side,id):
        now_pos = np.array(my_car.get_odom()) 
        start_time = time.time()
        sport_time = start_time
        x_speed = int(random.random()+0.5)
        y_speed = int(random.random()+0.5)
        if x_speed==0:
            x_speed-=1
        if y_speed==0:
            y_speed-=1
        while time.time()-start_time<3:
            if side==-1:
                sensor_val = my_car.right_sensor.get()/1000
            else:
                sensor_val = my_car.left_sensor.get()/1000
            if sensor_val<0.8:
                # my_car.beep()
                my_car.AI_FLAG = id
            if time.time()-sport_time>0.2:
                sport_time = time.time()
                x_speed = int(random.random()+0.5)
                y_speed = int(random.random()+0.5)
                if x_speed==0:
                    x_speed-=1
                if y_speed==0:
                    y_speed-=1
            my_car.mecanum_wheel(x_speed/75, y_speed/75, 0)

        my_car.set_pos(now_pos)

        print("my_car.AI_FLAG = ",my_car.AI_FLAG)

    # 计算IoU，矩形框的坐标形式为xyxy，这个函数会被保存在box_utils.py文件中
    def box_iou_xyxy(box1, box2):
        # 获取box1左上角和右下角的坐标
        x1min, y1min, x1max, y1max = box1[0], box1[1], box1[2], box1[3]
        # 计算box1的面积
        s1 = (y1max - y1min + 1.) * (x1max - x1min + 1.)
        # 获取box2左上角和右下角的坐标
        x2min, y2min, x2max, y2max = box2[0], box2[1], box2[2], box2[3]
        # 计算box2的面积
        s2 = (y2max - y2min + 1.) * (x2max - x2min + 1.)
        
        # 计算相交矩形框的坐标
        xmin = np.maximum(x1min, x2min)
        ymin = np.maximum(y1min, y2min)
        xmax = np.minimum(x1max, x2max)
        ymax = np.minimum(y1max, y2max)
        # 计算相交矩形行的高度、宽度、面积
        inter_h = np.maximum(ymax - ymin + 1., 0.)
        inter_w = np.maximum(xmax - xmin + 1., 0.)
        intersection = inter_h * inter_w
        # 计算相并面积
        union = s1 + s2 - intersection
        # 计算交并比
        iou = intersection / union
        return iou
    
    def hanoi_tower_func():
        #my_car.lane_dis_offset(0.3, 0.5)
        # 走一段距离足够近之后才退出
        my_car.set_pos_offset([0.7,0,0], 1)

        #det_side = my_car.lane_det_dis2pt(0.2, 0.19)
        # 获取岔路走哪一边
        side = my_car.get_card_side()
        print("side = ",side)
        # 调整检测方向
        my_car.task.arm.switch_side(side*-1)
        # 调整车子朝向
        my_car.set_pos_offset([0, 0, math.pi/4*side], 1)
        
        # 第一个要抓取的圆柱
        cylinder_id = 1
        # 调整抓手位置，获取要抓取的圆柱信息
        pt = my_car.task.pick_up_cylinder(cylinder_id, True)
        # 走一段距离
        my_car.lane_dis_offset(0.3,0.66)
        # 第二次感应到侧面位置
        #my_car.lane_sensor(0.2, value_h=0.3, sides=side*-1)
        my_car.lane_sensor(0.2, value_l=0.3, sides=side*-1, stop=True)
        # 记录此时的位置
        pos_start = np.array(my_car.get_odom())
        logger.info("start pos:{}".format(pos_start))
        # my_car.lane_dis(0.2, 0.1)
        # return
        # 根据给定信息定位目标
        my_car.lane_det_location(0.2, pt, side=side*-1)
        # 抓取圆柱
        my_car.task.pick_up_cylinder(cylinder_id)
        # 计算走到记录位置的距离
        run_dis = my_car.calculation_dis(pos_start, np.array(my_car.get_odom()))
        # print("run_dis:{}".format(run_dis))
        # 后移刚才计算的距离，稍微多走一点儿
        my_car.set_pos_offset([0-(run_dis+0.065), 0, 0])
    
        # # print("stop pos:{}".format(my_car.get_odom()))
        tar_pos = my_car.get_odom()
        # 记录位置
        logger.info("tar_pos:{}".format(tar_pos))
        my_car.task.put_down_cylinder(cylinder_id)
        
        # 抓取2号圆柱
        cylinder_id = 2
        pt = my_car.task.pick_up_cylinder(cylinder_id, True)
        my_car.lane_det_location(0.2, pt, dis_out=0.5, side=-1*side)
        my_car.task.pick_up_cylinder(cylinder_id)
        my_car.set_pos(tar_pos)
        # print(my_car.get_odom())
        my_car.task.put_down_cylinder(cylinder_id)

        # 抓取3号圆柱
        cylinder_id = 3
        pt = my_car.task.pick_up_cylinder(cylinder_id, True)
        my_car.lane_dis_offset(0.2, 0.1)
        my_car.lane_det_location(0.2, pt, dis_out=0.5, side=-1*side)
        my_car.task.pick_up_cylinder(cylinder_id)
        my_car.set_pos(tar_pos)
        # print(my_car.get_odom())
        my_car.task.put_down_cylinder(cylinder_id)


    def grap_block_func():
        need_detect = 1
        if need_detect ==1:
            # 设置手臂位置,返回抓取位置的信息
            # 巡航右侧感应器感应到物块，并继续巡航2cm
            # return
            my_car.lane_sensor(0.25, value_h=0.3, sides=-1, stop=False)
            my_car.set_pos_offset([0.08, 0.02 ,0], 0.5)            
            my_car.task.servo_ball.set_angle(80, -110)
            pt_tar = my_car.task.pick_up_block(arm_set=True)
            res = my_car.lane_det_location(0.2, pt_tar, side=-1)
            if res:
                my_car.task.pick_up_block(0)
                my_car.task.put_down_self_block() 
            pt_tar = my_car.task.pick_up_block(arm_set=True)                   
            my_car.lane_dis_offset(0.3, 0.05)    
            my_car.lane_sensor(0.25, value_h=0.3, sides=-1, stop=False)
            res = my_car.lane_det_location(0.2, pt_tar, side=-1)
            if res:
                my_car.task.pick_up_block(1)                             

        else:
            pt_tar = my_car.task.pick_up_block(arm_set=True)

            my_car.lane_sensor(0.25, value_h=0.3, sides=-1, stop=False)
            my_car.task.servo_ball.set_angle(80, -110)
            my_car.set_pos_offset([0.115, 0.02  , 0], 0.5)
            my_car.beep()
            i = 0
            my_car.task.pick_up_block(i=i)
            my_car.task.put_down_self_block()

            pt_tar = my_car.task.pick_up_block(arm_set=True)
            my_car.set_pos_offset([0.305, 0, 0], 1.3)
            my_car.beep()
            i = 1
            my_car.task.pick_up_block(i=i)


    def release_block_func():
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN)
        # 调整机械臂的位置
        my_car.task.arm.switch_side(-1)
        # 速度0.3m/s巡航2.3m

        my_car.set_pos_offset([0.275, 0,0],  sp=[0.4, 0.3, math.pi / 5])  
        my_car.set_pos_offset([0.0, 0, math.pi/2.75-15*math.pi/180],  sp=[0.4, 0.3, math.pi / 4])    
        # AI_Read(1,1)   
        my_car.set_pos_offset([0.7,0, 0], 3)
        my_car.lane_dis_offset(0.3, 1.3, stop=False)
        # 速度0.3m/s巡航右侧感应器感应到0.4m到障碍
        my_car.lane_sensor(0.3, value_h=0.5, sides=-1, stop=False)
        # my_car.lane_time(0.2,0.5)
        # time.sleep(3)
        #my_car.lane_sensor(0.2, value_h=0.01, sides=-1, stop=False,angle_offest=10)
        # 速度0.3m/s巡航0.1m

        for i in range(9):
            my_car.lane_dis_offset(0.3-(i*0.03), 0.01, stop=False)
        my_car.lane_dis_offset(0.03, 0.01, stop=True)

        # 调整位置,这个根据巡航效果调整
        my_car.set_pos_offset([0.09, 0.03, -0.3+math.pi/36],error_need = True)

        # my_car.lane_sensor(0.2, value_l=0.3, sides=-1,stop=False)
        my_car.beep()
        # time.sleep(3)
        #my_car.set_pos_offset([0, 0.15, 0],sp=[0.05, 0.05, math.pi / 5],read_inface = 1)
        base = 0
        infornt = 0.08
        aa_speed = [0.1,0.1,math.pi/5]
        ok_flag = my_car.set_pos_offset([base+infornt, 0, 0],sp = aa_speed,read_inface=2,error_need=True)
        dir = 1
        times = 0
        my_car.beep()
        # time.sleep(2)
        while ok_flag == 0:
            my_car.set_pos_offset([0, -0.015, 0],sp = aa_speed,read_inface=2,error_need=True)
            ok_flag = my_car.set_pos_offset([base-infornt, 0, 0],sp = aa_speed,read_inface=2,error_need=True)
            dir = -1
            times+=1
            if times>3:
                break
            if ok_flag ==1:
                break
            my_car.set_pos_offset([0, -0.015, 0],sp = aa_speed,read_inface=2,error_need=True)
            ok_flag = my_car.set_pos_offset([base+infornt, 0, 0],sp = aa_speed,read_inface=2,error_need=True)
            dir = 1
            times+=1
        GPIO.setmode(GPIO.BCM)
        input_array = []
        for i in range(50):
            input_array.append(GPIO.setup(18, GPIO.IN))
        mode = statistics.mode(input_array)
        if mode == 1:
            if dir == 1:
                my_car.set_pos_offset([-0.05, 0, 0], sp = [0.05,0.05,math.pi/5],read_inface=2)
            else:
                my_car.set_pos_offset([0.05, 0, 0], sp = [0.05,0.05,math.pi/5],read_inface=2)
        my_car.beep()
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(18, GPIO.IN)
        # input_state = GPIO.input(18)
        # print(input_state)


        my_car.set_pos_offset([0, 0.2, 0],sp=[0.01, 0.01, math.pi / 5],read_inface = 1)

        GPIO.cleanup()
        my_car.beep()
        # time.sleep(3)

        # my_car.set_pos_offset([0.08, -use, 0], 1)
        n = 0
        my_car.beep()
        # my_car.set_pos_offset([0.1, 0, 0])
        # 放下一个方块
        my_car.set_pos_offset([0, -0.01, 0])
        my_car.task.put_down_block(n)
        # 前移0.08m
        my_car.set_pos_offset([0.11, 0, 0])
        my_car.beep()
        n = 1
        # 放第二个方块
        my_car.task.pick_up_block_self()
        my_car.task.put_down_block(n)
        my_car.set_pos_offset([0.15, 0,0],  sp=[0.4, 0.3, math.pi / 5])  
        my_car.set_pos_offset([0.0, 0, -math.pi/2.75+(15)*math.pi/180],  sp=[0.4, 0.3, math.pi / 4]) 
        # if my_car.AI_FLAG==0:
        #     AI_Read(1,2)   


        # my_car.set_pos_offset([0.5,0, 0], 2)
        my_car.lane_dis_offset(0.3, 0.5)

    def get_ball_func():
        get_ball_flag = 1

        if get_ball_flag == 1 :
            test = 1
            if test == 1:
                now = 1
                my_car.task.arm.switch_side(1)
                # 调整机械手位置准备抓球，返回识别目标的位置
                pt = my_car.task.pick_up_ball(arm_set=True)
                my_car.lane_dis_offset(0.15, 0.1)
                my_car.lane_dis_offset(0.3, 0.1)
                my_car.lane_dis_offset(0.3, 0.1)
                my_car.lane_sensor(0.3, value_h=0.4, sides=1, stop=True)
                start_dis = my_car.get_dis_traveled()

                if now == 1:
                    for i in range(3):
                        # 根据给定目标和位置、方向定位调整车子的位置
                        pt = my_car.task.pick_up_ball(arm_set=True)
                        res,rect = my_car.lane_det_location(0.2, pt, side=1,get_ball=1)
                        print("rect= ",rect)
                        if i < 2:
                            if res:
                                my_car.beep()
                                my_car.task.pick_up_ball()
                                my_car.task.put_down_self_ball()
                                use_times = 0
                                for times in range(3):
                                    use_times = times
                                    start_time = time.time()
                                    iou_flag = -1
                                    while time.time() - start_time<0.3:
                                        my_car.task.pick_up_ball(arm_set=True)
                                        detect_rect = my_car.lane_det_location(0.2, pt, side=1,get_ball=2)#识别并返回rect
                                        if detect_rect!=None:
                                            iou = box_iou_xyxy(rect,detect_rect)
                                            print("iou = ",iou,rect,detect_rect)
                                            if iou>0.5:
                                                iou_flag =0
                                                rect = detect_rect
                                                my_car.set_pos_offset([0,0.01,0])
                                                my_car.beep()
                                                my_car.task.pick_up_ball()
                                                my_car.task.put_down_self_ball()
                                                break
                                            else:
                                                break
                                    if iou_flag==-1:
                                        break   
                            
                                out_flag = 0
                                if use_times==2:
                                    start_time = time.time()
                                    while time.time() - start_time<0.3:
                                        my_car.task.pick_up_ball(arm_set=True)
                                        detect_rect = my_car.lane_det_location(0.2, pt, side=1,get_ball=2)#识别并返回rect
                                        if detect_rect!=None:
                                            iou = box_iou_xyxy(rect,detect_rect)
                                            if iou>0.5:
                                                out_flag = 1
                                                break
                                if out_flag ==1:
                                    if i !=2:
                                        my_car.lane_sensor(0.2, value_h=0.5, sides=1) #0.2
                                    break

                            else:
                                # 距离超过0.4m就跳出
                                if my_car.get_dis_traveled() - start_dis < 0.40:
                                    logger.info("dis out {}".format(i))
                                else:
                                    logger.info("can not find ball")
                                    break
                                continue


                        else:
                            pt = my_car.task.pick_up_ball(arm_set=True)
                            res,rect = my_car.lane_det_location(0.2, pt, side=1,get_ball=1)
                            if res:
                                my_car.beep()
                                my_car.task.pick_up_ball()
                                my_car.task.arm.set_arm_angle(70,restart_flag=True)
                                time.sleep(2)



    def elevation_pole_func():
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)
        # 巡航0.2m
        my_car.lane_dis_offset(0.3, 0.4)
        # 巡航到左侧感应器感应到障碍
        my_car.lane_sensor(0.15, value_h=0.35, sides=1) #0.2
        my_car.set_pos_offset([0.3, 0, 0],sp=[0.05, 0.05, math.pi / 5],left_inface = 2)

        my_car.set_pos_offset([0, -0.1, 0],sp=[0.01, 0.01, math.pi / 5],left_inface = 1)

        start_pos = np.array(my_car.get_odom())

        pos = [0,0.03,0]
        start_angle = start_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, pos[2]]) + start_pos
        print(start_pos,tar_pos)
        my_car.set_pos(tar_pos,sp=[0.05, 0.05, math.pi / 5],error_need = True)
        

        my_car.set_pos_offset([0.2, 0, 0],sp=[0.01, 0.01, math.pi / 5],left_inface = 2)
        now_pos = np.array(my_car.get_odom())
        my_car.set_pos_offset([0.2, 0, 0],sp=[0.01, 0.01, math.pi / 5],left_inface = 1)
        start_pos = np.array(my_car.get_odom())
        error_pos = start_pos-now_pos
        
        print(start_pos,tar_pos)
        pos = [0,0.04,0]
        start_angle = start_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, pos[2]]) + tar_pos + error_pos
        my_car.set_pos(tar_pos,sp=[0.05, 0.05, math.pi / 5],error_need = True)


        GPIO.cleanup()
        my_car.beep()
 
        # time.sleep(2)
        my_car.task.elevation_pole()
        my_car.set_pos(start_pos,0.5)
        # my_car.set_pos_offset([0.00, -0.04, 0], 1)
        #my_car.set_pos_offset([0.0, -0.04-0.00, 0], 0.5)
        my_car.task.servo_rotate.set_angle(90, 90)
        
        my_car.task.arm.set(0, 0.12,speed=[0.2, 0.2])
        my_car.task.arm.set(-0.22, 0.12,speed=[0.2, 0.2])
        my_car.task.arm.set_arm_angle(90,restart_flag=True)
        my_car.task.arm.set(-0.22, 0,speed=[0.2, 0.2])


        my_car.set_pos_offset([-0.05, 0, 0],sp=[0.1, 0.1, math.pi / 5])
        my_car.task.arm.set_grap_angle(60)
        time.sleep(0.5)
        my_car.task.arm.set_grap_angle(0)
        time.sleep(0.5)

        # time.sleep(5)

    def get_high_ball_func():
        # 调整位置准备抓球
        pt = my_car.task.pick_high_ball(arm_set=True)
        my_car.lane_dis_offset(0.3, 1)
        # 飞机停车坪移开位置
        pt = my_car.task.pick_high_ball(arm_set=True)
        my_car.lane_sensor(0.3, value_h=0.4, sides=1,stop = False)
        my_car.lane_sensor(0.15, value_l=0.3, sides=1,stop = False)
        my_car.lane_sensor(0.15, value_h=0.4, sides=1,stop = False)
        my_car.lane_sensor(0.15, value_l=0.3, sides=1,stop = False)
        my_car.set_pos_offset([0.02,0,0],sp = [0.05,0.05,math.pi/5],error_need=True)
        my_car.beep()
        # time.sleep(2)
        infornt = 0.05
        aa_speed = [0.1,0.1,math.pi/5]
        ok_flag = my_car.set_pos_offset([infornt, 0, 0],sp = aa_speed,left_inface=2,error_need=True)
        dir = 1
        times = 0
        my_car.beep()
        # time.sleep(2)
        while ok_flag == 0:
            my_car.set_pos_offset([0, 0.03, 0],sp = aa_speed,left_inface=2,error_need=True)
            ok_flag = my_car.set_pos_offset([-infornt, 0, 0],sp = aa_speed,left_inface=2,error_need=True)
            dir = -1
            times+=1
            if times>3:
                break
            if ok_flag ==1:
                break
            my_car.set_pos_offset([0, 0.03, 0],sp = aa_speed,left_inface=2,error_need=True)
            ok_flag = my_car.set_pos_offset([infornt, 0, 0],sp = aa_speed,left_inface=2,error_need=True)
            dir = -1
            times+=1
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)

        if GPIO.input(23) != 0:
            if dir == 1:
                my_car.set_pos_offset([-0.05, 0, 0], sp = [0.01,0.01,math.pi/5],left_inface=2)
            else:
                my_car.set_pos_offset([0.05, 0, 0], sp = [0.01,0.01,math.pi/5],left_inface=2)
        GPIO.cleanup()
        my_car.set_pos_offset([0, -0.2, 0],sp=[0.01, 0.01, math.pi / 5],left_inface = 1)

        my_car.set_pos_offset([-0.22,0.06, 0],sp = [0.1,0.1,math.pi/5],error_need=True)
        my_car.beep()
        #time.sleep(3)
        # my_car.lane_advance(0.2, dis_offset=0.01, value_h=0.2, sides=1)
        # my_car.set_pos_offset([0.20, -0.15, 0], 1)
        # # 调整位置准备抓球
        my_car.task.pick_high_ball()
        my_car.set_pos_offset([0.22,-0.06, 0], 1)
        my_car.task.arm.set_offset(0, -0.11)
        #my_car.set_pos_offset([0, -0.07, 0], 0.7)


    def pull_ball_func():
        my_car.task.servo_ball.set_angle(60, -95)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(23, GPIO.IN)
        my_car.lane_dis_offset(0.3, 1)
        my_car.lane_sensor(0.2, value_h=0.50, sides=1,stop=False)    
        # time.sleep(0.5)    

        # my_car.lane_sensor(0.3, value_l=0.5, sides=1,stop=False)   
        my_car.set_pos_offset([0.3-0.05, 0, 0],error_need=True) 
        my_car.set_pos_offset([0, 0.05+0.015, 0],error_need=True) 
        # my_car.beep()
        my_car.task.put_down_ball()
 



    
    #进入修整营地 
    def camp_fun_old():
        # my_car.lane_advance(0.3, value_h=0.3, sides=-1)
        # time.sleep(25)
        my_car.lane_dis_offset(0.2, 1.05)
        # 调整位置准备进行ocr识别
        light.set_light(4, 70, 70, 70)
        my_car.task.ocr_arm_ready(-1)
        # 感应到右侧障碍距离小于0.4
        my_car.lane_sensor(0.3, value_h=0.4, dis_offset=0.02, sides=-1, stop=True)

        start_pos = np.array(my_car.get_odom())
        # end_pos = start_pos + np.array(pos)
        pos = [-0.07, 0.00, 27.5*math.pi/180]
        start_angle = start_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, pos[2]]) + start_pos

        my_car.set_pos(tar_pos,error_need=True)
        start_pos = tar_pos
        # for i in range(3,7):
        #     light.set_light(i, 140, 140, 140)
        # light.set_light(4, 140, 140, 140)
        a = time.time()
        i =0
        actions_map = None
        actions_map2 = None
        # while True:
        text = my_car.get_ocr()
        # text = '蜂鸣器每间隔一秒叫一次，叫三次,然后等待三秒'
        my_car.beep()
        # logger.info("text:{}".format(text))
        try:
            actions_map1 = my_car.yiyan_get_actions(text)
            actions_map2 = actions_map1
            print('actions_map1: ',actions_map1)
        except:
            actions_map1 = None
        while True:
            if time.time()-a>35:
                actions_map = None
                if actions_map2  != None:
                    actions_map2 = actions_map
                print('超时退出营地识别！！！')
                break
            else:
                text = my_car.get_ocr()
                # text = '蜂鸣器每间隔一秒叫一次，叫三次,然后等待三秒'
                logger.info("text:{}".format(text))
                try:
                    actions_map = my_car.yiyan_get_actions(text)
                except:
                    pass
                if actions_map != None:
                    actions_map2 = actions_map
                if actions_map1==actions_map :
                    i += 1
                else:
                    actions_map1 = actions_map
                    i = 0
                if i ==1 and actions_map!= None:
                    break
            print('actions_map: ',actions_map)
        # text ='原地旋转180度'
        # logger.info("text:{}".format(text))
        # actions_map = my_car.yiyan_get_actions(text)
        print('finally actions_map: ',actions_map)

        # 前移到营地左侧
        pos = [0.07,0, -27.5*math.pi/180]
        start_angle = start_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, pos[2]]) + start_pos

        my_car.set_pos(tar_pos,error_need=True)
        start_pos = tar_pos


        # 做任务
        my_car.lane_dis_offset(0.2, 0.5) 
       

        if actions_map != None: 
            my_car.lane_det_location(speed=0.1, pt_tar=[0, 0, 'stop',  0, -0.28125, 0.5649038461538461, 0.8605769230769231, 0.6394230769230769],side=-1,det='redstop',camp=1)
            # pos_start = np.array(my_car.get_odom())

            
            time.sleep(0.5)
            _pos_ = np.array(my_car.get_odom())
            my_car.set_pos_offset([0, -0.44,0])

            my_car.do_action_list(actions_map)
            my_car.set_pos(_pos_, error_need=True)#

            # my_car.set_pos_offset([-0.04, 0.44-0.06,0])


        for i in range(1,9):
            light.set_light(i, 0, 0, 0)

    # 找到罪犯打击罪犯
    def find_criminal():
        # my_car.lane_dis_offset(0.3, 0.5)
        # my_car.task.arm.switch_side(-1)
        # 调整位置准备进行ocr识别
        my_car.task.ocr_arm_ready(-1)
        # 感应到右侧障碍距离小于0.4
        my_car.lane_sensor(0.2, value_h=0.5, dis_offset=0.02, sides=-1, stop=False)
        light.set_light(4, 70, 70, 70)

        pt_tar = my_car.task.punish_crimall(arm_set=True)
        # 巡航到识别位置
        my_car.AI_FLAG = random.randint(0,3) 
        # my_car.AI_FLAG = 2
        print('打击第',my_car.AI_FLAG+1,'个罪犯')
        my_car.lane_sensor(0.1, value_h=0.5, sides=-1, stop=True)

        if my_car.AI_FLAG == 1:
            my_car.set_pos_offset([0.1, 0, 0],error_need=True,sp=[0.1, 0.1, math.pi / 20])

        elif my_car.AI_FLAG == 2:
            my_car.set_pos_offset([0.18, 0, 0],error_need=True,sp=[0.1, 0.1, math.pi / 20])

        elif my_car.AI_FLAG == 3:
            my_car.set_pos_offset([0.25, 0, 0],error_need=True,sp=[0.1, 0.1, math.pi / 20])

        my_car.task.punish_crimall()




    def go_start():
        my_car.lane_sensor(0.3, value_l=0.4, sides=-1)
        my_car.set_pos_offset([0.85, 0, 0], 2.8)
        # 前移


    my_car.beep()
    time.sleep(0.2)
    functions = [ hanoi_tower_func, grap_block_func, release_block_func, get_ball_func, elevation_pole_func, get_high_ball_func, 
                 pull_ball_func, camp_fun_old, find_criminal, go_start]
    my_car.manage(functions, 0)


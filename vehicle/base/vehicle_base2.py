#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择
import sys, os

dir_file = os.path.dirname(__file__)
sys.path.append(os.path.abspath(dir_file))
root_path = os.path.join(dir_file, "..", "..")
sys.path.append(root_path)
# 导入自定义log模块
from log_info import logger
from controller2 import *

import shutil
import math
import numpy as np
from simple_pid import PID
from threading import Thread
import yaml
import RPi.GPIO as GPIO
from vehicle import My_Beep
def limit_val(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class MotoConvert:
    def __init__(self, perimeter=None) -> None:
        # 编码器一圈的编码值 次
        self.encoder_resolution = 2016
        # 编码速度转换值 m/s
        self.speed_rate = 100
        # 周长
        if perimeter is None:
            perimeter = 0.06 * math.pi
        # 距离分辨率，一个数走多远  m/次
        self.dis_resolution = perimeter / self.encoder_resolution

    #  通过周长修改新的分辨率
    def set_perimeter(self, perimeter):
        self.dis_resolution = perimeter / self.encoder_resolution
        # print(self.dis_resolution)

    # 通过直径计算新的分辨率
    def set_diameter(self, diameter):
        self.dis_resolution = diameter * math.pi / self.encoder_resolution

    # 速度转化为编码器转速
    def sp2virtual(self, speed):
        # 速度转为encoder输出 m/s m/次 =  次/秒
        #  30cm/s 2cm/次 15次/秒
        speed_encoder = speed / self.dis_resolution
        # encoder转为控制器设置值 次/秒  100  = 次/秒
        speed_out = int(speed_encoder / self.speed_rate)
        speed_out = limit_val(speed_out, -100, 100)
        return int(speed_out)

    # 距离 次 * m/次 = m
    def dis2true(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    #  实际速度
    def sp2true(self, speed):
        # 控制器速度转为encoder输出
        speed_encoder = int(speed * self.speed_rate)
        speed_out = speed_encoder * self.dis_resolution
        return speed_out

    # 同dis2ture
    def encoder2dis(self, encoder_dis):
        dis_out = encoder_dis * self.dis_resolution
        return dis_out

    # 距离转化为编码器计数次数
    def dis2encoder(self, dis):
        encoder_out = dis / self.dis_resolution
        return encoder_out


class ArmBase():
    def __init__(self) -> None:
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = '/home/nvidia/Desktop/vehicle_wbt/vehicle/base/config_arm.yaml'
        flag_reset = False
        while True:
            try:
                with open(self.yaml_path, 'r') as stream:
                    self.config = yaml.load(stream, Loader=yaml.FullLoader)
                self.horiz_threshold = self.config["horiz_threshold"]
                self.vert_threshold = self.config["vert_threshold"]
                self.horiz_moto_param = self.config["horiz_moto_param"]
                self.vert_moto_param = self.config["vert_moto_param"]
                self.horiz_limit_port = self.config["horiz_limit_port"]
                self.vert_limit_port = self.config["vert_limit_port"]
                self.horiz_pid_param = self.config["horiz_pid_param"]
                self.vert_pid_param = self.config["vert_pid_param"]
                self.grap_servo_port = self.config["grap_servo_port"]
                self.arm_servo_id = self.config["arm_servo_id"]
                self.side = self.config["side"]
                print('读取到了机械臂yml')
                # 定义初始化属性
                self.pos_enable = self.config["pos_enable"]
                if not self.pos_enable:
                    raise Exception("pos_enable is False")
                self.pos_vert = self.config["pos_vert"]
                # self.pos_vert = 0
                self.pos_vert_start = self.pos_vert
                self.pos_horiz = self.config["pos_horiz"]
                # self.pos_horiz = 0
                self.pos_horiz_start = self.pos_horiz
                self.angle_list = self.config["angle_list"]
                # if flag_reset:
                #
                break
            except:
                print('进入机械臂复位')
                # break
                # # 默认设置
                # self.config = {"pos_enable": True, "side": -1, "pos_vert": 0, "pos_horiz": 0,
                #                "angle_list": [90, 0, -100],
                #                "horiz_moto_param": {"port_id": 5, "reverse": -1},
                #                "vert_moto_param": {"port_id": 6, "reverse": 1},
                #                "horiz_limit_port": 3, "vert_limit_port": 4, "grap_servo_port": 1, "arm_servo_id": 1,
                #                "vert_pid_param": {"Kp": 20, "Ki": 0.0, "Kd": 0.4, "setpoint": 0},
                #                "horiz_pid_param": {"Kp": 20, "Ki": 0.0, "Kd": 0.4, "setpoint": 0},
                #                "vert_threshold": [-0.23, 0.23], "horiz_threshold": [-0.23, 0.23]}
                # self.pos_vert = 0
                # self.pos_horiz = 0
                # self.save_yaml()
                # flag_reset = True

        # 定义竖直移动电机 限位传感器，速度转换参数
        self.vert_moto = Motor(**self.vert_moto_param)
        self.encoder_vert = EncoderMotor(self.vert_moto_param["port_id"])
        # self.encoder_vert.reset()
        # 获取编码器encoder_vert_start开机时的值
        self.encoder_vert_start = self.encoder_vert.get()
        # print('encoder start: ',self.encoder_vert_start)
        self.limit_vert = SensorAi(self.vert_limit_port)
        vert_perimeter = 0.037 * math.pi / 56 * 8
        self.vert_convert = MotoConvert(vert_perimeter)
        self.vert_pid = PID(**self.vert_pid_param)

        # 定义水平移动电机 限位传感器 速度转换参数
        self.horiz_moto = Motor(**self.horiz_moto_param)
        self.encoder_horiz = EncoderMotor(self.horiz_moto_param["port_id"])
        # self.encoder_horiz.reset()
        # 开机时encoder_horiz_start的值
        self.encoder_horiz_start = self.encoder_horiz.get()
        self.limit_horiz = SensorAi(self.horiz_limit_port)
        horiz_perimeter = 0.06 / 15 * 8
        self.horiz_convert = MotoConvert(horiz_perimeter)
        self.horiz_pid = PID(**self.horiz_pid_param)

        self.key = Key4Btn(2)
        # 抓取的舵机
        self.grap_servo = ServoPwm(self.grap_servo_port)
        self.grap_servo.set(50, 0)

        # 机械臂舵机,旋转角度
        self.arm_servo = ServoBus(self.arm_servo_id)

        # 未有默认设置文件
        if flag_reset:
            self.reset()

        # 位置调整
        if self.side == 0:
            self.switch_side(-1)
        # self.arm_servo.set_angle(50, 90)

    def save_yaml(self, pos_enable=True):
        self.config["pos_enable"] = pos_enable
        self.config["pos_vert"] = self.pos_vert
        self.config["pos_horiz"] = self.pos_horiz
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream)

    def set_pos_start(self, pos_vert):
        self.pos_vert_start = self.pos_vert
        self.pos_horiz_start = self.pos_horiz
        self.save_yaml()

    # 通过按键设置轮子的转速移动一段距离
    def set_by_yourself(self):
        logger.info("set arm by yourself...")
        while True:
            val = self.key.get_key()
            if val == 1:
                self.vert_moto.set(30)
            elif val == 3:
                self.vert_moto.set(-30)
            elif val == 4:
                self.horiz_moto.set(30)
            elif val == 2:
                self.horiz_moto.set(-30)
            else:
                self.horiz_moto.set(0)
                self.vert_moto.set(0)

            # 更新目前的位置
            self.update_pos(0)
            self.update_pos(1)
            # time.sleep(0.1)

    def reset(self):
        self.reset_pos_dir(2, 0.06)
        # 回到初始位置

        # self.vert_moto.set(0)
        # self.horiz_moto.set(0)

    # 切换机械臂方向 -1右边 1左边
    def switch_side(self, side):
        if self.side != side:
            self.side = side
            logger.info("change side to {}".format(self.side))
            # print("change side to {}".format(self.side))

        else:
            return
        angle_tar = self.angle_list[side + 1]
        self.config["side"] = side
        self.save_yaml()
        self.set(-0.12, 0.08+0.05, speed=0.3)#0.06
        logger.info("change to -0.12, 0.08")
        # angle_tar = -1*side*90
        self.set_arm_angle(angle_tar, 120)#80
        time.sleep(1.5)

    # 直到到达预定动作。设置回默认
    def reset_pos_dir(self, dir=0, speed=0.1):
        vert_flag = True
        horiz_flag = True
        if dir == 0:
            horiz_flag = False
        elif dir == 1:
            vert_flag = False
        else:
            horiz_flag = False
            vert_flag = False

        while True:
            if vert_flag and horiz_flag:
                break

            if not horiz_flag:
                horiz_val = self.limit_horiz.get()
                # print("horiz_val:", horiz_val)
                if horiz_val > 1000:
                    self.move_horiz(0)
                    self.pos_horiz = 0
                    self.pos_horiz_start = 0
                    self.encoder_horiz.reset()

                    self.pos_horiz_start = 0
                    self.pos_horiz = 0
                    horiz_flag = True
                else:
                    self.move_horiz(speed)

            if not vert_flag:
                vert_val = self.limit_vert.get()
                # print("vert_val:", vert_val)
                if vert_val > 1000:
                    self.move_vert(0)
                    self.encoder_vert.reset()

                    self.pos_horiz_start = 0
                    self.pos_vert = 0
                    vert_flag = True
                else:
                    self.move_vert(0 - abs(speed))

    # 设置抓取角度
    def set_grap_angle(self, angle):
        self.grap_servo.set(50, angle)

    # 设置机械臂的关节角度
    def set_arm_angle(self, angle, speed=80,restart_flag = False):
        restart_flag = True
        # 设置
        self.arm_servo.set_angle(speed, angle,restart_flag = restart_flag)
        if angle in self.angle_list:
            self.side = self.angle_list.index(angle) - 1
        else:
            self.side = 0

    def set_arm_dir(self, dir=0, speed=80):
        assert dir == 0 or dir == 1 or dir == -1, "dir should be 0 or 1 or -1"
        self.set_arm_angle(self.angle_list[dir + 1], speed)

    # 设置竖直转速
    def move_vert(self, speed_vert):
        speed_out = self.vert_convert.sp2virtual(speed_vert)
        # print(speed_out)
        self.vert_moto.set(speed_out)

    # 设置水平转速
    def move_horiz(self, speed_horiz):
        speed_out = self.horiz_convert.sp2virtual(speed_horiz)
        # print(speed_out)
        self.horiz_moto.set(speed_out)

    # 更新当前的位置
    def update_pos(self, axis):
        if axis == 0:
            # 相对开始时的编码值
            encoder_vert = self.encoder_vert.get() - self.encoder_vert_start
            # print('encoder_vert: ',encoder_vert)
            self.pos_vert = self.pos_vert_start + self.vert_convert.dis2true(encoder_vert)
            # print('pos_vert: ',self.pos_vert)
            self.config["pos_vert"] = self.pos_vert
        else:
            # 相对开始时的编码值 水平方向
            encoder_horiz = 0 - (self.encoder_horiz.get() - self.encoder_horiz_start)
            self.pos_horiz = self.pos_horiz_start + self.horiz_convert.dis2true(encoder_horiz)
            self.config["pos_horiz"] = self.pos_horiz

        # print(self.pos_vert, self.pos_horiz)

    # 设置移动距离，调用set函数(相对位置) 水平 垂直
    def set_offset(self, horiz_offset, vert_offset, time_run=None, speed=[0.1, 0.05]):
        horiz_pos = self.pos_horiz + horiz_offset
        vert_pos = self.pos_vert + vert_offset
        self.set(horiz_pos, vert_pos, time_run, speed)

    # 电机转动,偏移到达指定位置(物理位置),没有时间限制就用速度(根据速度算时间如果速度太大,限制时间),或者直接根据时间
    def set(self, horiz_pos, vert_pos, time_run=None, speed=[0.1, 0.05]):
        # print(horiz_pos)
        # 控制上下限
        horiz_pos = limit_val(horiz_pos, self.horiz_threshold[0], self.horiz_threshold[1])
        # print(horiz_pos)
        vert_pos = limit_val(vert_pos, self.vert_threshold[0], self.vert_threshold[1])

        # 获取结束时间和对应速度
        time_start = time.time()
        if time_run is not None:
            assert isinstance(time_run, int) or isinstance(time_run, float), "wrong time args"
            # 根据时间求速度
            time_end = time_start + time_run
            vert_time = time_run
            horiz_time = time_run
        elif speed is not None:
            # 根据速度求时间
            if isinstance(speed, int) or isinstance(speed, float):
                # print(speed)
                speed_horiz = speed
                speed_vert = speed
            elif isinstance(speed, list) or isinstance(speed, tuple):
                speed_horiz = speed[0]
                speed_vert = speed[1]
            else:
                logger.error("wrong speed args")
                return
            horiz_time = abs(horiz_pos - self.pos_horiz) / speed_horiz
            vert_time = abs(vert_pos - self.pos_vert) / speed_vert
            time_run = max(horiz_time, vert_time)
        else:
            logger.error("wrong args")
            return
        # 超时时间
        time_end = time_start + time_run

        # 定义结束标志和到达位置标记量
        vert_flag = False
        horiz_flag = False
        count_vert = 0
        count_horiz = 0
        # print('vert_time:',vert_time)
        # print('horiz_time:',horiz_time)
        # 获取对应的速度和pid位置
        if vert_time < 0.01:
            speed_vert = 0.1
            vert_flag = True
        else:
            speed_vert = abs(vert_pos - self.pos_vert) / vert_time

        self.vert_pid.setpoint = vert_pos
        self.vert_pid.output_limits = (-speed_vert, speed_vert)

        if horiz_time < 0.01:
            speed_horiz = 0.1
            horiz_flag = True
        else:
            speed_horiz = abs(horiz_pos - self.pos_horiz) / horiz_time
        self.horiz_pid.setpoint = horiz_pos
        self.horiz_pid.output_limits = (-speed_horiz, speed_horiz)
        # 开始移动前，位置信息定义，如果中间中断此时位置信息无用
        self.save_yaml(pos_enable=False)
        while True:
            # 到达结束标志结束
            if vert_flag and horiz_flag:
                break
            # 获取剩余时间
            time_remain = time_end - time.time()
            # print("time remain:", time_remain)
            # 超时处理
            if time_remain < -1.5:
                logger.warning("timeout")
                # 超时停止
                self.move_horiz(0)
                self.move_vert(0)
                break
            if not vert_flag:
                dis_vert = vert_pos - self.pos_vert
                # logger.debug("dis_vert:", dis_vert)
                # logger.debug("pos vert:", self.pos_vert)
                # 到达定义位置，确认十次以上就可以到达目标位置了
                if abs(dis_vert) < 0.005:
                    count_vert += 1
                    # print("count_vert:", count_vert)
                    if count_vert > 10:
                        vert_flag = True
                        self.move_vert(0)
                        continue
                else:
                    count_vert = 0
                # print('vert_limit:',self.limit_vert.get())
                # 重置初始化位置
                if self.limit_vert.get() > 1000 and False:
                    print(111111111111111111111)
                    self.pos_vert = 0
                    self.pos_vert_start = 0
                    self.encoder_vert_start = self.encoder_vert.get()

                    self.save_yaml()
                    # 移动方向如果向下，则停止移动
                    if dis_vert < 0:
                        self.move_vert(0)
                        vert_flag = True
                        continue
                # pid控制输出
                speed_vert = self.vert_pid(self.pos_vert)
                # print("speed_vert:", speed_vert)
                # 转化为编码器转速
                self.move_vert(speed_vert)
                self.update_pos(0)

            if not horiz_flag:
                dis_horiz = horiz_pos - self.pos_horiz
                # print("dis_horiz:", dis_horiz)
                # print("pos horiz:", self.pos_horiz)
                # 到达定义位置
                if abs(dis_horiz) < 0.005:
                    count_horiz += 1
                    # print("count_horiz:", count_horiz)
                    if count_horiz > 10:
                        horiz_flag = True
                        self.move_horiz(0)
                        continue
                else:
                    count_horiz = 0
                # 重置初始化位置
                if self.limit_horiz.get() > 1000 and False:

                    self.pos_horiz = 0
                    self.pos_horiz_start = 0
                    self.encoder_horiz_start = self.encoder_horiz.get()
                    # 移动方向如果向右，则停止移动
                    if dis_horiz > 0:
                        self.move_horiz(0)
                        horiz_flag = True
                        continue

                speed_horiz = self.horiz_pid(self.pos_horiz)
                # print("speed_horiz:", speed_horiz)
                self.move_horiz(speed_horiz)
                self.update_pos(1)

            # self.save_yaml()
            # time.sleep(0.005)
        self.save_yaml()
        # logger.debug("pos vert:{}, horiz:{}".format(self.pos_vert, self.pos_horiz))


def get_pid(kp, ki, kd, val, setpoint=0):
    my_pid = PID(kp, ki, kd, setpoint=setpoint)
    my_pid.output_limits = (-val, val)
    return my_pid


class VehicleBase:
    def __init__(self) -> None:
        # self.ser = SerialWrap()
        # self.ser.ping_port()
        # print("VehicleBase init ok")
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, "config_vehicel.yaml")

        self.buzzer = Beep()

        wheel_perimeter = 0.06 * math.pi
        self.wheel_convert = MotoConvert(wheel_perimeter)

        self.base_moto = Motor4()

    def beep(self):
        self.buzzer.set(200, 10)

    def run2(self, speed_l, speed_r):
        speed_l = self.wheel_convert.sp2virtual(speed_l)
        speed_r = self.wheel_convert.sp2virtual(speed_r)
        # print(speed_l, speed_r)
        self.base_moto.set(speed_l, 0 - speed_r, speed_l, 0 - speed_r)

    def run4(self, *speeds):
        # print(speeds)
        dir = -1
        if isinstance(speeds[0], list):
            sp_tar = speeds[0]
        else:
            sp_tar = list(speeds)
        for i in range(4):
            dir = dir * -1
            sp_tar[i] = int(dir * self.wheel_convert.sp2virtual(sp_tar[i]))
        # print(sp_tar)
        self.base_moto.set(*sp_tar)


class MecanumBase():
    def __init__(self) -> None:
        logger.info("mecanum init")
        # 配置文件存储路径
        self.yaml_name = "config_vehicel.yaml"
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.yaml_path = os.path.join(self.path_dir, self.yaml_name)
        # 根据路径获取配置信息
        self.get_yaml_config()

        self.buzzer = My_Beep()
        # 电机
        self.base_moto = Motor4()
        # 四个底盘电机编码器
        self.encoders_motors = EncoderMotors()
        self.encoders_motors.reset()

        # 计算近似半径
        self.radius1 = (self.rx + self.ry) / 2
        self.radius2 = (self.rx + self.ry) * 2

        self.dis_now = 0
        self.pos_start = np.array([0, 0, 0])
        # 车子整体前进的路程变量
        self.dis_traveled = 0
        # 车子起始位置相对世界坐标系
        self.odom_x = 0
        self.odom_y = 0
        self.odom_theta = 0
        self.speed_now = np.array([0, 0, 0])

        # pid控制
        self.pid_turn = PID(self.pid_turn_params["kp"], self.pid_turn_params["ki"], self.pid_turn_params["kd"])
        self.pid_turn.setpoint = 0
        self.pid_turn.output_limits = (
        -abs(self.pid_turn_params["output_limit"]), abs(self.pid_turn_params["output_limit"]))

        self.pid_dis_x = PID(self.pid_dis_x_params["kp"], self.pid_dis_x_params["ki"], self.pid_dis_x_params["kd"])
        self.pid_dis_x.setpoint = 0
        self.pid_dis_x.output_limits = (
        -abs(self.pid_dis_x_params["output_limit"]), abs(self.pid_dis_x_params["output_limit"]))

        self.pid_dis_y = PID(self.pid_dis_y_params["kp"], self.pid_dis_y_params["ki"], self.pid_dis_y_params["kd"])
        self.pid_dis_y.setpoint = 0
        self.pid_dis_y.output_limits = (
        -abs(self.pid_dis_y_params["output_limit"]), abs(self.pid_dis_y_params["output_limit"]))

        self.last_encoders = self.encoders_motors.get()
        self.odom_update_flag = True
        self.last_time = time.time()
        # 多线程一直获得里程计信息并更新
        self.odom_thread = Thread(target=self.odom_update, args=())
        self.odom_thread.daemon = True
        self.odom_thread.start()
        time.sleep(0.2)

        # self.base_motors.set_speed(speeds)

    def get_yaml_config(self):
        while True:
            try:
                with open(self.yaml_path, 'r') as stream:
                    self.config = yaml.load(stream, Loader=yaml.FullLoader)
                # 轮胎直径
                self.wheel_diameter = self.config["wheel_diameter"]
                # 根据存储的轮胎直径获取轮胎的周长 
                self.wheel_perimeter = self.wheel_diameter * math.pi
                self.wheel_convert = MotoConvert(self.wheel_perimeter)
                self.rx = self.config["rx"]
                self.ry = self.config["ry"]
                self.pid_turn_params = self.config["pid_turn_params"]
                self.pid_dis_x_params = self.config["pid_dis_x_params"]
                self.pid_dis_y_params = self.config["pid_dis_y_params"]
                break
            except Exception as e:
                # print(e)
                self.config = {
                    "wheel_diameter": 0.06,
                    "rx": 0.305,
                    "ry": 0.28,
                    "pid_turn_params": {
                        "kp": 5,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 1.5
                    },
                    "pid_dis_x_params": {
                        "kp": 20,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 0.3
                    },
                    "pid_dis_y_params": {
                        "kp": 20,
                        "ki": 0.0,
                        "kd": 0.1,
                        "output_limit": 0.3
                    }
                }
                with open(self.yaml_path, 'w') as stream:
                    yaml.dump(self.config, stream)

    def save_yaml(self):
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream)

    def beep(self):
        self.buzzer.set(0.2, 1)

    def run2(self, speed_l, speed_r):
        speed_l = self.wheel_convert.sp2virtual(speed_l)
        speed_r = self.wheel_convert.sp2virtual(speed_r)
        # print(speed_l, speed_r)
        self.base_moto.set(speed_l, 0 - speed_r, speed_l, 0 - speed_r)

    def run4(self, *speeds):
        # print(speeds)
        dir = -1
        if isinstance(speeds[0], list):
            sp_tar = speeds[0]
        else:
            sp_tar = list(speeds)
        for i in range(4):
            dir = dir * -1
            sp_tar[i] = int(dir * self.wheel_convert.sp2virtual(sp_tar[i]))
        # print(sp_tar)
        self.base_moto.set(*sp_tar)

    def stop(self):
        self.run2(0, 0)

    def get_encoders(self):
        re_list = self.encoders_motors.get()
        if not re_list:
            return [0, 0, 0, 0]
        re_list[1] = re_list[1] * -1
        re_list[3] = re_list[3] * -1
        return re_list

    # 计算每个轮子的速度
    def mecanum_inverse(self, vx, vy, vomega):
        speed_out = [0, 0, 0, 0]
        speed_out[0] = vx - vy - self.radius1 * vomega
        speed_out[1] = vx + vy + self.radius1 * vomega
        speed_out[2] = vx + vy - self.radius1 * vomega
        speed_out[3] = vx - vy + self.radius1 * vomega
        return speed_out

    # 计算整车的速度
    def mecanum_forward(self, d_vect):
        car_dx = (d_vect[0] + d_vect[1] + d_vect[2] + d_vect[3]) / 4.0  # 车x方向位移速度
        car_dy = (0 - d_vect[0] + d_vect[1] + d_vect[2] - d_vect[3]) / 4.0;  # 车y方向位移速度
        car_domega = (0 - d_vect[0] + d_vect[1] - d_vect[2] + d_vect[3]) / self.radius2  # 车角速度
        return car_dx, car_dy, car_domega

    # 计算速度并且使用run4来运行
    def mecanum_wheel(self, speed_vx, speed_vy, speed_vomega):
        # self.odom_update()
        speeds = self.mecanum_inverse(speed_vx, speed_vy, speed_vomega)
        # speeds = self.
        # print(speeds)
        self.run4(speeds)

    # 设定运动到的位置，然后停止
    def set_pos(self, pos, time_dur=None, sp=[0.2, 0.2, math.pi / 4],read_inface = False,error_need = True,left_inface = False,sensor = 0):
        start_pos = np.array(self.get_odom())
        tar_pos = np.array(pos)

        if time_dur is not None:
            # 平均速度
            sp = (tar_pos - start_pos) / time_dur
        else:
            time_dur = np.max(abs((tar_pos - start_pos) / sp))
            sp = (tar_pos - start_pos) / time_dur

        # print("time_dur:{}".format(time_dur))
        # print("start{}, tar{}, sp{}".format(start_pos, tar_pos, sp))

        self.pid_dis_x.setpoint = tar_pos[0]
        self.pid_dis_x.output_limits = (-abs(sp[0]), abs(sp[0]))

        self.pid_dis_y.setpoint = tar_pos[1]
        self.pid_dis_y.output_limits = (-abs(sp[1]), abs(sp[1]))

        self.pid_turn.setpoint = tar_pos[2]
        self.pid_turn.output_limits = (-abs(sp[2]), abs(sp[2]))

        start_time = time.time()
        count_flag = 0

        filter = 0
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.IN)
        GPIO.setup(23, GPIO.IN)
        ok_flag = 0
        while True:
            current_time = time.time()
            dt = current_time - start_time
            if dt > time_dur + 0.3 +0.2:
                print("move pos time out")
                break
            if read_inface != 0:
                input_state = GPIO.input(18)
                if input_state != read_inface-1:
                    filter+=1
                else:
                    filter = 0
                if filter>10:
                    ok_flag = 1
                    GPIO.cleanup()
                    break
            if left_inface != 0:
                input_state = GPIO.input(23)
                if input_state != left_inface-1:
                    filter+=1
                else:
                    filter = 0
                if filter>10:
                    GPIO.cleanup()
                    ok_flag = 1
                    break
            now_pos = np.array(self.get_odom())
            # print("now_pos = ",now_pos,"  tar_pos = ",tar_pos)
            # print("now_pos:{}".format(now_pos))
            # 获取剩余距离值和角度值
            error_dis = tar_pos - now_pos
            # print("error_dis:{}".format(error_dis))
            if error_need  == True:
                if abs(error_dis[0]) < 0.01 and abs(error_dis[1]) < 0.01 and abs(error_dis[2]) < math.pi/180: 
                    count_flag += 1
                    if count_flag > 20:
                        break
                else:
                    count_flag = 0
            else:
                if abs(error_dis[0]) < 0.01 and abs(error_dis[1]) < 0.01 and abs(error_dis[2]) < 0.5:
                    count_flag += 1
                    if count_flag > 20:
                        break
                else:
                    count_flag = 0
            out_x = self.pid_dis_x(now_pos[0])
            out_y = self.pid_dis_y(now_pos[1])
            out_omega = self.pid_turn(now_pos[2])
            # 世界输出转为机器人输出
            angle_robot = now_pos[2]
            # dis_omega = error_dis[2]
            # 根据误差角度计算实际运行速度和角度值
            sp_x = math.cos(angle_robot) * out_x + math.sin(angle_robot) * out_y
            sp_y = 0 - math.sin(angle_robot) * out_x + math.cos(angle_robot) * out_y
            sp_omega = out_omega

            # print("pid in_x:{}, in_y:{}, in_omega:{}".format(now_pos[0], now_pos[1], now_pos[2]))
            # print("pid out_x:{}, out_y:{}, out_omega:{}".format(out_x, out_y, out_omega))
            # print("out_x", out_x, "out_y", out_y, "out_omega", out_omega)
            self.mecanum_wheel(sp_x, sp_y, sp_omega)
        # print(self.get_odom())
        self.run4([0, 0, 0, 0])
        return ok_flag

    # 相对当前的位置的继续前进
    def set_pos_offset(self, pos, time_dur=None, sp=[0.2, 0.2, math.pi / 4],read_inface = False,error_need = True,left_inface = False,sensor = 0):
        start_pos = np.array(self.get_odom())
        # end_pos = start_pos + np.array(pos)

        start_angle = start_pos[2]
        dis_x = math.cos(start_angle) * pos[0] - math.sin(start_angle) * pos[1]
        dis_y = math.sin(start_angle) * pos[0] + math.cos(start_angle) * pos[1]
        tar_pos = np.array([dis_x, dis_y, pos[2]]) + start_pos
        # tar_pos = start_pos + np.array(pos)
        return self.set_pos(tar_pos, time_dur, sp,read_inface = read_inface,error_need = error_need,left_inface = left_inface,sensor = sensor)

    # 更新里程计信息
    def odom_update(self):
        self.last_encoders = np.array(self.get_encoders())
        self.last_time = time.time()
        while True:
            if not self.odom_update_flag:
                break
            try:
                # 获取当前编码器值

                current_enc = np.array(self.get_encoders())

                d_enc = current_enc - self.last_encoders
                dt = time.time() - self.last_time
                if dt == 0:
                    dt = 0.05
                self.last_time = time.time()
                self.last_encoders = current_enc
                # 计算编码器值变化
                d_dis = self.wheel_convert.dis2true(d_enc)

                # 计算这个瞬间的位移并叠加
                self.dis_traveled += np.average(d_dis)

                # 计算当前位移角度变化
                car_dx, car_dy, car_domega = self.mecanum_forward(d_dis)

                self.speed_now = np.array([car_dx, car_dy, car_domega]) / dt
                # print("speed_closed", self.speed_closed)
                # print(self.speed_closed)
                # print("car move",car_dx, car_dy, car_domega)
                self.dis_now += car_dx

                dx = car_dx * math.cos(self.odom_theta) - car_dy * math.sin(self.odom_theta)
                dy = car_dx * math.sin(self.odom_theta) + car_dy * math.cos(self.odom_theta)

                domega = car_domega
                # print(dx, dy, domega)
                # print(self.odom_x, self.odom_y, self.odom_theta)
                self.odom_x += dx
                self.odom_y += dy
                self.odom_theta += domega

            except Exception as e:
                pass
            # print("odom x", self.odom_x, "odom y", self.odom_y, "odom theta", self.odom_theta)
            time.sleep(0.05)

    # 重新设置参考的初始点位
    def set_pos_relative(self, pos_src=None):
        if pos_src is None:
            pos_src = self.get_odom()
        self.pos_start = pos_src

    # 计算相对位移
    def get_pos_relative(self, pos_dst=None, pos_src=None):
        if pos_dst is None:
            pos_dst = self.get_odom()
        if pos_src is None:
            pos_src = self.pos_start
        pos = np.array(pos_dst) - np.array(pos_src)
        dis_x = pos[0] * math.cos(pos_src[2]) - pos[1] * math.sin(pos_src[2])
        dis_y = pos[1] * math.cos(pos_src[2]) + pos[0] * math.sin(pos_src[2])
        dis_angle = pos[2]
        return dis_x, dis_y, dis_angle

    # 获得里程计信息
    def get_odom(self):
        return self.odom_x, self.odom_y, self.odom_theta

    # 计算路程
    def get_dis_traveled(self):
        return self.dis_traveled

    def get_speed_closed(self):
        return self.speed_now

    def close(self):
        self.odom_update_flag = False

# 加上按键调
if __name__ == '__main__':
    # ser = SerialWrap()
    car = MecanumBase()
    car.set_pos_offset([0.15, 0, 0])
    car.set_pos_offset([0.3, 0, -math.pi/5])
    # car = MecanumBase()

    # arm = ArmBase()
    # car.arm.set_by_yourself()

    # car.pull_init()

    # car = Car()
    # car.beep()
    # while True:
    #     # arm.set(0, 0.05, 1)
    #     # time.sleep(1)
    #     # arm.set(0, 0.1, 1)
    #     # time.sleep(1)
    #     # car.pull_down()
    #     # time.sleep(5)
    #     for i in range(10):
    #         car.run2(0.1, 0.1)
    #         time.sleep(0.1)
    #     car.run2(0, 0)

    #     time.sleep(1)
        # car.servo_rotate.set_rotate(100)
        # time.sleep(1)
        # car.servo_rotate.set_rotate(0)
        # time.sleep(2)
        # pass
        # car.mecanum_wheel_closed(0.0, 0.1, 0.0)
        # car.mecanum_wheel(0, 0.1, 0)
        # time.sleep(0.05)
    # car.pull_down()
    # arm  = ArmBase(ser)
    # arm.reset()
    # arm.horiz_moto.set(-20)
    # arm.set(-0.15, 0.1, 2)
    # arm.set_by_yourself()
    # vehicle = VehicleBase(ser)
    # vehicle.beep()
    # chasiss = MecanumBase(ser)
    # chasiss.encoders_motors.reset()
    # chasiss.run2(0.1, 0.1)
    # time.sleep(1)
    # last_time =time.time()
    # count = 0
    # chasiss.move_closed([0.25, -0.25, 0], 2.5)
    # chasiss.process_encoder()
    # while True:
    #     chasiss.mecanum_wheel(0.0, -0.3, 0.0)
    #     time.sleep(0.2)
    # time.sleep(5)
    # chasiss.close()
    # print("time cost:", time.time() - last_time)
    # print(count)

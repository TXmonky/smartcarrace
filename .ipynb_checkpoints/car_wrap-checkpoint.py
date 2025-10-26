#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import platform
import signal
from camera import Camera
import numpy as np
from vehicle import MecanumBase, SensorAi, SerialWrap, ArmBase, ScreenShow, Key4Btn, Infrared
from simple_pid import PID
import cv2, math
from task_func import MyTask
from infer_front import ClintInterface
from wbt_client_infer import CnnClient, TaskDetectClient, FrontDectectClient
from log_info import logger

def sellect_program(programs, order, win_order):
    dis_str = ''
    start_index = 0
    
    start_index = order - win_order
    for i, program in enumerate(programs):
        if i < start_index:
            continue
        now = str(program)
        if i == order:
            now = '>>> ' + now
        else:
            now = str(i+1) + '.' + now
        if len(now) >= 19:
            now = now[:19]
        else:
            now = now + '\n'
        dis_str += now
        if i-start_index == 4:
            break
    return dis_str

def kill_other_python():
    import psutil
    pid_me = os.getpid()
    # logger.info("my pid ", pid_me, type(pid_me))
    python_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'python' in proc.info['name'].lower() and len(proc.info['cmdline']) > 1 and len(proc.info['cmdline'][1]) < 30:
                    python_processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process['pid']) != pid_me:
            os.kill(int(process['pid']), signal.SIGKILL)
            time.sleep(0.3)
    # logger.info("    ")
            
def limit(value, value_range):
    return max(min(value, value_range), 0-value_range)

class LaneInterface():
    def __init__(self):
        y_out_limit = 0.7
        self.pid_y = PID(5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        # angle_speed = self.limit((angle) * k_angle, self.speed_limit_angler_z)
        # logger.info("before:---------------")
        # logger.info("angle:{} error_dy:{}".format(angle, dy))
        # angle_speed = 0-self.angle_pid(angle)
        # angle_speed = self.filter_angle.filter(angle_speed)
        # speed_dy = 0-self.y_pid(dy) + dy_offset
        # speed_dy = self.filter_y.filter(speed_dy)
        # speed_dy = self.limit((dy) * k_dy, self.speed_limit_y) 
        # speed_dy = 0

        return pid_y_out, pid_angle_out
    
class DetInterface():
    def __init__(self):
        y_out_limit = 0.7
        self.pid_y = PID(0.3, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        angle_out_limit = 1.5
        self.pid_angle = PID(2, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        # angle_speed = self.limit((angle) * k_angle, self.speed_limit_angler_z)
        # logger.info("before:---------------")
        # logger.info("angle:{} error_dy:{}".format(angle, dy))
        # angle_speed = 0-self.angle_pid(angle)
        # angle_speed = self.filter_angle.filter(angle_speed)
        # speed_dy = 0-self.y_pid(dy) + dy_offset
        # speed_dy = self.filter_y.filter(speed_dy)
        # speed_dy = self.limit((dy) * k_dy, self.speed_limit_y) 
        # speed_dy = 0

        return pid_y_out, pid_angle_out
    

class LocationPid():
    def __init__(self):
        y_out_limit = 0.3
        self.pid_y = PID(0.5, 0, 0)
        self.pid_y.setpoint = 0
        self.pid_y.output_limits = (-y_out_limit, y_out_limit)

        x_out_limit = 0.3
        self.pid_x = PID(0.5, 0, 0)
        self.pid_x.setpoint = 0
        self.pid_x.output_limits = (-x_out_limit, x_out_limit)
    
    def set_target(self, x, y):
        self.pid_y.setpoint = y
        self.pid_x.setpoint = x

    def get_out(self, error_x, error_y):
        pid_y_out = self.pid_y(error_y)
        pid_x_out = self.pid_x(error_x)
        return pid_x_out, pid_y_out


class MyCar(MecanumBase):
    STOP_PARAM = False
    def __init__(self):
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        # kill_other_python()
        # 关闭后台省电策略
        logger.info("my car init ok {}".format(time.time() - start_time))
        self.key = Key4Btn(2)
        self.task = MyTask()
        self.left_sensor = Infrared(1)
        self.right_sensor = Infrared(6)

        self.display = ScreenShow()
        # 初始化前后摄像头设置
        self.cap_front = Camera(1)
        self.cap_front.set_size(640, 480)
        self.cap_front.start()
        # 侧面摄像头
        self.cap_side = Camera(2)
        self.cap_side.set_size(640, 480)
        self.cap_side.start()

        self.crusie = CnnClient()
        self.front_det = FrontDectectClient()
        self.task_det = TaskDetectClient()

        # 速度上限设置 单位m/s
        self.speed_limit_x = 0.7
        self.speed_limit_y = 0.7
        self.speed_limit_angler = 3.0

        self.lane_pid = LaneInterface()
        self.det_pid = DetInterface()
        # 偏移系数设置，调整这个值可以修改小车在道路上的修正能力
        self.k_dy = 8.0
        self.k_angle = 3.2
        self.kx = 1.0
        self.dy_offset = 0.0

        self.pos_middle = [80, 160]
        # 相关临时变量设置
        self.last_error = 0
        self._stop_flag = False

        self._end_flag = False
        self.key_thread = threading.Thread(target=self.key_check)
        self.key_thread.setDaemon(True)
        self.key_thread.start()
        
        self.last_det = None

    # 延时函数
    def delay(self, time_hold):
        start_time = time.time()
        while True:
            if time.time() - start_time > time_hold:
                break
            if self._stop_flag:
                return
    
    def key_check(self):
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_btn()
                # print(key_val)
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)

    def image_walk_det(self, speed, dy_offset=0, k_dy=0, k_angle=0):
        image = self.cap_front.read()
        res_det = self.front_det(image)
        det_tmp = None
        if len(res_det) > 0:
            # print(res_det)
            for det in res_det:
                if det_tmp is None:
                    det_tmp = det
                if det_tmp[2] < det[2]:
                    det_tmp = det
        if det_tmp is not None:
            self.last_det = det_tmp
        # print(self.last_det)
        if self.last_det is None:
            dy = 0
            dangle = 0
            dis = 0
            id = None
        else:
            dy = self.last_det[3] + dy_offset
            dis = 1 - self.last_det[4]
            dangle = dy/dis
            id = self.last_det[0]
        # print(dy, dangle, dis)
        self.det_forward(speed, dy, dangle, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
        
        return dy, dangle, dis, id
    
    def det_forward(self, speed_x, error_y, error_angle, dy_offset=0.0, d_angle_offset=0.0, k_dy=None, k_angle=None):
        # 滤波
        # angle= self.filter_t.filter(speed_angle)
        # 把数据转为右手定则数据, 同时加上偏移量
        angle = (error_angle + d_angle_offset)
        dy = (error_y)
        # 根据对应的参数进行设置使用值
        if k_dy==None:
            k_dy = self.k_dy
        if k_angle==None:
            k_angle = self.k_angle
        
        speed_dy, angle_speed = self.det_pid.get_out(dy, angle)
        # print("out:", speed_dy, angle_speed)
        # 计算速度
        # angle_speed = angle*self.k_angle
        # speed_dy = dy * self.k_dy
        self.mecanum_wheel(speed_x, speed_dy, angle_speed)

    def walk_det_time(self, speed, time_hold, dy_offset=0, k_dy=0, k_angle=0):
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            self.image_walk_det(speed, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
            timeout = time.time()
            if timeout - start_time > time_hold:
                break
        self.mecanum_wheel(0, 0, 0)

    def walk_det_advance(self, speed, det_dis, dy_offset=0, k_dy=0, k_angle=0):
        while True:
            if self._stop_flag:
                return
            _x, _angle, _dis, id = self.image_walk_det(speed, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
            if _dis < det_dis and _dis != 0:
                break
        print("detect lane out...")
        self.mecanum_wheel(0, 0, 0)
        return id

    def image_walk_lane(self, speed, dy_offset=0, k_dy=0, k_angle=0):
        image = self.cap_front.read()
        res = self.crusie(image)
        dy = res[0]
        dangle = res[1]
        self.lane_forward(speed, dy, dangle, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)

    # 根据对应的便宜设置运动的速度
    def lane_forward(self, speed_x, error_y, error_angle, dy_offset=0.0, d_angle_offset=0.0, k_dy=None, k_angle=None):
        # 滤波
        # angle= self.filter_t.filter(speed_angle)
        # 把数据转为右手定则数据, 同时加上偏移量
        angle = 0-(error_angle + d_angle_offset)
        dy = (error_y + dy_offset)
        # 根据对应的参数进行设置使用值
        if k_dy==None:
            k_dy = self.k_dy
        if k_angle==None:
            k_angle = self.k_angle
        
        speed_dy, angle_speed = self.lane_pid.get_out(dy, angle)
        # 计算速度
        # angle_speed = angle*self.k_angle
        # speed_dy = dy * self.k_dy
        self.mecanum_wheel(speed_x, speed_dy, angle_speed)
        # logger.info("angle:{} error_dy:{}".format(angle_speed, speed_dy))

    # 按照给定速度沿着道路前进给定的时间
    def lane_time(self, speed, time_hold, dy_offset=0.0, k_dy=0, k_angle=0, stop=STOP_PARAM):
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            self.image_walk_lane(speed, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
            # self.lane_forward(speed, lane_info)
            timeout = time.time()
            if timeout - start_time > time_hold:
                break
        if stop:
            self.stop()

    def move_advance(self, sp, value_l=None, value_h=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
        _sensor_usr = self.right_sensor
        if sides != -1:
            _sensor_usr = self.left_sensor
            
        counter = 0
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0

        for i in range(times):
            stage = 0
            start_pos = self.get_odom()
            logger.info("pass{}".format(stage))
            while True:
                if self._stop_flag:
                    return
                now_pos = self.get_odom()
                pos_dis = np.array(now_pos) - np.array(start_pos)
                dis = math.sqrt(pos_dis[0]**2 + pos_dis[1]**2)
                if dis > dis_out:
                    print("dis out")
                    break
                # dy, dangle = self.get_pos()
                # self.lane_forward(speed, dy, dangle, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
                self.mecanum_wheel(sp[0], sp[1], sp[2])
                # dis = self.right_sensor.get()
                dis = _sensor_usr.get() / 1000      # mm转m
                # dis = self.control.get_tof(sensor_id)
                logger.info(dis)
                if stage == 1:
                    if value_l <= dis < value_h:
                        break
                elif dis < value_l or dis > value_h+1:
                    stage = 1
                    logger.info("pass{}".format(stage))
        if stop:
            self.stop()

    def calculation_dis(self, pos_dst, pos_src):
        return math.sqrt((pos_dst[0] - pos_src[0])**2 + (pos_dst[1] - pos_src[1])**2)
    
    def lane_task_location(self, speed, pt_tar=[0, -0.025, -0.23, 0.41, 0.54], dis_out=0.22, side=-1, kp=None):
        
        target_label = pt_tar[0]
        target_x = pt_tar[1]
        target_y = pt_tar[2]
        # 此处位置可以用宽度定位，摄像头角度影响会小一些
        target_w = pt_tar[3]
        target_h = pt_tar[4]
        # 设置侧面比例
        det_portation = side * -1
        target_x = target_x *det_portation
        target_y = target_y
    
        # dis_out = 0.15
        # 重置此时的相对位置
        self.set_pos_relative()
        # pos_start = self.get_odom()
        # cfg =
        import yaml
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        self.config_path = os.path.join(self.path_dir, "location_pid.yaml")
        while True:
            try:
                # 读取配置文件
                with open("location_pid.yml", 'r', encoding='utf-8') as f:
                    self.config = yaml.load(f, Loader=yaml.FullLoader)
                x_kp = self.config['pid_x']['kp']
                x_ki = self.config['pid_x']['ki']
                x_kd = self.config['pid_x']['kd']
                self.pid_x = PID(x_kp, x_ki, x_kd)

                y_kp = self.config['pid_y']['kp']
                y_ki = self.config['pid_y']['ki']
                y_kd = self.config['pid_y']['kd']
                pid_y = PID(y_kp, y_ki, y_kd)
                break
            except Exception as e:
                self.config = {
                    'pid_x': {
                        'kp': 0.5,
                        'ki': 0,
                        'kd': 0.005
                    },
                    'pid_y': {
                        'kp': 1.2,
                        'ki': 0,
                        'kd': 0.005
                    }
                }
                with open("location_pid.yml", 'w', encoding='utf-8') as f:
                    yaml.dump(self.config, f)

        self.pid_x.setpoint = target_x
        self.pid_x.output_limits = (-abs(speed), abs(speed))

        # pid_y = PID(1, 0, 0)
        pid_y.setpoint = target_w
        pid_y.output_limits = (-abs(speed), abs(speed))

        error_x = 100
        error_y = 100
        error_w = 100
        out_x = speed
        out_y = 0
        out_w = 0
        out_count = 0
        state_loc = False
        # logger.info(time.time())
        find_tar = False
        while True:
            if self._stop_flag:
                break
            _pos_x, _pos_y, _pos_omage = self.get_pos_relative() # 用来计算距离

            if abs(_pos_x) > dis_out or abs(_pos_y) > dis_out:
                if not find_tar:
                    logger.info("task location dis out")
                    break
            img_side = self.cap_side.read()
            det_ret = self.task_det(img_side)
            if len(det_ret) > 0:
                index = None
                dis_min = 2
                # 找到离最近的目标
                for i, det in enumerate(det_ret):
                    if det[1] == target_label:
                        # 找到了目标
                        find_tar = True
                        dis = math.sqrt(det[3]**2 + det[4]**2)
                        if dis < dis_min:
                            dis_min = dis
                            index = i
                
                if index is not None:
                    ret = det_ret[index]
                    # logger.info("task location".format(ret))
                        
                    if ret[1] == target_label:
                        # 获取位置
                        pos_x = ret[3]
                        pos_y = ret[4]
                        width = ret[5]

                        # 计算偏差
                        error_x = pos_x * det_portation - target_x
                        error_y = pos_y - target_y
                        error_w = width - target_w
                        # 转换方向
                        in_x = pos_x * det_portation
                        in_y = pos_y
                        in_w = width
                        if abs(in_x) > 0.2:
                            in_y = target_y
                            in_w = target_w

                        # 计算pid
                        out_x = self.pid_x(in_x)
                        # out_y = pid_y(in_y) * det_portation * -1
                        out_w = pid_y(in_w) * det_portation * -1

                        # logger.info("pos_x:{}, pos_y:{}".format(pos_x, pos_y))
                        # logger.info("out_x:{}, out_y:{}".format(out_x, out_y))
                # self.mecanum_wheel(out_x, out_y, 0)
                self.mecanum_wheel(out_x, out_w, 0)
            else:
                # self.mecanum_wheel(out_x, out_y, 0)
                self.mecanum_wheel(out_x, out_w, 0)
            # logger.info("error_x:{}, error_y:{}".format(error_x, error_y))
            # if abs(error_x) < 0.04 and abs(error_y) < 0.04:
            if abs(error_x) < 0.04 and abs(error_w) < 0.02:
                out_count += 1
                if out_count > 20:
                    state_loc = True
                    break
            else:
                out_count = 0
        self.mecanum_wheel(0, 0, 0)
        return state_loc
                
    def lane_dis(self, speed, dis_end, dy_offset=0.0, k_dy=0, k_angle=0, stop=STOP_PARAM):
        dis_start = self.get_dis_traveled()
        pid_dis = PID(5, 0, 0)
        # 设置结束位置
        pid_dis.setpoint = dis_end
        pid_dis.output_limits = (-abs(speed), abs(speed))
        count_flag = 0
        
        walk_dir = 1
        speed_out = speed
    
        if dis_end < dis_start:
            walk_dir = -1
            speed_out = 0 - speed
        while True:
            if self._stop_flag:
                return
            now_dis = self.get_dis_traveled()
            dis_error = now_dis - dis_end
            # logger.info("dis_error:{}, dis_start:{}".format(dis_error, dis_start))
            # logger.info("now:".format(now))
            if not stop:
                if dis_error*walk_dir > 0.01:
                    logger.info("now_dis:{}, dis_start:{}, dis_end:{}".format(now_dis, dis_start, dis_end))
                    break
            else:
                speed_out = pid_dis(now_dis)
                if abs(dis_error) < 0.01:
                    count_flag += 1
                    if count_flag > 20:
                        break
                else:
                    count_flag = 0
            self.image_walk_lane(speed_out, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
        if stop:
            self.stop()

    def lane_dis_offset(self, speed, dis_hold, dy_offset=0.0, k_dy=0, k_angle=0, stop=STOP_PARAM):
        dis_start = self.get_dis_traveled()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle, stop=stop)

    # 按照给定的速度前进到sides(0左1右)侧传感器的值在value_l~value_l之间，times是重复的次数
    def lane_advance(self, speed, value_l=None, value_h=None, times=1, sides=1, dis_offset = 0, dy_offset=0.0, k_dy=0, k_angle=0, stop=STOP_PARAM):
        _sensor_usr = self.right_sensor
        if sides != -1:
            _sensor_usr = self.left_sensor
        # if value_h != 0:
        counter = 0
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        for i in range(times):
            stage = 0
            logger.info("pass{}".format(stage))
            while True:
                if self._stop_flag:
                    return
                
                # dy, dangle = self.get_pos()
                # self.lane_forward(speed, dy, dangle, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
                self.image_walk_lane(speed, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle)
                # dis = self.right_sensor.get()
                dis_sensor = _sensor_usr.get() / 1000      # mm转m
                # dis = self.control.get_tof(sensor_id)
                # logger.info(dis)
                if stage == 1:
                    if value_l <= dis_sensor < value_h:
                        break
                elif dis_sensor < value_l or dis_sensor > value_h+1:
                    stage = 1
                    logger.info("pass{}".format(stage))
        dis_time_offset = dis_offset / speed
        if dis_time_offset > 0:
            # self.lane_forward(speed, lane_info, dy_offset, k_dy=k_dy,k_angle=k_angle)
            # self.delay(dis_time_offset) 
            self.lane_time(speed, dis_time_offset, dy_offset=dy_offset, k_dy=k_dy, k_angle=k_angle, stop=stop)
        if stop:
            self.stop()

    def lane_test(self):
        while True:
            if self._stop_flag:
                return
            self.image_walk_lane(0.3)

    def debug(self):
        # self.arm.arm_init()
        # self.set_xyz_relative(0, 100, 60, 0.5)
        while True:
            if self._stop_flag:
                return
            image = self.cap_front.read()
            res = self.crusie(image)
            det_front = self.front_det(image)
            error = res[0]
            angle = res[1]
            image = self.cap_side.read()
            det_task = self.task_det(image)
            logger.info("")
            logger.info("--------------")
            logger.info("error:{} angle{}".format(error, angle))
            logger.info("front:{}".format(det_front))
            logger.info("task:{}".format(det_task))
            logger.info("left:{} right:{}".format(self.left_sensor.get(),self.right_sensor.get()))
            self.delay(0.5)

    def walk_lane_test(self):
        while True:
            if self._stop_flag:
                return
            self.image_walk_lane(0.3)

    def close(self):
        self._stop_flag = False
        self._end_flag = True
        self.key_thread.join()
        self.cap_front.close()
        self.cap_side.close()
        # self.grap_cam.close()

    def manage(self, programs, order_index=0):
        
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5
        win_order = 0
        # 把programs的函数名转字符串
        logger.info(order_index)
        programs_str = [str(i.__name__) for i in programs]
        # logger.info(programs_str)
        dis_str = sellect_program(programs_str, order_index, win_order)
        self.display.show(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # self.button_all.event()
            btn = self.key.get_btn()
            # logger.info(btn)
            # button_num = car.button_all.clicked()
            
            if btn != 0:
                # logger.info(btn)

                if btn == 5:
                    # run_flag = True
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 2:
                        self.beep()
                        if order_index == 0:
                            order_index = len(programs)-1
                            win_order = win_num-1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 4:
                        self.beep()
                        if order_index == len(programs)-1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)

                    elif btn == 3:
                        
                        # 调用别的程序
                        dis_str = "\n{} running......\n".format(str(programs_str[order_index]))
                        self.display.show(dis_str)
                        self.beep()
                        self._stop_flag = False
                        programs[order_index]()
                        self._stop_flag = True
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.stop()
                        self.beep()

                        # 自动跳转下一条
                        if order_index == len(programs)-1:
                            order_index = 0
                            win_order = 0
                        else:
                            order_index += 1
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        self.display.show(dis_str)
                    logger.info(programs_str[order_index])
            else:
                self.delay(0.02)
                
            time.sleep(0.02)

        for i in range(2):
            self.beep()
            time.sleep(0.4)
        time.sleep(0.1)
        self.close()

if __name__ == "__main__":
    # kill_other_python()
    my_car = MyCar()
    # my_car.task.reset()
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.lane_time(0.3, 5)
    # my_car.debug()
    
    # my_car.debug()

            
    # my_car.task.pick_up_block()
    # my_car.task.put_down_self_block()
    # my_car.lane_time(0.2, 2)
    # my_car.lane_advance(0.3, dis_offset=0.01, value_h=500, sides=-1)
    # my_car.lane_task_location(0.3, 2)
    # my_car.task.pick_up_block()
    # my_car.close()
    # logger.info(time.time())
    # my_car.lane_task_location(0.3, 2)


    # my_car.debug()
    # programs = [func1, func2, func3, func4, func5, func6]
    # my_car.manage(programs)
    # import sys
    # test_ord = 0
    # if len(sys.argv) >= 2:
    #     test_ord = int(sys.argv[1])
    # logger.info("test:", test_ord)
    # car_test(test_ord)


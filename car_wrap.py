#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import threading
import os
import platform
import signal
import atexit
from camera import Camera
import numpy as np
from vehicle import MecanumBase, SensorAi, SerialWrap, ArmBase, ScreenShow, Key4Btn, Infrared, LedLight
from simple_pid import PID
import cv2, math
from task_func import MyTask
from infer_cs import ClintInterface, Bbox
from ernie_bot import ErnieBotWrap, ActionPrompt, HumAttrPrompt
from tools import CountRecord, get_yaml, IndexWrap
from vehicle import My_Beep
from log_info import logger
# order=4 ,win_order=0 order-win_order = start_order，start为执行的函数并且打印接下来执行的4个的函数
exit_value = False

# 全局MyCar实例，用于异常退出时清理
_car_instance = None

def cleanup_car_resources():
    """清理车辆资源"""
    global _car_instance
    if _car_instance is not None:
        try:
            logger.info("开始清理车辆资源...")
            _car_instance.emergency_cleanup()
            logger.info("车辆资源清理完成")
        except Exception as e:
            logger.error(f"清理车辆资源时出错: {e}")

def car_signal_handler(signum, frame):
    """车辆信号处理函数"""
    logger.info(f"接收到信号 {signum}，开始清理车辆资源...")
    cleanup_car_resources()
    os._exit(0)  # 强制退出

# 注册信号处理和退出清理
signal.signal(signal.SIGINT, car_signal_handler)   # Ctrl+C
signal.signal(signal.SIGTERM, car_signal_handler)  # 终止信号
atexit.register(cleanup_car_resources)             # 程序退出时清理

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
            # 出现异常的时候捕获 不存在的异常，权限不足的异常， 僵尸进程
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
    for process in python_processes:
        # logger.info(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
        # logger.info("this", process['pid'], type(process['pid']))
        if int(process['pid']) != pid_me:
            os.kill(int(process['pid']), signal.SIGKILL)
            time.sleep(0.3)
            
def limit(value, value_range):
    return max(min(value, value_range), 0-value_range)

# 两个pid集合成一个
class PidCal2():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out

class LanePidCal():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
        # y_out_limit = 0.7
        # self.pid_y = PID(5, 0, 0)
        # self.pid_y.setpoint = 0
        # self.pid_y.output_limits = (-y_out_limit, y_out_limit)
        print(cfg_pid_y)
        print(cfg_pid_angle)
        self.pid_y = PID(**cfg_pid_y)
        print(self.pid_y)

        angle_out_limit = 1.5
        self.pid_angle = PID(3, 0, 0)
        self.pid_angle.setpoint = 0
        self.pid_angle.output_limits = (-angle_out_limit, angle_out_limit)
    
    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
    
class DetPidCal():
    def __init__(self, cfg_pid_y=None, cfg_pid_angle=None):
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
        return pid_y_out, pid_angle_out
    

class LocatePidCal():
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
    STOP_PARAM = True
    def __init__(self):
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        # 获取自己文件所在的目录路径
        self.path_dir = os.path.abspath(os.path.dirname(__file__))
        
        # 注册到全局实例
        global _car_instance
        _car_instance = self
        
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 任务
        self.task = MyTask()
        # 显示
        self.display = ScreenShow()
        self.task.pick_up_block(arm_set=True)
        time.sleep(1)
        # 设置手臂准备位置
        self.task.pick_up_block(arm_set=True)
        # 获取配置
        cfg = get_yaml('/home/nvidia/Desktop/vehicle_wbt/config_car.yml')
        # 根据配置设置sensor
        self.sensor_init(cfg)

        self.car_pid_init(cfg)
        
        self.camera_init(cfg)
        # paddle推理初始化
        self.paddle_infer_init()
        # 文心一言分析初始化
        self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 按键线程结束标志
        self._end_flag = False
        self.thread_key = threading.Thread(target=self.key_thread_func)
        self.thread_key.setDaemon(True)
        self.thread_key.start()
        self.AI_FLAG = 0
    
    def sensor_init(self, cfg):
        cfg_sensor = cfg['io']
        # print(cfg_sensor)
        self.key = Key4Btn(cfg_sensor['key'])
        self.light = LedLight(cfg_sensor['light'])
        self.left_sensor = Infrared(cfg_sensor['left_sensor'])
        self.right_sensor = Infrared(cfg_sensor['right_sensor'])
    
    def car_pid_init(self, cfg):
        # lane_pid_cfg = cfg['lane_pid']
        # self.pid_y = PID(lane_pid_cfg['y'], 0, 0)
        # self.lane_pid = LanePidCal(**cfg['lane_pid'])
        # self.det_pid = DetPidCal(**cfg['det_pid'])
        self.lane_pid = PidCal2(**cfg['lane_pid'])
        self.det_pid = PidCal2(**cfg['det_pid'])

    def camera_init(self, cfg):
        # 初始化前后摄像头设置
        cfg_camera = cfg['camera']
        
        logger.info(f"开始初始化摄像头: front={cfg_camera['front']}, side={cfg_camera['side']}")
        
        # 使用线程方式同时初始化两个摄像头（基于测试结果）
        self.cap_front = Camera(cfg_camera['front'])
        self.cap_side = Camera(cfg_camera['side'])
        
        # 创建初始化结果字典
        init_results = {}
        
        def init_camera_thread(camera_obj, camera_name):
            """线程初始化摄像头"""
            try:
                # Camera对象在创建时已经自动初始化，只需检查是否成功
                success = camera_obj.cap is not None and camera_obj.cap.isOpened()
                init_results[camera_name] = success
                logger.info(f"摄像头{camera_name}初始化: {'成功' if success else '失败'}")
            except Exception as e:
                init_results[camera_name] = False
                logger.error(f"摄像头{camera_name}初始化异常: {e}")
        
        # 创建线程
        thread_front = threading.Thread(target=init_camera_thread, args=(self.cap_front, f"front({cfg_camera['front']})"))
        thread_side = threading.Thread(target=init_camera_thread, args=(self.cap_side, f"side({cfg_camera['side']})"))
        
        # 同时启动线程
        logger.info("使用线程方式同时初始化摄像头...")
        thread_front.start()
        thread_side.start()
        
        # 等待线程完成
        thread_front.join()
        thread_side.join()
        
        # 检查初始化结果
        front_success = init_results.get(f"front({cfg_camera['front']})", False)
        side_success = init_results.get(f"side({cfg_camera['side']})", False)
        
        if front_success and side_success:
            logger.info("所有摄像头初始化成功！")
        else:
            error_msg = f"摄像头初始化失败 - front: {'成功' if front_success else '失败'}, side: {'成功' if side_success else '失败'}"
            logger.error(error_msg)
            raise RuntimeError(error_msg)

    
    def paddle_infer_init(self):
        # 巡航
        self.crusie = ClintInterface('lane')
        # 前置左右方向识别,岔路的转向
        self.front_det = ClintInterface('front')
        # 任务识别
        self.task_det = ClintInterface('task')
        # 人体跟踪
        self.mot_hum = ClintInterface('mot')
        # 人体属性
        self.attr_hum = ClintInterface('humattr')
        # ocr识别
        self.ocr_rec = ClintInterface('ocr')
        # 识别为None
        self.last_det = ClintInterface('redstop')

    def ernie_bot_init(self):
        self.hum_analysis = ErnieBotWrap()
        self.hum_analysis.set_promt(str(HumAttrPrompt()))

        self.action_bot = ErnieBotWrap()
        self.action_bot.set_promt(str(ActionPrompt()))

    @staticmethod
    def get_cfg(path):
        from yaml import load, Loader
        # 把配置文件读取到内存
        with open(path, 'r') as stream:
            yaml_dict = load(stream, Loader=Loader)
        port_list = yaml_dict['port_io']
        # 转化为int
        for port in port_list:
            port['port'] = int(port['port'])
        print(yaml_dict)

    # 延时函数
    def delay(self, time_hold):
        start_time = time.time()
        while True:
            if self._stop_flag:
                return
            if time.time() - start_time > time_hold:
                break
    # 按键检测线程
    # def key_thread_func(self):
    #     while True:
    #         if not self._stop_flag:
    #             if self._end_flag:
    #                 return
    #             key_val = self.key.get_btn()
    #             # print(key_val)
    #             if key_val == 3:
    #                 self._stop_flag = True
    #                 print(1111111111111111111111111111111111)
    #             time.sleep(0.2)        


    # 按键检测线程，短按按键3就会首先失效这个函数（启动一个任务之后就只能等完成）,然后只有通过close或者长按按键1才会真正正常结束整个程序
    def key_thread_func(self):
        count = 0
        while True:
            if not self._stop_flag:
                if self._end_flag:
                    return
                key_val = self.key.get_btn()
                # print(key_val)
                if key_val == 3:
                    self._stop_flag = True
                time.sleep(0.2)

            key_val = self.key.get_key()
            if key_val == 1 :
                count += 1
            if count == 2:
                global exit_value
                exit_value = True
                # return
                print('停车！！！！！！！！！！！！！')
            time.sleep(0.2)

    

    
    
    # 根据某个值获取列表中匹配的结果
    @staticmethod
    def get_list_by_val(list, index, val):
        for det in list:
            if det[index] == val:
                return det
        return None

    # 计算速度并且使用run4来运行
    def move_base(self, sp, end_fuction, stop=STOP_PARAM):
        self.mecanum_wheel(sp[0], sp[1], sp[2])
        while True:
            if exit_value:
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            if self._stop_flag:
                return
            if end_fuction():
                break
            self.mecanum_wheel(sp[0], sp[1], sp[2])
        self.mecanum_wheel(0, 0, 0)


    #  高级移动，按着给定速度进行移动，直到满足条件并且停止，side选择使用哪一边的红外传感器
    def move_advance(self, sp, value_h=None, value_l=None, times=1, sides=1, dis_out=0.2, stop=STOP_PARAM):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        if value_h is None:
            value_h = 1200
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.get() / 1000
            print("val:", val_sensor)
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False
            
        # 直到 val在一定范围才会退出
        for i in range(times):
            self.move_base(sp, end_fuction, stop=False)
        if stop:
            self.stop()
    
    
    def do_action_list(self, acitons):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        def move_func(x, y, angle):
            if angle>math.pi or angle<-math.pi:
                angle = angle*math.pi/180
            print('x,y,::',x,y,angle)
            self.set_pos_offset([x, y, angle],error_need = True)

        def light_func(time_dur=0.2,count=1,interval=0.3):
            if interval==0:
                interval += 0.2
            while count>0:
                a = time.time()
                for i in range(1, 5):
                    self.light.set_light(i, 255, 255, 255)
                while time.time() -a <time_dur:
                    pass
                if interval!=0:
                    for i in range(1, 5):
                        self.light.set_light(i, 0, 0, 0)
                    time.sleep(interval)

                count -= 1

        def beep_func(time_dur=0.2,count=1,interval=0.2):
            if interval==0:
                interval += 0.2
            beep = My_Beep()
            beep.set(time_dur,count,interval)

        def wait_fun(time_dur=0):
            now = time.time()
            while time.time() - now < time_dur:
                pass
        action_map = {'move': move_func, 'light': light_func, 'beep': beep_func,'wait':wait_fun}
        for action in acitons:
            func = action_map[action['func']]
            # 删除func, 剩下的都是params
            action.pop('func')
            # 执行
            func(**action)

    # 计算两个坐标的距离
    def calculation_dis(self, pos_dst, pos_src):
        return math.sqrt((pos_dst[0] - pos_src[0])**2 + (pos_dst[1] - pos_src[1])**2)
    
    # 侧面摄像头进行位置定位，任务定位
    # [0, 1, 'pedestrian', 0, -0.02, 0.4, 0.22, 0.82]   -0.15, -0.48, 0.24, 0.82
    def lane_det_location(self, speed, pt_tar=[0, 1, 'pedestrian',  0, -0.15, -0.48, 0.24, 0.82], \
                          dis_out=0.22, side=1, det='task',see = 0,aim =0,get_ball = 0,common =0,camp=0,yingdi=0):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        infer = self.task_det
        if det!="task":
            infer = self.mot_hum
                    # 用于相同记录结果的计数类
            x_count = CountRecord(2)
            y_count = CountRecord(2)
            w_count = CountRecord(2)

        if det == "redstop":
            infer = self.last_det
            # 用于相同记录结果的计数类
            x_count = CountRecord(2)
            y_count = CountRecord(2)
            w_count = CountRecord(2)

        else :
            x_count = CountRecord(5)
            y_count = CountRecord(5)
            w_count = CountRecord(5)

        if(pt_tar[2]=="blue_ball"):
            pid_x = PID(0.5, 0, 0.02, setpoint=0, output_limits=(-speed/2, speed/2))
        else:
            pid_x = PID(0.5, 0, 0.02, setpoint=0, output_limits=(-speed, speed))
        pid_y = PID(1.3, 0, 0.01, setpoint=0, output_limits=(-0.15, 0.15))
        pid_w = PID(1.0, 0, 0.02, setpoint=0, output_limits=(-0.15, 0.15))

        # # 用于相同记录结果的计数类
        # x_count = CountRecord(5)
        # y_count = CountRecord(5)
        # w_count = CountRecord(5)
        
        out_x = speed
        out_y = 0
        # 坐标位置error转换相对位置
        error_adjust = np.array([-1, 1, 1, 1])
        if side == -1:
            error_adjust = np.array([1, -1, -1, -1])
        
        # 此时设置相对初始位置,固定进入任务的self.pos_start
        self.set_pos_relative()
        find_tar = False
        # 类别, id, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
        flag_location = False
        now = time.time()
        while True:
            if time.time() -now >15:
                print('超时退出！')
                if yingdi==1:
                    return False
                break
            if exit_value:
                self.close()
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            if self._stop_flag:
                return
            _pos_x, _pos_y, _pos_omage = self.get_pos_relative() # 用来计算距离
            # 保证任务相较于进入任务时不会偏离太多，否则提前结束任务
            if (abs(_pos_x) > dis_out or abs(_pos_y) > dis_out) and common == 0:
                if not find_tar:
                    logger.info("task location dis out!!!!!!!!!!!!!")
                    break
            elif (abs(_pos_x) > 0.28 or abs(_pos_y) > 0.28) and common == 1:
                if not find_tar:
                    logger.info("!!!!!!!!!!!!!!!!!!!task location dis out!!!!!!!!!!!!!")
                    break
            if camp == 1:
                if (abs(_pos_x) > 0.4 or abs(_pos_y) > 0.2) :
                    print('营地提前退出')
                    if yingdi==1:
                        return False
                    
                    break
                
            img_side = self.cap_side.read()
            dets_ret = infer(img_side)

            # if dets_ret is not None:
            #     for i in range(len(dets_ret)):
            #         det_rect_normalise = dets_ret[i][4:]
            #         rect = Bbox(box=det_rect_normalise, size=img_side.shape[:2][::-1]).get_rect()
            #         print('dets_ret',dets_ret)
            #         cv2.rectangle(img_side, rect[:2], rect[2:], (0, 255, 0), 2)
            # cv2.imshow("side", img_side)
            # cv2.waitKey(1)
            
            # 进行排序，此处排列按照由近及远的顺序

            # if(pt_tar[2]=="blue_ball"):
            #     dets_ret.sort(key=lambda x: (x[4]-0)**2)
            # else:
            list1 = []
            if get_ball ==2:
                if dets_ret is not None:
                    for i in range(len(dets_ret)):
                        det_rect_normalise = dets_ret[i][4:]
                        id = dets_ret[i][2]
                        rect = Bbox(box=det_rect_normalise, size=img_side.shape[:2][::-1]).get_rect()
                        list1.append([id,rect[0],rect[1],rect[2],rect[3]])  
                    list1.sort(key=lambda x: (x[1]-tar_bbox[0])**2 + (x[2]-tar_bbox[1])**2)       
                    for det in list1:
                        if det[0] == "blue_ball":
                            return det[1],det[2],det[3],det[4]
                    return None                  
                else:
                    return None
            if see !=0:
                if see == 1 :
                    if dets_ret is not None:
                        for i in range(len(dets_ret)):
                            det_rect_normalise = dets_ret[i][4:]
                            id = dets_ret[i][2]
                            rect = Bbox(box=det_rect_normalise, size=img_side.shape[:2][::-1]).get_rect()
                            list1.append([id,rect[0],rect[1]])
                        list1.sort(key=lambda x: (x[1]-208)**2+(x[2]-0)**2)
                        for det in list1:
                            if det[0] == "cylinder1":
                                return det[0]
                            if det[0] == "cylinder2":
                                return det[0]
                            if det[0] == "cylinder3":
                                return det[0]
                        det = None
                if see == 2 :
                    if dets_ret is not None:
                        for i in range(len(dets_ret)):
                            det_rect_normalise = dets_ret[i][4:]
                            id = dets_ret[i][2]
                            rect = Bbox(box=det_rect_normalise, size=img_side.shape[:2][::-1]).get_rect()
                            list1.append([id,rect[0],rect[1]])
                        list1.sort(key=lambda x: (x[1]-208)**2+(x[2]-0)**2)#reverse=True
                        for det in list1:
                            if det[0] == "cylinder1":
                                self.mecanum_wheel(0, 0, 0)
                                return det[0]
                            if det[0] == "cylinder2":
                                self.mecanum_wheel(0, 0, 0)
                                return det[0]
                            if det[0] == "cylinder3":
                                self.mecanum_wheel(0, 0, 0)
                                return det[0]
                        det = None
                    else:
                        det = None
                det = None
            else:
                dets_ret.sort(key=lambda x: (x[4]-tar_bbox[0])**2 + (x[5]-tar_bbox[1])**2)
                det = self.get_list_by_val(dets_ret, 2, tar_label)
            # print(dets_ret)
            # 找到最近对应的类别，类别存在第一个位置


            # 如果没有，就重新获取
            if det is not None:
                find_tar = True
                # 结果分解
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]

                """
                获取坐标
                """
                if get_ball == 1:
                    det_rect_normalise = det[4:]
                    rect = Bbox(box=det_rect_normalise, size=img_side.shape[:2][::-1]).get_rect()
                
                # print(det_bbox)
                # 计算偏差, 并进行偏差转换为输入pid的输入值
                bbox_error = ((np.array(det_bbox) - np.array(tar_bbox)) * error_adjust).tolist()
                # 离得远时. ywh值进行滤波为0，最终仅使用了w的值
                if abs(bbox_error[0]) > 0.1:
                    bbox_error[1] = 0
                    bbox_error[2] = 0
                    bbox_error[3] = 0
                out_x = pid_x(bbox_error[0])
                # out_y = pid_y(bbox_error[1])
                out_y = pid_w(bbox_error[2])
                # print(bbox_error)
                # 检测偏差值连续小于阈值时，跳出循环
                flag_x = x_count(abs(bbox_error[0]) < 0.02)
                flag_y = y_count(abs(bbox_error[2]) < 0.025)
                if aim==0:
                    if det == "redstop": 
                        if time.time() - now >15:
                            # pass
                            break
           
                    if flag_x:
                        out_x = 0
                    if flag_y:
                        out_y = 0
                    if flag_x and flag_y:
                        print("location ok")
                        flag_location = True
                        break
                elif aim ==1:
                     if flag_x:
                        out_x = 0   
                        flag_location = True                
                        break
                elif aim ==2:
                     if flag_y:
                        out_y = 0 
                        flag_location = True                  
                        break               
                
                # print("error_x:{:.2}, error_y:{:.2}, out_x:{:.2}, out_y:{:2}".format(bbox_error[0], bbox_error[2], out_x, out_y))
            else:
                x_count(False)
                y_count(False)
            # self.mecanum_wheel(0, 0, 0)  
            if aim==0: 
                # if det is None:
                #     def aa():
                #         return True
                #     self.lane_base(speed,end_fuction = aa,out_flag =1)
                # else:
                #     # print("outx= ",out_x)
                if see != 0:
                    self.mecanum_wheel(out_x, 0, 0)
                else:
                    self.mecanum_wheel(out_x, out_y, 0)
                    # self.mecanum_wheel(0, 0, 0)  
            elif aim==1:
                self.mecanum_wheel(out_x,0, 0)
            elif aim ==2:
                self.mecanum_wheel(0, out_y, 0)
            # print(out_x,out_y)
        # 停止
        self.mecanum_wheel(0, 0, 0)
        if yingdi == 1:
            return True
        if get_ball==1:
            if find_tar:
                return flag_location,rect
            else:
                return flag_location,None
        return flag_location,None
            
    # 巡航前进
    def lane_base(self, speed, end_fuction, stop=STOP_PARAM, y_offest = 0,angle_offest = 0,y_go = [0,0],out_flag = 0):
        start_time = time.time()
        while True:
            if exit_value:
                self.close()
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            if self._stop_flag:
                return
            image = self.cap_front.read()
            error_y, error_angle = self.crusie(image)
  
                
            y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
            y_go_flag = 0
            y_go_mode = 0
            if y_offest!=0:
                print("angle_speed = ",angle_speed ,"y_speed = ",y_speed)
                if  abs(angle_speed)<0.08:#abs(y_speed) < 0.0004*3 and
                    y_offest-=1
                else :
                    y_offest=10
                if y_offest ==0:
                    break
            if angle_offest!=0:
                temp_angle = math.pi - self.odom_theta
                if temp_angle>math.pi:
                    temp_angle-=2*math.pi
                if abs(temp_angle)<math.pi/36:
                    angle_offest-=1
                else:
                    angle_offest=10
                if angle_offest==0:
                    break
            
            if y_go[0]!=0 and y_go[1]!=0:
                y_go_flag+=1
                if y_go_flag> y_go[1]:
                  y_go_flag = 0
                  if y_go_mode==0:
                    y_go_mode = 1
                  elif y_go_mode == 1:
                    y_go_mode = 0
                if y_go_mode ==0:
                    y_speed+=y_go[0]
                    angle_speed = 0
            #    print(y_speed,temp)

            # angle_speed = angle_speed*1.1
            # speed_dy, angle_speed = process(image)
            # cv2.imshow('img',image)
            # if cv2.waitKey(1) == ord('q'):
            #     break
            # print(error_y,error_angle)
            self.mecanum_wheel(speed, y_speed, angle_speed)
            #print(speed)
            if end_fuction():
                #print("break")
                break

        if stop:
            if out_flag:
                pass
            else:
                self.stop()

    # 基于检测到的位置进行定位移动,岔路检测
    def lane_det_base(self, speed, end_fuction, stop=STOP_PARAM):

        y_speed = 0
        angle_speed = 0
        while True:        
            if exit_value:
                self.close()
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            # 此处检测简单不需要排序
            # dets_ret.sort(key=lambda x: x[4]**2 + (x[5])**2)
            if len(dets_ret)>0:
                det = dets_ret[0]
                det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
                error_y = det_bbox[0]
                dis_x = 1 - det_bbox[1]
                print(dis_x)
            
                if end_fuction(dis_x):
                    break
                error_angle = error_y/dis_x
                y_speed, angle_speed = self.det_pid.get_out(error_y, error_angle)
            self.mecanum_wheel(speed, y_speed, angle_speed)
            # self.mecanum_wheel(0, 0, 0)
            if end_fuction(0):
                break
        if stop:
            self.stop()
    
    #根据时间来退出 检测岔路
    def lane_det_time(self, speed, time_dur, stop=STOP_PARAM):
        time_end = time.time() + time_dur
        end_fuction = lambda x: time.time() > time_end
        self.lane_det_base(speed, end_fuction, stop=stop)
   
    # 给定速度移动一定的距离,检测岔路
    def lane_det_dis2pt(self, speed, dis_end, stop=STOP_PARAM):
        # lambda定义endfunction
        end_fuction = lambda x: x < dis_end and x != 0
        self.lane_det_base(speed, end_fuction, stop=stop)
    
    # 根据时间判断是否停止
    def lane_time(self, speed, time_dur, stop=STOP_PARAM):
        time_end = time.time() + time_dur
        end_fuction = lambda: time.time() > time_end
        self.lane_base(speed, end_fuction, stop=stop)
    
    # 巡航一段路程
    def lane_dis(self, speed, dis_end, stop=STOP_PARAM,y_go = [0,0]):
        # lambda重新endfunction
        end_fuction = lambda: self.get_dis_traveled() > dis_end
        self.lane_base(speed, end_fuction, stop=stop,y_go = y_go)
   
    # 从当前位置再移动dis_hold距离
    def lane_dis_offset(self, speed, dis_hold, stop=STOP_PARAM,y_go = [0,0]):
        dis_start = self.get_dis_traveled()
        dis_stop = dis_start + dis_hold
        self.lane_dis(speed, dis_stop, stop=stop,y_go = y_go)

    # 按着给定速度巡航进行移动，直到满足条件，side选择使用哪一边的红外传感器
    def lane_sensor(self, speed, value_h=None, value_l=None, dis_offset=0.0, times=1, sides=1, stop=STOP_PARAM,y_offest = 0,angle_offest = 0,y_go = [0,0],y_use_offest = 0):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        if value_h is None:
            value_h = 1200
        # else:
        #     value_h -= 0.10
        if value_l is None:
            value_l = 0
        _sensor_usr = self.left_sensor
        if sides == -1:
            _sensor_usr = self.right_sensor
        # 用于检测开始过渡部分的标记
        flag_start = False
        def end_fuction():
            nonlocal flag_start
            val_sensor = _sensor_usr.get() / 1000
            #print("val_sensor = ",val_sensor)
            # print("val:", val_sensor)
            if val_sensor < value_h and val_sensor > value_l:
                return flag_start
            else:
                flag_start = True
                return False
        if y_use_offest==0:
            for i in range(times):
                self.lane_base(speed, end_fuction, stop=False,y_offest = y_offest,angle_offest = angle_offest,y_go = y_go)
        else:
            self.lane_time(speed,time_dur = 0.2)
            self.set_pos_offset([0,0.01*y_use_offest,0])
        # 根据需要是否巡航
        self.lane_dis_offset(speed, dis_offset, stop=stop)
    
    # 检测应该左移还是右移
    def get_card_side(self):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        # 检测卡片左右指示
        count_side = CountRecord(3)
        while True:
            if exit_value:
                self.close()
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            if self._stop_flag:
                return
            image = self.cap_front.read()
            dets_ret = self.front_det(image)
            if len(dets_ret) == 0:
                count_side(-1)
                continue
            det = dets_ret[0]
            det_cls, det_id, det_label, det_score, det_bbox = det[0], det[1], det[2], det[3], det[4:]
            # 联系检测超过3次
            print("det_label = ",det_label)
            if count_side(det_label):
                if det_label == 'turn_right':
                    return -1
                elif det_label == 'turn_left':
                    return 1
                else:
                    print(f"检测到非转向标签: {det_label}，默认返回左转")
                    return 1  # 默认左转
                
    # 获取人物属性和人体追踪
    def get_hum_attr(self, pt_tar, show=False):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        # 类别, id, 置信度, 归一化bbox[x_c, y_c, w, h]
        tar_cls, tar_id, tar_label, tar_score, tar_bbox = pt_tar[0], pt_tar[1], pt_tar[2], pt_tar[3], pt_tar[4:]
        tar_count = CountRecord(4)
        while True:
            if exit_value:
                self.close()
                raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            if self._stop_flag:
                return
            image = self.cap_side.read()
            dets_ret = self.mot_hum(image)
            # 排序, 按照距离中心的远近,选最近
            dets_ret.sort(key=lambda x: (x[4]-tar_bbox[0])**2 + (x[5]-tar_bbox[1])**2)
            if len(dets_ret) != 0:
                det_rect_normalise = dets_ret[0][4:]
                # print(det_rect_normalise)
                rect = Bbox(box=det_rect_normalise, size=image.shape[:2][::-1]).get_rect()
                # print(rect)
                if show:
                    cv2.rectangle(image, rect[:2], rect[2:], (0, 255, 0), 2)
                image_hum = image[rect[1]:rect[3], rect[0]:rect[2]]
                # cv2.imshow("hum", image_hum)
                # cv2.waitKey(1)
                # image_hum = image[int(det_rect[1]):int(det_rect[3]), int(det_rect[0]):int(det_rect[2])]
                res = self.attr_hum(image_hum)
                print(res)
                if tar_count(res):
                    # print(res)
                    return res
            if show:
                cv2.imshow("hum", image)
                cv2.waitKey(1)
            # print(res)
            # image_hum
            # logger.info(res)
    
    # 返回相应的属性
    def compare_humattr(self, crimall_attr, hum_attr):
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        if crimall_attr is None:
            return False
        for key, val in crimall_attr.items():
            if key not in hum_attr:
                return False
            if type(val) is bool:
                if val != hum_attr[key]:
                    return False
            elif val.lower() != hum_attr[key].lower():
                return False
        return True
    
    # 文字检测，设置感兴趣区域
    def get_ocr(self):
        # 简单滤波,三次检测到相同的值，认为稳定并返回
        # if exit_value:
        #     raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
        # text_count = CountRecord(1)
        # while True:
        if exit_value:
            self.close()
            raise Exception("我的变量为True，因此我将抛出一个异常并结束程序。")
            
        if self._stop_flag:
            return
        img = self.cap_side.read()
        text = self.ocr_rec(img[180:, 200:440])
        print(text)
        # if text_count(text):
        return text
        
    # 获取文心一言的回答:人物属性判断
    def yiyan_get_humattr(self, text):
        return self.hum_analysis.get_res_json(text)
    # 获得文心一言的回答:动作判断
    def yiyan_get_actions(self, text):
        return self.action_bot.get_res_json(text)
    
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
            det_hum = self.mot_hum(image)
            
            logger.info("")
            logger.info("--------------")
            logger.info("error:{} angle{}".format(error, angle))
            logger.info("front:{}".format(det_front))
            logger.info("task:{}".format(det_task))
            logger.info("hum_det:{}".format(det_hum))
            logger.info("left:{} right:{}".format(self.left_sensor.get(),self.right_sensor.get()))
            self.delay(0.5)

    def walk_lane_test(self):
        end_function = lambda: True
        self.lane_base(0.3, end_function, stop=self.STOP_PARAM)

    def close(self):
        """确保完全释放所有资源"""
        try:
            self._stop_flag = False
            self._end_flag = True
            
            # 等待按键线程结束
            if hasattr(self, 'thread_key') and self.thread_key.is_alive():
                self.thread_key.join(timeout=1.0)
            
            # 释放摄像头
            if hasattr(self, 'cap_front'):
                self.cap_front.close()
            if hasattr(self, 'cap_side'):
                self.cap_side.close()
                
            logger.info("车辆资源已释放")
        except Exception as e:
            logger.error("释放车辆资源时出错: {}".format(e))

    def emergency_cleanup(self):
        """紧急清理资源，用于异常退出"""
        try:
            logger.info("执行紧急资源清理...")
            self._stop_flag = True
            self._end_flag = True
            
            # 强制释放摄像头
            if hasattr(self, 'cap_front') and self.cap_front is not None:
                self.cap_front.force_release()
            if hasattr(self, 'cap_side') and self.cap_side is not None:
                self.cap_side.force_release()
                
            logger.info("紧急资源清理完成")
        except Exception as e:
            logger.error("紧急清理时出错: {}".format(e))

    def __del__(self):
        """析构函数，确保异常退出时也能释放资源"""
        try:
            # 只做最基本的资源释放，避免复杂操作
            if hasattr(self, 'cap_front') and self.cap_front is not None:
                self.cap_front.cap.release() if hasattr(self.cap_front, 'cap') else None
            if hasattr(self, 'cap_side') and self.cap_side is not None:
                self.cap_side.cap.release() if hasattr(self.cap_side, 'cap') else None
        except:
            # 静默处理异常，避免析构函数中的异常导致程序崩溃
            pass

    def manage(self, programs_list:list, order_index=0):

        # 导入函数列表
        def all_task():
            for func in programs_list:
                func()
        
        def lane_test():
            self.lane_dis_offset(0.3, 3)
        
        programs_suffix = [all_task, lane_test, self.task.arm.reset, self.debug]
        # 不改变传进函数的列表
        programs = programs_list.copy()
        
        programs.extend(programs_suffix)
        # print(programs)
        # 选中的python脚本序号
        # 当前选中的序号
        win_num = 5
        win_order = 0
        logger.info(order_index)
        # 把programs的函数名转字符串
        programs_str = [str(i.__name__) for i in programs]
        logger.info(programs_str)
        dis_str = sellect_program(programs_str, order_index, win_order)
        # self.display.show(dis_str)
        print(dis_str)

        self.stop()
        run_flag = False
        stop_flag = False
        stop_count = 0
        while True:
            # self.button_all.event()
            btn = self.key.get_btn()
            # 短按1=1,2=2,3=3,4=4
            # 长按1=5,2=6,3=7,4=8
            # logger.info(btn)
            # button_num = car.button_all.clicked()
            
            if btn != 0:
                # logger.info(btn)
                # 长按1按键，退出
                if btn == 5 or btn==1:
                    # run_flag = True
                    self.beep()
                    self._stop_flag = True
                    self._end_flag = True
                    break
                else:
                    if btn == 2:
                        # 序号减1, order_index = 4 win_order=0 win_num=5
                        self.beep()
                        # 为零则从最后一个开始
                        if order_index == 0:
                            order_index = len(programs)-1
                            win_order = win_num-1
                        else:
                            order_index -= 1
                            if win_order > 0:
                                win_order -= 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        # self.display.show(dis_str)

                    elif btn == 4:
                        self.beep()
                        # 序号加1 超过就从零开始
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
                        # self.display.show(dis_str)

                    elif btn == 3:
                        # 确定执行
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
                            # 设置win_num最大值为列表最大值
                            if len(programs) < win_num:
                                win_num = len(programs)
                            if win_order != win_num-1:
                                win_order += 1
                        # res = sllect_program(programs, num)
                        dis_str = sellect_program(programs_str, order_index, win_order)
                        # self.display.show(dis_str)

                    logger.info(programs_str[order_index])

            # 还没有按下
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
    # def aa():
    #     return False
    # my_car.lane_base(0.3,aa)
    #my_car.lane_det_location(0.2, [2, 0, "cylinder2", 0, -0.1105, 0.1923, 0.596, 0.875], side=-1)
    
    my_car.lane_dis_offset(0.15, 20.5, stop=False)
    #my_car.lane_sensor(0.3, 0.5)

    #机械爪测试
    # print("测试机械爪张开...")
    # my_car.task.arm.set_grap_angle(0)
    # time.sleep(2)
    
    # print("测试机械爪闭合...")
    # my_car.task.arm.set_grap_angle(90)
    # time.sleep(2)
    
    # print("测试机械臂移动...")
    # my_car.task.arm.set(0, 0.1, speed=[0.2, 0.2])
    # time.sleep(2)
    # my_car.debug()

    # text = "犯人没有带着眼镜，穿着短袖"
    # criminal_attr = my_car.hum_analysis.get_res_json(text)
    # print(criminal_attr)
    # my_car.task.reset()
    # pt_tar = my_car.task.punish_crimall(arm_set=True)
    # hum_attr = my_car.get_hum_attr(pt_tar)
    # print(hum_attr)
    # res_bool = my_car.compare_humattr(criminal_attr, hum_attr)
    # print(res_bool)
    # pt_tar = [0, 1, 'pedestrian',  0, 0.02, 0.4, 0.22, 0.82]
    # for i in range(4):
    #     my_car.set_pos_offset([0.07, 0, 0])
    #     my_car.lane_det_location(0.1, pt_tar, det="mot", side=-1)
    # my_car.close()
    # text = my_car.get_ocr()
    # print(text)
    # pt_tar = my_car.task.pick_up_ball(arm_set=True)
    # my_car.lane_det_location(0.1, pt_tar)
    
    my_car.close()
    # my_car.debug()
    # while True:
    #     text = my_car.get_ocr()
    #     print(text)

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

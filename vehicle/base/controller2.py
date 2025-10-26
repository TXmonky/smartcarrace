#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 开始编码格式和运行环境选择

import os
from queue import Queue
from serial.tools import list_ports
from threading import Lock, Thread
from multiprocessing import Process 
import serial
import time
import struct
import io
import numpy as np
import sys
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
# 添加上本地目录
sys.path.append(os.path.abspath(os.path.dirname(__file__))) 
from pydownload import Scratch_Download_MC602P
import Jetson.GPIO as GPIO


# 导入自定义log模块
# print(e)
try:
    from log_info import logger
except Exception as e:
    import logging
    logger = logging.getLogger("this")
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.DEBUG)
    logger.info("set this logger")


dev_map = { 1:["Motors", '<bbbbbbbb' ],
            2:["Motor", '<bbbb'],
            3:["Motor2", '<bb']
            }

class StructData():
    def __init__(self, format=None) -> None:
        if format is None:
            format=''
        self.format = '<b'+ format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1
    # 设置数据格式
    def set_fortmat(self, format):
        self.format = '<b'+ format
        self.size = struct.calcsize(self.format)
        self.len = len(self.format)-1
        
    def __sizeof__(self) -> int:
        return self.size
    # 解包收到的数据
    def unpack_data(self, data, index_start):
        try:
            s = index_start
            e = index_start + self.size
            
            # print(data[s:e])
            re_list = list(struct.unpack(self.format, data[s:e]))
        except Exception as e:
            pass
            return []
        return re_list
    # 发送数据
    def pack_data(self, data):
        bytes_t = struct.pack(self.format, *data)
        return bytes_t

    # 定义len函数的定义
    def __len__(self):
        return self.len



class SerialBytes:
    def __init__(self) -> None:

        self.head_cmd = b"\x77\x68"
        self.head1 = b'\x77'
        self.head2 = b'\x68'
        self.rear = b'\x0a'

        self.cmd_arr = bytearray(256)
        self.cmd_arr[0] = 0x77
        self.cmd_arr[1] = 0x68

    @staticmethod
    def check_sum(data, lenght):
        i = 2
        sum = 0
        while i< lenght-1:
            sum += data[i]
            i += 1
        sum = 255 - (sum % 256)
        return sum

    def set_bytes_start(self):
        self.index_now = 3

    def set_bytes(self, bytes_info=None):
        if bytes_info is not None:
            # for 迭代循环，带有数字标号的
            count = 1
            if isinstance(bytes_info, int):
                self.cmd_arr[5] = bytes_info
            else:
                for index, byte in enumerate(bytearray(bytes_info)):
                    self.cmd_arr[index+self.index_now] = byte
                    count = index
            # 标号位置根据数据变化
            self.index_now += count

    # 补充完成
    def get_bytes_whole(self):
        self.cmd_arr[2] = self.index_now + 1
        self.cmd_arr[self.index_now] = 0x0A
        return bytes(self.cmd_arr[:self.index_now+1])

    def get_bytes_frame(self, bytes_tmp):
        bytes_len = (len(bytes_tmp) + 4).to_bytes(1, 'big')
        return self.head_cmd + bytes_len + bytes_tmp + self.rear

    def __repr__(self) -> str:
        return str(bytes(self.cmd_arr[:self.index_now+1]))


class SerialWrap(serial.Serial):
    def __init__(self):
        super(SerialWrap, self).__init__()
        # 根据平台判断系统
        # import platform
        # if platform.system() == 'Windows':
        #     self.port = "COM3"
        # else:
        #     self.port = "/dev/ttyUSB0"
        self.port = None
        self.last_port = self.port
        # self.baudrate = 115200
        self.baudrate = 1000000
        self.bytesize = serial.EIGHTBITS
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.timeout = 0.1
        self.xonxoff = False
        self.rtscts = False
        self.dsrdtr = False

        self.rts = False
        self.dtr = False

        self.connect_flag = False
        
        self.lock = Lock()

        self.protocol = SerialBytes()
        while True:
            if self.ping_port():
                break

            result, msg = Scratch_Download_MC602P("RunA", isrun=True)
            if result:
                continue
            logger.info("no controller!")
            # logger.critical("未接控制器或者控制器没有开机,或者程序运行错误!")
            while True:
                time.sleep(1)

    def set_port(self, port):
        if self.connect_flag:
            self.close()
            self.connect_flag = False
        self.port = port
        # print(self.port)
        
    def open(self):
        try:
            if self.port is None:
                return False
            self.connect_flag = True
            super(SerialWrap, self).open()
            return True
        except Exception as e:
            # print(e)
            # logger.critical("open error")
            self.connect_flag = False
            return False

    # 发送数据并且接受数据，
    def get_answer(self, bytes_tmp, time_dur=0.1):
        # 加锁, 不加锁会如果多线程会导致资源竞争，操作就会很慢
        self.lock.acquire()

        self.write(bytes_tmp)
        if self.connect_flag is False:
            return None
        # ret = None
        data = b''
        # data_tmp  = b''
        ret = False
        start_time = time.time()
        len_res = 0

        # 读取前三个字节
        while True:
            if time.time() - start_time > time_dur:
                # 释放锁
                self.lock.release()

                return None, None
            if len_res==3:
                break
            res = self.read(3-len_res)
            # print(res)
            len_res = len(res)

        # 第三个字节为表示帧长
        length = res[2]
        # 加上前面的数据
        data = res + self.read(length-3)
        # 如果最后的字节为0x0a设置ret为True
        if data[-1] == 0x0a:
            ret = True
        # 释放锁
        self.lock.release()
        return ret, data

    def get_answer1(self):
        if self.connect_flag is False:
            return None
        start_time = time.time()

        flag = 0
        data = b''
        ret = False

        while True:
            res = self.read(1)
            if time.time() - start_time > self.timeout:
                break
            if flag == 2:
                len = res[0]
                data = data + res
                data = data + self.read(len-3)
                if data[-1] == 0x0a:
                    ret = True
                    break

            elif res == self.protocol.head1 and flag == 0:
                flag = 1
                data = res
            elif res == self.protocol.head2 and flag == 1:
                flag = 2
                data = data + res
        # data_tmp = self.readline()
        # print(data_tmp)
        return ret, data
    
    def get_serial_list(self):
        port_list = list_ports.comports()
        # for port in port_list:
        #     print(port)
        #     print('端口号：' + port[0] + '   端口名：' + port[1])
        port_list = [port for port in port_list if "CH340" in port[1] or "USB" in port[1]]


        #     print(port)
        # print(port_list)
        return port_list
    
    def ping_port(self):
        bytes_tmp = SerialBytes()
        ch340_list = self.get_serial_list()
        # print(ch340_list)
        if len(ch340_list) == 0:
            logger.error("未找到串口,查看是否插入了串口,或者查看下位机是否开机")
            # print("未找到串口,查看是否插入了串口,或者查看下位机是否开机")
            while True:
                time.sleep(1)
        
        for serial in ch340_list:
            try:
                # print(list(serial))
                self.set_port(serial[0])
                time.sleep(0.1)
                self.open()
                self.reset_input_buffer()
                self.reset_output_buffer()
                # 发送进入debug界面
                self.write(bytearray([0x55, 0xAA, 0x00, 0x01, 0x08, 0x00, 0x00, 0xF7]))
                ret = self.read(11)
                if ret == b'f\xbb\x01\x01\n\x00Z\x02\x00v':
                    # print(ret)
                    # 启动控制器加载程序
                    time.sleep(0.2)
                    self.write(bytearray([0x55, 0xAA, 0x00, 0x40, 0x0B, 0x00, 0x00, 0xD0, 0x00, 0x08, 0xDD]))
                    time.sleep(1)
                    self.reset_input_buffer()
                    self.reset_output_buffer()
                    
                bytes_tmp.set_bytes_start()
                tx = bytes_tmp.get_bytes_whole()
                for i in range(10):
                    self.write(tx)
                    ret = self.read(4)
                    # print(ret)
                    if ret==b'wh\x04\n':
                        # logger.info("ping:{} ok, res:{}".format(serial[0],res))
                        # print("ping ok!!")
                        logger.info("ping:{} ok, res:{}".format(serial[0],ret))
                        return True
                    time.sleep(0.1)

                # self.write(tx)
                # print(tx)
                # ret, res = self.get_answer(tx, time)
                # print("ping:", serial[0], "res:", res)
                    
                
                self.close()
            except Exception as e:
                # logger.error(e)
                pass
        return False
        
serial_wrap = SerialWrap()
class DevInterface:
    def __init__(self, dev_id = None, port_id=None, mode=None) -> None:
        
        self.ser = serial_wrap
        # self.data_structs = []
        # self.data_structs.append(StructData())
        self.data_struct = StructData()
        self.dev_id = dev_id
        self.mode = mode
        self.port_id = port_id
        self.time_out = 0.2
        self.last_data = None
        self.data = None
        self.s = 1
        
    def set_ser(self, ser):
        self.ser = ser

    def set_time_out(self, time_out):
        self.time_out = time_out

    def set_port(self, port_id):
        self.port_id = port_id

    # 获取最新的接受的数据
    def get_data(self):
        start_time = time.time()
        while True:
            if time.time() - start_time > self.time_out:
                return None
            if self.data is not None:
                self.last_data = self.data
                self.data = None
                return self.last_data

    # 构建发送指令包,dev_id必须有,每个外设都有,port_id不一定
    def get_bytes(self, *args, mode=None, port_id=None):
        data = []
        # print(args)
        data.append(self.dev_id)
        self.s = 3

        if mode is not None:
            data.append(mode)
        elif self.mode is not None:
            data.append(self.mode)
        else:
            self.s -= 1
            data.append(0)

        if port_id is not None:
            data.append(port_id)
        elif self.port_id is not None:
            data.append(self.port_id)
        else:
            self.s -= 1
        # 构建发送长度,整合成一个数据包
        d_len = len(self.data_struct) - len(data)
        args_list= list(args)
        # print(d_len)
        # print(args_list)
        # 根据情况去除参数或者补齐参数
        while True:
            if len(args_list) > d_len:
                args_list.pop(0)
            elif len(args_list) < d_len:
                args_list.append(0)
            else:
                break
        data = data + args_list
        # print(data)
        return self.data_struct.pack_data(data)
    
    @staticmethod
    def get_bytes_frame(bytes_tmp):
        bytes_len = (len(bytes_tmp) + 4).to_bytes(1, 'big')
        return b'\x77\x68' + bytes_len + bytes_tmp + b'\x0a'
    # 解码串口获取到的数据
    def get_result(self, bytes_all, index):
        self.data = self.data_struct.unpack_data(bytes_all, index)[self.s:]
        # print(self.data)
        if len(self.data) == 1:
            self.data = self.data[0]
        return self.data

    # 发送数据并且接受数据,并且返回数据
    def send_get(self, bytes_tmp,restart_flag = False):
        bytes_tx = self.get_bytes_frame(bytes_tmp)
        # print(bytes_tx.hex())
        # self.ser.write(bytes_tx)
        ret, bytes_rx = self.ser.get_answer(bytes_tx, self.time_out)
        if(restart_flag):
            print("duoduoji = ",ret)
            print("bytes_rx = ",bytes_rx)
            print("bytes_tx=",bytes_tx)
        if ret:
            # print(bytes_rx.hex())
            self.last_data = self.get_result(bytes_rx, 3)
            return self.last_data
        return self.last_data
    
    def act_mode(self, *args, mode=None, port_id=None,restart_flag = False):
        data_bytes = self.get_bytes(*args, mode=mode, port_id=port_id)
        return self.send_get(data_bytes,restart_flag = restart_flag)
    
    def reset(self, port_id=None):
        data_bytes = self.get_bytes(mode=3,port_id=port_id)
        return self.send_get(data_bytes)
    
    def start(self, port_id=None):
        data_bytes = self.get_bytes(mode=4,port_id=port_id)
        return self.send_get(data_bytes)
    
    def stop(self, port_id=None):
        data_bytes = self.get_bytes(mode=5,port_id=port_id)
        return self.send_get(data_bytes)
    
    def set(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, mode=2, port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)
    
    def get(self, *args, mode=None, port_id=None):
        data_bytes = self.get_bytes(*args, mode=1, port_id=port_id)
        return self.send_get(data_bytes)
    
    def read(self, port_id=None):
        data_bytes = self.get_bytes(port_id=port_id)
        # print(data_bytes)
        return self.send_get(data_bytes)
    
    def act_default(self, *args, port_id=None):
        data_bytes = self.get_bytes(*args, port_id=port_id)
        return data_bytes

class DevListWrap:
    def __init__(self, dev_list=None) -> None:
        if dev_list is None:
            self.dev_list = []
        else:
            self.dev_list = dev_list

    def append(self, dev):
        self.dev_list.append(dev)

    def send_all(self, args):
        bytes_all = b''
        for i in range(len(self.dev_list)):

            bytes_all += self.dev_list[i].act_default(args[i])

        bytes = DevInterface.get_bytes_frame(bytes_all)
        # serial_wrap.write(bytes)
        ret, res = serial_wrap.get_answer(bytes)
        data_ret = []
        if ret:
            index = 3
            for i in range(len(self.dev_list)):
                data = self.dev_list[i].get_result(res, index)
                index += self.dev_list[i].data_struct.size
                data_ret.append(data)

        return data_ret


    def __getattr__(self, name):
        return getattr(self.dev_list, name)


class Beep(DevInterface):
    def __init__(self) -> None:
        super().__init__(dev_id=10)
        self.data_struct.set_fortmat('BB')

class My_Beep():
    def __init__(self) -> None:
        pass
    def set(self,dur_time=0.2,count=1,interval=0.2):
        GPIO.setmode(GPIO.BCM) 
        
        GPIO.setup(22, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(22, GPIO.HIGH)
        while count>0:
            # print('jiao')
            a = time.time()
            # 叫
            GPIO.output(22, GPIO.LOW)
            while time.time() -a <dur_time:
                pass

            if interval!=0:
                #不叫的电平
                GPIO.output(22, GPIO.HIGH)
                time.sleep(interval)
            count -= 1
        
        GPIO.cleanup(22)
        
            
class Motor(DevInterface):
    def __init__(self, port_id=None, reverse=1) -> None:
        super().__init__(2, port_id=port_id)
        self.data_struct.set_fortmat('bbb')
        self.reverse = reverse
    # 改变电机的转向
    def set_dir(self, reverse):
        self.reverse = reverse

    # 是否改变速度的方向
    def set(self, *args):
        args = list(args)
        if len(args) == len(self.data_struct):
            args[1] = args[1] * self.reverse
        else:
            args[0] = args[0] * self.reverse
        return super().set(*args)
    
    def act_default(self, *args, port_id=None):
        return super().act_default(*args, port_id=port_id)
    
# 磁传感器
class SensorAi(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=7, mode=0, port_id=port_id)
        self.data_struct = StructData('bbH')

# 红外传感器
class Infrared(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=7, mode=1, port_id=port_id)
        self.data_struct = StructData('bbH')

class TestBytes(DevInterface):
    def __init__(self, ser_obj=None) -> None:
        super().__init__(ser_obj)
        self.bytes = SerialBytes(1)
        self.data_struct = StructData(5, '<b')

# 蓝牙手柄类
class BluetoothPad(DevInterface):
    def __init__(self) -> None:
        # 调用父对象初始化
        super().__init__(dev_id=9)
        self.data_struct = StructData('BBBBi')

        self.throsheld_mid = [97, 97, 97, 97, 0]
        self.stick_min = 40
        self.stick_max = 160
        self.divisor_min = [42, 42, 42, 42]
        self.divisor_max = [56, 56, 56, 56]
        
        self.margin = 6

    def calibrate(self):
        info_tmp = self.read()

        # print(info_tmp)
        for i in range(4):
            if abs(info_tmp[i] - self.throsheld_mid[i]) < 10:
                self.throsheld_mid[i] = info_tmp[i]
        # logger.info(str(self.throsheld_mid))
        for i in range(4):
            self.divisor_max[i] = self.stick_max - self.throsheld_mid[i] - self.margin
            self.divisor_min[i] = self.throsheld_mid[i] - self.stick_min - self.margin

    def get_stick(self):
        data = self.read()
        re_data = []
        tmp = 0.0
        for i in range(4):
            tmp = data[i] - self.throsheld_mid[i]
            if abs(tmp) < self.margin:
                tmp = 0
            if tmp > 0:
                tmp = (tmp-self.margin) / self.divisor_max[i]
            elif tmp<0:
                tmp = (tmp+self.margin) / self.divisor_min[i]
            tmp = min(1, max(-1, tmp))
            re_data.append(tmp)
        if data[4] == 49152:
            self.calibrate()
        re_data.append(data[4])
        return re_data


class SensorTof(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=7, port_id=port_id, mode=11)
        self.data_struct.set_fortmat('bbH')

class BoardKey(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=13)
        self.data_struct.set_fortmat('bi')

# led灯
class LedLight(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=14, port_id=port_id)
        self.data_struct.set_fortmat('bbBBBB')
    
    def set_light(self, led_id, r, g, b, port_id=None):
        return super().set(led_id, r, g, b, port_id=port_id)
    
    def set(self, *args, port_id=None):
        return super().set(*args, port_id=port_id)


# led灯
class Buguang(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=14, port_id=port_id)
        self.data_struct.set_fortmat('bbBBBB')
    
    def set_light(self, led_id, r, g, b, port_id=None):
        return super().set(led_id, r, g, b, port_id=port_id)
    
    def set(self, *args, port_id=None):
        return super().set(*args, port_id=port_id)

# 四个按键类
class Key4Btn(SensorAi):

    btn_sta = []
    btn_sta_last = []
    state = []
    state_read = []
    limit = 0.05
    stop_time = 0.05
    bak_time = 0.0
    last_time = 0.0
    long_time = 0.7
    short_time = 0.4

    def __init__(self, port_id=None) -> None:
        super().__init__(port_id=port_id)
        self.key_map = {3:355,1:1366,2:2137, 4:2988}
        self.threshold = 0.1

        for i in range(5):
            self.btn_sta.append([False, 0.0, 0.0])

    def key_map_btn(self, val):
        r_key = 0
        diff = 1
        for key, value in self.key_map.items():
            try:
                tmp = abs(value - val) / value
                if tmp < self.threshold and tmp < diff:
                    r_key = key
                    diff = tmp
            except:
                pass
        return r_key

    # 返回匹配的按键操作
    def get_key(self, port_id=None):
        val = self.read(port_id=port_id)
        return self.key_map_btn(val)
    
    # def get_result(self, bytes_all, index):
    #     res = super().get_result(bytes_all=bytes_all, index=index)
    #     return res
        # if res is not None:
            # self.data = self.key_map_btn(res)
            # return self.data
            
    # 短按1=1,2=2,3=3,4=4
    # 长按1=5,2=6,3=7,4=8
    def get_btn(self, port_id=None):
        self.event()
        if len(self.state) > 0:
            key_v, key_state = self.state[0][0], self.state[0][1]
            del self.state[0]
            return key_v + 1 + (key_state-1)*4
        else:
            return 0

    def event(self):
            self.bak_time = time.time()
            index = 0
            while len(self.state) > index:
                for i in range(len(self.state)):
                    if self.bak_time - self.state[index][2] > self.limit:
                        del self.state[index]
                        continue
                index = + 1

            button_num = self.get_key()
            # if button_num != 0:
            #     print(button_num)
            if button_num != 0:
                # print(button_num)
                index = button_num - 1
            else:
                index = 4
                
            # 对应的按键按下，更新状态
            if self.btn_sta[index][0]:
                # 更新按键按下时间
                self.btn_sta[index][1] += (self.bak_time - self.btn_sta[index][2])
                # 发送连续按下
                if self.btn_sta[index][1] > self.long_time and index != 4:
                    self.state.append([index, 3, self.bak_time])
            else:
                self.btn_sta[index][0] = True
                self.btn_sta[index][1] = 0
            self.btn_sta[index][2] = self.bak_time
            # print(self.btn_sta)
            for i in range(4):
                btn_state, time_dur, time_last = self.btn_sta[i][0], self.btn_sta[i][1], self.btn_sta[i][2]
                # 如果有记录按下
                if btn_state:
                    # 如果长时间没有更新
                    if self.bak_time - time_last > self.stop_time:
                        if self.limit < time_dur < self.long_time:
                            self.state.append([i, 1, time_last])
                        elif time_dur > self.long_time:
                            self.state.append([i, 2, time_last])
                        self.btn_sta[i][0] = False
                        self.btn_sta[i][1] = 0.0






    # def event(self):
    #     self.bak_time = time.time()
    #     index = 0
    #     # 如果超时未读取，删除临时保存状态
    #     while index < len(self.state):
    #         if self.bak_time - self.state[index][2] > self.limit:
    #             del self.state[index]
    #             continue
    #         index = + 1
    #     button_num = self.get_key()
    #     if button_num != 0:
    #         # print(button_num)
    #         for i in range(4):
    #             if i == button_num - 1:
    #                 tmp_time = self.btn_sta[i][1]
    #                 self.btn_sta[i][0] = True
    #                 self.btn_sta[i][1] += (self.bak_time - self.last_time)
    #                 self.btn_sta[i][2] = self.bak_time
    #                 # 该状态上传长按一次
    #                 if tmp_time < self.long_time < self.btn_sta[i][1]:
    #                     self.state.append([i, 2, self.btn_sta[i][2]])
    #                 # 该状态上传长按连续
    #                 elif tmp_time > self.long_time :
    #                     self.state.append([i, 3, self.btn_sta[i][2]])
    #             elif button_num != 0:
    #                 self.btn_sta[i][0] = False
    #                 self.btn_sta[i][1] = 0.0
    #         self.btn_sta[4][0] = False
    #         self.btn_sta[4][1] = 0.0
    #     else:
    #         for i in range(4):
    #             if self.btn_sta[i][0]:

    #                 if self.btn_sta[i][1] < self.long_time:
    #                     # self.btn_sta_last.append([i, 1, self.btn_sta[i][2]])
    #                     self.state.append([i, 1, self.btn_sta[i][2]])
    #                 self.btn_sta[i][0] = False
    #                 self.btn_sta[i][1] = 0.0
    #         self.btn_sta[4][0] = True
    #         self.btn_sta[4][1] += (self.bak_time - self.last_time)
    #         # self.state_read[0]
    #     # print(self.btn_sta)
    #     # print(self.state)
    #     self.last_time = self.bak_time
    #     # time.sleep(0.1)

    
class Key4Btn1(SensorAi):
    btn_sta = []
    btn_sta_last = []
    state = []
    state_read = []
    limit = 0.5
    bak_time = 0.0
    last_time = 0.0
    long_time = 0.7
    stop_time = 0.05

    def __init__(self, port_id=None) -> None:
        super().__init__(port_id=port_id)
        self.key_map = {3:355,1:1366,2:2137, 4:2988}
        self.threshold = 0.1

        for i in range(5):
            self.btn_sta.append([False, 0.0, 0.0])

    def key_map_btn(self, val):
        r_key = 0
        diff = 1
        for key, value in self.key_map.items():
            try:
                tmp = abs(value - val) / value
                if tmp < self.threshold and tmp < diff:
                    r_key = key
                    diff = tmp
            except:
                pass
        return r_key
    
    def get_key(self, port_id=None):
        val = self.read(port_id=port_id)
        return self.key_map_btn(val)
    
    # def get_result(self, bytes_all, index):
    #     res = super().get_result(bytes_all=bytes_all, index=index)
    #     return res
        # if res is not None:
            # self.data = self.key_map_btn(res)
            # return self.data
    
    def get_btn(self, port_id=None):
        self.event()
        if len(self.state) > 0:
            key_v, key_state = self.state[0][0], self.state[0][1]
            del self.state[0]
            return key_v + 1 + (key_state-1)*4
        else:
            return 0
    def event(self):
        self.bak_time = time.time()
            
        index = 0
        while len(self.state) > index:
            for i in range(len(self.state)):
                if self.bak_time - self.state[index][2] > self.limit:
                    del self.state[index]
                    continue
            index = + 1

        button_num = self.get_key()
        # if button_num != 0:
        #     print(button_num)
        if button_num != 0:
            # print(button_num)
            index = button_num - 1
        else:
            index = 4
            
        # 对应的按键按下，更新状态
        if self.btn_sta[index][0]:
            # 更新按键按下时间
            self.btn_sta[index][1] += (self.bak_time - self.btn_sta[index][2])
            # 发送连续按下
            if self.btn_sta[index][1] > self.long_time and index != 4:
                self.state.append([index, 3, self.bak_time])
        else:
            self.btn_sta[index][0] = True
            self.btn_sta[index][1] = 0
        self.btn_sta[index][2] = self.bak_time
        # print(self.btn_sta)
        for i in range(4):
            btn_state, time_dur, time_last = self.btn_sta[i][0], self.btn_sta[i][1], self.btn_sta[i][2]
            # 如果有记录按下
            if btn_state:
                # 如果长时间没有更新
                if self.bak_time - time_last > self.stop_time:
                    if self.limit < time_dur < self.long_time:
                        self.state.append([i, 1, time_last])
                    elif time_dur > self.long_time:
                        self.state.append([i, 2, time_last])
                    self.btn_sta[i][0] = False
                    self.btn_sta[i][1] = 0.0        

class Motor4(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=1)
        self.data_struct.set_fortmat('bbbbb')


class Motors():
    def __init__(self, ports) -> None:
        self.moto_ports = ports
        self.motors = []
        for i in ports:
            self.motors.append(Motor(i))
    
    # 设置速度
    def set_speed(self, speeds):
        bytes_all = b''
        for i in range(len(self.moto_ports)):

            bytes_all += self.motors[i].get_bytes(speeds[i], mode=2)
        bytes_tmp = DevInterface.get_bytes_frame(bytes_all)
        # serial_wrap.write(bytes)
        ret, res = serial_wrap.get_answer(bytes_tmp)
        if ret:
            index = 3
            ret = []
            for motor in self.motors:
                if motor.dev_id == res[index]:
                    data_res = motor.get_result(res, index)
                    index += motor.data_struct.size
                    ret.append(data_res)
            # print(ret)

class EncoderMotor(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=4, port_id=port_id)
        self.data_struct.set_fortmat('bbi')

class EncoderMotors(DevInterface):
    def __init__(self, port_id=None) -> None:
        super().__init__(dev_id=3, port_id=port_id)
        self.data_struct.set_fortmat('biiii')


class ServoPwm(DevInterface):
    def __init__(self, port_id=None, mode=None) -> None:
        super().__init__(5, port_id=port_id, mode=mode)
        self.data_struct.set_fortmat('bbBB')

# 伺服电机
class ServoBus(DevInterface):
    def __init__(self,port_id=None) -> None:
        self.now_angle = 0
        super().__init__(6, port_id)
        self.data_struct.set_fortmat('bbbbh')
        self.set_time_out(1)
    
    def set_angle(self, speed, angle,restart_flag = False):
        # if self.port_id == 1 and abs(self.now_angle-angle)>30:
        #     speed = 40
        self.now_angle = angle
        if(self.port_id==3):
            angle-=20
        self.act_mode(1, speed, angle, mode=2,restart_flag = restart_flag)

    def set_rotate(self, speed):
        self.act_mode(2, speed, mode=2)

class SerialWireless(DevInterface):
    def __init__(self) -> None:
        super().__init__(40)
        self.data_struct = StructData(5, '<bb')


class ScreenShow(DevInterface):
    def __init__(self) -> None:
        super().__init__(11)
        format_t = 'b' * 101
        self.data_struct.set_fortmat(format_t)
    
    def show(self, args):
        if type(args) != str:
            args = str(args)
        int_values = [ord(arg) for arg in args]
        int_values = tuple(int_values)
        self.set(*int_values)
    

class Battry(DevInterface):
    def __init__(self, ser_obj=None) -> None:
        super().__init__(ser_obj, 12)
        self.data_struct.set_fortmat('bi')
    
    def get(self):
        res = super().get()
        bat = float(res) / 1000
        return bat        

def beep_test():
    beep = Beep()
    beep.set(200, 10)
    for i in range(10):
        beep.set(200, 10)
        time.sleep(0.5)

def motor_test():
    motor = Motor(1)
    for i in range(10):
        motor.set(40)
        time.sleep(1)
        motor.set(0)
        time.sleep(1)

def sensor_ai_test():
    sensor4 = SensorAi(4)
    while(1):
        print(sensor4.read())
        time.sleep(1)

def sensor_tof_test():
    tof = SensorTof(4)
    while(1):
        print(tof.read())
        time.sleep(1)

def show_test():
    show = ScreenShow()
    show.show("my_test\n\nok")

def servo_bus_test():
    servo_bus = ServoBus(1)
    while(1):
        servo_bus.set_angle(100, -100)
        time.sleep(1)
        servo_bus.set_angle(100, 0)
        time.sleep(1)

def servo_pwm_test():
    servo_pwm = ServoPwm(1)
    while(1):
        servo_pwm.set(100, 50)
        time.sleep(1)
        servo_pwm.set(100, 90)
        time.sleep(1)

def key_test():
    flag = False
    key = Key4Btn(4)
    # def my_thread():
    #     nonlocal flag, key
    #     while True:
    #         res = key.get_btn()
    #         if res == 5:
    #             flag = True
    #             os._exit(0)
    #             # return True
    #         time.sleep(0.1) 
    #         print(res)
    import sys
    import subprocess
    # key = Key4Btn(4)
    show = ScreenShow()
    index = 1
    # thread_t = Thread(target=my_thread)
    # thread_t.daemon = True
    # thread_t.start()

    last_time = time.time()
    while True:

        res = key.get_btn()
        now = time.time()
        if res!=0:
            print(res)
        try:
            fps = (1 / (now - last_time))
        except ZeroDivisionError:
            fps = 100
        last_time = now
        if flag:
            sys.exit(0)
        # print("fps:", fps)
        # print(res)
        # print(res)
        if res == 2:
            index += 1
            show.show(f'\n{index}')
        elif res == 3:
            index -= 1
            show.show(f'\n{index}')
        elif res == 1:
            serial_wrap.close()
            subprocess.Popen(sys.argv)
            serial_wrap.ping_port()

if __name__ == "__main__":
    # beep = My_Beep()
    # beep.set(0.2,2,1)
    # import threading
    # import time
    # _stop_flag = False
    # _end_flag  = False
    # key = Key4Btn(4)
    # def key_thread_func():
    #     nonlocal _stop_flag
    #     nonlocal _end_flag
    #     while True:
    #         # flase进入这里
    #         if not _stop_flag:
    #             if _end_flag:
    #                 return
    #             key_val = key.get_btn()
    #             # print(key_val)
    #             if key_val == 3:
    #                 _stop_flag = True
    #                 enable = True 
    #             time.sleep(0.2)
    #         if enable == True:
    #             key_val = key.get_btn()
    #             print('here !!!!!!!!!!!!!!!!!!!!!!1')
    #             if key_val == 1 or key_val == 5:
    #                 _stop_flag = True
    #                 _end_flag = True


    # thread = threading.Thread(target=key_thread_func)
    # thread.start()
    # thread.join()


    # beep_test()
    # motor_test()
    # sensor_ai_test()
    # sensor_tof_test()
    # key = Key4Btn(4)
    # last_num = 10
    # ServoBus(2)
    servo_high = ServoPwm(2)
    servo_high.set(50, 0)
    # arm_servo = ServoBus(1)
    # for i in range(10):

    #     arm_servo.set_angle(80, 60,restart_flag=True)
    #     time.sleep(1)
    #     arm_servo.set_angle(80, 90,restart_flag=True)
    #     time.sleep(1)
    # light = Buguang(5)
    # for i in range(1,9):
    #     light.set_light(i, 0, 0, 0)
    # for i in range(3,7):

    #     light.set_light(i, 255, 255, 255)

    # arm_servo.set_rotate(100)
    # time.sleep(5)
    # arm_servo.set_rotate(0)
    # arm_servo.set_angle(90, 0)
    # time.sleep(0.3)
    # time.sleep(1)
    # servo = ServoPwm(2)
    # servo.set(50, 0)
    # LedLight()
    # while True:
   
    #     num = key.get_key()
    #     if num!=last_num:
    #         print(num)
    #         last_num=num
    # key_test()
    # servo_bus_test()
    # servo_pwm_test()


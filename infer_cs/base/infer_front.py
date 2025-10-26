#!/usr/bin/python
# -*- coding: utf-8 -*-
import zmq
import cv2
import numpy as np
import time
import json
import subprocess
import os, sys
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 

import psutil
from log_info import logger
 

def get_zmp_client(port):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    res = socket.connect(f"tcp://127.0.0.1:{port}")
    # print(res)
    return socket

def get_python_processes():
    
    # print("----------")
    python_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if 'python' in proc.info['name'].lower() and len(proc.info['cmdline']) > 1 and len(proc.info['cmdline'][1]) < 100:
                info = [proc.info['pid'], proc.info['cmdline'][1]]
                python_processes.append(info)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return python_processes
    # for process in python_processes:
    #     print(f"PID: {process['pid']}, Name: {process['name']}, Cmdline: {process['cmdline']}")
    # print("    ")

class Bbox:
    def __init__(self, box=None, rect=None, size=[640, 480]) -> None:
        self.size = np.array(size) / 2
        self.size_concat = np.concatenate((self.size, self.size))

        if box is not None:
            box_np = np.array(box)
            # 如果所有值的绝对值都小于1，表示归一化
            # np.abs(box_np, out=box_np)
            if (np.abs(box_np) < 2).all():
                self.box_normalise = box_np
                self.box = self.denormalise(box_np, self.size)
                # print(self.box)
            else:
                self.box = box_np
                self.box_normalise = self.normalise(box_np, self.size)
            self.rect = self.box_to_rect(self.box, self.size)
        elif rect is not None:
            self.rect = np.array(rect)
            self.box = self.rect_to_box(self.rect, self.size)
            self.box_normalise = self.normalise(self.box, self.size)
    
    def get_rect(self):
        return self.rect
    
    def get_box(self):
        return self.box

    @staticmethod
    def normalise(box, size):
        return box / np.concatenate((size, size))
    
    @staticmethod
    # 去归一化
    def denormalise(box_nor, size):
        return (box_nor * np.concatenate((size, size))).astype(np.int32)

    @staticmethod
    def rect_to_box(rect, size):
        pt_tl = rect[:2]
        pt_br = rect[2:]
        pt_center = (pt_tl + pt_br) / 2 - size
        box_wd = pt_br - pt_tl
        return np.concatenate((pt_center, box_wd)).astype(np.int32)

    @staticmethod
    def box_to_rect(box, size):
        pt_center = box[:2]
        box_wd = box[2:]
        pt_tl = (size + pt_center - box_wd / 2).astype(np.int32)
        pt_br = (size + pt_center + box_wd / 2).astype(np.int32)
        # print(pt_tl, pt_br)
        rect = np.concatenate((pt_tl, pt_br))
        # 限制最大最小值
        max_size = np.concatenate((size, size))*2
        # print(max_size)
        np.clip(rect, 0, max_size, out=rect)
        return rect

class ClintInterface:
    configs = [
            {'name':'lane', 'infer_type': 'LaneInfer', 'params': [], 'port':5001, 'img_size':[128, 128]},
            {'name':'task', 'infer_type': 'YoloeInfer', 'params': ['task_model3'], 'port':5002, 'img_size':[416, 416]},
            {'name':'front', 'infer_type':'YoloeInfer', 'params': ['front_model2'], 'port':5003, 'img_size':[416, 416]},
            {'name':'redstop', 'infer_type':'YoloeInfer', 'params': ['redstop'], 'port':5004, 'img_size':[416, 416]},
            {'name':'ocr', 'infer_type':'OCRReco', 'params': [], 'port':5005,'img_size':None},
            {'name':'humattr', 'infer_type':'HummanAtrr', 'params': [], 'port':5006, 'img_size':None},
            {'name':'mot', 'infer_type':'MotHuman', 'params': [], 'port':5007, 'img_size':None}
            ]
    
    def __init__(self, name):
        logger.info("{}连接服务器...".format(name))
        model_cfg = self.get_config(name)
        self.img_size = model_cfg['img_size']
        self.client = self.get_zmp_client(model_cfg['port'])
        
        infer_back_end_file = "infer_back_end.py"
        # 检查后台程序是否运行, 如果未开启, 则开启
        self.check_back_python(infer_back_end_file)

        flag = False
        while True:
            if self.get_state():
                if flag:
                    logger.info("")
                break
            # 输出一个提示信息，不换行
            print('.', end='', flush=True)
            # logger.info(".")
            time.sleep(1)
            flag = True
        # print(self.client)
        # print("连接服务器成功")
        logger.info("{}连接服务器成功".format(name))
    
    def check_back_python(self, file_name):
        dir_file = os.path.abspath(os.path.dirname(__file__))
        file_path = os.path.join(dir_file, file_name)
        # print(file_path)
        if not os.path.exists(file_path):
            raise Exception("后台脚本文件不存在")
        # 获取正在运行的python脚本
        py_lists = get_python_processes()
        for py_iter in py_lists:
            # 检测是否存在后台运行的脚本
            if file_name in py_iter[1]:
                return
        else:
            # 开启后台脚本，后台运行, 忽略输入输出
            # 使用subprocess调用脚本
            logger.info("开启{}脚本, 后台运行中, 请等待".format(file_name))
            cmd_str = 'python3 ' + file_path + ' &'
            # shell=True告诉subprocess模块在运行命令时使用系统的默认shell。这使得可以像在命令行中一样执行命令，包括使用通配符和其他shell特性
            subprocess.Popen(cmd_str, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1)
            # 这里的> /dev/null 2>&1将标准输出和标准错误都重定向到/dev/null，实现与之前subprocess.Popen相同的效果
            # os.system(cmd_str + " > /dev/null 2>&1")
        

    def get_config(self, name):
        for conf in self.configs:
            if conf['name'] == name:
                return conf
            
    @staticmethod
    # 请求者
    def get_zmp_client(port):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        res = socket.connect(f"tcp://127.0.0.1:{port}")
        # print(res)
        return socket
    
    # 实例调用get_infer发送图片给后端
    def __call__(self, *args, **kwds):
        return self.get_infer(*args, **kwds)
    
    # 发送连接请求，等待回应
    def get_state(self):
        data = bytes('ATATA', encoding='utf-8')
        self.client.send(data)
        # 接收服务器数据
        response = self.client.recv()
        
        # 把bytes转为json
        response = json.loads(response)
        return response
    
    def get_infer(self, img):
        # img转jpeg格式压缩，并转为bytes
        if self.img_size is not None:
            img = cv2.resize(img, self.img_size)
        img = cv2.imencode('.jpg', img)[1].tobytes()
        data = bytes('image', encoding='utf-8') + img
        self.client.send(data)
        # 接收服务器数据
        response = self.client.recv()
        
        # 把bytes转为json
        response = json.loads(response)
        return response

def main_client():
    from camera import Camera
    cap = Camera(2, 640, 480)
    # cap.set_size(640, 480)
    # cap.start_back_thread()
    # infer_client = ClintInterface('lane')
    infer_client = ClintInterface("redstop")
    # infer_client = ClintInterface('task')
    # infer_client = ClintInterface('mot')
    # infer_client = ClintInterface('front')
    # infer_client = infer_clint
    # while True:
    #     print(infer_client.get_state())
    #     time.sleep(1)
    # infer_client = TaskDetectClient()
    last_time = time.time()
    while True:
        img = cap.read()
        # img = cv2.resize(img, (128, 128))
        dets_ret = infer_client.get_infer(img)
        # dets_ret = infer_client.get_infer(img)
        # dets_ret = infer_client.get_infer(img)
        print(dets_ret)
        for det in dets_ret:
            cls_id, obj_id, label, score, bbox = det[0], det[1], det[2], det[3], det[4:]
            rect = Bbox(box=bbox, size=[640, 480]).get_rect()
            # print(rect)
            cv2.rectangle(img, rect[0:2], rect[2:4], (255, 0, 0), 2)
        
        # response = task_det_client.get_infer(img)
        cv2.imshow("img", img)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        # print(response)
        fps = 1 / (time.time() - last_time)
        last_time = time.time()
        print("fps:", fps)
    cap.close()
    cv2.destroyAllWindows()

def stop_process(py_str):
    py_lists = get_python_processes()
    print(py_lists)
    for py_procees in py_lists:
        pid_id, py_name = py_procees[0], py_procees[1]
        # print(pid_id, py_name)
        if py_str in py_procees[1]:
            psutil.Process(pid_id).terminate()
            print("stop", py_name)
            return


if __name__ == '__main__':
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--op', type=str, default="infer")
    args = args.parse_args()
    print(args)
    if args.op == "infer":
        main_client()
    if args.op == "stop":
        stop_process("infer_back_end.py")

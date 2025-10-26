#!/usr/bin/python3
# -*- coding: utf-8 -*-

import threading
from multiprocessing import Process
import time
import signal
import atexit

import cv2
import platform
import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 
from log_info import logger
from vehicle import Buguang
import re
import subprocess
from vehicle import Beep,Buguang
import subprocess
import re

def get_video_device_ids():
    try:
        # 使用shell=True让shell解析通配符
        output = subprocess.check_output('ls /dev/video*', shell=True).decode('utf-8')
        
        # 使用正则表达式匹配所有数字
        device_ids = [int(re.search(r'\d+', name).group()) for name in output.split()]
        # print(device_ids)
        return device_ids
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
        return []

class Camera:
    def __init__(self, index=1, width=640, height=480):
        self.width = width
        self.height = height
        self.index = index
        
        self.cap = None
        self.frame = None
        # 暂停标志
        self.pause_flag = False
        self.stop_flag = False

        self.init()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        # thread是否运行标志
        self.flag_thread = False
        self.start_back_thread()

    def __del__(self):
        """析构函数，只做最基本的释放"""
        try:
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        except:
            pass

    def force_release(self):
        """强制释放摄像头资源"""
        try:
            self.stop_flag = True
            time.sleep(0.2)  # 等待线程停止
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            logger.info("强制释放摄像头 {} 完成".format(getattr(self, 'src', 'unknown')))
        except Exception as e:
            logger.error("强制释放摄像头时出错: {}".format(e))

    def close(self):
        """确保完全释放摄像头资源"""
        try:
            self.stop_flag = True
            
            # 等待线程结束
            if self.flag_thread and hasattr(self, 'cap_thread'):
                self.cap_thread.join(timeout=2.0)
                if self.cap_thread.is_alive():
                    logger.warning("线程未能正常结束，强制释放")
                    self.force_release()
                    return
            
            # 释放摄像头
            if self.cap is not None:
                self.cap.release()
                self.cap = None
                
            logger.info("摄像头 {} 资源已释放".format(getattr(self, 'src', 'unknown')))
        except Exception as e:
            logger.error("释放摄像头资源时出错: {}".format(e))
            # 出错时强制释放
            self.force_release()

    def init(self):
        while True:
            try:
                if 'Windows' in platform.platform():
                    self.src = self.index
                    self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
                else:
                    # 保持使用 /dev/video 路径
                    self.src = "/dev/video" + str(self.index)

                    self.cap = cv2.VideoCapture(self.src)
                
                # 检查是否成功打开
                if self.cap.isOpened():
                    logger.info("摄像头{}打开成功".format(self.src))
                    break
                else:
                    logger.error("摄像头{}打开失败".format(self.src))
                    if self.cap is not None:
                        self.cap.release()
                    time.sleep(1)
                    
            except Exception as e:
                logger.error("init:摄像头{}打开错误: {}".format(self.src, e))
                if self.cap is not None:
                    self.cap.release()
                time.sleep(1)
    
    def start_back_thread(self):
        # 如果未开启线程，开启线程
        if not self.flag_thread:
            self.cap_thread = threading.Thread(target=self.update, args=())
            self.cap_thread.daemon = True
            self.cap_thread.start()
            self.flag_thread = True
        time.sleep(0.5)
            
    def update(self):
        while True:
            if self.stop_flag:
                break
            if self.pause_flag:
                continue
            try:
                ret, frame = self.cap.read()
                if self.index == 0 and ret:
                    frame = cv2.flip(frame, -1)

                if ret:
                    self.frame = frame
                else:
                    logger.error("read:读取图像错误!!!!{}".format(self.src))
                    self.cap.release()
                    self.init()
                    self.set_size(self.width, self.height)
            except Exception as e:
                logger.error("exception:摄像头{}错误: {}".format(self.src, e))
                self.cap.release()
                self.init()
                self.set_size(self.width, self.height)

    def set_size(self, width, height):
        self.width = width
        self.height = height
        if self.cap is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self):
        while self.frame is None:
            time.sleep(0.1)
        return self.frame


def main():
    light = Buguang(5)
    light.set_light(4, 70, 70, 70)
    
    # 测试摄像头0和摄像头2，使用参考代码的简化方法
    print("正在测试简化的摄像头实现（使用/dev/video路径）...")
    
    try:
        camera0 = Camera(0, 640, 480)
        camera2 = Camera(2, 640, 480)
        
        print("摄像头初始化完成，开始显示画面...")
        print("按 'q' 键退出测试")
        
        while True:
            try:
                img0 = camera0.read()
                img2 = camera2.read()
                
                cv2.imshow("Camera 0 (/dev/video0)", img0)
                cv2.imshow("Camera 2 (/dev/video2)", img2)
                
                key = cv2.waitKey(1)
                if key == ord('q'):
                    print("用户退出...")
                    break
                    
            except Exception as e:
                logger.error(f"读取摄像头画面错误: {e}")
                break
        
        # 清理资源
        camera0.close()
        camera2.close()
        
    except Exception as e:
        print(f"摄像头测试失败: {e}")
    
    # 关闭灯光
    for i in range(1, 9):
        light.set_light(i, 0, 0, 0)
    
    cv2.destroyAllWindows()
    logger.info("测试完成")


if __name__ == "__main__":
    main()
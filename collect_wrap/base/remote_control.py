
import cv2
import threading
import time
import json
import subprocess
import os, sys
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))) 


from vehicle import MecanumBase, BluetoothPad, ScreenShow
from camera import Camera
from log_info import logger


class RemoteControlCar:
    def __init__(self, cap: Camera = None) -> None:
        # 获取当前存储目录
        path_dir = os.path.abspath(os.path.dirname(__file__))
        # 获取模型存储目录
        self.dir = os.path.join(path_dir, "6")
        if not os.path.exists(self.dir):
            os.mkdir(self.dir)

        self.index = 0
        if cap is None:
            self.cap = Camera(2, 320, 240)
        else:
            self.cap = cap
        
        # self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        # self.cap.open()

        # while self.cap.isOpened() is False:
        #     print("摄像头未打开")
        #     time.sleep(1)

        # self.cap = None
        self.car = MecanumBase()
        self.car.beep()
        self.display = ScreenShow()

        self.blue_pad = BluetoothPad()

        self.state_base = [0.15, 0.15, 0.3]
        self.state_start = [0.3, 0.3, 0.5]
        self.speed_x = 0.15     # m/s
        self.speed_y = 0.15     # m/s
        self.ration_omage = 0.5
        self.car_state = [0.0, 0.0, 0.0]
        self.run_flag = False
        self.exit_flag = False
        self.json_data = []
        # self.car_thread = threading.Thread(target=self.car_process, args=())
        # self.car_thread.daemon = True
        # self.car_thread.start()
        logger.info("remote control start!!")
        # self.display.show("press btn control\n 3 start\n 4+2 stop\n 4+v del 30pic\n 4+o del all\n")
        self.img_thread = threading.Thread(target=self.image_process, args=())
        self.img_thread.daemon = True
        self.img_thread.start()
        
        self.car_process()
        
 
    def car_process(self):
        pad_exit_flag = False
        while not self.exit_flag:
            keys_val = self.blue_pad.get_stick()
            #keys_val = list(self.blue_pad.get_stick().values())
            #print(f"类型: {type(keys_val)}, 内容: {keys_val}")
            # print(keys_val)
            if keys_val == [-1, -1, -1, -1, 0]:
                pad_exit_flag = False
                self.car_state = [0.0, 0.0, 0.0]
                logger.error("no bluepad")
                self.display.show("no bluepad\n")
                self.car.beep()
                time.sleep(1)
                
                continue
            else:
                if not pad_exit_flag:
                    self.display.show("press btn control\n 3 pressing record\n 4+2 stop\n 4+v del 30pic\n 4+o del all\n")
                pad_exit_flag = True
            
            if (keys_val[4] & 1024) != 0:
                self.run_flag = True
            else:
                self.run_flag = False
            if keys_val[4] == 34816:
                self.close()
            elif keys_val[4] == 2052:
                # 删除3s内的图片
                self.del_last3s()
            elif keys_val[4] == 2304:
                # 删除所有图片和数据
                self.restart()
            if self.run_flag:
                self.car_state[0] = self.state_base[0]
                self.car_state[1] = -1 * self.state_base[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_base[2] * keys_val[2]
            else:
                self.car_state[0] = self.state_start[0] * keys_val[1]
                self.car_state[1] = -1 * self.state_start[1] * keys_val[0]
                self.car_state[2] = -3.14 * self.state_start[2] * keys_val[2]
            # print(*self.car_state)
            self.car.mecanum_wheel(*self.car_state)
            time.sleep(0.05)

    def image_process(self):
        name_length = 4
        
        if os.path.exists(self.dir) is not True:
            os.mkdir(self.dir)
        json_name = "data.json"
        self.json_path = os.path.join(self.dir, json_name)
        
        while not self.exit_flag:
            if self.run_flag:
                data_dict = dict()
                # 获取图片
                image = self.cap.read()

                img_name = (name_length - len(str(self.index)))*'0' + str(self.index) +'.jpg'
                data_dict["img_path"] = img_name
                image_path = os.path.join(self.dir, img_name)
                cv2.imwrite(image_path, image)
                data_dict["state"] = self.car_state.copy()
                self.json_data.append(data_dict)
                # print(img_name)
                logger.info("image:{}".format(img_name))
                self.index += 1
                if self.index%10 == 0:
                    self.save_json(self.json_data, self.json_path)
                if self.index%20 == 0:
                    self.display.show("image:{}\n".format(self.index))
                time.sleep(0.05)
            

    def del_last3s(self):
        self.car.beep()
        for i in range(30):
            # 尝试使用 pop() 方法移除最后一个元素
            try:
                data = self.json_data.pop()
                path = os.path.join(self.dir, data['img_path'])
                os.remove(path)
                self.index -= 1
                
            except IndexError:
                # 输出: 列表为空，无法使用 pop() 方法
                logger.info("image data zero now")
                return
        self.display.show("image:{}\n".format(self.index))

    # 写一个网页，具有遥控控件的界面
    def controller_html(self):
        # 具体实现
        pass


    def restart(self):
        self.car.beep()
        time.sleep(0.4)
        self.car.beep()
        
        # 使用rm命令删除*.jpg
        # subprocess.run(["rm", "-rf", self.dir + "/*.jpg"])
        # 删除文件
        subprocess.run(["find", self.dir, "-name", "*.jpg", "-delete"])
        self.json_data = []
        self.index = 0
        self.display.show("image:{}\n".format(self.index))

    @staticmethod
    def save_json(json_data, path):
        with open(path, 'w') as fp:
            json.dump(json_data, fp)

    def get_state(self):
        return self.car_state

    def close(self):
        self.save_json(self.json_data, self.json_path)
        self.display.show("control end!\nimage:{}\n".format(self.index))
        self.exit_flag = True
        self.img_thread.join()
        self.cap.close()

        for i in range(3):
            self.car.beep()
            time.sleep(0.4)


if __name__ == "__main__":
    remote_car = RemoteControlCar()

    # while True:
        # time.sleep(0.5)
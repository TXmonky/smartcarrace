# --*-- coding: utf-8 --*--
# infer_back_end.py

import zmq, json, cv2
import numpy as np
from threading import Thread
from infer_front import ClintInterface
import time


class InferServer:
    def __init__(self):
        # 导入推理客户端的配置,每个推理服务一个线程
        configs = ClintInterface.configs
        
        self.flag_infer_initok = False
    
        self.flag_end = False
        # 开启对应的线程和服务
        self.threads_list = []
        self.server_dict = {}
        
        # self.lane_server = self.get_server(5001)
        for conf in configs:
            # 创建获取zmq服务
            server = self.get_server(conf['port'])
            print('server: ',server)
            # 完成端口号和名称匹配
            self.server_dict[conf['name']] = server
            print('serber_dict: ',self.server_dict)

            # 创建线程
            # 带参数线程，此处参数为各种推理模型，系统调度
            thread_tmp = Thread(target=self.process_demo, args=(conf['name'],))
            # thread_tmp = Thread(target=self.lane_process)
            thread_tmp.daemon = True
            thread_tmp.start()
            # 添加进程
            self.threads_list.append(thread_tmp)
        
        from paddle_jetson import YoloeInfer, LaneInfer, OCRReco, HummanAtrr, MotHuman
        # 创建推理模型
        self.infer_dict = {}
        for conf in configs:
            # 创建推理模型, eval字符转为对象，infer_tmp就是一个实例
            #     configs = [
            # {'name':'lane', 'infer_type': 'LaneInfer', 'params': [], 'port':5001, 'img_size':[128, 128]},
            # {'name':'task', 'infer_type': 'YoloeInfer', 'params': ['task_model3'], 'port':5002, 'img_size':[416, 416]},
            # {'name':'front', 'infer_type':'YoloeInfer', 'params': ['front_model2'], 'port':5003, 'img_size':[416, 416]},
            # {'name':'ocr', 'infer_type':'OCRReco', 'params': [], 'port':5004,'img_size':None},
            # {'name':'humattr', 'infer_type':'HummanAtrr', 'params': [], 'port':5005, 'img_size':None},
            # {'name':'mot', 'infer_type':'MotHuman', 'params': [], 'port':5006, 'img_size':None}
            # ]
            infer_tmp = eval(conf['infer_type'])(*conf['params'])
            # 完成名字和对应推理模型的匹配
            self.infer_dict[conf['name']] = infer_tmp
            
        # 创建推理模型
        # self.lane_infer = LaneInfer()
        # self.front_infer = YoloInfer("front_model2") # "trt_fp32")
        # self.task_infer = YoloInfer("task_model3") # "trt_fp32")
        # self.ocr_infer = OCRReco()
        # self.humattr_infer = HummanAtrr()
        # self.mot_infer = MotHuman()
        
        # 新建一个空白图片，用于预先图片推理
        img = np.zeros((240, 240, 3), np.uint8)
        # 预加载推理几张图片，刚开始推理时速度慢，会有卡顿，加载多个模型进内存
        for i in range(3):
            for conf in configs:
                infer_tmp = self.infer_dict[conf['name']]
                infer_tmp(img)
        print("infer init ok")

        self.flag_infer_initok = True

    # 响应
    def get_server(self, port):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://127.0.0.1:{port}")
        return socket
    
    # 完成端口号、推理类型，端口的匹配
    def process_demo(self, name):
        
        print(time.strftime("%Y-%m-%d %H:%M:%S"), "{} process start".format(name))
        # 获得
        server = self.server_dict[name]
        # lambda定义推理函数，含有归一化处理参数为True, 此处定义方便后续调用
        func = lambda x: self.infer_dict[name](x, True)

        while True:
            if self.flag_end:
                return
            # 此处等待，如果没有收到信息
            response = server.recv()

            head = response[:5]
            res = []
            if head == b"ATATA":
                if self.flag_infer_initok:
                    res = True
                else:
                    res = False
            elif head == b"image":
                # 把bytes转为jpg格式
                img = cv2.imdecode(np.frombuffer(response[5:], dtype=np.uint8), 1)
                if self.flag_infer_initok:
                    # res = self.lane_infer(img).tolist()
                    # lambda函数
                    res = func(img)
                    
            json_data = json.dumps(res)
            json_data = bytes(json_data, encoding='utf-8')
            server.send(json_data)

    def close(self):
        print("closing...")
        self.flag_end = True
        for thread in self.threads_list:
            # 等待结束
            thread.join()
            # 关闭
            thread.close()


def main():
    infer_back = InferServer()

    while True:
        try:
            time.sleep(1)
        except Exception as e:
            print(e)
            break
    time.sleep(0.1)
    infer_back.close()

if __name__ == "__main__":
    main()

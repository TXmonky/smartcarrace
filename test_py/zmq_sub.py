import zmq
import sys
import cv2
import numpy as np
import time
 
 
def main():
    context = zmq.Context()
    print("连接服务器...")
    skt = context.socket(zmq.SUB)
    skt.connect("tcp://127.0.0.1:7889")
 
    skt.setsockopt(zmq.SUBSCRIBE, 'image'.encode('utf-8'))  # 只订阅 服务器时间 开头的消息
    last_time = time.time()
    while True:
        response = skt.recv()
        head = response[:5]
        # print(head)
        image_jpg_bytes = response[5:]
        # 把bytes转为jpg格式
        image = cv2.imdecode(np.frombuffer(image_jpg_bytes, dtype=np.uint8), 1)
        fps = 1 / (time.time() - last_time)
        last_time = time.time()
        print("fps:", fps)
        # cv2.imshow('image', image)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        # print("接收到服务器数据: ", response)
    sys.exit()
 
 
if __name__ == '__main__':
    main()
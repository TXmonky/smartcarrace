import zmq
import sys
import time
from camera import Camera
import cv2
 
cap = Camera(1)
def main():
    context = zmq.Context()
    skt = context.socket(zmq.PUB)
    skt.bind("tcp://127.0.0.1:7889")
 
    while True:
        # try:
            time.sleep(0.05)
            img = cap.read()
            # img转jpeg格式
            img = cv2.imencode('.jpg', img)[1].tobytes()
            data = bytes('image', encoding='utf-8') + img
            # # 发送数据
            # data = bytes('服务器时间:@' + time.strftime("%Y-%m-%d %H:%M:%S %a"), encoding='utf-8')
 
            skt.send(data)
 
            print('发送消息')
        # except Exception as e:
        #     print('异常:', e)
        #     sys.exit()
 
 
if __name__ == '__main__':
    main()
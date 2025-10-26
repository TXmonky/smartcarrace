# 添加上层目录到sys
import os, sys
parent_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, parent_path)
import time

from vehicle import Beep
if __name__ == '__main__':
    beep = Beep()
    beep.set(100, 10)
    time.sleep(0.5)
    beep.set(10, 10)
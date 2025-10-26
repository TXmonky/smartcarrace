# -*- coding:utf-8 -*-
# tools_class.py
from simple_pid import PID
import yaml

# 读取yaml文件
def get_yaml(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.load(f, Loader=yaml.FullLoader)
    except Exception as e:
        print('{} not found'.format(path))
        print(e)
        return None

# pid参数进行封装
class PidWrap:

    def __init__(self, kp, ki, kd, setpoint=0, output_limits=1):
        self.pid_t = PID(kp, ki, kd, setpoint, output_limits=(0-output_limits, output_limits))
        
    def set_target(self, target):
        self.pid_t.setpoint = target

    def set(self, kp, ki, kd):
        self.pid_t.kp = kp
        self.pid_t.ki = ki
        self.pid_t.kd = kd

    def get(self, val_in):
        return self.pid_t(val_in)


# 次数记录进行简单滤波，到达一定次数为真
class CountRecord:
    def __init__(self, stop_count=2) -> None:
        self.last_record = None
        self.count = 0
        self.stop_cout = stop_count

    def get_count(self, val):
        try:
            if val == self.last_record:
                self.count += 1
            else:
                self.count=0
                self.last_record = val
            return self.count
        except Exception as e:
            print(e)
    
    def __call__(self, val):
        self.get_count(val)
        # print("count:{}, val:{}".format(self.count, val))
        # 检测val的类型
        if self.count >= self.stop_cout:
            if type(val) == bool:
                return val
            return True
        else:
            return False

class IndexWrap:
    def __init__(self, num, circle=False) -> None:
        self.index = 0
        self.max = num-1
        self.min = 0
        # 巡航设置
        self.circle = circle
    
    def next(self):
        self.index += 1
        if self.index > self.max:
            if self.circle:
                self.index = self.min
            else:
                self.index = self.max
        return self.index

    def before(self):
        self.index -= 1
        if self.index < self.min:
            if self.circle:
                self.index = self.max
            else:
                self.index = self.min
        return self.index

    def get_index(self):
        return self.index
    
    def __call__(self):
        return self.index

    def __str__(self) -> str:
        return 'min:{}, max:{}, index:{}'.format(self.min, self.max, self.index)
    
    def __repr__(self) -> str:
        return 'min:{}, max:{}, index:{}'.format(self.min, self.max, self.index)
    
def count_test():
    import numpy as np
    count1 = CountRecord(30)
    count2 = CountRecord(20)
    cdd = np.array([0.1,0.1, .1, .1, .1]).tolist()
    while True:
        flag1 = count1(cdd[0]<0.2)
        flag2 = count2(cdd[1]<0.2)
        if flag1 and flag2:
            break
        # print("cout1:",count1.count)
        # print("cout2:", count2.count)

    print(count1.count, count2.count)

def compare_test():
    a = {"test": "a"}
    b = {"test": "a"}
    print(a==b)

if __name__ == "__main__":
    # pid_name = PidWrap
    # name = pid_name.__name__.lower()
    # print(name)
    # count_test()
    compare_test()

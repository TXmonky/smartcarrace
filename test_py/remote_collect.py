import os, sys
# 添加上两层目录
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))) 

from collect_wrap import RemoteControlCar
# 后台检测关闭程序
rc = RemoteControlCar()

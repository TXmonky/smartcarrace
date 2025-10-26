
# t = None
# assert isinstance(t, int) or isinstance(t, float), "Wrong type"
# assert False, "True"
# rc = RemoteControlCar()
from typing import List, Dict, Tuple

class Det:
    def __init__(self, name:str):
        self.name = name
    def pp(self):
        print(self.name)

    def __str__(self):
        return "name:"+self.name
    
def get_test()->List[Det]:
    ret = []
    for i in range(5):
        ret.append(Det(str(i)))
    return ret

for ord in get_test():
    ord.pp()
# print(get_test("test"))

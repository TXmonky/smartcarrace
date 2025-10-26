# -*- coding:utf-8 -*-

import numpy as np

a = np.array([1, 2, 4, -5])
print(a.tobytes())
b = np.array([3, 4])
# a b合并
a = np.concatenate((a, a))
print(a)

print((np.abs(a) < 6).all())

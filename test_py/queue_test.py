from collections import deque
import collections

 
# 定义队列的最大容量为5
max_size = 5
q = deque(maxlen=max_size)

res = q.popleft()
print(res)
# 向队列中添加元素
for i in range(10):
    q.append(i)
    # print(q)
print(q)
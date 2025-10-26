import struct

# 打包数据
data = [123, b'abc', 3.14]
packed_data = struct.pack('i 3s f', *data)

#
a = 'b' * 101
data_format = '<i3sf'+a
print('len: ',struct.calcsize(data_format))
# # 解包数据
# unpacked_data = struct.unpack('i 3s f', packed_data)
# print(unpacked_data)  # 输出：(123, b'abc', 3.140000104904175)
# # 和b‘’一样
# print(len('ab'.encode('ascii')))
# b = b'ab'
# for i in range(2):
#     print(b[i])

# a = b'\x77\x78'
# print(f'{a}')
dev_map = { 1:["Motors", '<bbbbbbbb' ],
            2:["Motor", '<bbbb'],
            3:["Motor2", '<bb']
            }
print(dev_map[1])

print('b' * 101)
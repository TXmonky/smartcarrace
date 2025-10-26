from ernie_bot import ErnieBotWrap

def ernie_func():
    ernie = ErnieBotWrap()
    while True:
        print("用户")
        str_tmp = input("输入:")
        if len(str_tmp)<1:
            continue
        # Create a chat completion
        print("文心一言")
        _, str_res = ernie.get_res(str_tmp)
        print("输出:",str_res)

if __name__ == "__main__":
    ernie_func()
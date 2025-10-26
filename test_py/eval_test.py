# def eval_test():
#     print('ok')

class EvalTest:
    def __init__(self):
        eval("self.eval_test()")

    def eval_test(self):
        print('ok')

class JsonTest:
    def __init__(self):
        list1 = ['a', 'b', 'c']
        list2 = ('a', 'b', 'c')
        self.print_test(*list1)
        self.print_test(*list2)
        import json
        json_obj = json.dumps(list1)
        print(json_obj)
    def print_test(self, a, b, c):
        print(a, b, c)

if __name__ == "__main__":
    EvalTest()
    JsonTest()
import argparse
def arg_test():
    parser = argparse.ArgumentParser()
    parser.add_argument('--foo', help='foo help')
    args = parser.parse_args()
    print(args.foo)

def list_test():
    list1= ['a', 'b', 'c']
    a, b, c = list1
    print(a, b,c )

if __name__ == "__main__":
    # arg_test()
    list_test()

    def add(a,b,c):
        print('a: ',a,' b: ',b,' c: ',c)

    dic = {'b':2,'a':1,'c':3}
    add(**dic)


    def add1(**kwargs):
        print(kwargs)
    add1(**dic)


    def add3(kwargs):
        print(kwargs)
    add3(dic)

    def show(a):
        print(a)
    a = [1,2,3]
    show(a)

    def add4(*kwargs,a):
        print('add4: ',kwargs)
    add4((a,1),a=1)

    def add2(a:int, b:int, c:int):
        print('a: ', a, ' b: ', b, ' c: ', c)
    add2(2.2,1,2)


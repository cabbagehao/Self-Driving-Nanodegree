"""
Given the starting point of any `x` gradient descent
should be able to find the minimum value of x for the
cost function `f` defined below.
"""
import random
from gd import gradient_descent_update


def f(x):
    """
    Quadratic function.

    It's easy to see the minimum value of the function
    is 5 when is x=0.
    """
    return x**2 + 5


def df(x):
    """
    Derivative of `f` with respect to `x`.
    """
    return 2*x


# Random number better 0 and 10,000. Feel free to set x whatever you like.
x = random.randint(0, 10000)
learning_rate = 0.1
epochs = 100

for i in range(epochs+1):
    cost = f(x)
    gradx = df(x)
    if i % 10 == 0:
        print("EPOCH {}: Cost = {:.3f}, x = {:.3f}".format(i, cost, gradx))
    x = gradient_descent_update(x, gradx, learning_rate)
"""
可以看到对1元函数来说直接对x求导，使用梯度下降结果也是正确的。
output:
    EPOCH 0: Cost = 35295486.000, x = 11882.000
    EPOCH 10: Cost = 406934.191, x = 1275.820
    EPOCH 20: Cost = 4696.574, x = 136.990
    EPOCH 30: Cost = 59.090, x = 14.709
    EPOCH 40: Cost = 5.624, x = 1.579
    EPOCH 50: Cost = 5.007, x = 0.170
    EPOCH 60: Cost = 5.000, x = 0.018
    EPOCH 70: Cost = 5.000, x = 0.002
    EPOCH 80: Cost = 5.000, x = 0.000
    EPOCH 90: Cost = 5.000, x = 0.000
    EPOCH 100: Cost = 5.000, x = 0.000
"""
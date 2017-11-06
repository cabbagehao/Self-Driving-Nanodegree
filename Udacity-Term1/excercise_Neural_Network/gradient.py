#*-coding:utf-8-*
"""
    梯度下降： 
        我们有一个关于各个权重wi的函数，想找出这个函数值下降最快的方法。
        没有直接求下降最快的方法，但有求上升最快的方法： 梯度。
        某一点的梯度就是曲面函数在该点最大那个方向导数
        计算目标函数的梯度，也就是计算函数对于所有参数的偏导数（这里的参数就是wi)

    E = 0.5*(y-y_hat)**2
    y_hat = f(h)
    h = ∑wi*xi
    对E求wi的偏导即可推出del_w的公式
"""
import numpy as np

def sigmoid(x):
    """
    Calculate sigmoid
    """
    return 1/(1+np.exp(-x))

# (1/f(x))' = -f(x)' / f(x)**2
def sigmoid_prime(x):
    return sigmoid(x) * (1 - sigmoid(x))

learnrate = 0.5
x = np.array([1, 2])
y = np.array(0.5)

# Initial weights
w = np.array([0.5, -0.5])

# Calculate one gradient descent step for each weight
# TODO: Calculate output of neural network
nn_output = sigmoid(np.dot(x, w))


# TODO: Calculate error of neural network
error = y - nn_output

error_term = error * sigmoid_prime(np.dot(x,w))

# TODO: Calculate change in weights
del_w = learnrate * error_term * x

print('Neural Network output:')
print(nn_output)
print('Amount of Error:')
print(error)
print('Change in Weights:')
print(del_w)

# wi*xi结果为-0.5. -0.5在sigmod函数上值为0.38左右，所以error为0.12左右。
# error实际上由error函数控制，所以我们可以改变wi的值改变error。 于是通过梯度下降调整wi.
# 从下面的循环可以看到，循环10次后error已经缩小到0.028. 确实梯度下降是有用的。
# 可有一个问题，对于给定的x和y，是否只有唯一的一组值使得error最小，所以用梯度往那里靠？
#   由于sigmod是单调函数，如果y唯一则wi*xi的值也唯一。 而xi固定，wi在变换
#   也就是说取不同的wi对x向量进行组合，最后得到等于/接近y的值。 那这个组合肯定不是唯一的，那梯度下降只是随便找了一组最优解？    
#   那不同的最优解差异很大的话对神经网络有啥影响呢？ 一组权值代表了这个网络关注的特性，另一组权值关注的不同特性？ 
#   如果这样的话就有点奇怪了，大家都用同一个框架，那不是都只学到了同样的特性同样的权值，但同一个事物可能有多个特性能准确识别，如何找出所有特性呢？ TODO

# A:对于只给定一个x和y确实有多个解。 但实际上我们是用大量数据去训练，力求找到一组最优解同时满足所有训练样本。 因此几乎不存在1个解能同时满足，只可能存在多个
# 较优的解。 也就是局部最优的情况是存在的，但避免陷入局部最优也有一些方法可以使用。

"""
    Neural Network output:
    0.377540668798
    Amount of Error:
    0.122459331202
    Change in Weights:
    [ 0.0143892  0.0287784]
"""

print('Aplly del_w, once again.')
for i in range(10):
    nn_output = sigmoid(np.dot(x, w))
    error = y - nn_output
    error_term = error * sigmoid_prime(np.dot(x,w))
    del_w = learnrate * error_term * x
    w += del_w
    print('time: ', i)
    print('Neural Network output: ', nn_output)
    print('Amount of Error: ', error)
    print('Change in Weights: ', del_w)

"""
Aplly del_w, once again.
time:  0
Neural Network output:  0.377540668798
Amount of Error:  0.122459331202
Change in Weights:  [ 0.0143892  0.0287784]
time:  1
Neural Network output:  0.394591112475
Amount of Error:  0.105408887525
Change in Weights:  [ 0.01259051  0.02518102]
time:  2
Neural Network output:  0.409725210676
Amount of Error:  0.0902747893235
Change in Weights:  [ 0.0109165  0.021833 ]
time:  3
Neural Network output:  0.422988072384
Amount of Error:  0.0770119276155
Change in Weights:  [ 0.00939812  0.01879624]
time:  4
Neural Network output:  0.434496556514
Amount of Error:  0.065503443486
Change in Weights:  [ 0.0080474   0.01609481]
time:  5
Neural Network output:  0.444407950655
Amount of Error:  0.0555920493448
Change in Weights:  [ 0.0068631   0.01372621]
time:  6
Neural Network output:  0.45289613814
Amount of Error:  0.0471038618604
Change in Weights:  [ 0.00583573  0.01167145]
time:  7
Neural Network output:  0.4601354916
Amount of Error:  0.0398645084003
Change in Weights:  [ 0.00495139  0.00990278]
time:  8
Neural Network output:  0.466291143686
Amount of Error:  0.0337088563144
Change in Weights:  [ 0.00419446  0.00838891]
time:  9
Neural Network output:  0.47151388342
Amount of Error:  0.0284861165804
Change in Weights:  [ 0.00354921  0.00709841]
"""
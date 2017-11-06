"""
This script builds and runs a graph with miniflow.

There is no need to change anything to solve this quiz!

However, feel free to play with the network! Can you also
build a network that solves the equation below?

(x + y) + y
"""

from miniflow import *

X, W, b = Input(), Input(), Input()

## Test Add
# f = Add(x, y)
# feed_dict = {x: 10, y: 5}

## Test Sigmoid
# f = Linear(X, W, b)
# g = Sigmoid(f)

# X_ = np.array([[-1., -2.], [-1, -2]])
# W_ = np.array([[2., -3], [2., -3]])
# b_ = np.array([-3., -5])

# feed_dict = {X: X_, W: W_, b: b_}

## Test cost
# y, a = Input(), Input()
# cost = MSE(y, a)

# y_ = np.array([1, 2, 3])
# a_ = np.array([4.5, 5, 10])

# feed_dict = {y: y_, a: a_}

## Test backpaprogation
X, W, b = Input(), Input(), Input()
y = Input()
f = Linear(X, W, b)
a = Sigmoid(f)
cost = MSE(y, a)

X_ = np.array([[-1., -2.], [-1, -2]])
W_ = np.array([[2.], [3.]])
b_ = np.array([-3.])
y_ = np.array([1, 2])

feed_dict = {
    X: X_,
    y: y_,
    W: W_,
    b: b_,
}
graph = topological_sort(feed_dict)
output = forward_and_backward(cost, graph)
print(output)
gradients = [t.gradients[t] for t in [X, y, W, b]]
print(gradients)
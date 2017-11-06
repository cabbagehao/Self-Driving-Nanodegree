import numpy as np

class Node(object):
    def __init__(self, inbound_nodes=[]):
        # Nodes from which this Node receives values
        self.inbound_nodes = inbound_nodes
        # Nodes to which this Node passes values
        self.outbound_nodes = []
        # A calculated value
        self.value = None
        # Their values are the partials of this node with respect to that input
        self.gradients = {}
        # Add this node as an outbound node on its inputs.
        for n in self.inbound_nodes:
            n.outbound_nodes.append(self)

    # These will be implemented in a subclass.
    def forward(self):
        """
        Forward propagation.

        Compute the output value based on `inbound_nodes` and
        store the result in self.value.
        """
        # raise NotImplemented
        raise NotImplementedError

    def backward(self):
        """
        Every node that uses this class as a base class will
        need to define its own `backward` method.
        """
        raise NotImplementedError

class Input(Node):
    def __init__(self):
        # an Input node has no inbound nodes,
        # so no need to pass anything to the Node instantiator
        Node.__init__(self)

    # NOTE: Input node is the only node that may
    # receive its value as an argument to forward().
    #
    # All other node implementations should calculate their
    # values from the value of previous nodes, using
    # self.inbound_nodes
    #
    # Example:
    # val0 = self.inbound_nodes[0].value
    def forward(self, value=None):
        if value is not None:
            self.value = value

    def backward(self):
        # An Input node has no inputs so the gradient (derivative)
        # is zero.
        # The key, `self`, is reference to this object.
        self.gradients = {self: 0}
        # Weights and bias may be inputs, so you need to sum
        # the gradient from output gradients.
        for n in self.outbound_nodes:
            grad_cost = n.gradients[self]
            self.gradients[self] += grad_cost * 1

class Add(Node):
    def __init__(self, x, y):
        # You could access `x` and `y` in forward with
        # self.inbound_nodes[0] (`x`) and self.inbound_nodes[1] (`y`)
        # 调用父类设置inbound_nodes.
        Node.__init__(self, [x, y])
        self.value = 0

    def forward(self):
        """
        """
        for node in self.inbound_nodes:
            self.value += node.value
        # x_value = self.inbound_nodes[0].value
        # y_value = self.inbound_nodes[1].value
        # self.value = x_value + y_value
    def backward(self):
        pass

class Mul(Node):
    def __init__(self, *inputs):
        Node.__init__(self, inputs)
        self.value = 1
        
    def forward(self):
        for node in self.inbound_nodes:
            self.value *= node.value  
    def backward(self):
        pass

class Linear(Node):
    def __init__(self, inputs, weights, bias):
        Node.__init__(self, [inputs, weights, bias])

        # NOTE: The weights and bias properties here are not
        # numbers, but rather references to other nodes.
        # The weight and bias values are stored within the
        # respective nodes.

    def forward(self):
        """
        """
        input_values = self.inbound_nodes[0].value
        weight_values = self.inbound_nodes[1].value
        bia_value = self.inbound_nodes[2].value
        
        self.value = np.dot(input_values, weight_values) + bia_value

    def backward(self):
        """
        Calculates the gradient based on the output values.
        """
        # Initialize a partial for each of the inbound_nodes.
        self.gradients = {n: np.zeros_like(n.value) for n in self.inbound_nodes}
        # Cycle through the outputs. The gradient will change depending
        # on each output, so the gradients are summed over all outputs.
        for n in self.outbound_nodes:
            # Get the partial of the cost with respect to this node.
            grad_cost = n.gradients[self]
            print(grad_cost)
            # Set the partial of the loss with respect to this node's inputs.
            self.gradients[self.inbound_nodes[0]] += np.dot(grad_cost, self.inbound_nodes[1].value.T)
            # Set the partial of the loss with respect to this node's weights.
            self.gradients[self.inbound_nodes[1]] += np.dot(self.inbound_nodes[0].value.T, grad_cost)
            # Set the partial of the loss with respect to this node's bias.
            self.gradients[self.inbound_nodes[2]] += np.sum(grad_cost, axis=0, keepdims=False)

class Sigmoid(Node):
    def __init__(self, node):
        Node.__init__(self, [node])

    def _sigmoid(self, x):
        """
        This method is separate from `forward` because it
        will be used later with `backward` as well.
        """
        return 1.0 / (1 + np.exp(-x))
        
    def forward(self):
        """
        """
        # This is a dummy value to prevent numpy errors
        # if you test without changing this method.
        
        input_value = self.inbound_nodes[0].value
        self.value = self._sigmoid(input_value) 

    def backward(self):
        """
        Calculates the gradient using the derivative of
        the sigmoid function.
        """
        # Initialize the gradients to 0.
        self.gradients = {n: np.zeros_like(n.value) for n in self.inbound_nodes}
        # Sum the derivative with respect to the input over all the outputs.
        for n in self.outbound_nodes:
            grad_cost = n.gradients[self]
            sigmoid = self.value
            self.gradients[self.inbound_nodes[0]] += sigmoid * (1 - sigmoid) * grad_cost

class MSE(Node):
    def __init__(self, y, a):
        """
        The mean squared error cost function.
        Should be used as the last node for a network.
        """
        # Call the base class' constructor.
        Node.__init__(self, [y, a])

    def forward(self):
        """
        Calculates the mean squared error.
        """
        # NOTE: We reshape these to avoid possible matrix/vector broadcast errors.
        # 
        # For example, if we subtract an array of shape (3,) from an array of shape
        # (3,1) we get an array of shape(3,3) as the result when we want
        # an array of shape (3,1) instead.
        #
        # Making both arrays (3,1) insures the result is (3,1) and does
        # an elementwise subtraction as expected.
        y = self.inbound_nodes[0].value.reshape(-1, 1)
        a = self.inbound_nodes[1].value.reshape(-1, 1)

        self.m = self.inbound_nodes[0].value.shape[0]
        # Save the computed output for backward.
        self.diff = y - a
        self.value = np.mean(self.diff**2)

    def backward(self):
        """
        Calculates the gradient of the cost.

        This is the final node of the network so outbound nodes
        are not a concern.
        """
        # 保存这个节点前一层的所有gradient. 前一层计算时过来取。
        self.gradients[self.inbound_nodes[0]] = (2 / self.m) * self.diff
        self.gradients[self.inbound_nodes[1]] = (-2 / self.m) * self.diff

def topological_sort(feed_dict):
    """
    Sort generic nodes in topological order using Kahn's Algorithm.

    `feed_dict`: A dictionary where the key is a `Input` node and the value is the respective value feed to that node.

    Returns a list of sorted nodes.
    """

    input_nodes = [n for n in feed_dict.keys()]

    # 将所有的node都放到G里去，包含它的out 和 in节点，类似于邻接表。
    G = {}
    nodes = [n for n in input_nodes]
    while len(nodes) > 0:
        n = nodes.pop(0)
        if n not in G:
            G[n] = {'in': set(), 'out': set()}
        for m in n.outbound_nodes:
            if m not in G:
                G[m] = {'in': set(), 'out': set()}
            G[n]['out'].add(m)
            G[m]['in'].add(n)
            nodes.append(m)
    # 初始化时，S = [x, y] G : {x, y, f}
    # 但这里有问题，如果y在x前面，y会被先输出出来。 因此需要优化为先输出所有input。
    L = []
    S = set(input_nodes)
    while len(S) > 0:
        n = S.pop()

        # 用value替换input的placeholder.
        if isinstance(n, Input):
            n.value = feed_dict[n]

        L.append(n)
        for m in n.outbound_nodes:
            G[n]['out'].remove(m)
            G[m]['in'].remove(n)
            # if no other incoming edges add to S
            # 如果节点没有了其他输入节点，表示可以计算了，加入到S.
            if len(G[m]['in']) == 0:
                S.add(m)
    return L


def forward_and_backward(sorted_nodes):
    """
    Performs a forward pass through a list of sorted nodes.

    Arguments:

        # `output_node`: A node in the graph, should be the output node (have no outgoing edges).
        `sorted_nodes`: A topologically sorted list of nodes.

    Returns the output Node's value
    """
    # 顺序调用每个node的forward函数，执行计算,得到output节点的值。
    for n in sorted_nodes:
        n.forward()

    # Backward pass
    # see: https://docs.python.org/2.3/whatsnew/section-slices.html
    for n in sorted_nodes[::-1]:
        n.backward()

    # return output_node.value

def sgd_update(trainables, learning_rate=1e-2):
    """
    Updates the value of each trainable with SGD.

    Arguments:

        `trainables`: A list of `Input` Nodes representing weights/biases.
        `learning_rate`: The learning rate.
    """
    # Input 都会保存 gradients[t]，weights和biases都是Input。也只有Input类需要更新。
    # 每个节点都保存自己上一层每个节点的grad，反向传播到某节点时，它会从自己的output_bounds
    # 里获取自己贡献给其的grad,然后存入gradients[self].  该节点所有输出节点上保存grad的和为它的grad.
    for t in trainables:
        partial = t.gradients[t]
        t.value -= learning_rate * partial

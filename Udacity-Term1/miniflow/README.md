**实现了一个类似tensorflow的miniflow，并测试了波士顿房价测试集，训练1000次后loss在4.48。**  

#### 定义了几类Node：  
    * 1. Input  
    * 2. Add / Mul  
    * 3. Linear   
    * 4. Sigmod  
    * 5. MSE  
#### 和几个核心函数：
    1. forward_and_backward：  
        执行正向运算得到输出和反向传播梯度更新权值  
    2. sgd_update：  
        执行权值和bias的更新  
    3. topological_sort：  
        确定计算图，返回排好执行顺序的node列表  

#### 运行过程：
    1. 和tensorflow类似，定义x, y, w, b等node，用placeholder表示，并确定每一层node个数。  
    2. 定义相邻2层间的运算，比如Linear， Sigmod等。    
    3. 用定义好的node填充feed_dict，放入`topological_sort()`得到计算图graph。    
    4. 开始循环运行epochs次：    
        执行设定的steps_per_epoch次：  
            每一次X,y从resample函数中获取一部分数据  
            执行forward_and_backward,计算整个图并执行反向传播函数。  
            执行sgd_update，更新权值和biases  
        计算这次循环的平均loss.  


#### Node设计：  

self.gradients:  
        每一个节点都有这个字典，用来存放上一层每个节点对自己的error梯度的贡献。   
        每个节点获取自己的error梯度时，从自己下一层所有的节点的那个字典里获取加起来就好了。  

**Node**:  
    >定义了一个基类`Node`，包含4个变量和2个函数：  
        >> `inbound_nodes`   前一层所有节点  
        >> `outbound_nodes`  后一层所有节点  
        >> `value`           当前节点计算的值  
        >> `gradients`       前一层每个节点对本节点error梯度的贡献  
        >> `forward()`       计算图运行时本节点所做的操作  
        >> `backward()`      反向传播error梯度时本节点所做的操作。 和forward一样，继承者都必须实现。  

**Input**:  
    非功能性节点，比如input，label， w, b等。  
        `self.gradients` : 存的自己给下一层所有节点造成的error梯度总和。    
        `forward()` ：  如果传给forward一个参数，它将其存为自己的value，暂时没用到。  
        `backward()`：  从所有下一层节点获取自己给其造成的error梯度，**累加**存入self.gradients[self],梯度更新时用。  
    
**Add/Mul**:  
    执行加法/乘法的节点，仅测试时会用这些简单节点。  
        `forward()`：   执行乘法或者加法，存入self.value.  
        `backward()`:   pass. 不需要。  

**Linear**:    
    执行线性运算的节点，类似于全连接层节点    
        `forward()`:    执行wx+b，存入self.value  
        `backward()`:   对每个输入输入节点都创建一个向量，放到self.gradients字典里  
                        从下一层 **每个节点** 得到自己对其造成的error梯度(grad_cost)：  
                            1. 更新x：  
                                    当前grad_cost * 上一层所有节点到本节点的权值向量w,得到上一层每个节点对本节点grad_cost的贡献  
                                    将上一步得到的结果加到该节点在self.gradients对应的项 (上一层所有节点的grad_cost都存储在本节点的self.gradients字典里）  
                            2. 更新w:  
                                    将上一层节点的value * grad_cost,得到每个w(权重）节点对本节点grad_cost的贡献  
                                    将上一步得到的结果加到w在self.gradients对应的项(每个w就是当前节点与上一层每个节点的边的权值)  
                            3. 更新b: 本节点储存的所有b的grad_cost都直接加上sum(grad_cost)  


**Sigmoid**:    
    执行Sigmod函数的节点    
        `forward()`:    将输入的值执行sigmod函数，存到self.value.    
        `backward()`:   对每个输入输入节点都创建一个向量，放到self.gradients字典里  
                        从下一层 **每个节点** 得到自己对其造成的grad_cost：  
                        计算error梯度 sigmoid * (1 - sigmoid) * grad_cost, 加到该节点在self.gradients对应的项实际上sigmoid节点就一个输入节点。  
  

**MSE**:    
    计算mean Error的节点    
        `forward()`:    计算y - a， 并对其平方并平均后存入self.value  
        `backward()`:   更新self.gradients中储存的label y和输出层的梯度 (2 / self.m) * self.diff 和 (-2 / self.m) * self.diff， 没太明白。  


#### TODO  
    对Linear和MSE的backward还需要进一步理解。    

        

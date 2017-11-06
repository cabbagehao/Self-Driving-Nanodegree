# CarND-LeNet-Lab
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![LeNet-5 Architecture](lenet.png)
Implement the LeNet-5 deep neural network model.  

P2的一个exercise. 熟悉使用LeNet网络去识别手写数字.  
从mnist里加载数据,并padding到LeNet使用的32*32*C  
### 网络结构:  
    conv1:      28*28*6  
    Activation  
    Pooling:    14*14*6  

    conv2:      10*10*16  
    Activation  
    Pooling:    5*5*16  

    Flatten: (使用tf.contrib.layers.flatten)  
    Fc:         120  
    Activation  

    Fc:         84  
    Activation  

    Fc:         10(Logits)  

### Dependencies
This lab requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

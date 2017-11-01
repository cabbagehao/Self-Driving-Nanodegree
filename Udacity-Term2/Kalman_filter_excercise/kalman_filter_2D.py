#*_coding:utf-8_*
from Matrix import matrix

"""
    2维状态下预测2个值： 坐标x和速度x'
    x = x + x'  预测点=旧点+速度
        则状态转换矩阵F可表示为[ [1, 1], [0, 1]]
    x' = x' 新速度=旧速度（假定不变）
        测量矩阵H可表示为[1, 0]
    P为预测的方差，因此这个值也需要进行更新。

    Prediction：
        x = F*x + u   (u为外部误差,代表的是一个均值为0的高斯分布)
        P = F*P*F.tranpose
    Measurment Update:
        y = z - H*x   (z是测量值，H为测量矩阵<即如何测量x的，见上>，y为偏差)
        S = H*P*H.tranpose + R (R为测量误差(厂商提供)， H为测量矩阵，这里相当于把预测方差P投影到测量维度上来)
        K = P*H.transpose*S.inverse （K称为卡尔曼系数）
        用这个系数对x和P进行更新。
        
    卡尔曼滤波及扩展卡尔曼滤波笔记：
        http://note.youdao.com/noteshare?id=78d19e07970fe7aef0285f430ae9ad80

"""

def update(x, P, z):
    y = z - H * x
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - K*H) * P
    
    return x, P
    
def predict(x, P, z):
    x = F * x + u
    P = F * P * F.transpose()
    
    return x, P
    
def kalman_filter(x, P):
    for n in range(len(measurements)):
        z = matrix([[measurements[n]]])
        # measurement update
        x,P = update(x, P, z)
        # prediction
        x,P = predict(x, P, z)
        print 'predict: ', x
    return x,P


measurements = [1., 2., 3.]


x = matrix([[0.], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

print kalman_filter(x, P)
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]

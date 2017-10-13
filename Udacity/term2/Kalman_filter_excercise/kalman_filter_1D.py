#*_coding:utf-8_*
"""
    一维卡尔曼滤波： 预测目标位置。
    刚开始预测不准，所以给一个很大的方差。 
    根据不断的测量及预测/更新，能够降低方差提高预测精度。
"""

def update(mean1, var1, mean2, var2):
    """
        update就是将实际测量结果和预测的结果进行合并，得到更精确的高斯误差
        这样新的高斯分布能够比之前的准确，以提高预测准确性。（因为测量误差是固定的，不需提高，而预测误差是变化的）
    """    
    new_mean = float(var2*mean1 + var1*mean2) / (var1 + var2)
    new_var = 1.0 / (1.0/var1 + 1.0/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    """
        当前位置有误差，移动操作也有误差。
        因此新的距离直接相加，两个误差也直接相加，作为新的预测结果。
    """
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]


measurements = [5., 6.5, 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.
mu = 0.
sig = 10000.

for i in range(len(measurements)):
    mu, sig = update(measurements[i], measurement_sig, mu, sig)
    mu, sig = predict(motion[i], motion_sig, mu, sig)
    print mu, sig

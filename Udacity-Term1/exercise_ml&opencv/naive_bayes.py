# 贝叶斯文档里的例子. 觉得非常具有启发性
# 机器学习本质上就是这样,训练样本就是一些列表,标签就是一个值. 
# 将其对应起来fit拟合一下,ok就可以用来预测了.  就像是放一堆点在哪里用曲线拟合一样,这个不过是多维的.

import numpy as np
from sklearn.naive_bayes import GaussianNB

X = np.array([[-1, -1], [-2, -1], [-3, -2], [1, 1], [2, 1], [3, 2]])
Y = np.array([1, 1, 1, 2, 2, 2])

clf = GaussianNB()
clf.fit(X, Y)
GaussianNB(priors=None)
print(clf.predict([[-0.8, -1]]))
# [1]

clf_pf = GaussianNB()
clf_pf.partial_fit(X, Y, np.unique(Y))
GaussianNB(priors=None)
print(clf_pf.predict([[-0.8, -1]]))
# [1]
#*-coding:utf-8-* 
# 将map划分为多个格子并预测每个格子的概率，实际上就是直方图滤波器histogram filter。
# 机器人初始状态不知道自己在哪里，所以5个位置概率都是0.2
# 通过2次的测量和move，就能比较准确地知道自己的位置了

# 第一次测量为green， 但是有3个green，所以3个green格子的概率稍微大些，但是仍不能确定位置
# 于是move了一格，再次测量发现为red，map里green和red前后挨着的地方只有一处，因此能定位到第二次测量时应该是在第二格
# 所以再次move后第3格概率最大。

# 直观上判断和运行结果一样。 但是机器人并不需要保存前面几次测量结果就能通过当前测量结果更新准确率，因此效率很高，这就是马尔可夫链的作用。
# 这个测量-更新过程类似于卡尔曼滤波的 预测-更新。 都是通过当前测量值来更新概率/权重。

#starting with a uniform prior distribution.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['green', 'red']
motions = [1,1]
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i-U) % len(p)]
        s = s + pOvershoot * p[(i-U-1) % len(p)]
        s = s + pUndershoot * p[(i-U+1) % len(p)]
        q.append(s)
    return q
#
# ADD CODE HERE
#
print p
for i in range(len(measurements)):
    Z = measurements[i%len(measurements)]
    p = sense(p, Z)
    print 'p sense: ', p 
    p = move(p, 1)
    print 'p move: ', p 

"""
[0.2, 0.2, 0.2, 0.2, 0.2]

# sense到了green，因此0 3 4位置概率高一些
p sense: [0.272727, 0.090909, 0.090909,  0.272727, 0.272727]

# move了一格，概率高的3个都往后一了一格
p move:  [0.272727, 0.254545, 0.109090,  0.109090, 0.254545]

# 又sense到了red，由于上一次sense到了green，因此在第二个的概率应该最高
p sense: [0.157894, 0.44210,  0.189473,  0.063157, 0.147368]

# move一格，概率最高的变为第3格，但由于move的不确定性，最高的概率降低了。
p move:  [0.13999,  0.185263, 0.3884210, 0.202105, 0.084210]
"""

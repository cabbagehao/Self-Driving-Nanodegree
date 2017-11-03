#*-coding:utf-8-*
# 机器人原本在第2个格子， 移动准确的概率是0.8，超前一格和差一格的概率都是0.1
# 由于没有参照物，移动一千步后每个格子的概率都是0.2了。
# 由于不确定性，移动实际上丢失了信息，使系统信息熵逐渐降到0

p=[0, 1, 0, 0, 0]
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


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
for i in range(1000):
    p = move(p, 1)
print p

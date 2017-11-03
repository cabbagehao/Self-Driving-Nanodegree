# *-coding:utf-8-*
# 粒子滤波器用来定位
# 生成particle：
#   就是当我的car不知道自己在map坐标系的哪个位置时，我就在map里随机生成一堆particle。
#   虽然是随机生成，但我还是知道这些particle我是知道它在map的哪个位置的。
#   这些particle有坐标位置，还有方向。 一个particle可以看做是一个car，因为它会移动，会测量（本练习里会测量）。
# 权重与重采样：
#   car和这些particle都测量到所有landmark的距离（本练习会测量，实际应用时particle不测量，使用car的测量结果就好了）
#       并对比particle和car到landmark的距离差异，给每个particle分配权重。
#   如果particle到所有landmark距离和car很接近，那么它的位置就很可能和car位置非常近，它的权重也相应更大。
#   然后有一个resample过程，概率小的particle自然被采样到的概率也更小，因此很可能被淘汰，留下概率大的，也
#   就是离car近的。
# 移动和循环：
#   经过一次重采样后，距离远的大多数被淘汰了，距离近的大部分能留下。可还是不能确定car的位置到底在哪些particle周围。  
#   于是car移动1次，并将所有particle也做同样的移动。再次测量到所有landmark的距离，然后重复上面的权重计算和重采样过程。
#   因为particle是有方向的，所以即使上一个周期位置和car很近的particle也move一次后可能离car就远了，上一次离car
#   远的particle这次离car又近了。   所以这样一轮resample后又有一些particle被淘汰，这次剩下的应该比上一个
#   周期采样过后的particle更能反映car的位置。

#   不断重复这个循环，最终只有和car的位置/方向都相近的particle才能留下。 
#   这些particle位置都知道，所以也就对car进行了定位。

# In this exercise, write a program that will
# run your previous code twice.
# Please only modify the indicated area below!

from math import *
import random

landmarks  = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError, 'X coordinate out of bound'
        if new_y < 0 or new_y >= world_size:
            raise ValueError, 'Y coordinate out of bound'
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        """
            sense即为robot的测量，每次move后都测量一遍所有landmark的距离，
            然后将每个particle也测量一遍到landmark的距离，通过这些距离的不同，
            确定出哪些particle离真实的robot很近，即定位到robot的位置。
        """
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError, 'Robot cant move backwards'         
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement):
        # measurement is the result of myrobot.sense().
        # calculates how likely a measurement should be
        # particle的概率权重计算：
        #   对每一个landmark：
        #       以myrobot到该landmark的测量值为均值，测量方差(恒定)为方差建立一个高斯函数
        #   对每一个particle：
        #       它到每一个landmark的距离作为x，放入该landmark对应的高斯函数则得到一个概率，
        #       表示了它离这个landmark与myrobot离这个landmark的距离偏差有多大。
        #       将particle在每个landmark高斯函数得到的概率相乘得到权重。 
        #       表示该particle到所有landmark距离与myrobot到所有landmark距离的偏差有多大，即该particle与myrobot偏差多大。
        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob
      
    def __repr__(self):
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))

def eval(r, p):
    sum = 0.0;
    for i in range(len(p)): # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err
    return sum / float(len(p))

#myrobot = robot()
#myrobot.set_noise(5.0, 0.1, 5.0)
#myrobot.set(30.0, 50.0, pi/2)
#myrobot = myrobot.move(-pi/2, 15.0)
#print myrobot.sense()
#myrobot = myrobot.move(-pi/2, 10.0)
#print myrobot.sense()

myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
Z = myrobot.sense()

N = 1000
p = []

def init(p):
    for i in range(N):
        x = robot()
        x.set_noise(0.05, 0.05, 5.0)
        p.append(x)

def move_it():
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    return p2

w = [0] * N
def sense():
    for i in range(N):
        w[i] = p[i].measurement_prob(Z)

def resample(w):
    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])
    return p3

init(p)
# range 2次看不到规律，range20次后就会发现打印出来的p方向和位置都是差不多的。
for i in range(20):
    p = move_it()
    sense()
    p = resample(w)

print len(p), #p

p = []
init(p)
myrobot = robot()
myrobot = myrobot.move(0.1, 5.0)
for i in range(20):
    # car move
    myrobot = myrobot.move(0.1, 5.0)
    
    # sense
    Z = myrobot.sense()

    # filter move 
    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.1, 5.0))
    p = p2

    # calc weights.
    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))   

    p = resample(w)
# 验证剩下的这些particle离car真实距离多近。（这里假设知道car的真实位置，而实际环境是不知道的）        
print eval(myrobot, p)
# 2.56022056477


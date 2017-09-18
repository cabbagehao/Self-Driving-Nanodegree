# -*- coding: UTF-8 -*-
# 左转、右转、直行分别有不同权重
# A-star  寻找最优权重路径
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.
## 失败方法：
    # 如何走
        # 在3个方向上遍历邻居。当前方向 i,下一个方向i+1, 上一个方向i+2
        # value记录节点被访问的方式（0,1,2,3)，代表进来的方向，
        #    下次访问这个点时就不能向其反方向走（反方向走原路）
        #    也不能按原方向走（重复走原路）
    # 如何结束：
        # 每条路径记录各自的cost：
        #    如果cost > 已成功cost_min，则该路径废弃
        #    如果路径成功cost < cost_min，则更新该值。 新路径为更优路径
        # 每条路径记录路径上各点的
## 失败原因：
    # 每条路径如何记录是个问题，广度优先不好判断当前节点属于哪条路径。

## 新方法：
# pop一个cost最小的节点，并认为所有pop出的点即为路径
    # 根据其direction和父节点，设置 父节点 的path为action。
# 3个方向遍历邻居
    # value记录访问过的节点的方向，下次访问时不能反方向走，也不能同一个方向走。
# 存在的问题：
    # 过程中会尝试多余的点，如果多余的点没有被正确的路径覆盖，则也会显示在path上。
    # 需要一个回溯机制，清除这些尝试失败的路径。

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------


def calc_action_id(foward_diff):
    
    if foward_diff == -1 or foward_diff == 3:
        action_id = 0
    elif foward_diff == 0:
        action_id = 1
    elif foward_diff == 1 or foward_diff == -3:
        action_id = 2
    else:
        print foward_diff, " It's a unexpected forward_diff"

    return action_id

def optimum_policy2D(grid,init,goal,cost):
    # use A* to find the best path.
    value = [ [ -1 for col in range(len(grid[0]))] for row in range(len(grid))]
    path = [ [ ' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    value[init[0]][init[1]] = 0
    # open_list [ g, x, y, direction, parent]
    open_list = [ [0, init[0], init[1], init[2], [init[0], init[1]]] ]
    while open_list != []:
        open_list.sort()
        open_list.reverse()
        next_point = open_list.pop()

        g = next_point[0]
        x = next_point[1]
        y = next_point[2]
        direction = next_point[3]
        parent = next_point[4]
        p_x, p_y = parent[0], parent[1]
        # 父节点和当前点direction区别，用来计算action
        foward_diff = direction - value[p_x][p_y]
        action_id = calc_action_id(foward_diff)
        
        path[p_x][p_y] = action_name[action_id]
        if [x, y] == goal:
            path[x][y] = '*'
            break
            

        for i in range(len(forward)):
            if abs(i - direction) == 2:  # 不能反方向
                continue
            a = forward[i]
            x2 = x + a[0]
            y2 = y + a[1]
            if x2 >= 0 and y2 >= 0 and x2 < len(grid) and y2 < len(grid[0]) and grid[x2][y2] == 0 and i!= value[x2][y2]: 
                # 找到邻居后，计算出action，用于计算这个邻居的cost
                action_id = calc_action_id(i - value[x][y])
                cost_new = g + cost[action_id]
                neighbor = [cost_new, x2, y2, i, [x, y]]
                value[x2][y2] = i
                open_list.append(neighbor)
                  
    return path



result = optimum_policy2D(grid,init,goal,cost)
for row in result:
    print row


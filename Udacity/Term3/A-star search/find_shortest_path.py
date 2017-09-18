# -*- coding: UTF-8 -*-
# 先广度搜索找到一条路径，然后反向查询到最短路径。
# search a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 0, 1 ], # go down
         [ 1, 0 ]] # go right

delta_name = ['^', '<',  '>', 'v']

def search(grid,init,goal,cost):
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1

    # parent： 每个点父点的位置（也可以取消这个变量，由parent_action可以推算出）
    # parent_action： 每个点父点的动作（因为当前点的动作还未计算出，因此由下一个点储存当前点的动作）
    parent = [[ ' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    parent_action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    
    expand = [[ ' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    expand[goal[0]][goal[1]] = '*'
    
    x = init[0]
    y = init[1]
    g = 0

    open = [[g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[1]
            y = next[2]
            g = next[0]
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            parent[x2][y2] = [x, y]     # save how to reach this point.
                            parent_action[x2][y2] = i   # save which action to reach this point

    # recall
    # 反向查找当前点的上一个点位置 和 action
    now = goal[:]
    while now != init:
        
        parent_point = parent[now[0]][now[1]]
        p_x = parent_point[0]
        p_y = parent_point[1]
        expand[p_x][p_y] = delta_name[ parent_action[now[0]][now[1]] ]
        now = parent_point
        
        
        
    expand[goal[0]][goal[1]] = '*'
    return expand # make sure you return the shortest path
    
path_list =  search(grid,init,goal,cost)    
for row in path_list:
    print row

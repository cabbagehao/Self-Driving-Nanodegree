# -*- coding: UTF-8 -*-
# Dynamic programming.
# User Instructions:
# 
# Write a function optimum_policy that returns
# a grid which shows the optimum policy for robot
# motion. This means there should be an optimum
# direction associated with each navigable cell from
# which the goal can be reached.
# 
# Unnavigable cells as well as cells from which 
# the goal cannot be reached should have a string 
# containing a single space (' '), as shown in the 
# previous video. The goal cell should have '*'.
# ----------

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def find_neighbors(point, cost, grid, value):
    g = point[0]
    x = point[1]
    y = point[2]
    neighbors = []
    for action in delta:
        x += action[0]
        y += action[1]
        
        if x >= 0 and y >=0 and x < len(grid) and y < len(grid[0]) and value[x][y] == 99 and grid[x][y] != 1:
            if grid[x][y] == 1:
                neighbors.append([99, x, y])
            else:
                neighbors.append([g+cost, x, y])
        
        x -= action[0]
        y -= action[1]
    
    return neighbors

def optimum_policy(grid,goal,cost):
    # ----------------------------------------
    # modify code below
    # ----------------------------------------
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    open_list = [ [0, goal[0], goal[1]] ]
    while open_list != []:
        open_list.sort()
        open_list.reverse()
        next_point = open_list.pop()
        value[next_point[1]][next_point[2]] = next_point[0]
        neighbors = find_neighbors(next_point, cost, grid, value)
        open_list.extend(neighbors)

    
    open_list =  [ [value[goal[0]], goal[0], goal[1]] ]
    while open_list != []:
        open_list.sort()
        open_list.reverse()
        point = open_list.pop()
        x = point[1]
        y = point[2]
        for a in range(len(delta)):
            x2 = x + delta[a][0]
            y2 = y + delta[a][1]

            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0 :
                if value[x2][y2] > value[x][y] and value[x2][y2] != 99:
                    policy[x2][y2] = delta_name[(a+2) % 4]
                    open_list.append([value[x2][y2], x2, y2])
            
            
    policy[goal[0]][goal[1]] = '*'
    
    return policy
# delta_name = ['^', '<', 'v', '>']
result = optimum_policy(grid,goal,cost)    

for row in result:
    print row




# -*- coding: UTF-8 -*-
# 广度优先搜索
# Return [optimal path length, row, col] if there is a path.
# If there is no valid path from the start point
# to the goal, return 'fail'

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']


def find_valid_neighbors(point, cost, closed, open_list):
    neighbors = []
    for action in delta:
        x = point[1] + action[0]
        y = point[2] + action[1]

        # If it in the grid
        if not (x >= 0 and x <= len(grid)-1 and y >= 0 and y <= len(grid[0])-1):
            continue

        # If the postion is a wall.
        if grid[x][y] == 1:       
            continue

        # If it not in the closed or open_list
        if [x,y] not in closed and [x,y] not in [ [p[1], p[2]] for p in open_list]:
                neighbors.append([ point[0] + cost, x, y])

    return neighbors
        
        
def search(grid,init,goal,cost):
    open_list = [[0, init[0], init[1]]]
    closed = []
    # 也可以这样标记closed，省去同时判断open和closed的麻烦。
    # closed = [ [0 for row in range(len(grid[0]))] for col in range(len(grid)) ]
    while open_list != []:
        open_list.sort()
        next_point = open_list.pop(0)
        closed.append([next_point[1], next_point[2]])

        next_neighbors = find_valid_neighbors(next_point, cost, closed, open_list)
        for point in next_neighbors:
            if [point[1], point[2]] == goal:
                return point
            else:
                open_list.extend(next_neighbors)
        
    
    return 'fail'
    
print search(grid, init, goal, cost)

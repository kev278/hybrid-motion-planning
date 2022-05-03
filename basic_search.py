
# Basic searching algorithms
import numpy as np
from operator import attrgetter
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.patches import Rectangle# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = np.Infinity         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node
        self.color = 0
        
def draw_path(grid, path, goal, start, title="BFS"):
    '''Visualization of the result
    '''
    # Create empty map
    np.where(grid == 1, 1, 0)
    fig, ax = plt.subplots(1)
    img = 255 * np.dstack((grid, grid, grid))
    ax.imshow(img)
    # Draw Final Path if found
    cur = goal
    while cur.col != start.col and cur.row != start.row:
        plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
        cur = cur.parent
        plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
    plt.plot(start.col, start.row, markersize=5, marker='o', color='g')
    plt.plot(goal.col, goal.row, markersize=5, marker='o', color='r')

    # show image
    plt.show()

#reconstruct path given the goal node
def reconstructPath(node):
    path= []
    path.append([node.row, node.col])
    while node.parent != None:
        node = node.parent
        path.append([node.row, node.col])
    path.reverse()
    return path
#returns the locations of adjacent nodes that are not obstacles
def getAdjacentNodes(node_grid, node):
    adj_nodes = []
    row = node.row
    col = node.col
    #using order right,down,left, up
    locations = [[row, col+1], [row+1, col], [row, col-1], [row-1, col], [row-1, col-1], [row+1, col+1], [row-1, col+1], [row+1, col+1]]
    for location in locations:
        node_row = location[0]
        node_col = location[1]
        #check if the value is less than 0 or greater than grid size. If so, node is an edge case
        if not ((node_row < 0) or (node_col < 0) or (node_col >= len(node_grid[1])) or (node_row >= len(node_grid[0]))):
            #check if node is obstacle
            if not (node_grid[location[0], location[1]].is_obs):
                adj_nodes.append(node_grid[node_row][node_col])
    return adj_nodes
            
def manhattanDistance(current, goal):
    return abs(goal[0] - current[0]) + abs(goal[1] - current[1])

def createNodeGrid(grid, goal, useH, is_bfs):
    node_grid = np.empty((len(grid[0]), len(grid[1])), dtype=object)
    for i in range(len(grid[0])):
        for j in range(len(grid[1])):
            node_grid[i][j] = Node(i, j, grid[i][j], 0)
            if useH:
                node_grid[i][j].h = manhattanDistance([i, j], goal)
            if is_bfs:
                node_grid[i][j].cost = 0
    return node_grid

def checkGoal(current, goal):
    if (current.row == goal.row) and (current.col == goal.col):
        return True
    return False
#UseH parameter basically allows the algorithm to use an Heuristic or not
# if heuristic is used, the solution is A*. If not, the solution is the same 
#as disjkstra's. If "is_bfs" is set to true, for each neighbor of current node,
# The bfs procedure is done. Can also add dfs to this, but use it as a stack instead of a 
# queue
def findPath(start, goal, grid, useH, is_bfs):
    found = False
    openSet = []
    #createNodeGrid function creates a 2d array of nodes
    node_grid = createNodeGrid(grid, goal, useH, is_bfs)
    node_grid[start[0]][start[1]].g = 0
    node_grid[start[0], start[1]].cost = node_grid[start[0], start[1]].h
    steps = 0
    if is_bfs:
        node_grid[start[0], start[1]].color = 1
    openSet.append(node_grid[start[0], start[1]])
    goal_node = node_grid[goal[0], goal[1]]
    while len(openSet) != 0:
        #get minimum cost in open set.
        current = min(openSet, key=attrgetter('cost'))
        steps = steps + 1
        if checkGoal(current, goal_node):
            #print("goal found steps are", steps)
            path = reconstructPath(current)
            return path, True, steps, node_grid
        openSet.remove(current)
        adj_nodes = getAdjacentNodes(node_grid, current)
        for neighbor in adj_nodes:
            #add g score of current node plus moving cost
            tentativeGscore = current.g + 1
            if is_bfs:
                if neighbor.color == 0:
                    neighbor.color = 1
                    neighbor.g = tentativeGscore
                    neighbor.parent = current
                    openSet.append(neighbor)
                    
            else:
                if tentativeGscore < neighbor.g:
                    neighbor.parent= current
                    neighbor.g = tentativeGscore
                    neighbor.cost =  tentativeGscore + neighbor.h
                    if neighbor not in openSet:
                        openSet.append(neighbor)
            current.color = 2
    #print("here :(")               
    return [], False, steps, node_grid
def bfs(grid, start, goal, plot_on):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    path, found, steps, node_grid = findPath(start, goal, grid, 0, 1)
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    if (plot_on == "1"):
        draw_path(grid, path, node_grid[goal[0], goal[1]], node_grid[start[0], start[1]], title="BFS")
    return path  



#dfs visit function
def dfs_visit(node_grid, u, time, steps, found, goal):
    time = time + 1
    u.cost = time
    u.color = 1
    if checkGoal(u, node_grid[goal[0]][goal[1]]):
        found = True
        return True
    if found:
        return True
    adj_nodes = getAdjacentNodes(node_grid, u)
    for v in adj_nodes:
        if v.color == 0:
            v.parent = u
            found = dfs_visit(node_grid, v, time, steps, found, goal)
    u.color = 2
    time = time + 1
    u.cost = time
    return found
    
#running this as a recursive algorithm.
def dfs(grid, start, goal, plot_on):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    time = 0
    node_grid = createNodeGrid(grid, goal, 0, 0)
    found = dfs_visit(node_grid, node_grid[start[0], start[1]], time, steps, found, goal)
    goal_node = node_grid[goal[0], goal[1]]
    if found:
        path = reconstructPath(goal_node)
        steps = goal_node.cost
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path


def dijkstra(grid, start, goal, plot_on):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    path, found, steps, node_grid = findPath(start, goal, grid, 0, 0)
    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    if (plot_on == "1"):
        draw_path(grid, path, node_grid[goal[0], goal[1]], node_grid[start[0], start[1]], title="")

    return path


def astar(grid, start, goal, plot_on):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    path, found, steps, node_grid = findPath(start, goal, grid, 1, 0)
    goal_node = node_grid[goal[0], goal[1]]
    if found:
        steps = goal_node.cost
        print(f"It takes {steps} nodes to find a path using A*")
    else:
        print("No path found")
    if (plot_on == "1"):
        draw_path(grid, path, node_grid[goal[0], goal[1]], node_grid[start[0], start[1]], title="BFS")

    return path


# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()

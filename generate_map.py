import numpy as np

def base_map(env):
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    x_size = env["dimensions"]["x_max"] - x_min
    y_size = env["dimensions"]["y_max"] - y_min

    grid = [[0] * x_size]
    for _ in range(y_size):
        grid.append([0] * x_size)

    for obstacle in env["obstacles"]:
        grid[obstacle[1] - y_min][obstacle[0] - x_min] = 1
    
    return grid

def to_astar(point, env):
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    return (int(point[1] * 10) - y_min, int(point[0] * 10) - x_min)
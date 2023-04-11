"""Generates an environment to simulate the agents in using a .yaml file"""
import numpy as np

def base_map(env):
    """Generates the map of the drone environment"""
    #Find the size of the environment
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    x_size = env["dimensions"]["x_max"] - x_min
    y_size = env["dimensions"]["y_max"] - y_min

    #Create a grid the size of the environment 
    grid = [[0] * x_size]
    for _ in range(y_size):
        grid.append([0] * x_size)

    #Sets the grid to 1 where there would be an obstacle for the drone.
    for obstacle in env["obstacles"]:
        grid[obstacle[1] - y_min][obstacle[0] - x_min] = 1
    
    return grid

def to_astar(point, env):
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    return (int(point[1] * 10) - y_min, int(point[0] * 10) - x_min)

def from_astar(path, env):
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    new_path = []
    for point in path:
        new_path.append((((point[1] + x_min) / 10), ((point[0] + y_min) / 10)))
    
    return new_path

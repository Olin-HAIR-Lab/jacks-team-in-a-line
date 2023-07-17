"""Generates an environment to simulate the agents in using a .yaml file"""
import numpy as np

class Node:
    """The base unit of the environment"""

    def __init__(self, id=None, pos=None, parent=None, children=None):
        self.id = id
        self.pos = pos
        self.parent = parent
        self.children = children

        # Total cost function
        self.f = 0
        # Cost to reach node function
        self.g = 0
        # Heuristic function
        self.h = 0

    def __eq__(self, other):
        """Check if two nodes are the same"""
        return self.pos == other.pos

    def manhattan_dist(self, other):
        """Find the manhattan distance between two nodes"""
        return abs(self.pos[0] - other.pos[0]) + abs(self.pos[1] - other.pos[1])
    
    def euclidean_dist_node(self, node):
        """Finds the euclidean distance between node and another node"""
        return np.sqrt((self.pos[0] - node.pos[0])**2 + 
                       (self.pos[1] - node.pos[1])**2 +
                       (self.pos[2] - node.pos[2])**2)
    
    def euclidean_dist_point(self, point):
        """Finds the euclidean distance between node and a point"""
        return np.sqrt((self.pos[0] - point[0])**2 + 
                       (self.pos[1] - point[1])**2 +
                       (self.pos[2] - point[2])**2)
    

def create_graph(env):
    """Generates a directed graph of nodes describing the environment"""

    graph = dict()

    # iterate through nodes in env and create Node objects
    nodes = env["nodes"]
    for node in nodes:
        n = Node(id=node[0], pos=[node[1][0], node[1][1], node[1][2]])
        n.children = []
        for child in node[2]:
            n.children.append(child)
    
        # add node to graph
        graph[n.id] = n

    return graph


def base_map(env):
    """Generates the map of the drone environment"""
    # Find the size of the environment
    x_min = env["dimensions"]["x_min"]
    y_min = env["dimensions"]["y_min"]

    x_size = env["dimensions"]["x_max"] - x_min
    y_size = env["dimensions"]["y_max"] - y_min

    # Create a grid the size of the environment
    grid = [[0] * x_size]
    for _ in range(y_size):
        grid.append([0] * x_size)

    # Sets the grid to 1 where there would be an obstacle for the drone.
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
    if path is not None:
        for point in path:
            new_path.append((((point[1] + x_min) / 10), ((point[0] + y_min) / 10)))

    return new_path


def get_node_id(point, graph):
    """Returns the corresponding (or closest) node to the given position"""

    min_dist = 999
    min_id = None

    # iterate over all the nodes in the graph to find the one closest to the [x,y] point
    for node in graph.values():
        if isinstance(point, tuple):
            dist = node.euclidean_dist_point(point)
        else:
            dist = node.euclidean_dist_node(point)
        if dist < min_dist:
            min_id = node.id
            min_dist = dist
    #         print(f"Min node is {min_id} and distance is {dist}")
    # print("-------------------")

    return min_id    
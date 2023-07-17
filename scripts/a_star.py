"""Implements A* pathfinding for the agents"""
from copy import deepcopy

class Node:
    """The base unit of the environment"""

    def __init__(self, pos=None, parent=None):
        self.pos = pos
        self.parent = parent

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


def astar(start_node_id, end_node_id, init_graph):
    """Calculate the A* path between given nodes"""
    
    # Mark the initial and final nodes
    graph = deepcopy(init_graph) #TODO: Check this out
    start = graph[start_node_id]
    end = graph[end_node_id]

    ##Create list of nodes adjacent to explored nodes
    open_list = [start]
    # Create list of explored nodes
    closed_list = []

    # Until all nodes have been explored, run A*
    while len(open_list) > 0:
        #
        q = open_list[0]
        q_idx = 0

        # Find the node with the lowest cost from adjacent nodes
        for i, n in enumerate(open_list):
            if n.f < q.f:
                q = n
                q_idx = i

        # Remove the current node from nodes to explore
        open_list.pop(q_idx)

        # goal acheived
        if q == end:
            # print("goal achieved!")
            path = []
            c = q
            # Find the optimal path by checking the parent of each node
            while c is not None or c:
                # path.append(c.pos)
                path.append(c)
                c = c.parent
            return path[::-1]


        # creates a heuristic and adds the childen to the list of nodes to explore
        # for child in children:
        for child_id in q.children:
            child = graph[child_id]

            child.g = q.g + 1
            child.h = ((child.pos[0] - end.pos[0]) ** 2) + (
                (child.pos[1] - end.pos[1]) ** 2) + (
                (child.pos[2] - end.pos[2]) ** 2)
            
            child.f = child.g + child.h
            # Check that the child is not previously explored
            if child not in open_list and child not in closed_list:
                # assign parent
                child.parent = q
                open_list.append(child)
            for n in open_list:
                # If the child is already in the list, choose the path with the lower cost function
                if child == n and n.f > child.f:
                    # assign parent
                    child.parent = q
                    open_list.append(child)

        # Adds this node to the list of explored nodes
        closed_list.append(q)

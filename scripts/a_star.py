"""Implements A* pathfinding for the agents"""


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


def astar(start_pos, end_pos, map):
    """Calculate the A* path between given points"""
    # Mark the initial and final nodes
    start = Node(start_pos)
    end = Node(end_pos)
    node_dict = {
        "S1": "A11",
        "S2": "A12",
        "S3": "A16",
        "S4": "A1",
        "A1": "A2",
        "A2": "A3",
        "A3": "A4",
        "A4": "A5",
        "A5": "A6",
        "A6": "A7",
        "A7": "A8",
        "A8": "A9",
        "A9": "A10",
        "A10": ["B1", "A11"],
        "A11": "A12",
        "A13": "A14",
        "A14": "A15",
        "A15": "A16",
        "A16": "A1",
        "B1": "B2",
        "B2": "B3",
        "B3": "B4",
        "B4": "B5",
        "B5": "A2",
    }

    ##Create list of nodes adjacent to explored nodes
    open_list = [start]
    # Create list of explored nodes
    closed_list = []

    # Until all nodes have been explored, run A*
    while len(open_list) > 0:
        #
        q = open_list[0]
        q_i = 0
        # print(curr.pos)
        # list of nodes = ["A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9", "A10", "A11", "A12", "A13", "A14", "A15", "A16", "B1", "B2", "B3", "B4", "B5"]
        # dictionary of points {A1: A2, A2: A3, A3: A4, A4: A5, A5: A6, A6: A7, A7: A8, A8: A9, A9: A10, A10: [B1, A11], A11: A12, A13: A14, A14: A15, A15: A16, A16: A1, B1: B2, B2, B3, B3: B4, B4: B5, B5: A2}
        # Find the node with the lowest cost from adjacent nodes
        for i, n in enumerate(open_list):
            if n.f < q.f:
                q = n
                q_i = i

        # Remove the current node from nodes to explore
        open_list.pop(q_i)

        # goal acheived
        if q == end:
            # print("goal achieved!")
            path = []
            c = q
            # Find the optimal path by checking the parent of each node
            while c is not None:
                path.append(c.pos)
                c = c.parent
            return path[::-1]

        # generate children
        children = []

        # change -1 for different step size
        for new_pos in [
            (0, -1),
            (0, 1),
            (-1, 0),
            (1, 0),
            (-1, -1),
            (1, 1),
            (-1, 1),
            (1, -1),
        ]:
            node_pos = (q.pos[0] + new_pos[0], q.pos[1] + new_pos[1])

            # If the node is off of the map, do not count it
            if any(
                [
                    node_pos[0] > len(map) - 1,
                    node_pos[1] > len(map[0]) - 1,
                    node_pos[0] < 0,
                    node_pos[1] < 0,
                ]
            ):
                continue

            # If the node is an obstacle, do not count it
            if map[node_pos[0]][node_pos[1]] != 0:
                continue

            # Assign the new nodes to their parent
            new_child = Node(node_pos, q)
            children.append(new_child)

        # creates a heuristic and adds the childen to the list of nodes to explore
        for child in children:
            child.g = q.g + 1
            child.h = ((child.pos[0] - end.pos[0]) ** 2) + (
                (child.pos[1] - end.pos[1]) ** 2
            )
            child.f = child.g + child.h
            # Check that the child is not previously explored
            if child not in open_list and child not in closed_list:
                open_list.append(child)
            for n in open_list:
                # If the child is already in the list, choose the path with the lower cost function
                if child == n and n.f > child.f:
                    open_list.append(child)

        # Adds this node to the list of explored nodes
        closed_list.append(q)

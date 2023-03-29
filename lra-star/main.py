"""
Currently: ignoring shelves as obstacles, assuming all moves are valid
"""

from a_star import astar
from itertools import product

map_grid = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

# tracks current pos of each agent
agent_pos = {
    0: (0, 0),
    1: (0, 3)
}

# agent_number: [start_pos, pick_pos, drop_pos]
agents = {
    0: [(0, 0), (2, 2), (4, 4)],
    1: [(0, 3), (2, 3), (4, 3)]
}

# will contain list of tuples for agent path in points
agent_paths = {}

def get_hitbox(pos):
    """
    Given a center position, returns a list containing the
    8 points immeditaely surrounding it (and the point itself)
    """
    neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0), \
                        (-1, -1), (1, 1), (-1, 1), (1, -1)]
    hitbox1 = [tuple(map(sum, zip(pos, n))) for n in neighbors]
    hitbox1.append(pos)
    return hitbox1


# go through the agents and A* for each one
for agent_k, agent_v in agents.items():
    agent_paths[agent_k] = []
    for loc_i in range(len(agent_v) - 1):
        agent_paths[agent_k].append(astar( \
            agent_v[loc_i], agent_v[loc_i + 1], map_grid))


step = 0
while not all([agent_pos[i] == agents[i][-1] for i in agents]):
    # make agent_next_pos !!! TODO
    agent_next_pos = agent_pos

    all_hitboxes = []
    # find all the hitboxes
    for agent in agents:
        all_hitboxes.append(get_hitbox(agent_pos[agent]))

    # are there any points in common between the hitboxes
    if len(set(all_hitboxes)) < len(all_hitboxes):
        # IF SO: points overlap - problem aaaaaaa
        duplicates = {x for x in all_hitboxes if all_hitboxes.count(x) > 1}
        overlap = duplicates[8::9]
        bad_agents = [agent_next_pos.values().index(o) for o in overlap]

        # TODO: figure out pairs of overlapping agents, make one wait 1 timestep
        # by inserting a repeat point into the path list

        # agent_paths[agent].insert(step, )



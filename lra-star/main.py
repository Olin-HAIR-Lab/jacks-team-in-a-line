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
    0: [(0, 0), (2, 2), (4, 0)],
    1: [(0, 3), (2, 3), (4, 3)]
}

# will contain list of tuples for agent path in points
agent_paths = {}

def get_hitbox(pos):
    """
    Given a center position, returns a list containing the
    8 points immeditaely surrounding it (and the point itself)
    """
    print(pos)
    neighbors = [(0, -1), (0, 1), (-1, 0), (1, 0), \
                        (-1, -1), (1, 1), (-1, 1), (1, -1)]
    hitbox1 = [tuple(map(sum, zip(pos, n))) for n in neighbors]
    hitbox1.append(pos)
    return hitbox1


# go through the agents and A* for each one
for agent_k, agent_v in agents.items():
    agent_paths[agent_k] = []
    for loc_i in range(len(agent_v) - 1):
        agent_paths[agent_k] += (astar( \
            agent_v[loc_i], agent_v[loc_i + 1], map_grid))


agent_next_pos = {}
step = 0
while not all([agent_pos[i] == agents[i][-1] for i in agents]) \
    and step < 10:
    print(f"agent 0 position: {agent_pos[0]} at step {step}")
    print(f"agent 1 position: {agent_pos[1]} at step {step}")
    # make agent_next_pos !!! TODO
    
    for agent in range(len(agent_pos)):
        if agent_pos[agent] == agent_paths[agent][-1]:
            # print("the fuck is happening")
            # print(agent_pos[agent])
            agent_next_pos[agent] = agent_pos[agent]
        else:
            agent_next_pos[agent] = agent_paths[agent][step + 1]

    all_hitboxes = []
    # find all the hitboxes
    for agent in agents:
        all_hitboxes += get_hitbox(agent_pos[agent])
    
    print(all_hitboxes)

    # are there any points in common between the hitboxes
    # print(all_hitboxes)
    # print(set(all_hitboxes))
    print(f"hitbox set length: {len(set(all_hitboxes))}")
    print(f"hitbox list length: {len(all_hitboxes)}")
    if len(set(all_hitboxes)) < len(all_hitboxes):
        print("is this running")
        # IF SO: points overlap - problem aaaaaaa
        duplicates = [x for x in all_hitboxes if all_hitboxes.count(x) > 1]
        print(f"duplicates: {duplicates}")
        drone_points = [point for point in list(agent_next_pos.values())\
                    if point in duplicates]
        print(f"overlap: {drone_points}")
        bad_agents = [list(agent_next_pos.values()).index(o) for o in drone_points]

        # TODO: figure out pairs of overlapping agents, make one wait 1 timestep
        # by inserting a repeat point into the path list

        print("next position dict before")
        print(agent_next_pos)

        print("agent paths before")
        print(agent_paths)

        # is it good code? no. but that's okay
        fixed_pairs = []

        for agent_a in bad_agents:
            for agent_b in bad_agents:
                if {agent_a, agent_b} not in fixed_pairs and \
                agent_a != agent_b and \
                (agent_next_pos[agent_a] in get_hitbox(agent_next_pos[agent_b]) or \
                agent_next_pos[agent_b] in get_hitbox(agent_next_pos[agent_a])):
                    print("should run once each time")
                    agent_paths[agent_b].insert(step + 1, agent_pos[agent_b])
                    fixed_pairs.append({agent_a, agent_b})
        
        print(f"fixed pairs: {fixed_pairs}")
        
        print("next position dict after")
        print(agent_next_pos)

        print("agent paths after")
        print(agent_paths)

    for agent in range(len(agent_pos)):
        # print("do we ever get here")
        # print("bitch")
        # print(agent_next_pos[agent])
        agent_pos[agent] = agent_next_pos[agent]
    
    step += 1


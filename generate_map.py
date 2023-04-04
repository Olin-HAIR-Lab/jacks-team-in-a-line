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
    
    '''for loc in pick_locs + drop_locs:
        loc_x = int(10 * loc[1][0] - x_min)
        loc_y = int(10 * loc[1][1] - y_min)

        # print((loc_x, loc_y))

        grid[loc_y][loc_x] = 1'''
    
    return grid

def agent_map(map, agent_locs):
    for loc in agent_locs:
        map[int(loc[1] * 10 + 9)][int(loc[0] * 10 + 9)] = 0
    
    return map

def find_points_of_interest(agent_list,task_assignment):
    agent_paths=[]
    for agent_instance in agent_list:
        #Create list of tuples that the agent wants to visit
        agent_pois=[]
        #Find the starting position of the agent
        start_pos=(agent_list[agent_instance].get_pos().x,agent_list[agent_instance].get_pos().y)
        #The first point of intrest is the starting position
        agent_pois.append(start_pos)

        #Finds the list of tasks for this agent
        agent_task_list=task_assignment[agent_list[agent_instance]]
        #If there is more than one task, the tasks will be a list and will be added sequentially
        if type(agent_task_list) is list:
            for task in agent_task_list:
                pick_loc=(task.pick_loc.x,task.pick_loc.y)
                agent_pois.append(pick_loc)
                drop_loc=(task.drop_loc.x,task.drop_loc.y)
                agent_pois.append(drop_loc)
        #If there is only a single task, the pick and drop locations are added directly
        else:
            pick_loc=(agent_task_list.pick_loc.x,agent_task_list.pick_loc.y)
            agent_pois.append(pick_loc)
            drop_loc=(agent_task_list.drop_loc.x,agent_task_list.drop_loc.y)
            agent_pois.append(drop_loc)
        agent_paths.append(agent_pois)
    return agent_paths
        
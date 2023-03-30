import time

def generate_heuristic(env,agent, task, dist_weight=1, time_weight=0.05):
    """
    Generates a heuristic value for task assignment
    Args:
        env: The warehouse layout
        agent_list: an agent's position in the warehouse and availability
        task: a task's pic and drop location, time of input, and whether the task has the priority flag
    Returns:
        heuristic value: a float that represents the cost for this drone to complete this task
    """

    distance = get_manhattan_distance(agent.loc,task.pick_loc)
    time = get_time_since_input(task.time_input)
    cost = distance*dist_weight-time*time_weight-task.priority
    return cost

def get_time_since_input(time_input):
    """
    Finds how many seconds ago the task was inputted
    """
    current_time=time.time()
    return current_time-time_input

def get_manhattan_distance(pos_1,pos_2):
    """
    Takes in 2 positions as 3 element lists [x,y,z] and finds the manhattan distance between them 
    """
    x_dist=pos_1[0]-pos_2[0]
    y_dist=pos_1[1]-pos_2[1]
    z_dist=pos_1[2]-pos_2[2]
    return abs(x_dist)+abs(y_dist)+abs(z_dist)

def get_euclidian_distance(pos_1,pos_2):
    """
    Takes in 2 positions as 3 element lists [x,y,z] and finds the euclidian distance between them 
    """
    x_dist=pos_1[0]-pos_2[0]
    y_dist=pos_1[1]-pos_2[1]
    z_dist=pos_1[2]-pos_2[2]
    return (x_dist**2+y_dist**2+z_dist**2)**0.5
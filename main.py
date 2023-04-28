from scripts.ground_control import GroundControlSystem
from scripts.Quadrotor import Quadrotor
from scripts.utils import Task, Position, State
import time
import yaml
import plotly.graph_objects as go
from simulation.Simulation import Simulation
from itertools import product


def define_env(map):
    """
    Take in the two corners of rectangles from the yaml file
    and define all the points as obstacles
    """
    env = map
    corners = map["obstacles"]
    obstacles = []

    x_vals = []
    y_vals = []
    for pair in corners:
        x_vals += [x for x in range(pair[0][0], pair[1][0] + 1)]
        y_vals += [y for y in range(pair[0][1], pair[1][1] + 1)]
        obstacles += product(x_vals, y_vals)

    env['obstacles'] = list(set(obstacles))
    return env


if __name__ == '__main__':
    # load configuration file from YAML file
    original_map = './base_config.yaml'
    new_map = "new_map_config.yaml"
    test_maps = ["test_1_config.yaml", "test_2_config.yaml",
                 "test_3_config.yaml", "test_4_config.yaml"]

    active_map = new_map
    with open(test_maps[1], 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    use_hardware = config['use_hardware']
    agent_init = config['agent_init']
    colors = config['agent_colors']
    time_delta = config['time_delta']
    env_simplified = config['map'].copy()
    print(env_simplified)
    env = define_env(config['map'])
    num_agents = len(agent_init)

    print(
        f'Number of Agents: [{num_agents}] -> {[agent_init[i][0] for i in range(len(agent_init))]}')
    print(f'Use Hardware: [{use_hardware}]')
    print(f'Time Delta (dt): [{time_delta}]')

    # ---------------------------------------------------------------------------------------------------------
    # STEP 1.A: Define the task list
    # ---------------------------------------------------------------------------------------------------------

    # create the task list
    task_list = dict()
    pick_locations = config['pick_loc']
    drop_locations = config['drop_loc']

    print('-----------------------')
    print('Task List')
    print('-----------------------')

    for i in range(len(pick_locations)):
        t = Task(pick_loc=Position(x=pick_locations[i][1][0], y=pick_locations[i][1][1], z=pick_locations[i][1][2]),
                 drop_loc=Position(
                     x=drop_locations[i][1][0], y=drop_locations[i][1][1], z=0.0),
                 pick_id=pick_locations[i][0],
                 drop_id=drop_locations[i][0],
                 id='T'+str(i),
                 priority=i,
                 time_input=time.time())
        task_list['T'+str(i)] = t

        print(f'Task {t.id}: {t.pick_id} -> {t.drop_id}')

    print('-----------------------')

    # ---------------------------------------------------------------------------------------------------------
    # STEP 1.B: Define the agent list
    # ---------------------------------------------------------------------------------------------------------

    print('----------------------------------')
    print('Agent List (with home position)')
    print('----------------------------------')

    # Plotly colors
    colors = [
        '#1f77b4',  # muted blue
        '#ff7f0e',  # safety orange
        '#2ca02c',  # cooked asparagus green
        '#d62728',  # brick red
        '#9467bd',  # muted purple
        '#8c564b',  # chestnut brown
        '#e377c2',  # raspberry yogurt pink
        '#7f7f7f',  # middle gray
        '#bcbd22',  # curry yellow-green
        '#17becf'   # blue-teal
    ]

    agent_list = dict()
    for i in range(num_agents):
        # define initial state
        start = State(x_pos=agent_init[i][1][0],
                      y_pos=agent_init[i][1][1])
        # define the appropriate UR1
        uri = 'radio://0/'+agent_init[i][0][2:]+'0/2M/E7E7E7E7E7'
        # define agent as Quadrotor
        agent = Quadrotor(init_state=start,
                          color=colors[i],
                          id=agent_init[i][0],
                          uri=uri,
                          take_off_height=agent_init[i][2],
                          hardware_flag=use_hardware,
                          dt=time_delta)
        agent_list[agent._id] = agent

        if use_hardware:
            print(f'Agent {agent._id}: {agent_list[agent._id].get_pos().x, agent_list[agent._id].get_pos().y} \
                ---> {uri}')
        else:
            print(
                f'Agent {agent._id}: {agent_list[agent._id].get_pos().x, agent_list[agent._id].get_pos().y} ')

    print('----------------------------------')

    if use_hardware:
        print('\n !!!!!!!!Please ensure you confirm the Crazyflies are connected to the right radio channels!!!!!!!!')

    # #########################################################################################################
    # STEP 2: Implement Multi-Agent Task Assignment
    # #########################################################################################################

    # ---------------------------------------------------------------------------------------------------------
    # STEP 2.A. Define the Ground Constrol System & compute assignment
    # ---------------------------------------------------------------------------------------------------------

    # instantiate ground control system object
    gcs = GroundControlSystem(agent_list=agent_list,
                              task_list=task_list,
                              env=env)

    # SETUP FOR SIMLUATION
    fig1 = go.Figure()
    fig2 = go.Figure()
    fig_animated = go.Figure()
    fig3d_animated = go.Figure()
    sim = Simulation(env=env_simplified, fig1=fig1, fig2=fig2,
                     fig_animated=fig_animated, fig3d_animated=fig3d_animated)
    sim.add_agents(agent_list)
    sim.set_task_list(task_list)
    sim.init_plot()

    for agent in agent_list.values():
        agent.initialize_agent()

    if use_hardware:
        time.sleep(3)

    i = 0
    while i < 115:
        # update the drone for it's next step
        gcs.update()
        # create a map of where the drone has gone
        gcs.set_task_graph(draw=False)
        # get tasks for task assignment
        task_assignment = gcs.get_task_assignment(draw=False)

        i += 1
        # Print Timestep
        # print(i+5)
        if use_hardware:
            time.sleep(time_delta)

    for agent in agent_list.values():
        agent.land()

    sim.update_plot()

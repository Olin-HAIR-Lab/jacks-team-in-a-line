""" This script holds the functions used in the ground control system"""


import numpy as np
import matplotlib.pyplot as plt
import math
import time
import networkx as nx
from scripts.a_star import astar
from scripts.generate_map import base_map, to_astar, from_astar, create_graph, get_node_id
from dataclasses import dataclass
from scripts.Quadrotor import Quadrotor
from scripts.utils import Task, Node


class GroundControlSystem:
    def __init__(self, agent_list=None, task_list=None, env=None):
        self._agent_list = agent_list
        self._task_list = task_list
        self._task_assignment = dict()
        self._agent_paths = dict()
        self._env = env
        self._available_agents = list(self._agent_list.keys())
        self._task_queue = list(self._task_list.keys())
        self._completed_tasks = dict()
        self._agents_active = False

        self._map = base_map(env)
        self._graph = create_graph(env)
        # print(self._graph)

    def update(self):
        self._agents_active = False
        for agent_id, agent in self._agent_list.items():
            # set agent as active if task is incomplete
            if not agent.tasks_complete:
                self._agents_active = True
            # check if agent is at the end of it's path
            if agent.path_complete():
                # if agent is not at the base station, plan a path to the base station
                if not agent.at_base_station():
                    end_point = get_node_id(agent.get_path_end(), self._graph)
                    base_loc = get_node_id(
                        (agent.base_station.x, agent.base_station.y), self._graph
                    )
                    return_path = astar(end_point, base_loc, self._graph)
                    agent.clear_tasks_and_path()
                    agent.add_to_path(return_path)
                    agent.add_to_path([Node(pos=[agent.base_station.x, agent.base_station.y])] * 2)
                
                # if agent is at base station, add base node and set task as complete
                else:
                    agent.clear_tasks_and_path()
                    agent.add_to_path([Node(pos=[agent.base_station.x, agent.base_station.y])] * 2)
                    agent.set_tasks_complete(True)

            if agent.available_for_task() and agent_id not in self._available_agents:
                self._available_agents.append(agent_id)
            elif not agent.available_for_task() and agent_id in self._available_agents:
                self._available_agents.remove(agent_id)

        self.assign_tasks()

        for agent_id in self._available_agents:
            agent = self._agent_list[agent_id]
            if agent.task_to_plan:
                task = self._task_list[agent.task_to_plan]

                base_loc = get_node_id(agent.get_path_end(), self._graph)
                pick_loc = get_node_id((task.pick_loc.x, task.pick_loc.y), self._graph)
                drop_loc = get_node_id((task.drop_loc.x, task.drop_loc.y), self._graph)

                # plan and add paths to agent path
                # base_loc -> pick_loc
                astar_path = astar(base_loc, pick_loc, self._graph)
                agent.add_to_path(astar_path)
                    # agent stays at pick location for two timesteps
                agent.add_to_path(Node(id=pick_loc, pos=[task.pick_loc.x, task.pick_loc.y]))
                # pick_loc -> drop_loc
                astar_path = astar(pick_loc, drop_loc, self._graph)
                agent.add_to_path(astar_path)
                    # agent stays at drop location for two timesteps
                agent.add_to_path(Node(id=drop_loc, pos=[task.drop_loc.x, task.drop_loc.y]))
                # drop_loc -> base_loc
                astar_path = astar(drop_loc, base_loc, self._graph)
                agent.add_to_path(astar_path)
                agent.add_to_path(Node(id=base_loc, pos=[agent.base_station.x, agent.base_station.y]))
               
                agent.remove_planned_task()

            # print(f'Agent: {agent.get_path()}')
            path = []
            for n in agent.get_path():
                path.append(n.id)
            
            print(path)


        self.repair_conflicts()

        for agent in self._agent_list.values():
            agent.update()

        print('---------------------------------------------------')

  

    @property
    def agents_active(self):
        return self._agents_active

    def assign_tasks(self):
        """Assign tasks to available agents using Hungarian algorithm"""
        if len(self._available_agents) < 1 or len(self._task_queue) < 1:
            return

        tasks_to_assign = self._task_queue.copy()

        # Create NxN matrix based on number of agents/tasks
        if len(self._available_agents) > len(tasks_to_assign):
            self._cost_matrix = np.zeros(
                (len(self._available_agents), len(self._available_agents))
            )
        else:
            self._cost_matrix = np.zeros((len(tasks_to_assign), len(tasks_to_assign)))

        # Calculate costs
        for i, agent_id in enumerate(self._available_agents):
            for j, task_id in enumerate(tasks_to_assign):
                self._cost_matrix[i][j] = self.generate_heuristic(
                    self._agent_list[agent_id], self._task_list[task_id]
                )

        print(f"Cost matrix:\n{np.round(self._cost_matrix.copy(), 2)}")

        assignments = self.hungarian_algorithm()
        total_cost, ans_cost_matrix = self.ans_calculation(
            self._cost_matrix, assignments
        )
        for a in assignments:
            if a[0] < len(self._available_agents) and a[1] < len(tasks_to_assign):
                agent_id = self._available_agents[a[0]]
                task_id = tasks_to_assign[a[1]]
                if self._agent_list[agent_id] in self._task_assignment:
                    self._task_assignment[self._agent_list[agent_id]].append(
                        self._task_list[task_id]
                    )
                else:
                    self._task_assignment[self._agent_list[agent_id]] = [
                        (self._task_list[task_id])
                    ]
                self._completed_tasks[task_id] = self._task_list[task_id]

                self._agent_list[agent_id].add_task(self._task_list[task_id])

                self._task_queue.remove(task_id)

        print("Assignments: ")
        for assignment in self._task_assignment.items():
            print(f"{assignment[0].id}: {[a.id for a in assignment[1]]}")

        print("Remaining tasks: ")
        for task in self._task_queue:
            print(f"{task}")

    def set_task_graph(self, draw=False):
        """creates a directed graph based on the agents and task list using networkx"""
        self.G = nx.DiGraph()

        # add agent start locations:
        for a in self._agent_list.values():
            self.G.add_node(a.id, pos=(a.get_pos().x, a.get_pos().y))

        # add pick and drop locations of tasks:
        for t in self._task_list.values():
            self.G.add_node(t.pick_id, pos=(t.pick_loc.x, t.pick_loc.y))
            self.G.add_node(t.drop_id, pos=(t.drop_loc.x, t.drop_loc.y))
            # add edges connecting pick and drop locations
            self.G.add_edge(t.pick_id, t.drop_id)

        pos = nx.get_node_attributes(self.G, "pos")

        if draw:
            nx.draw(self.G, pos, with_labels=True)
            plt.show()

    def get_task_assignment(self, draw=False):
        """return the task assignment and draw if required"""

        j = 0
        colors = ["b", "g", "r", "m", "y", "c"]

        for agent, task in self._task_assignment.items():
            if type(task) == list:
                for i in range(len(task)):
                    if i > 0:
                        self.G.add_edge(
                            task[i - 1].drop_id, task[i].pick_id, color=colors[j]
                        )
                        self.G.add_edge(
                            task[i].pick_id, task[i].drop_id, color=colors[j]
                        )
                    else:
                        self.G.add_edge(agent.id, task[i].pick_id, color=colors[j])
                        self.G.add_edge(
                            task[i].pick_id, task[i].drop_id, color=colors[j]
                        )
            else:
                self.G.add_edge(agent.id, task.pick_id, color=colors[j])
                self.G.add_edge(task.pick_id, task.drop_id, color=colors[j])
            j += 1

        if draw:
            color_scheme = nx.get_edge_attributes(self.G, "color").values()
            pos = nx.get_node_attributes(self.G, "pos")
            nx.draw(self.G, pos, with_labels=True, edge_color=color_scheme)
            plt.show()

        return self._task_assignment

    def generate_heuristic(self, agent, task, dist_weight=1, time_weight=0.05):
        """
        Generates a heuristic value for task assignment
        Args:
            env: The warehouse layout
            agent_list: an agent's position in the warehouse and availability
            task: a task's pic and drop location, time of input, and whether the task has the priority flag
        Returns:
            heuristic value: a float that represents the cost for this drone to complete this task
        """
        return Heuristic(agent=agent, task=task)

    def get_time_since_input(self, time_input):
        """
        Finds how many seconds ago the task was inputted
        """
        current_time = time.time()
        return current_time - time_input

    def min_zero_row(self, zero_mat, mark_zero):
        """
        The function can be splitted into two steps:
        #1 The function is used to find the row which containing the fewest 0.
        #2 Select the zero number on the row, and then marked the element corresponding row and column as False
        """

        # Find the row
        min_row = [99999, -1]

        for row_num in range(zero_mat.shape[0]):
            if np.sum(zero_mat[row_num] == True) > 0 and min_row[0] > np.sum(
                zero_mat[row_num] == True
            ):
                min_row = [np.sum(zero_mat[row_num] == True), row_num]

        # Marked the specific row and column as False
        zero_index = np.where(zero_mat[min_row[1]] == True)[0][0]
        mark_zero.append((min_row[1], zero_index))
        zero_mat[min_row[1], :] = False
        zero_mat[:, zero_index] = False

    def mark_matrix(self, mat):
        """
        Finding the returning possible solutions for LAP problem.
        """

        # Transform the matrix to boolean matrix(0 = True, others = False)
        cur_mat = mat
        zero_bool_mat = cur_mat == 0
        zero_bool_mat_copy = zero_bool_mat.copy()

        # Recording possible answer positions by marked_zero
        marked_zero = []
        while True in zero_bool_mat_copy:
            self.min_zero_row(zero_bool_mat_copy, marked_zero)

        # Recording the row and column positions seperately.
        marked_zero_row = []
        marked_zero_col = []
        for i in range(len(marked_zero)):
            marked_zero_row.append(marked_zero[i][0])
            marked_zero_col.append(marked_zero[i][1])

        # Step 2-2-1
        non_marked_row = list(set(range(cur_mat.shape[0])) - set(marked_zero_row))

        marked_cols = []
        check_switch = True
        while check_switch:
            check_switch = False
            for i in range(len(non_marked_row)):
                row_array = zero_bool_mat[non_marked_row[i], :]
                for j in range(row_array.shape[0]):
                    # Step 2-2-2
                    if row_array[j] == True and j not in marked_cols:
                        # Step 2-2-3
                        marked_cols.append(j)
                        check_switch = True

            for row_num, col_num in marked_zero:
                # Step 2-2-4
                if row_num not in non_marked_row and col_num in marked_cols:
                    # Step 2-2-5
                    non_marked_row.append(row_num)
                    check_switch = True
        # Step 2-2-6
        marked_rows = list(set(range(mat.shape[0])) - set(non_marked_row))

        return (marked_zero, marked_rows, marked_cols)

    def adjust_matrix(self, mat, cover_rows, cover_cols):
        cur_mat = mat
        non_zero_element = []

        # Step 4-1
        for row in range(len(cur_mat)):
            if row not in cover_rows:
                for i in range(len(cur_mat[row])):
                    if i not in cover_cols:
                        non_zero_element.append(cur_mat[row][i])
        min_num = min(non_zero_element)

        # Step 4-2
        for row in range(len(cur_mat)):
            if row not in cover_rows:
                for i in range(len(cur_mat[row])):
                    if i not in cover_cols:
                        cur_mat[row, i] = cur_mat[row, i] - min_num
        # Step 4-3
        for row in range(len(cover_rows)):
            for col in range(len(cover_cols)):
                cur_mat[cover_rows[row], cover_cols[col]] = (
                    cur_mat[cover_rows[row], cover_cols[col]] + min_num
                )
        return cur_mat

    def hungarian_algorithm(self):
        dim = self._cost_matrix.shape[0]
        cur_mat = self._cost_matrix.copy()

        # Step 1 - Every column and every row subtract its internal minimum
        for row_num in range(self._cost_matrix.shape[0]):
            cur_mat[row_num] = cur_mat[row_num] - np.min(cur_mat[row_num])

        for col_num in range(self._cost_matrix.shape[1]):
            cur_mat[:, col_num] = cur_mat[:, col_num] - np.min(cur_mat[:, col_num])
        zero_count = 0
        while zero_count < dim:
            # Step 2 & 3
            ans_pos, marked_rows, marked_cols = self.mark_matrix(cur_mat)
            zero_count = len(marked_rows) + len(marked_cols)

            if zero_count < dim:
                cur_mat = self.adjust_matrix(cur_mat, marked_rows, marked_cols)

        return ans_pos

    def ans_calculation(self, mat, pos):
        total = 0
        ans_mat = np.zeros((mat.shape[0], mat.shape[1]))
        for i in range(len(pos)):
            total += mat[pos[i][0], pos[i][1]]
            ans_mat[pos[i][0], pos[i][1]] = mat[pos[i][0], pos[i][1]]
        return total, ans_mat

    def repair_conflicts(self):
        """
        Performs the local repair operation by stepping through the agent paths to find and fix any overlaps
        """

        # find the max agent path length
        K = 0
        for agent in self._agent_list.values():
            K = max(K, agent.get_path_length())

        # iterate through K time steps to resolve all overlaps
        for k in range(K):
            # define the priority order of the agents
            agent_order = self.define_agent_priority(k)

            # get all overlapping agents ids
            overlapping_agents = self.find_overlapping_agents(agent_order, k)

            # cycle through until there's no overlap along the priority list
            while overlapping_agents:
                self.fix_overlaps(agent_order, overlapping_agents, k)
                overlapping_agents.clear()

                overlapping_agents = self.find_overlapping_agents(agent_order, k)

    def find_overlapping_agents(self, priority_order, path_idx):
        """ Returns a list containing indices of the overlapping agents in the agent priority order """

        # populate the node list, i.e. nodes that agents are in at the time step/index
        node_list = []
        for id in priority_order:
            agent_path = self._agent_list[id].get_path()

            if len(agent_path) <= path_idx:
                node_list.append("")
            else:
                node_list.append(agent_path[path_idx].id)
        
        # find any duplicate nodes (indicating overlapping agents)
        overlaps = [idx for idx, id in enumerate(node_list) 
                    if node_list.count(id) > 1 
                    and id != ""
                    and id != None] # TODO: Improve this list comprehension

        return overlaps
        
    def define_agent_priority(self, path_idx):
        """Returns the agent priority order (preferably as a function of the current local state of the environment)"""
        #TODO: Make this dependent on the local state of the environment
        # return ['HB4', 'HB3', 'HB2', 'HB1']
        return ['HB1']
    
    def fix_overlaps(self, agent_order, overlapping_agents, path_idx):
        """Fixes the overlapping agents by delaying the lower priority agents by one timestep"""
        
        # delay the lower priority agent by adding an additional node at the path_idx
        lower_priority_agent = agent_order[overlapping_agents[-1]] #TODO: how about when multiple agents overlap?
        agent_path = self._agent_list[lower_priority_agent].get_path()
        # duplicate the node at the path index
        agent_path.insert(path_idx, agent_path[path_idx-1])

        # update agent's path
        self._agent_list[lower_priority_agent].set_path(agent_path)


@dataclass
class Heuristic:
    """
    Represents the cost of assigning an agent to a task.

    Attributes:
        agent (Quadrotor): Agent with preexisting path and tasks.
        task (Task): The task object being assigned.
        dist_weight (float): weighting of the distance drone will
            cover before reaching pick loc (incl current path)
        time_weight (float): weighting of the time since task assigned
        priority_weight (int): weighting of task priority

    Methods:
        steps_left_in_path: returns steps left in currently held path
        path_end_loc: returns position (x y z) of the last path point
        get_euclidian_distance: returns manhattan distance between 2 points
        cost: uses scaling factors to generate a weighted cost
        __float__(): Returns the cost as an float AUTOMATICALLY TRIGGERED
            ON NUMERICAL COMPARISONS OF CLASS OBJ, CAN BE USED AS VALUE.


    """

    agent: Quadrotor
    task: Task
    dist_weight: float = 1
    time_weight: float = 0.05
    priority_weight: int = 0.5
    # unsure about priority weighting, may cause problems

    def steps_left_in_path(self):
        return self.agent.get_path_length()

    def path_end_loc(self):
        return self.agent.get_path_end()

    def get_euclidian_distance(self, pos_1, pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the euclidian (manhattan) distance between them
        """
        x_dist = pos_1[0] - pos_2.x
        y_dist = pos_1[1] - pos_2.y
        return math.sqrt(abs(x_dist) ** 2 + abs(y_dist) ** 2)

    def get_manhattan_distance(self, pos_1, pos_2):
        return abs(pos_1.pos[0] - pos_2.x) + abs(pos_1.pos[1] - pos_2.y)

    @property
    def cost(self):
        """Return total cost"""
        distance_from_path = self.get_manhattan_distance(
            self.path_end_loc(), self.task.pick_loc
        )
        steps_left = self.steps_left_in_path() * 0.1
        total_distance = distance_from_path + steps_left
        curr_time = time.time() - self.task.time_input
        priority = self.task.priority * self.priority_weight

        cost = total_distance - curr_time + priority
        return cost

    def __float__(self):
        """
        method to allow for direct comparison of Heuristic values like floats!
        """
        return float(self.cost)

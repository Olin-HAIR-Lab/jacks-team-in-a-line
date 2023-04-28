""" This script holds the functions used in the ground control system"""


from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import networkx as nx
from scripts.a_star import astar
from scripts.generate_map import base_map, to_astar, from_astar
import math

from scripts.Quadrotor import Quadrotor
from scripts.utils import Task


class GroundControlSystem():
    def __init__(self, agent_list=None, task_list=None, env=None):
        self._agent_list = agent_list
        self._task_list = task_list
        self._task_assignment = dict()
        self._agent_paths = dict()
        self._env = env
        self._available_agents = list(self._agent_list.keys())
        self._task_queue = list(self._task_list.keys())
        self._completed_tasks = dict()

        self._map = base_map(env)
        # print(self._map)

    def update(self):
        for agent_id, agent in self._agent_list.items():
            if agent.path_complete():
                if not agent.at_base_station():
                    end_point = to_astar(agent.get_path_end(), self._env)
                    start_loc = to_astar(
                        (agent.get_base_station().x, agent.get_base_station().y), self._env)
                    return_path = astar(end_point, start_loc, self._map)
                    agent.clear_tasks_and_path()
                    agent.add_to_path(from_astar(return_path, self._env))
                    agent.add_to_path(
                        [(agent.get_base_station().x, agent.get_base_station().y)]*2)
                else:
                    agent.clear_tasks_and_path()
                    agent.add_to_path(
                        [(agent.get_base_station().x, agent.get_base_station().y)]*2)
            if agent.available_for_task() and agent_id not in self._available_agents:
                self._available_agents.append(agent_id)
            elif not agent.available_for_task() and agent_id in self._available_agents:
                self._available_agents.remove(agent_id)

        self.assign_tasks()

        for agent_id in self._available_agents:
            agent = self._agent_list[agent_id]
            if agent.get_task_to_plan():
                task = self._task_list[agent.get_task_to_plan()]
                agent_pos = to_astar(agent.get_path_end(), self._env)
                pick_loc = to_astar(
                    (task.pick_loc.x, task.pick_loc.y), self._env)
                drop_loc = to_astar(
                    (task.drop_loc.x, task.drop_loc.y), self._env)

                target_points = [agent_pos, pick_loc, drop_loc]

                raw_path = []
                for loc_i in range(2):
                    astar_path = astar(
                        target_points[loc_i], target_points[loc_i + 1], self._map)

                    raw_path += astar_path

                    agent.add_to_path(from_astar(astar_path, self._env))
                    agent.remove_planned_task()
                print(agent._path)

                # plt.plot([a[1] for a in raw_path], [a[0] for a in raw_path])
                # plt.xlim([0, 30])
                # plt.ylim([0, 20])
                # plt.show()

        self.find_collisions()

        for agent in self._agent_list.values():
            agent.update()

    def assign_tasks(self):
        if len(self._available_agents) < 1 or len(self._task_queue) < 1:
            return

        tasks_to_assign = self._task_queue.copy()

        # Create NxN matrix based on number of agents/tasks
        if len(self._available_agents) > len(tasks_to_assign):
            self._cost_matrix = np.zeros(
                (len(self._available_agents), len(self._available_agents)))
        else:
            self._cost_matrix = np.zeros(
                (len(tasks_to_assign), len(tasks_to_assign)))

        # Calculate costs
        for i, agent_id in enumerate(self._available_agents):
            for j, task_id in enumerate(tasks_to_assign):
                self._cost_matrix[i][j] = Heuristic(
                    self._agent_list[agent_id], self._task_list[task_id])

        print(f"Cost matrix:\n{np.round(self._cost_matrix.copy(), 2)}")

        assignments = self.hungarian_algorithm()
        total_cost, ans_cost_matrix = self.ans_calculation(
            self._cost_matrix, assignments)
        for a in assignments:
            if a[0] < len(self._available_agents) and a[1] < len(tasks_to_assign):
                agent_id = self._available_agents[a[0]]
                task_id = tasks_to_assign[a[1]]
                if self._agent_list[agent_id] in self._task_assignment:
                    self._task_assignment[self._agent_list[agent_id]].append(
                        self._task_list[task_id])
                else:
                    self._task_assignment[self._agent_list[agent_id]] = [
                        (self._task_list[task_id])]
                self._completed_tasks[task_id] = self._task_list[task_id]

                self._agent_list[agent_id].add_task(self._task_list[task_id])

                self._task_queue.remove(task_id)

        print("Assignments: ")
        for assignment in self._task_assignment.items():
            print(f"{assignment[0]._id}: {[a.id for a in assignment[1]]}")

        print("Remaining tasks: ")
        for task in self._task_queue:
            print(f"{task}")

    def set_task_graph(self, draw=False):
        """creates a directed graph based on the agents and task list using networkx"""
        self.G = nx.DiGraph()

        # add agent start locations:
        for a in self._agent_list.values():
            self.G.add_node(a._id, pos=(a.get_pos().x, a.get_pos().y))

        # add pick and drop locations of tasks:
        for t in self._task_list.values():
            self.G.add_node(t.pick_id, pos=(t.pick_loc.x, t.pick_loc.y))
            self.G.add_node(t.drop_id, pos=(t.drop_loc.x, t.drop_loc.y))
            # add edges connecting pick and drop locations
            self.G.add_edge(t.pick_id, t.drop_id)

        pos = nx.get_node_attributes(self.G, 'pos')

        if draw:
            nx.draw(self.G, pos, with_labels=True)
            plt.show()

    def get_task_assignment(self, draw=False):
        """return the task assignment and draw if required"""

        j = 0
        colors = ['b', 'g', 'r', 'm', 'y', 'c']

        for agent, task in self._task_assignment.items():
            if type(task) == list:
                for i in range(len(task)):
                    if i > 0:
                        self.G.add_edge(task[i-1].drop_id,
                                        task[i].pick_id, color=colors[j])
                        self.G.add_edge(task[i].pick_id,
                                        task[i].drop_id, color=colors[j])
                    else:
                        self.G.add_edge(
                            agent._id, task[i].pick_id, color=colors[j])
                        self.G.add_edge(task[i].pick_id,
                                        task[i].drop_id, color=colors[j])
            else:
                self.G.add_edge(agent._id, task.pick_id, color=colors[j])
                self.G.add_edge(task.pick_id, task.drop_id, color=colors[j])
            j += 1

        if draw:
            color_scheme = nx.get_edge_attributes(self.G, 'color').values()
            pos = nx.get_node_attributes(self.G, 'pos')
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
        distance = self.get_manhattan_distance(agent.get_pos(), task.pick_loc)
        time = self.get_time_since_input(task.time_input)
        cost = (distance*dist_weight-time*time_weight)/(task.priority+1)
        return cost

    def get_time_since_input(self, time_input):
        """
        Finds how many seconds ago the task was inputted
        """
        current_time = time.time()
        return current_time-time_input

    def get_manhattan_distance(self, pos_1, pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the manhattan distance between them 
        """
        x_dist = pos_1.x-pos_2.x
        y_dist = pos_1.y-pos_2.y
        z_dist = pos_1.z-pos_2.z
        return abs(x_dist)+abs(y_dist)+abs(z_dist)

    def get_euclidian_distance(self, pos_1, pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the euclidian distance between them 
        """
        x_dist = pos_1.x-pos_2.x
        y_dist = pos_1.y-pos_2.y
        z_dist = pos_1.z-pos_2.z
        return (x_dist**2+y_dist**2+z_dist**2)**0.5

    def min_zero_row(self, zero_mat, mark_zero):
        '''
        The function can be splitted into two steps:
        #1 The function is used to find the row which containing the fewest 0.
        #2 Select the zero number on the row, and then marked the element corresponding row and column as False
        '''

        # Find the row
        min_row = [99999, -1]

        for row_num in range(zero_mat.shape[0]):
            if np.sum(zero_mat[row_num] == True) > 0 and min_row[0] > np.sum(zero_mat[row_num] == True):
                min_row = [np.sum(zero_mat[row_num] == True), row_num]

        # Marked the specific row and column as False
        zero_index = np.where(zero_mat[min_row[1]] == True)[0][0]
        mark_zero.append((min_row[1], zero_index))
        zero_mat[min_row[1], :] = False
        zero_mat[:, zero_index] = False

    def mark_matrix(self, mat):
        '''
        Finding the returning possible solutions for LAP problem.
        '''

        # Transform the matrix to boolean matrix(0 = True, others = False)
        cur_mat = mat
        zero_bool_mat = (cur_mat == 0)
        zero_bool_mat_copy = zero_bool_mat.copy()

        # Recording possible answer positions by marked_zero
        marked_zero = []
        while (True in zero_bool_mat_copy):
            self.min_zero_row(zero_bool_mat_copy, marked_zero)

        # Recording the row and column positions seperately.
        marked_zero_row = []
        marked_zero_col = []
        for i in range(len(marked_zero)):
            marked_zero_row.append(marked_zero[i][0])
            marked_zero_col.append(marked_zero[i][1])

        # Step 2-2-1
        non_marked_row = list(
            set(range(cur_mat.shape[0])) - set(marked_zero_row))

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

        return(marked_zero, marked_rows, marked_cols)

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
                cur_mat[cover_rows[row], cover_cols[col]
                        ] = cur_mat[cover_rows[row], cover_cols[col]] + min_num
        return cur_mat

    def hungarian_algorithm(self):
        dim = self._cost_matrix.shape[0]
        cur_mat = self._cost_matrix.copy()

        # Step 1 - Every column and every row subtract its internal minimum
        for row_num in range(self._cost_matrix.shape[0]):
            cur_mat[row_num] = cur_mat[row_num] - np.min(cur_mat[row_num])

        for col_num in range(self._cost_matrix.shape[1]):
            cur_mat[:, col_num] = cur_mat[:, col_num] - \
                np.min(cur_mat[:, col_num])
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

    def get_hitbox(self, pos):
        """
        Given a center position, returns a list containing the
        8 points immeditaely surrounding it (and the point itself)
        """

        neighbors = [(0, -.1), (0, .1), (-.1, 0), (.1, 0), (0, 0),
                     (-.1, -.1), (.1, .1), (-.1, .1), (.1, -.1)]
        hitbox = [tuple(map(sum, zip(pos, n))) for n in neighbors]
        hitbox.append(pos)
        return hitbox

    @DeprecationWarning
    def init_astar(self):
        for agent in self._agent_list.values():
            for loc_i in range(len(agent._task_queue) - 1):
                agent._path += (astar(
                    agent._task_queue[loc_i], agent._task_queue[loc_i + 1], self._env))

        for agent in self._agent_list.values():
            print(agent._path)

    def _fix_collision(self, agent_a, agent_b):
        """
        Given agent moves one step left of it's current position as the next step in the list
        """
        a_pos = agent_a._path[agent_a._path_index]
        a_next_pos = agent_a.next_pos
        b_pos = agent_b._path[agent_b._path_index]
        b_next_pos = agent_b.next_pos
        a_next_pos = (a_pos[0], a_pos[1])
        if b_next_pos == a_pos:
            dy = b_pos[0]-a_pos[0]
            dx = b_pos[1]-a_pos[1]
            a_next_pos = (a_pos[0]+dx, a_pos[1]+dy)
        agent_a._path.insert(agent_a._path_index+1, a_next_pos)

    def find_collisions(self):
        """
        Find all potential collisions in the drones' next moves and then
        preferbaly don't collide pls :D
        """

        for agent in self._agent_list.values():
            agent.next_pos = agent._path[agent._path_index + 1]
            # print(agent.next_pos)
        # agent_next_pos = [agent._path[agent._path_index + 1] for agent in self._agent_list.values()]

        all_hitboxes = []
        # find all the hitboxes and adds the points to all_hitboxes list
        for agent in self._agent_list.values():
            all_hitboxes += self.get_hitbox(agent.next_pos)

        duplicates = [x for x in all_hitboxes if all_hitboxes.count(x) > 1]
        # print(f"duplicates: {duplicates}")
        bad_agents = []
        for agent in self._agent_list.values():
            if agent.next_pos in duplicates:
                bad_agents.append(agent)

        # print(f"bad agents: {bad_agents}")

        # is it good code? no. but that's okay
        fixed_pairs = []

        for agent_a in bad_agents:
            # print(f"bad agent a: {agent_a}")
            for agent_b in bad_agents:
                # print(f"agent b: {agent_b}")
                if {agent_a, agent_b} not in fixed_pairs and \
                        agent_a._id != agent_b._id and \
                        agent_b.next_pos in self.get_hitbox(agent_a.next_pos):
                    self._fix_collision(agent_b, agent_a)
                    fixed_pairs.append({agent_a, agent_b})
                if {agent_a, agent_b} not in fixed_pairs and \
                        agent_a._id != agent_b._id and \
                        agent_a.next_pos in self.get_hitbox(agent_b.next_pos):
                    self._fix_collision(agent_a, agent_b)
                    fixed_pairs.append({agent_a, agent_b})

    '''def go_and_dont_crash(self):
        # for now, we're assuming every agent always has a target next point
        # if that's not how the task assignment works we should figure that out

        self.find_collisions()
        for agent in self._agent_list.values():
            agent._path_index += 1'''


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
    time_weight: float = .05
    priority_weight: int = 10
    # unsure about priority weighting, may cause problems

    def steps_left_in_path(self):
        return self.agent.get_path_length()

    def path_end_loc(self):
        return self.agent.get_path_end()

    def get_euclidian_distance(self, pos_1, pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the euclidian (manhattan) distance between them 
        """
        x_dist = pos_1[0]-pos_2.x
        y_dist = pos_1[1]-pos_2.y
        # z_dist = pos_1[2]-pos_2.z
        return math.sqrt(abs(x_dist) ** 2+abs(y_dist) ** 2)  # +abs(z_dist) ^ 2

    def cost(self):
        distance_from_path = self.get_euclidian_distance(
            self.path_end_loc(), self.task.pick_loc)
        steps_left = self.steps_left_in_path()
        total_distance = (distance_from_path + steps_left) * self.dist_weight
        # time = time.time - self.task.time_input
        priority = self.task.priority * self.priority_weight

        cost = total_distance + priority  # - time
        return cost

    def __float__(self):
        """
        method to allow for direct comparison of Heuristic values like floats!
        """
        return float(self.cost())

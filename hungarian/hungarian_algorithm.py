import numpy as np
import time

class GroundControlSystem():
    def __init__(self, agent_list=None, task_list=None, env=None):
        self._agent_list = agent_list
        self._task_list = task_list
        self._task_assignment = None
        self._agent_paths = dict()
        self._env = env
        self._task_queue = task_list.copy()

    def assign_tasks(self):
        if len(self._agent_list) > len(self._task_list):
            self._cost_matrix = np.zeros((len(self._agent_list), len(self._agent_list)))
        else:
            self._cost_matrix = np.zeros((len(self._task_list), len(self._task_list)))

        self._agent_order = list(self._agent_list.keys())
        self._task_order = list(self._task_list.keys())

        for i, agent_id in enumerate(self._agent_order):
            for j, task_id in enumerate(self._task_order):
                self._cost_matrix[i][j] = self.generate_heuristic(self._agent_list[agent_id], self._task_list[task_id])

        print(f"Cost matrix: {self._cost_matrix}")

        assignments = self.hungarian_algorithm()
        total_cost, ans_cost_matrix = self.ans_calculation(self._cost_matrix, assignments)
        self._task_assignment = dict()
        for a in assignments:
            if a[0] < len(self._agent_list) and a[1] < len(self._task_list):
                agent_id = self._agent_order[a[0]]
                task_id = self._task_order[a[1]]
                self._task_assignment[self._agent_list[agent_id]] = self._task_list[task_id]
                del self._task_queue[task_id]

        print("Assignments: ")
        for assignment in self._task_assignment.items():
            print(f"{assignment[0]._id}: {assignment[1].id}")
        
        print("Remaining tasks: ")
        for task in self._task_queue.items():
            print(f"{task[1].id}")
    
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
        cost = distance*dist_weight-time*time_weight-task.priority
        return cost

    def get_time_since_input(self, time_input):
        """
        Finds how many seconds ago the task was inputted
        """
        current_time=time.time()
        return current_time-time_input

    def get_manhattan_distance(self, pos_1,pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the manhattan distance between them 
        """
        x_dist=pos_1.x-pos_2.x
        y_dist=pos_1.y-pos_2.y
        z_dist=pos_1.z-pos_2.z
        return abs(x_dist)+abs(y_dist)+abs(z_dist)

    def get_euclidian_distance(self, pos_1,pos_2):
        """
        Takes in 2 positions as 3 element lists [x,y,z] and finds the euclidian distance between them 
        """
        x_dist=pos_1.x-pos_2.x
        y_dist=pos_1.y-pos_2.y
        z_dist=pos_1.z-pos_2.z
        return (x_dist**2+y_dist**2+z_dist**2)**0.5

    def min_zero_row(self, zero_mat, mark_zero):
        
        '''
        The function can be splitted into two steps:
        #1 The function is used to find the row which containing the fewest 0.
        #2 Select the zero number on the row, and then marked the element corresponding row and column as False
        '''

        #Find the row
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

        #Transform the matrix to boolean matrix(0 = True, others = False)
        cur_mat = mat
        zero_bool_mat = (cur_mat == 0)
        zero_bool_mat_copy = zero_bool_mat.copy()

        #Recording possible answer positions by marked_zero
        marked_zero = []
        while (True in zero_bool_mat_copy):
            self.min_zero_row(zero_bool_mat_copy, marked_zero)
        
        #Recording the row and column positions seperately.
        marked_zero_row = []
        marked_zero_col = []
        for i in range(len(marked_zero)):
            marked_zero_row.append(marked_zero[i][0])
            marked_zero_col.append(marked_zero[i][1])

        #Step 2-2-1
        non_marked_row = list(set(range(cur_mat.shape[0])) - set(marked_zero_row))
        
        marked_cols = []
        check_switch = True
        while check_switch:
            check_switch = False
            for i in range(len(non_marked_row)):
                row_array = zero_bool_mat[non_marked_row[i], :]
                for j in range(row_array.shape[0]):
                    #Step 2-2-2
                    if row_array[j] == True and j not in marked_cols:
                        #Step 2-2-3
                        marked_cols.append(j)
                        check_switch = True

            for row_num, col_num in marked_zero:
                #Step 2-2-4
                if row_num not in non_marked_row and col_num in marked_cols:
                    #Step 2-2-5
                    non_marked_row.append(row_num)
                    check_switch = True
        #Step 2-2-6
        marked_rows = list(set(range(mat.shape[0])) - set(non_marked_row))

        return(marked_zero, marked_rows, marked_cols)

    def adjust_matrix(self, mat, cover_rows, cover_cols):
        cur_mat = mat
        non_zero_element = []

        #Step 4-1
        for row in range(len(cur_mat)):
            if row not in cover_rows:
                for i in range(len(cur_mat[row])):
                    if i not in cover_cols:
                        non_zero_element.append(cur_mat[row][i])
        min_num = min(non_zero_element)

        #Step 4-2
        for row in range(len(cur_mat)):
            if row not in cover_rows:
                for i in range(len(cur_mat[row])):
                    if i not in cover_cols:
                        cur_mat[row, i] = cur_mat[row, i] - min_num
        #Step 4-3
        for row in range(len(cover_rows)):  
            for col in range(len(cover_cols)):
                cur_mat[cover_rows[row], cover_cols[col]] = cur_mat[cover_rows[row], cover_cols[col]] + min_num
        return cur_mat

    def hungarian_algorithm(self): 
        dim = self._cost_matrix.shape[0]
        cur_mat = self._cost_matrix.copy()

        #Step 1 - Every column and every row subtract its internal minimum
        for row_num in range(self._cost_matrix.shape[0]): 
            cur_mat[row_num] = cur_mat[row_num] - np.min(cur_mat[row_num])
        
        for col_num in range(self._cost_matrix.shape[1]): 
            cur_mat[:,col_num] = cur_mat[:,col_num] - np.min(cur_mat[:,col_num])
        zero_count = 0
        while zero_count < dim:
            #Step 2 & 3
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
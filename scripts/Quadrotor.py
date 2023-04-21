""" This script holds the functions for controlling the crazyflies """

import numpy as np
from math import sin, cos
from scripts.utils import State, Position, VelCommand
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
import logging
import time
from threading import Event



# Quadrotor agent
class Quadrotor():
	
    def __init__(self, init_state=None, 
                 axes=None, color=None, 
                 id=None, uri=None,
                 take_off_height=0.5,
                 hardware_flag=False, dt=None):
        self._id = id
        self._ax = axes 
        self._color = color

        # initialize state
        self._state = init_state

        # set trajectory track
        self._x_track = []
        self._y_track = []
        self._z_track = []
        self._phi_track = []
        self._theta_track = []
        self._psi_track = []

        # task and trajectory
        self._task_queue = []
        self._task_to_plan = ""
        self._path = []
        self._path_index = 0

        # Used for LRA*
        self.next_pos=(0,0)

        # set up system
        # get/set simulation parameters
        self._time_delta = dt

        # parameters for simulated take-off and landing velocity control
        self._K, self._height_threshold = 2.0, 0.01
        self._vz_max = 0.15 # maximum velocity for take-off and landing in simulation

        # operating height for the crazyflie
        self._take_off_height = take_off_height

        # crazyflie hardware setup
        self._hardware_flag = hardware_flag
        self._is_flowdeck_attached = False
        self._deck_attached_event = Event()

        if self._hardware_flag:
            cflib.crtp.init_drivers(enable_debug_driver=False)
            self.URI = uri_helper.uri_from_env(default=uri)

            # initialize variables for logging
            self.range_left, self.range_front = 600, 600
            self.range_right, self.range_back = 600, 600

            # initiate Crazyflie objects
            self.scf = SyncCrazyflie(self.URI, cf=Crazyflie(rw_cache='./cache'))

            # open the syncCrazyflie link
            self.scf.open_link()

            # check decks (e.g. flowdeck)
            self.scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                cb=self.FlowDeckCheck)
            
            # reset the estimator
            self.reset_estimator()

            # Logging data
            logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework
                # position
            self.logconf_pos = LogConfig(name='Position', period_in_ms=10)
            self.logconf_pos.add_variable('stateEstimate.x', 'float')
            self.logconf_pos.add_variable('stateEstimate.y', 'float')
            self.logconf_pos.add_variable('stateEstimate.z', 'float')
            self.scf.cf.log.add_config(self.logconf_pos)
            self.logconf_pos.data_received_cb.add_callback(self.log_pos_callback)
                # orientation
            self.logconf_orient = LogConfig(name='Stabilizer', period_in_ms=10)
            self.logconf_orient.add_variable('stabilizer.roll', 'float')
            self.logconf_orient.add_variable('stabilizer.pitch', 'float')
            self.logconf_orient.add_variable('stabilizer.yaw', 'float')
            self.scf.cf.log.add_config(self.logconf_orient)
            self.logconf_orient.data_received_cb.add_callback(self.log_orient_callback)
                # multi-ranger
            self.logconf_range = LogConfig(name='Range', period_in_ms=10)
            self.logconf_range.add_variable('range.left', 'float')
            self.logconf_range.add_variable('range.front', 'float')
            self.logconf_range.add_variable('range.back', 'float')
            self.logconf_range.add_variable('range.right', 'float')
            self.scf.cf.log.add_config(self.logconf_range)
            self.logconf_range.data_received_cb.add_callback(self.log_range_callback)


    def initialize_agent(self):
        """handles agent take off and logger initiation"""
        if self._hardware_flag:
            if self._is_flowdeck_attached:
                self.logconf_pos.start()
                self.logconf_range.start()

                print(f'Agent [{self._id}] | Started MotionCommander!') 

                # instantiate the motion commander to enable use of start linear motion
                self.mc = MotionCommander(self.scf, default_height=self._take_off_height)

                self.take_off()
        else: 
            self.take_off()


    def plot2D(self, fig):
        """updates the given figure with new 2D position data"""
        fig.add_scatter(x=[self._state.x_pos], y=[self._state.y_pos], mode='markers',
                            marker=dict(color=self._color, size=5,  
                            symbol='circle'))

    def update(self):
        """Commands the drone to go to the next step of its path"""
        # if self._path_index + 2 >= len(self._path):
        #     self._path.append(self._path[-1])

        if self._path_index + 1 < len(self._path):
            self._path_index += 1
            next_pos = self._path[self._path_index]
            # kinda a fucky fix for the time being but yes
            pos_setpoint = [next_pos[1], next_pos[0], self._take_off_height, 0]
            if self._hardware_flag:
                self.position_setpoint_hw(pos_setpoint)
            else:
                self.position_setpoint_sim(pos_setpoint)

    def velocity_setpoint_sim(self, vel_cmd):
        """advances the system state using a simple kinematic model and the commanded velocity"""

        # advance state using vel_cmd
        self._state.x_pos += vel_cmd.vx * self._time_delta
        # self._state.x_pos += vel_cmd.vx * cos(self._state.psi) * self._time_delta
        self._state.y_pos += vel_cmd.vy * self._time_delta
        # self._state.y_pos += vel_cmd.vy * sin(self._state.psi) * self._time_delta
        self._state.z_pos += vel_cmd.vz * self._time_delta
        self._state.psi += vel_cmd.v_psi * self._time_delta

        # store updates
        self.update_state_trace()


    def velocity_setpoint_hw(self, vel_cmd):
        """calls the Motion commander module to send velocity setpoint packets to the Crazyflie"""
        # using motion commander 
        # (https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/positioning/motion_commander.py#L384)
        self.mc.start_linear_motion(vel_cmd.vx,
                                    vel_cmd.vy,
                                    vel_cmd.vz,
                                    vel_cmd.v_psi)

        # store updates
        self.update_state_trace()


    def position_setpoint_sim(self, pos_cmd):
        self._state.x_pos = pos_cmd[0]
        self._state.y_pos = pos_cmd[1]
        self._state.z_pos = pos_cmd[2]
        self._state.psi = pos_cmd[3]

        self.update_state_trace()


    def position_setpoint_hw(self, pos_cmd):
        """calls the Position commander module to send position setpoint packets to the Crazyflie"""
        cf = self.scf.cf
        cf.commander.send_position_setpoint(pos_cmd[0],
                                            pos_cmd[1],
                                            pos_cmd[2],
                                            pos_cmd[3])


    def path_complete(self):
        """Checks if a drone has reached the end of it's path"""
        if len(self._path) < 1:
            return False
        return self._path_index >= len(self._path) - 1
        # dist_thr = 0.05
        # point1 = np.asarray([self._state.x_pos, self._state.y_pos, self._state.z_pos])
        # point2 = np.asarray([self._path[-1][1], self._path[-1][0], .5])
        # return True if np.linalg.norm(point1-point2) < dist_thr else False
    
    def log_pos_callback(self, timestamp, data, logconf):
        """Logs drone position"""
        self._state.x_pos = data['stateEstimate.x']
        self._state.y_pos = data['stateEstimate.y']
        self._state.z_pos = data['stateEstimate.z']


    def log_orient_callback(self, timestamp, data, logconf):
        """Logs drone orientation"""
        self._state.phi = data['stabilizer.roll']
        self._state.theta = data['stabilizer.pitch']
        self._state.psi = data['stabilizer.yaw']

    
    def log_range_callback(self, timestamp, data, logconf):
        """Logs drone rangefinder readings"""
        self.range_left = data['range.left']
        self.range_right = data['range.right']
        self.range_front = data['range.front']
        self.range_back = data['range.back']

    def clear_tasks_and_path(self):
        """Resets the drone's tasks and paths"""
        self._task_queue = []
        self._task_to_plan = ""
        self._path = []
        self._path_index = 0

    def available_for_task(self):
        """Check if there is room in the drone's task queue for another task."""
        return len(self._task_queue) <= 3

    def add_task(self, task):
        """Add a task to the drone's queue"""
        self._task_queue.append(task)
        self._task_to_plan = task.id
    
    def get_task_queue(self):
        return self._task_queue
    
    def get_task_to_plan(self):
        return self._task_to_plan

    def remove_planned_task(self):
        self._task_to_plan = ""
    
    def set_path(self, path):
        self._path = path

    def add_to_path(self, new_path):
        self._path += new_path

    def get_next_pos(self):
        if self._path_index + 1 < len(self._path):
            return self._path[self._path_index + 1]
        return None
    
    def get_path_length(self):
        """
        number of steps remaining in agent's path
        """
        if self._path_index + 1 < len(self._path):
            return len(self._path)-self._path_index
        return 1
        
    def get_path_end(self):
        if self._path:
            return self._path[-1]
        else:
            return (self.get_pos().y, self.get_pos().x)

    def get_pos(self):
        return Position(x=self._state.x_pos, y=self._state.y_pos, z=self._state.z_pos)

    def get_attitude(self):
        return

    def take_off(self):
        """Commands the drone to begin flying"""
        if self._hardware_flag:
            self.mc.take_off(height=self._take_off_height)
            print(f'Agent [{self._id}] is Taking Off!!')
        else:
            print(f'Agent [{self._id}] is Taking Off!!')
            while np.sqrt((self._state.z_pos-self._take_off_height)**2) > self._height_threshold:
                # print(f'Agent [{self._id}] z pos: {self._state.z_pos}')
                vel = VelCommand()
                vel.vz = np.clip(self._K * (self._take_off_height - self._state.z_pos), -self._vz_max, self._vz_max)
                self.velocity_setpoint_sim(vel)


    def land(self):
        """Commands the drone to stop flying"""
        if self._hardware_flag:
            self.mc.land()
            print(f'Agent [{self._id}] is Landing!!')
        else:
            print(f'Agent [{self._id}] is Landing!!')
            while (self._state.z_pos**2) > self._height_threshold:
                vel = VelCommand()
                vel.vz = np.clip(self._K * (-self._state.z_pos), -self._vz_max, self._vz_max)
                self.velocity_setpoint_sim(vel)


    def update_state_trace(self):
        """Update the visualization of the drone path"""
        self._x_track.append(self._state.x_pos)
        self._y_track.append(self._state.y_pos)
        self._z_track.append(self._state.z_pos)


    def reset_estimator(self):
        """Resets the kalmal filter on the state estimator"""
        cf = self.scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

        self.wait_for_position_estimator()


    def wait_for_position_estimator(self):
        """Checks if the drone position is being tracked by the lighthouses"""
        # print('Waiting for estimator to find position...')
        print(f'Agent [{self._id}] | Waiting for estimator to find position...') 

        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(self.scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                # print("{} {} {}".
                #       format(max_x - min_x, max_y - min_y, max_z - min_z))

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    print(f'Agent [{self._id}] | Position estimate within threshold! Good to go!') 
                    break


    def FlowDeckCheck(self, name, value_str):
        """Checks if a flowdeck is attached"""
        value = int(value_str)
        # print(value)
        if value:
            self._deck_attached_event.set()
            self._is_flowdeck_attached = True
            # print(f'Agent [{self._id}] | FlowDeck is attached!') 
        # else:
            # print(f'Agent [{self._id}] | FlowDeck is NOT attached!') 

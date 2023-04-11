from numpy import *


class Simulation():

    def __init__(self, env=None, fig1=None, fig2=None):
        self._agent_list = None
        self._task_list = None
        self._env = env

        self._ply_fig1 = fig1
        self._ply_fig2 = fig2


    def add_agents(self, agent_list):
        self._agent_list = agent_list


    def set_task_list(self, task_list):
        self._task_list = task_list


    def init_plot(self):
        """creates the plotly 2D and 3D plots"""
        # 2D plot
        self.create_2D_plot()
        self._ply_fig1.show()

        # 3D plot
        # self.create_3D_plot()
        # self._ply_fig2.show()


    def update_plot(self):
        """updates the plotly 2D and 3D with drone traces"""
        # plot all the agents
        for agent in self._agent_list.values():
            self._ply_fig1.add_scatter(x=agent._x_track, y=agent._y_track, mode='lines+markers',
                            marker=dict(color=agent._color, size=5,  
                            symbol='circle'),
                            text=list(range(len(agent._x_track))))
            
            # self._ply_fig2.add_scatter3d(x=agent._x_track, y=agent._y_track,
            #                             z=agent._z_track, mode='lines+markers',
            #                             marker=dict(size=3, symbol='circle'))

        self._ply_fig1.show()
        # self._ply_fig2.show()


    def create_2D_plot(self):
        """creates 2D plot showing obstacles, agent start locations and task locations"""
        map_x = []
        map_y = []

        obstacles = self._env['obstacles']

        for i in range(len(obstacles)):
            map_x.append(obstacles[i][0]/10)
            map_y.append(obstacles[i][1]/10)

        self._ply_fig1.add_scatter(x=map_x, y=map_y, mode='markers',
                                   marker=dict(color='LightSkyBlue', size=20),
                                   showlegend=False)


        # plot agent start locations
        for agent in self._agent_list.values():        
            self._ply_fig1.add_scatter(x=[agent.get_pos().x], y=[agent.get_pos().y], mode='markers',
                                    marker=dict(color=agent._color, size=15,  
                                    symbol='arrow-right'),
                                    name=agent._id)
            
        # plot pick and drop locations
        pick_x, pick_y, drop_x, drop_y = [], [], [], []
        for t in self._task_list.values():
            pick_x.append(t.pick_loc.x)
            pick_y.append(t.pick_loc.y)
            drop_x.append(t.drop_loc.x)
            drop_y.append(t.drop_loc.y)
        
        self._ply_fig1.add_scatter(x=pick_x, y=pick_y, mode='markers',
                                marker=dict(color='Red', size=15),
                                name="Pick locations")
        self._ply_fig1.add_scatter(x=drop_x, y=drop_y, mode='markers',
                                marker=dict(color='DarkGreen', size=15,
                                            symbol='square'),
                                            name="Drop locations")

        self._ply_fig1.update_xaxes(range=[-1,2.2],
                                   constrain="domain")
        self._ply_fig1.update_yaxes(range=[-1,1.2],
                                   scaleanchor="x",
                                   scaleratio=1)


    def create_3D_plot(self):
        """creates 3D plot showing agent start locations and task locations"""
        
        # plot the obstacles
        map_x = []
        map_y = []
        map_z = []

        obstacles = self._env['obstacles']

        num = linspace(0,0.6,12)
        for j in num:
            for i in range(len(obstacles)):
                map_x.append(obstacles[i][0]/10)
                map_y.append(obstacles[i][1]/10)
                map_z.append(j)
        
        self._ply_fig2.add_scatter3d(x=map_x, y=map_y, z=map_z,
                                    mode='markers', showlegend=False)

        self._ply_fig2.update_layout(
            scene = dict(
                xaxis = dict(nticks=6, range=[-1,2.2],),
                            yaxis = dict(nticks=6, range=[-1,1.2],),
                            zaxis = dict(nticks=6, range=[0,2],),),
            width=700,
            margin=dict(r=20, l=10, b=10, t=10))
        

        # plot agent start locations
        for agent in self._agent_list.values():        
            self._ply_fig2.add_scatter3d(x=[agent.get_pos().x], y=[agent.get_pos().y],
                                        z=[agent.get_pos().z+0.075], mode='markers',
                                        marker=dict(size=10, symbol='circle'),
                                        name=agent._id)


        # plot pick and drop locations
        pick_x, pick_y, pick_z, drop_x, drop_y, drop_z = [], [], [], [], [], []
        for t in self._task_list.values():
            pick_x.append(t.pick_loc.x)
            pick_y.append(t.pick_loc.y)
            pick_z.append(t.pick_loc.z)
            drop_x.append(t.drop_loc.x)
            drop_y.append(t.drop_loc.y)
            drop_z.append(0.1)
        
        self._ply_fig2.add_scatter3d(x=pick_x, y=pick_y, z=pick_z, mode='markers',
                                marker=dict(color='rgba(240, 10, 10, 1.0)', size=10),
                                name="Pick locations")
        self._ply_fig2.add_scatter3d(x=drop_x, y=drop_y, z=drop_z, mode='markers',
                                marker=dict(color='rgba(0, 100, 0, 1.0)', size=10,
                                            symbol='square'), 
                                name="Drop locations")

from numpy import *
import pandas as pd
import plotly.express as px


class Simulation:
    def __init__(
        self, env=None, fig1=None, fig2=None, fig_animated=None, fig3d_animated=None
    ):
        self._agent_list = None
        self._task_list = None
        self._env = env

        self._ply_fig1 = fig1
        self._ply_fig2 = fig2
        self._ply_fig_animated = fig_animated
        self._ply_fig3d_animated = fig3d_animated

    def init_plot(self):
        """creates the plotly 2D and 3D plots"""
        # 2D plot
        self.create_2D_plot(self._ply_fig1)
        # self._ply_fig1.show()

        # 3D plot
        self.create_3D_plot(self._ply_fig2)
        # self._ply_fig2.show()

    def add_agents(self, agent_list):
        self._agent_list = agent_list

    def set_task_list(self, task_list):
        self._task_list = task_list

    def update_plot(self):
        """updates the plotly 2D and 3D with drone traces"""

        agent_traces = {
            "agent_id": [],
            "step": [],
            "x": [],
            "y": [],
            "z": [],
            "size": 1,
        }
        agent_colors, agent_symbol = [], []
        for agent in self._agent_list.values():
            agent_traces["agent_id"] += [agent._id] * len(agent._x_track)
            agent_traces["step"] += list(range(len(agent._x_track)))
            agent_traces["x"] += agent._x_track
            agent_traces["y"] += agent._y_track
            agent_traces["z"] += agent._z_track
            agent_colors.append(agent._color)
            agent_symbol.append(1)
            # TO DO: figure out how to add the rectangle around the moving point
            # Attemped to add a rectangle feature that would move with the centerpoint of the drone
            # fig_animated.add_shape(
            #     type="rect",
            #     x0=agent._state.x_pos - 1.5,
            #     y0=agent._state.y_pos - 1.5,
            #     x1=agent._state.x_pos + 1.5,
            #     y1=agent._state.y_pos + 1.5,
            #     line=dict(color=agent._color),
            #     fillcolor=agent._color,
            # )
        # print(f"xpos: {agent._state.x_pos}, y pos: {agent._state.y_pos}")

        df = pd.DataFrame(data=agent_traces)    

        agent_symbol = [0] * len(agent_traces["x"])
        symbols = ['square-dot', 'circle', 'square', 'diamond']
        marker_sizes = [40] * len(agent_traces["x"])

        self._ply_fig_animated = px.scatter(
            df,
            x="x",
            y="y",
            animation_frame="step",
            animation_group="agent_id",
            color="agent_id",
            color_discrete_sequence=agent_colors,
            symbol=agent_symbol,
            symbol_sequence=symbols,
            size=marker_sizes,
            size_max=80,
            range_x=[-1, 2],
            range_y=[-1, 1],
        )

        # self._ply_fig_animated.update_traces(marker=dict(size=25, symbol="circle"))
        # self._ply_fig_animated.update_traces(marker=dict(size=25, symbol="square-dot"))
        self.create_2D_plot(self._ply_fig_animated)
        self._ply_fig_animated.show()

        self._ply_fig3d_animated = px.scatter_3d(
            df,
            x="x",
            y="y",
            z="z",
            animation_frame="step",
            animation_group="agent_id",
            color="agent_id",
            color_discrete_sequence=agent_colors,
            range_x=[-1, 2],
            range_y=[-1, 1],
            range_z=[0, 2],
        )
        self._ply_fig3d_animated.update_traces(marker=dict(size=8, symbol="circle"))
        self.create_3D_plot(self._ply_fig3d_animated)
        self._ply_fig3d_animated.show()

        # plot all the agents
        for agent in self._agent_list.values():
            self._ply_fig1.add_scatter(
                x=agent._x_track,
                y=agent._y_track,
                mode="lines+markers",
                marker=dict(color=agent._color, size=5, symbol="circle"),
                text=list(range(len(agent._x_track))),
            )

            self._ply_fig2.add_scatter3d(
                x=agent._x_track,
                y=agent._y_track,
                z=agent._z_track,
                mode="lines+markers",
                marker=dict(color=agent._color, size=3, symbol="circle"),
            )

        self._ply_fig1.show()

        self._ply_fig2.show()

    def create_2D_plot(self, fig):
        """creates 2D plot showing obstacles, agent start locations and task locations"""

        obstacles = self._env["obstacles"]

        for obstacle in obstacles:
            fig.add_shape(
                type="rect",
                x0=obstacle[0][0] / 10,
                y0=obstacle[0][1] / 10,
                x1=obstacle[1][0] / 10,
                y1=obstacle[1][1] / 10,
                line=dict(color="LightSkyBlue"),
                fillcolor="LightSkyBlue",
            )

        bins = self._env["bins"]

        for drop_bin in bins:
            fig.add_shape(
                type="rect",
                x0=drop_bin[0][0] / 10,
                y0=drop_bin[0][1] / 10,
                x1=drop_bin[1][0] / 10,
                y1=drop_bin[1][1] / 10,
                line=dict(color="LightGreen"),
                fillcolor="LightGreen",
            )

        bases = self._env["bases"]

        for base in bases:
            fig.add_shape(
                type="rect",
                x0=base[0][0] / 10,
                y0=base[0][1] / 10,
                x1=base[1][0] / 10,
                y1=base[1][1] / 10,
                line=dict(color="Blue"),
                fillcolor="Blue",
            )
        # plot all nodes
        nodes = self._env["nodes"]

        for node in nodes:
            fig.add_scatter(
                x=[node[1][0]],
                y=[node[1][1]],
                mode="markers",
                marker=dict(color="LightGrey", size=15, line_width=3, symbol="x-thin"),
            )

        # plot agent start locations
        for agent in self._agent_list.values():
            fig.add_scatter(
                x=[agent.base_station.x],
                y=[agent.base_station.y],
                mode="markers",
                marker=dict(color=agent._color, size=15, symbol="arrow-left"),
                name=f"{agent._id} Base Station",
            )

        # plot pick and drop locations
        pick_x, pick_y, drop_x, drop_y = [], [], [], []
        for t in self._task_list.values():
            pick_x.append(t.pick_loc.x)
            pick_y.append(t.pick_loc.y)
            drop_x.append(t.drop_loc.x)
            drop_y.append(t.drop_loc.y)

        fig.add_scatter(
            x=pick_x,
            y=pick_y,
            mode="markers",
            marker=dict(color="Red", size=15),
            name="Pick locations",
        )
        fig.add_scatter(
            x=drop_x,
            y=drop_y,
            mode="markers",
            marker=dict(color="DarkGreen", size=15, symbol="circle"),
            name="Drop locations",
        )

        fig.update_xaxes(range=[0, 7], constrain="domain")
        fig.update_yaxes(range=[0, 5.5], scaleanchor="x", scaleratio=1)

        # fig.update_shapes(dict(xref="x", yref="y"))

    def create_3D_plot(self, fig):
        """creates 3D plot showing agent start locations and task locations"""

        # plot the obstacles
        map_x = []
        map_y = []
        map_z = []

        obstacles = self._env["obstacles"]

        # num = linspace(0,0.6,12)
        # for j in num:
        #     for i in range(len(obstacles)):
        #         map_x.append(obstacles[i][0]/10)
        #         map_y.append(obstacles[i][1]/10)
        #         map_z.append(j)

        map_obstacles = []
        for obstacle in obstacles:
            x0 = obstacle[0][0] / 10
            y0 = obstacle[0][1] / 10
            x1 = obstacle[1][0] / 10
            y1 = obstacle[1][1] / 10
            map_obstacles.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])

        for obstacle in map_obstacles:
            obs_x = [p[0] for p in obstacle]
            obs_y = [p[1] for p in obstacle]
            for i in linspace(0, 0.6, 12):
                obs_z = [i] * len(obstacle)
                fig.add_mesh3d(x=obs_x, y=obs_y, z=obs_z, color="skyblue")

        # self._ply_fig2.add_scatter3d(x=map_x, y=map_y, z=map_z,
        #                             mode='markers', showlegend=False)

        fig.update_layout(
            scene=dict(
                xaxis=dict(
                    nticks=6,
                    range=[0, 7],
                ),
                yaxis=dict(
                    nticks=6,
                    range=[0, 5.5],
                ),
                zaxis=dict(
                    nticks=6,
                    range=[0, 2],
                ),
            ),
            width=700,
            margin=dict(r=20, l=10, b=10, t=10),
        )

        # plot agent start locations
        for agent in self._agent_list.values():
            fig.add_scatter3d(
                x=[agent.base_station.x],
                y=[agent.base_station.y],
                z=[agent.base_station.z + 0.075],
                mode="markers",
                marker=dict(color=agent._color, size=10, symbol="circle"),
                name=agent._id,
            )

        # plot pick and drop locations
        pick_x, pick_y, pick_z, drop_x, drop_y, drop_z = [], [], [], [], [], []
        for t in self._task_list.values():
            pick_x.append(t.pick_loc.x)
            pick_y.append(t.pick_loc.y)
            pick_z.append(t.pick_loc.z)
            drop_x.append(t.drop_loc.x)
            drop_y.append(t.drop_loc.y)
            drop_z.append(0.1)

        fig.add_scatter3d(
            x=pick_x,
            y=pick_y,
            z=pick_z,
            mode="markers",
            marker=dict(color="rgba(240, 10, 10, 1.0)", size=10),
            name="Pick locations",
        )
        fig.add_scatter3d(
            x=drop_x,
            y=drop_y,
            z=drop_z,
            mode="markers",
            marker=dict(color="rgba(0, 100, 0, 1.0)", size=10, symbol="square"),
            name="Drop locations",
        )

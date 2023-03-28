from simulation import Simulation
import plotly.graph_objects as go
import yaml

with open("./base_config.yaml") as f:
    config = yaml.load(f, Loader = yaml.FullLoader)

env = config["map"]



# sim.init_plot()

from itertools import product

def define_env(corners):
    """
    Take in the two corners of rectangles from the yaml file
    and define all the points as obstacles
    """
    # corners = env["obstacles"]
    obstacles = []

    x_vals = []
    y_vals = []
    for pair in corners:
        x_vals += [x for x in range(pair[0][0], pair[1][0] + 1)]
        y_vals +=  [y for y in range(pair[0][1], pair[1][1] + 1)]
        obstacles += product(x_vals, y_vals)
        
    env["obstacles"] = list(set(obstacles))
    return env

# define_env(env["obstacles"])
# print(define_env(env["obstacles"]))

env = define_env(env["obstacles"])

fig1 = go.Figure()
fig2 = go.Figure()

sim = Simulation(env = env, fig1 = fig1, fig2 = fig2)
sim.init_plot()
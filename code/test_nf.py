import os
import sys

sys.path.append(os.getcwd())


import time
import matplotlib.pyplot as plt
import numpy as np


from NF.geometry import World
from NF.navigation import NavigationFunction
from NF.vis import plot_path_on_contour, plot_complexity, compute_total_cost

if __name__ == '__main__':
    # plot the navigation contour and navigation path

    fig = plt.figure(figsize=(9, 6))

    ax = fig.add_subplot(111)

    goal_pose = np.array([6.0,1.0,3.17]) #1.5,1.9,2.5
    start_pose = np.array([1.1075406863075947,2.719831148144997,-2.066782172437256])
    world_config = './CONFIG/squircle_world.yaml'


    world = World(world_config)
    # print(world.workspace)

    point = np.array([1.5, 2.5])

    nf = NavigationFunction(world, goal_pose, nf_lambda = 0.01, nf_mu = [1e20, 1e20, 1e20, 1e6])


    plot_path_on_contour(ax, nf, start_pose)

    plt.show()
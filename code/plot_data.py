import pickle
import os
import sys

sys.path.append(os.getcwd())

import numpy as np
import matplotlib.pyplot as plt
import datetime
from ENV.vis import plot_occupancy_world, plot_trajectory, plot_data, plot_cluster, plot_init_data, plot_cluster_segment, plot_fitting_squircle, plot_polygon_list, plot_nm_path


file_path = 'DATA/pickle_data/'
current_date_execution = datetime.datetime.now().strftime("execution_%Y_%m_%d")

# current_date_execution = 'execution_2024_04_26'

folder_path_execution = os.path.join(file_path, current_date_execution)

with open(folder_path_execution + '/execution.pickle', 'rb') as f:
    execution_data = pickle.load(f)

fig = plt.figure(figsize=(18, 6))

grid_resolution=0.05
grid_size_x = abs(execution_data[0].forest_world.workspace[0][0].x_limits()[0] - \
                  execution_data[0].forest_world.workspace[0][0].x_limits()[1])
grid_size_y = abs(execution_data[0].forest_world.workspace[0][0].y_limits()[0] - \
                  execution_data[0].forest_world.workspace[0][0].y_limits()[1])
grid_size_x = int(grid_size_x / grid_resolution)
grid_size_y = int(grid_size_y / grid_resolution)
grid_map = np.zeros((grid_size_y, grid_size_x))

grid_wall = np.zeros((grid_size_y, grid_size_x))

ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)
print("overall frame: ", len(execution_data))

plot_step = len(execution_data)


for i in range(0, plot_step - 1):
    # if i == 508 or i == 520:
    #     continue
    ax1.clear()
    ax2.clear()
    plot_data(ax1, execution_data[i].robot, execution_data[i].forest_world)
    plot_trajectory(ax1, execution_data[i].trajectory)
    plot_trajectory(ax2, execution_data[i].trajectory)

    plot_cluster(ax2, execution_data[i].sk_clu, execution_data[i].forest_world)
    plot_init_data(ax2, execution_data[i].robot, execution_data[i].forest_world)
    plot_cluster_segment(ax2, execution_data[i].forest_world)
    plot_fitting_squircle(ax2, execution_data[i].forest_world)
    # plot_polygon_list(ax2, execution_data[i].all_polygon_list)
    plot_nm_path(ax2, execution_data[i].navigation_map.path)
    # plot_occupancy_world(ax2, execution_data[i], grid_map, grid_wall, grid_resolution)
    file_path = './DATA/figure_data'
    current_date = datetime.datetime.now().strftime("snap_%Y_%m_%d")
    folder_path = os.path.join(file_path, current_date)
    # folder_path = './DATA/figure_data/snap_test'

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    file_name = f"{i}.png"
    # print("path", os.path.join(folder_path, file_name))
    plt.savefig(os.path.join(folder_path, file_name), dpi=200)
    print("=============frame " + str(i) + " is saved==============")

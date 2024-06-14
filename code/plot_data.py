import pickle
import os
import sys

sys.path.append(os.getcwd())

import numpy as np
import matplotlib.pyplot as plt
import datetime
from ENV.vis import plot_four_ax, plot_occupancy_world

file_path = 'DATA/pickle_data/'
current_date_execution = datetime.datetime.now().strftime("execution_%Y_%m_%d")

current_date_execution = 'execution_2024_04_26'

folder_path_execution = os.path.join(file_path, current_date_execution)

with open(folder_path_execution + '/execution.pickle', 'rb') as f:
    execution_data = pickle.load(f)

fig = plt.figure(figsize=(9, 6))

grid_resolution=0.05
grid_size_x = abs(execution_data[0].robot.real_world.x_limits[0] - execution_data[0].robot.real_world.x_limits[1])
grid_size_y = abs(execution_data[0].robot.real_world.y_limits[0] - execution_data[0].robot.real_world.y_limits[1])
grid_size_x = int(grid_size_x / grid_resolution)
grid_size_y = int(grid_size_y / grid_resolution)
grid_map = np.zeros((grid_size_y, grid_size_x))

grid_wall = np.zeros((grid_size_y, grid_size_x))

ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
print("overall frame: ", len(execution_data))

plot_step = len(execution_data)

# from matplotlib.patches import Polygon
#
# all_squircle = execution_data[-1].all_squircles
# for squircle_i in all_squircle:
#     center = squircle_i.center
#     width = squircle_i.width
#     height = squircle_i.height
#     polygon = [[center[0] - width / 2, center[1] - height / 2],
#                [center[0] - width / 2, center[1] + height / 2],
#                [center[0] + width / 2, center[1] + height / 2],
#                [center[0] + width / 2, center[1] - height / 2]]
#     poly = Polygon(polygon, edgecolor='red', alpha=0.7, fill=False, zorder=2)
#     ax4.add_patch(poly)
#     if squircle_i.ori_line is not None:
#         ax4.plot((squircle_i.ori_line[0][0], squircle_i.ori_line[1][0]),
#                  (squircle_i.ori_line[0][1], squircle_i.ori_line[1][1]), color='blue',
#                  linewidth=2.0, alpha=0.8, zorder=6)
# ax4.set_xlim([- 0.5, 7.5])
# ax4.set_ylim([- 0.5, 4.5])
# plt.show()d

for i in range(plot_step - 2, plot_step - 1):
    # if i == 508 or i == 520:
    #     continue
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    plot_four_ax(ax1, ax2, ax3, ax4, execution_data[i])
    plot_occupancy_world(ax2, execution_data[i], grid_map, grid_wall, grid_resolution)
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

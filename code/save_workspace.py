import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
import numpy as np
import copy
from NF.geometry import World
from ENV.geometry import RealWorld, RobotWorld, InitWorld, Squircle, ForestWorld

from NF.vis import plot_world, plot_task_area, plot_squircle, plot_init_world, plot_robot, plot_inflate_world
from ROBOT.robot import Robot
from ROBOT.vis import plot_robot_in_world


fig = plt.figure(figsize=(18, 6))

ax = fig.add_subplot(121)

start_pose = np.array([3.2, 0.4, -np.pi])  # 0.5, 0.5, 1.57
# start_pose = np.array([6.3, 1.5, -np.pi / 2])
p_1 = [0.4, 0.4, -np.pi / 2]
p_2 = [2.2, 3.5, -0.2]
d_1 = [0.9, 3.5, -np.pi]
d_2 = [6.3, 0.5, -np.pi / 2]
d_3 = [6.4, 3.0, -np.pi / 2]
u_1 = [4.6, 0.5, -np.pi]
goal_pose_list = [p_1, p_2, d_1, u_1, d_2, d_3]
# goal_pose_list = [d_3]

init_world_config = './CONFIG/workspace.yaml'
forest_config = './CONFIG/estimation_world.yaml'
robot_config = './ROBOT/robot_config.yaml'

init_world = InitWorld(init_world_config)
forest_world = World(forest_config)
# robot_world = RobotWorld()
# robot = Robot(start_pose, forest_world, init_world, robot_config=robot_config)

# extention = 0.1
# workspace = []
# obstacles = []
# for ws_group in forest_world.workspace:
#     ori_ws_group = [forest_world.workspace[0][0]]
#     for ws_i in ws_group[1:]:
#         ori_squircle = Squircle('Rectangular', ws_i.center, ws_i.width + 2 * extention,
#                                 ws_i.height + 2 * extention, ws_i.theta, ws_i.s)
#         ori_ws_group.append(ori_squircle)
#     workspace.append(ori_ws_group)
# for obs_group in forest_world.obstacles:
#     ori_obs_group = []
#     for obs_i in obs_group:
#         ori_squircle = Squircle('Rectangular', obs_i.center, obs_i.width + 2 * extention,
#                                 obs_i.height + 2 * extention, obs_i.theta, obs_i.s)
#         ori_obs_group.append(ori_squircle)
#     obstacles.append(ori_obs_group)
# ori_forest_world = ForestWorld(workspace, obstacles)

plot_world(ax, forest_world)
# plot_robot_in_world(ax, robot)
# plot_inflate_world(ax, ori_forest_world)
# plot_task_area(ax)

ec_color = "black"
face_color = "deepskyblue"
line_width = 1.0
task_area_r_0 = [1.6, 1.05, 0.35, 0.35]
circle_r_0 = plt.Rectangle((task_area_r_0[0]-task_area_r_0[2] / 2, task_area_r_0[1]-task_area_r_0[3] / 2),
                            width=task_area_r_0[2], height=task_area_r_0[3], fill=True,
                            ec=ec_color, facecolor=face_color, linewidth=line_width, zorder=29)

ax.add_patch(circle_r_0)
ax.text(task_area_r_0[0] - 0.13, task_area_r_0[1] - 0.05, '$p_1$', fontsize=13, color='k', zorder = 30)

file_path = './RESULT/workspace/'

file_name = "estimation_world.png"
# print("path", os.path.join(folder_path, file_name))
plt.savefig(os.path.join(file_path, file_name), dpi=200)
print("=============workspace is saved==============")

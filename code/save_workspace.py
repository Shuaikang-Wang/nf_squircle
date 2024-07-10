import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
import numpy as np
import copy
from NF.geometry import World
from ENV.geometry import RealWorld, RobotWorld, InitWorld
from NF.vis import plot_world, plot_task_area, plot_squircle, plot_init_world, plot_robot
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
forest_config = './CONFIG/forest_world.yaml'
robot_config = './ROBOT/robot_config.yaml'

init_world = InitWorld(init_world_config)
forest_world = World(forest_config)
# robot_world = RobotWorld()
robot = Robot(start_pose, forest_world, init_world, robot_config=robot_config)

plot_world(ax, forest_world)
plot_robot_in_world(ax, robot)
plot_task_area(ax)
file_path = './RESULT/workspace/'

file_name = "forest_world.png"
# print("path", os.path.join(folder_path, file_name))
plt.savefig(os.path.join(file_path, file_name), dpi=1200)
print("=============workspace is saved==============")

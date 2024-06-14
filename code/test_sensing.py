import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import datetime

from NF.geometry import World
from ENV.geometry import RealWorld, RobotWorld, InitWorld
from EXEC.execution import Execution
from ENV.vis import plot_data, plot_trajectory, plot_cluster, plot_init_data, plot_cluster_segment
from ENV.cluster import SklearnCluster
from ENV.cluster_split import ClusterSplit
from ROBOT.robot import Robot

# test
from ENV.estimation import SquircleEstimation

fig = plt.figure(figsize=(18, 6))

ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)

start_pose = np.array([3.2, 0.4, -np.pi])  # 0.5, 0.5, 1.57
p_1 = [0.5, 0.5, -np.pi / 2]
p_2 = [2.4, 3.5, -0.2]
d_1 = [0.8, 3.5, -np.pi]
d_2 = [6.3, 0.5, -np.pi / 2]
d_3 = [6.4, 3.0, -np.pi / 2]
u_1 = [4.6, 0.5, -np.pi]
goal_pose_list = [p_1, p_2, d_1, u_1, d_2, d_3]

real_world_config = './CONFIG/complex_world.yaml'
init_world_config = './CONFIG/workspace.yaml'
forest_config = './CONFIG/forest_world.yaml'
robot_config = './ROBOT/robot_config.yaml'

init_world = InitWorld(init_world_config)
forest_world = World(forest_config)
# robot_world = RobotWorld()
robot = Robot(start_pose, forest_world, init_world, robot_config=robot_config)
sk_clu = SklearnCluster(eps=0.12, min_samples=5)
cluster_split = ClusterSplit(window_size=5, curvature_threshold=5)

# estimate_squ = SquircleEstimation(forest_world)
# main_execute = Execution(robot, robot_world)
#
# main_execute.robot.goal_list = goal_pose_list
#
# main_execute.goal_index = 0W
# main_execute.robot.set_start(start_pose)
# main_execute.robot.set_goal(goal_pose_list[main_execute.goal_index])

path = robot.multi_goal_find_path(start_pose, goal_pose_list, forest_world)
path_x = [path_i[0] for path_i in path]
path_y = [path_i[1] for path_i in path]

robot.set_pre_path(path)

max_step = 1000

# estimate_squ.estimate()
robot.move_one_step(0)
robot.get_measurements_update_world()
cluster_points = sk_clu.cluster(np.array(forest_world.global_lidar_points))
sorted_cluster_points = sk_clu.sort_cluster_points(cluster_points)
forest_world.cluster_points = sorted_cluster_points
all_cluster_segments = cluster_split.split_all_cluster(forest_world.cluster_points)
forest_world.all_cluster_segments = all_cluster_segments

plot_data(ax1, robot, forest_world)
plot_trajectory(ax1, [path_x, path_y])

plot_cluster(ax2, sk_clu, forest_world)
plot_cluster_segment(ax2, forest_world)
plot_init_data(ax2, robot, forest_world)
plot_cluster_segment(ax2, forest_world)


def update_plot(frame):
    current_frame = frame
    print("=========current_frame==========", current_frame)
    robot.move_one_step(current_frame)
    robot.get_measurements_update_world()
    if current_frame % 2 == 0:
        cluster_points = sk_clu.cluster(np.array(forest_world.global_lidar_points))
        sorted_cluster_points = sk_clu.sort_cluster_points(cluster_points)
        forest_world.cluster_points = sorted_cluster_points
        all_cluster_segments = cluster_split.split_all_cluster(forest_world.cluster_points)
        forest_world.all_cluster_segments = all_cluster_segments
        print("num segments", [len(segment) for segment in all_cluster_segments])

    print("--------start plotting figure--------")
    ax1.clear()
    ax2.clear()
    plot_data(ax1, robot, forest_world)
    plot_trajectory(ax1, [path_x, path_y])

    plot_cluster(ax2, sk_clu, forest_world)
    plot_init_data(ax2, robot, forest_world)
    plot_cluster_segment(ax2, forest_world)
    print("--------end plotting figure--------")


ani = FuncAnimation(fig, update_plot, frames=range(max_step), interval=10, repeat=False)

plt.show()

import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import datetime

from NF.geometry import World
from ENV.geometry import RealWorld, RobotWorld, InitWorld
from ENV.construct_forest import ConstructForest
from EXEC.execution import Execution
from ENV.vis import plot_data, plot_trajectory, plot_cluster, plot_init_data, plot_cluster_segment, plot_fitting_squircle, plot_polygon_list, plot_nm_path
from ENV.cluster import SklearnCluster
from ENV.cluster_split import ClusterSplit
from ENV.squircle_estimation import SquircleEstimation
from ROBOT.robot import Robot
from NM.navigation_map import NavigationMap


# test
# from ENV.estimation import SquircleEstimation

fig = plt.figure(figsize=(18, 6))

ax1 = fig.add_subplot(121)
ax2 = fig.add_subplot(122)

start_pose = np.array([3.2, 0.4, -np.pi])  # 0.5, 0.5, 1.57
# start_pose = np.array([6.3, 1.5, -np.pi / 2])
p_1 = [0.4, 0.5, -np.pi / 2]
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
sk_clu = SklearnCluster(eps=0.12, min_samples=6)
cluster_split = ClusterSplit(window_size=5, curvature_threshold=10)

sq_esti = SquircleEstimation(robot)

navigation_map = NavigationMap()


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

current_frame = 0
# estimate_squ.estimate()
robot.move_one_step(0)
robot.get_measurements_update_world()

save_data_path = './RESULT/' + datetime.datetime.now().strftime("%Y%m%d~%H%H%S")
if not os.path.exists(save_data_path):
    os.makedirs(save_data_path)

current_goal_index = 0


for frame in range(0, max_step):
    if np.linalg.norm(np.asarray(goal_pose_list[-1][0:2]) - robot.pose[0:2]) <= 0.1:
        break

    if np.linalg.norm(np.asarray(goal_pose_list[current_goal_index][0:2]) - robot.pose[0:2]) <= 0.1:
        current_goal_index += 1
    
    if frame == 0:
        continue

    current_frame = frame
    print("=========current_frame==========", current_frame)
    robot.move_one_step(current_frame)
    robot.get_measurements_update_world()
    if current_frame % 2 == 0:
        cluster_points = sk_clu.cluster(np.array(forest_world.global_lidar_points))
        if len(cluster_points) == 0:
            continue
        # print("len(cluster_points)", len(cluster_points))
        sorted_cluster_points = sk_clu.sort_cluster_points(cluster_points)
        forest_world.cluster_points = sorted_cluster_points
        all_cluster_segments = cluster_split.split_all_cluster(forest_world.cluster_points)
        forest_world.all_cluster_segments = all_cluster_segments
        # print("num segments", [len(segment)-1 for segment in all_cluster_segments])

        squircle_data = sq_esti.fit_squircle_group(forest_world.all_cluster_segments)
        forest_world.squircle_data = squircle_data
        construct_forest = ConstructForest(forest_world.squircle_data)
        all_polygon_list = construct_forest.get_vis_rect_data(inflated_size=0.28)
        # print("all_polygon_list", all_polygon_list)
        navigation_map.construct_planner_rect_multi_goal(robot.pose, goal_pose_list[current_goal_index:], all_polygon_list)
        nm_path = navigation_map.path

        ax1.clear()
        ax2.clear()
        plot_data(ax1, robot, forest_world)
        plot_trajectory(ax1, [path_x, path_y])
 
        plot_cluster(ax2, sk_clu, forest_world)
        plot_init_data(ax2, robot, forest_world)
        plot_cluster_segment(ax2, forest_world)
        plot_fitting_squircle(ax2, forest_world)
        plot_polygon_list(ax2, all_polygon_list)
        plot_nm_path(ax2, nm_path)

        file_name = f"{current_frame}.png"
        plt.savefig(os.path.join(save_data_path, file_name), dpi=200)
        print("=============frame " + str(current_frame) + " is saved==============")

    # print("--------start plotting figure--------")
    
    # print("--------end plotting figure--------")


# ani = FuncAnimation(fig, update_plot, frames=range(max_step), interval=10, repeat=False)

# plt.show()

import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
import numpy as np
import copy
import itertools


from ENV.geometry import RealWorld, RobotWorld, InitWorld
from EXEC.execution import Execution
from ROBOT.robot import Robot
from ENV.squircle_estimation import SquircleEstimation
from ENV.vis import plot_robot_in_world, plot_start_and_goal
from NF.geometry import World
from NF.vis import plot_world
from  NF.geometry import Rectangular
import test_cluster
from ENV.cluster import SklearnCluster

import pickle
import datetime

start_pose = np.array([3.5, 0.5, 0.0])  # 0.5, 0.5, 1.57
goal_pose = np.array([6.0, 1.0, np.pi / 2])

real_world_config = './CONFIG/squircel_world.yaml'
init_world_config = './CONFIG/squircel_world.yaml'
robot_config = './ROBOT/robot_config.yaml'

real_world = World(real_world_config)
init_world = World(init_world_config)
robot_world = RobotWorld()

# start_pose_list = [np.array([3.5, 0.5, 0.0]), np.array([2.5, 0.5, 0.0]), np.array([0.8, 0.5, 0.0]),
#                    np.array([0.5, 1.8, 0.0]), np.array([0.5, 2.7, 0.0]), np.array([0.3, 3.5, 0.0]),
#                    np.array([2.5, 3.5, 0.0]), np.array([3.5, 3.5, 0.0]), np.array([3.5, 1.5, 0.0])]

start_pose_list = [np.array([3.2, 0.8, 0.0])]

# start_pose_list = [np.array([3.5, 0.5, 0.0])]

total_lidar_points = None

sk_clu = SklearnCluster(eps=0.12, min_samples=5)

for i, start_pose in enumerate(start_pose_list):
    # if i not in [2, 3, 5, 6, 7, 8]:
    #     continue

    robot = Robot(start_pose, real_world, init_world, robot_config=robot_config)
    sq_esti = SquircleEstimation(robot)

    x_ell_init_list = [1.0]
    y_ell_init_list = [1.0]
    a_init_list = [0.1]
    b_init_list = [0.1]
    s_init_list = [0.5]
    theta_init_list = [1.0]

    folder_path = './RESULT/fitting/multi_lidar_data_cluster_with_theta/goal_' + str(i)

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    total_lidar_points = real_world.global_lidar_points
    # lidar_obs_points = total_lidar_points
    # print("total_lidar_points", total_lidar_points)
    # print("total_lidar_points number", len(total_lidar_points))
    cluster_points = sk_clu.cluster(np.array(total_lidar_points))
    # print("cluster_points", cluster_points)
    
    print("%%%%%%%%%%%%%%%% cluster set number %%%%%%%%%%%%%%%%%%%%%%%%%", len(cluster_points))

    i = 0
    for combo in itertools.product(x_ell_init_list, y_ell_init_list,
                                   a_init_list, b_init_list, theta_init_list, s_init_list):
        x_ell_init = combo[0]
        y_ell_init = combo[1]
        a_init = combo[2]
        b_init = combo[3]
        s_init = combo[4]
        theta_init = combo[5]
        print("combo case", combo)

        fig = plt.figure(figsize=(6, 5))

        ax = fig.add_subplot(111)

        xmin, xmax = real_world.workspace[0][0].x_limits()
        ymin, ymax = real_world.workspace[0][0].y_limits()
        ax.set_xlim([xmin - 1.0, xmax + 1.0])
        ax.set_ylim([ymin - 1.0, ymax + 1.0])
        ax.set_aspect('equal', 'box')
        ax.set_axis_off()

        # plot on real world
        # ax = plot_world(real_world, start_pose, fig)
        for obs_group in real_world.obstacles:
            for obs in obs_group:
                sq_esti.plot_squircle(ax, obs.center, obs.width, obs.height, obs.theta, s=obs.s)
        plot_robot_in_world(ax, robot)
        plot_start_and_goal(ax, start_pose, goal_pose)

        # for point in total_lidar_points: 
        #     ax.scatter(point[0], point[1], s=20, c='g', marker='o', zorder=2)

        sk_clu.draw_results(ax, total_lidar_points)

        for k, point_set in enumerate(cluster_points):
            lidar_obs_points = np.array(point_set)

            fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s = \
                sq_esti.fitting(lidar_obs_points, x_ell_init, y_ell_init, a_init, b_init, theta_init, s_init)

            print("fitting results", fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s)

            sq_esti.plot_squircle(ax, fitting_center, fitting_width, fitting_height, fitting_theta, s=fitting_s, color='r', linestyle='--')

        # file_name = "theta_" + str(i) + ".png"
        # print("path", os.path.join(folder_path, file_name))
        # plt.savefig(os.path.join(folder_path, file_name), dpi=600)
        # print("=============figure is saved==============")
        i += 1
        plt.show()
        # plt.close('all')

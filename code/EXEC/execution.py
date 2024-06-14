import math
import os
import sys

sys.path.append(os.getcwd())

import numpy as np

from NF.controller import NFController
from ENV.construct_forest import ConstructForest
from NM.delaunay import DelaunayTriangulation
from NM.navigation_map import NavigationMap
from ENV.point_to_line import PointToLine


class Execution(object):
    def __init__(self, robot, robot_world, dt=0.4):
        self.robot = robot
        self.robot_world = robot_world
        self.point_to_line = PointToLine(robot_world, robot)
        self.dt = dt

        self.current_step = 0
        self.delaunay = None
        self.all_line_list = None
        self.all_squircles = None
        self.construct_forest = None
        self.current_goal = None
        self.trajectory = [[self.robot.pose[0]], [self.robot.pose[0]]]
        self.navigation_map = None
        self.all_polygon_list = None
        self.goal_index = 0
        self.grid_map = None

    def one_step_forward(self, current_step):
        lambda_ = 1e3
        mu_ = [1e10, 1e8, 1e6, 1e4, 1e2, 1e1]

        self.current_step = current_step

        self.robot.step = self.current_step
        self.point_to_line.update_line()

        # all_line_list = self.robot_world.all_line_list
        # for line_i in all_line_list:
        #     print("line_i", line_i[1], line_i[2])
        radius = 1.0
        all_line_list = self.robot_world.all_line_list
        # local_line_list = []
        # goal_point = self.robot.pose
        # from shapely.geometry import LineString, Point
        # for line_i in all_line_list:
        #     points = line_i[1]
        #     line_start_point = points[0]
        #     line_end_point = points[1]
        #     line_sp = LineString([line_start_point, line_end_point])
        #     point = Point(goal_point[0], goal_point[1])
        #     dis = line_sp.distance(point)
        #     if dis < radius:
        #         local_line_list.append(line_i)
        # self.all_line_list = local_line_list
        self.all_line_list = all_line_list
        print("current line length", len(self.all_line_list))

        self.construct_forest = ConstructForest(self.all_line_list)

        all_squircles = []
        for ws_group in self.construct_forest.semantic_world.workspace:
            if len(ws_group) == 1:
                continue
            else:
                all_squircles += ws_group[1:]
        for obs_group in self.construct_forest.semantic_world.obstacles:
            all_squircles += obs_group
        self.all_squircles = all_squircles

        print("forest world workspace structure:", len(self.construct_forest.forest_world.workspace))
        print("forest world obstacles structure", len(self.construct_forest.forest_world.obstacles))

        self.set_robot_goal(threshold=0.5)

        # test goal
        # self.current_goal = np.array([5.47447273, 3.12245789, 0.37140981])


        # if self.current_step == 0:
        #     self.set_robot_goal()
        # if self.robot.check_goal_reached(self.current_goal, 0.1):
        #     self.set_robot_goal()

        # print("path", self.delaunay.path)
        print("********Robot pose: " + str(self.robot.pose) + " ********")
        print("********Move to goal: " + str(self.current_goal) + " ********")

        nf_lambda = 1e40
        nf_mu = [1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200,
                 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200, 1e200]

        lambda_ = lambda_
        mu_ = mu_

        nf_controller = NFController(self.construct_forest.forest_world,
                                     np.array(self.robot.pose), np.array(self.current_goal),
                                     nf_lambda, nf_mu)

        # u = nf_controller.gradient_controller()
        # new_pose = [self.robot.pose[0] + self.dt * u[0],
        #             self.robot.pose[1] + self.dt * u[1],
        #             self.robot.pose[2]]

        v, omega = nf_controller.vector_follow_controller()
        print("********Control Input: " + str(v) + " " + str(omega) + " ********")
        new_pose = [self.robot.pose[0] + self.dt * v * np.cos(self.robot.pose[2]),
                    self.robot.pose[1] + self.dt * v * np.sin(self.robot.pose[2]),
                    self.robot.pose[2] + self.dt * omega]
        new_pose[2] = (new_pose[2] + np.pi) % (2 * np.pi) - np.pi

        self.robot.pose = new_pose
        self.trajectory[0].append(self.robot.pose[0])
        self.trajectory[1].append(self.robot.pose[1])

        # self.robot.move_one_step()
        # NF
        # self.robot.control_one_step(v, omega)

        self.robot.get_measurements_update_world()

    def set_robot_goal(self, threshold=1.0):
        # all_points, all_lines, all_holes = self.construct_forest.get_delaunay_data()
        # self.delaunay = DelaunayTriangulation(all_points, all_lines, all_holes)
        # path = self.delaunay.generate_navigation_map(self.robot.pose, self.robot.goal)
        # self.navigation_map = NavigationMap(inflated_size=0.12)
        # self.navigation_map.path = path
        round_num = 5
        all_polygon_list = self.construct_forest.get_vis_rect_data(inflated_size=0.25)
        self.all_polygon_list = all_polygon_list
        # print("all_polygon_list", all_polygon_list)

        # for i, polygon_i in enumerate(all_polygon_list):
        #     for j, vertex_j in enumerate(polygon_i):
        #         all_polygon_list[i][j] = np.round(all_polygon_list[i][j], round_num)
        #
        # robot_pose = [0.0, 0.0, 0.0]
        # goal_pose = [0.0, 0.0, 0.0]
        # for i, coord_i in enumerate(self.robot.pose):
        #     robot_pose[i] = np.round(self.robot.pose[i], round_num)
        #
        # for i, coord_i in enumerate(self.robot.goal):
        #     goal_pose[i] = np.round(self.robot.goal[i], round_num)

        robot_pose = self.robot.pose
        goal_pose = self.robot.goal

        self.navigation_map = NavigationMap()
        self.navigation_map.construct_planner_rect(robot_pose, goal_pose, all_polygon_list)
        path = self.navigation_map.path
        print("navigation path", path)
        next_path_index = 1
        current_index = 1
        next_path_goal = path[next_path_index]

        for path_goal in path[next_path_index:]:
            dis_to_goal = math.sqrt(
                (path_goal[0] - self.robot.pose[0]) ** 2 + (path_goal[1] - self.robot.pose[1]) ** 2)
            if dis_to_goal > 0.3:
                next_path_goal = path_goal
                break
            current_index += 1

        dis_to_goal = math.sqrt(
            (next_path_goal[0] - self.robot.pose[0]) ** 2 + (next_path_goal[1] - self.robot.pose[1]) ** 2)
        if dis_to_goal < threshold:
            self.current_goal = np.array([next_path_goal[0], next_path_goal[1], next_path_goal[2]])
        else:
            # new_theta = next_path_goal[2]
            new_theta = np.arctan2(next_path_goal[1] - self.robot.pose[1], next_path_goal[0] - self.robot.pose[0])
            if current_index == len(path) - 1:
                new_theta = np.arctan2(next_path_goal[1] - self.robot.pose[1], next_path_goal[0] - self.robot.pose[0])
            if abs(next_path_goal[0] - self.robot.pose[0]) < 1e-5:
                next_path_goal = [self.robot.pose[0],
                                  self.robot.pose[1] + threshold * (next_path_goal[1] - self.robot.pose[1]) / dis_to_goal,
                                  new_theta]
            elif abs(next_path_goal[1] - self.robot.pose[1]) < 1e-5:
                next_path_goal = [self.robot.pose[0] + threshold * (next_path_goal[0] - self.robot.pose[0]) / dis_to_goal,
                                  self.robot.pose[1],
                                  new_theta]
            else:
                next_path_goal = [self.robot.pose[0] + threshold * (next_path_goal[0] - self.robot.pose[0]) / dis_to_goal,
                                  self.robot.pose[1] + threshold * (next_path_goal[1] - self.robot.pose[1]) / dis_to_goal,
                                  new_theta]
            self.current_goal = np.array([next_path_goal[0], next_path_goal[1], next_path_goal[2]])
        # self.current_goal = np.array([2.8334661933800616, 4.249530909089547, 3.07484481770202578])

    def test_one_step(self, current_step):
        self.current_step = current_step

        self.robot.step = self.current_step
        self.point_to_line.update_line()

        all_line_list = self.robot_world.all_line_list
        self.all_line_list = all_line_list

        self.construct_forest = ConstructForest(all_line_list)

        all_polygon_list = self.construct_forest.get_vis_rect_data(inflated_size=0.16)
        self.all_polygon_list = all_polygon_list
        print("all_polygon_list", all_polygon_list)

        # round_num = 5
        # for i, polygon_i in enumerate(all_polygon_list):
        #     for j, vertex_j in enumerate(polygon_i):
        #         all_polygon_list[i][j] = np.round(list(all_polygon_list[i][j]), round_num)
        #
        # robot_pose = [0.0, 0.0, 0.0]
        # goal_pose = [0.0, 0.0, 0.0]
        # for i, coord_i in enumerate(self.robot.pose):
        #     robot_pose[i] = np.round(self.robot.pose[i], round_num)
        #
        # for i, coord_i in enumerate(self.robot.goal):
        #     goal_pose[i] = np.round(self.robot.goal[i], round_num)

        robot_pose = self.robot.pose
        goal_pose = self.robot.goal

        self.navigation_map = NavigationMap()
        self.navigation_map.construct_planner_rect(robot_pose, goal_pose, all_polygon_list)
        path = self.navigation_map.path
        print("navigation path", path)

        all_squircles = []
        for ws_group in self.construct_forest.forest_world.workspace:
            if len(ws_group) == 1:
                continue
            else:
                all_squircles += ws_group[1:]
        for obs_group in self.construct_forest.forest_world.obstacles:
            all_squircles += obs_group
        self.all_squircles = all_squircles

        print("forest world workspace structure:", len(self.construct_forest.forest_world.workspace))
        print("forest world obstacles structure", len(self.construct_forest.forest_world.obstacles))

        self.robot.move_one_step()

        self.trajectory[0].append(self.robot.pose[0])
        self.trajectory[1].append(self.robot.pose[1])

        self.robot.get_measurements_update_world()

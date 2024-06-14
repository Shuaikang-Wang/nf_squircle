import numpy as np
import matplotlib.pyplot as plt
import casadi as ca
import math


class SquircleEstimation(object):
    def __init__(self, robot):
        self.robot = robot

    @staticmethod
    def max_distance(points):
        max_dist = 0
        num_points = len(points)

        for i in range(num_points):
            for j in range(i + 1, num_points):
                dist = np.linalg.norm(np.array(points[i]) - np.array(points[j]))
                if dist > max_dist:
                    max_dist = dist

        return max_dist

    @staticmethod
    def compute_squicle_length_ray(width, height, q, s=0.99):
        normalized_q = q / np.linalg.norm(q)
        transformed_q = np.array([normalized_q[0] / width, normalized_q[1] / height])
        normalized_transformed_q = transformed_q / np.linalg.norm(transformed_q)
        scale = math.sqrt((normalized_transformed_q[0] * width) ** 2 + (normalized_transformed_q[1] * height) ** 2)
        rho_q = scale * math.sqrt(
            2 / (1 + math.sqrt(1 - 4 * s ** 2 * (normalized_transformed_q[0] * normalized_transformed_q[1]) ** 2)))
        return rho_q

    def plot_squircle(self, ax, center, width, height, rotation, color='b', linestyle='-', s=0.99):
        width = width / 2
        height = height / 2
        theta = np.linspace(0, 2 * np.pi, 200)
        traj_x = []
        traj_y = []
        for theta_i in theta:
            q = np.array([np.cos(theta_i), np.sin(theta_i)])
            rho_i = self.compute_squicle_length_ray(width, height, q, s=s)
            traj_x_i = center[0] + rho_i * np.cos(theta_i)
            traj_y_i = center[1] + rho_i * np.sin(theta_i)
            rotated_x_i = (traj_x_i - center[0]) * np.cos(rotation) - (traj_y_i - center[1]) * np.sin(rotation) + \
                          center[0]
            rotated_y_i = (traj_x_i - center[0]) * np.sin(rotation) + (traj_y_i - center[1]) * np.cos(rotation) + \
                          center[1]
            traj_x.append(rotated_x_i)
            traj_y.append(rotated_y_i)
        ax.plot(traj_x, traj_y, color=color, linewidth=4.0, linestyle=linestyle)

    @staticmethod
    def potential(x, y, center, width, height, rotation, s=0.99):
        x_0, y_0, a, b = center[0], center[1], width, height
        rotated_x = (x - x_0) * np.cos(rotation) + (y - y_0) * np.sin(rotation) + x_0
        rotated_y = -(x - x_0) * np.sin(rotation) + (y - y_0) * np.cos(rotation) + y_0
        x, y = rotated_x, rotated_y
        return (1 / (b / 2) ** 2) * (((b / a) * (x - x_0)) ** 2 + (y - y_0) ** 2 +
                                     (((b / a) * (x - x_0)) ** 4 + (y - y_0) ** 4 +
                                      ((2 - 4 * s ** 2) * ((b / a) * (x - x_0)) ** 2 * (y - y_0) ** 2)) ** 0.5) / 2 - 1

    @staticmethod
    def potential_ca(x, y, center_x, center_y, width, height, rotation, s):
        x_0, y_0, a, b = center_x, center_y, width, height
        rotated_x = (x - x_0) * np.cos(rotation) + (y - y_0) * np.sin(rotation) + x_0
        rotated_y = -(x - x_0) * np.sin(rotation) + (y - y_0) * np.cos(rotation) + y_0
        x, y = rotated_x, rotated_y
        return (1 / (b / 2) ** 2) * (((b / a) * (x - x_0)) ** 2 + (y - y_0) ** 2 +
                                     (((b / a) * (x - x_0)) ** 4 + (y - y_0) ** 4 +
                                      ((2 - 4 * s ** 2) * ((b / a) * (x - x_0)) ** 2 * (y - y_0) ** 2)) ** 0.5) / 2 - 1

    def check_valid(self, squircle, lidar_points):
        robot_point = self.robot.pose[0:2]
        for point in lidar_points:
            point_inner = [0.8 * (point[0] - robot_point[0]) + robot_point[0],
                           0.8 * (point[1] - robot_point[1]) + robot_point[1]]
            if squircle.check_point_inside_accurate(point_inner):
                return False
        return True

    def solve_nop_1(self, lidar_points, max_x_bound, max_y_bound, x_ell_init, y_ell_init, a_init, b_init, theta_init, s_init):
        opti = ca.Opti()
        opt_variables = opti.variable(1, 6)
        # print("opt_variables", opt_variables)
        x_ell = opt_variables[:, 0]
        y_ell = opt_variables[:, 1]
        a = opt_variables[:, 2]
        b = opt_variables[:, 3]
        theta = opt_variables[:, 4]
        s = opt_variables[:, 5]
        # print(b)

        # sum
        obj = 0.0
        for point in lidar_points:
            potential_ca_point = self.potential_ca(point[0], point[1], x_ell, y_ell, a, b, theta=0.0, s=0.0) ** 2
            obj += potential_ca_point

        opti.subject_to(opti.bounded(0.1, x_ell, 7.0))
        opti.subject_to(opti.bounded(0.1, y_ell, 4.0))
        opti.subject_to(opti.bounded(0.1, a, max_x_bound))
        opti.subject_to(opti.bounded(0.1, b, max_y_bound))
        opti.subject_to(opti.bounded(-np.pi, theta, np.pi))

        opti.minimize(obj)

        opts_setting = {'ipopt.max_iter': 10000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-5,
                        'ipopt.acceptable_obj_change_tol': 1e-3}

        opti.set_initial(x_ell, x_ell_init)
        opti.set_initial(y_ell, y_ell_init)
        opti.set_initial(a, a_init)
        opti.set_initial(b, b_init)
        opti.set_initial(theta, theta_init)
        opti.set_initial(s, s_init)
        opti.solver('ipopt', opts_setting)

        sol = opti.solve()
        # print(opti.value(obj))

        # obtain the results
        results = sol.value(opt_variables)
        fitting_center = results[0:2]
        fitting_width = results[2]
        fitting_height = results[3]
        fitting_theta = results[4]
        fitting_s = results[5]

        return fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s


    def solve_nop(self, lidar_points, max_x_bound, max_y_bound, x_ell_init, y_ell_init, a_init, b_init, theta_init, s_init):
        opti = ca.Opti()
        opt_variables = opti.variable(1, 6)
        # print("opt_variables", opt_variables)
        x_ell = opt_variables[:, 0]
        y_ell = opt_variables[:, 1]
        a = opt_variables[:, 2]
        b = opt_variables[:, 3]
        theta = opt_variables[:, 4]
        s = opt_variables[:, 5]
        # print(b)

        # sum
        obj = 0.0
        for point in lidar_points:
            potential_ca_point = self.potential_ca(point[0], point[1], x_ell, y_ell, a, b, theta, s) ** 2
            obj += potential_ca_point

        opti.subject_to(opti.bounded(0.1, x_ell, 7.0))
        opti.subject_to(opti.bounded(0.1, y_ell, 4.0))
        opti.subject_to(opti.bounded(0.1, a, max_x_bound))
        opti.subject_to(opti.bounded(0.1, b, max_y_bound))
        opti.subject_to(opti.bounded(-np.pi, theta, np.pi))
        opti.subject_to(opti.bounded(0.9, s, 0.999))

        opti.minimize(obj)

        opts_setting = {'ipopt.max_iter': 10000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-5,
                        'ipopt.acceptable_obj_change_tol': 1e-3}

        opti.set_initial(x_ell, x_ell_init)
        opti.set_initial(y_ell, y_ell_init)
        opti.set_initial(a, a_init)
        opti.set_initial(b, b_init)
        opti.set_initial(theta, theta_init)
        opti.set_initial(s, s_init)
        opti.solver('ipopt', opts_setting)

        sol = opti.solve()
        # print(opti.value(obj))

        # obtain the results
        results = sol.value(opt_variables)
        fitting_center = results[0:2]
        fitting_width = results[2]
        fitting_height = results[3]
        fitting_theta = results[4]
        fitting_s = results[5]

        return fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s

    @staticmethod
    def rotate_points(points, theta):
        rotation_matrix = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])
        rotated_points = points.dot(rotation_matrix)
        return rotated_points

    def fitting(self, lidar_points, x_ell_init, y_ell_init, a_init, b_init, theta_init, s_init):
        all_points_list = lidar_points
        max_distance = self.max_distance(all_points_list)
        max_x_bound = 10.0 * max_distance
        max_y_bound = 10.0 * max_distance
        bound_init = 0.5
        fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s = \
            self.solve_nop(lidar_points, max_x_bound, max_y_bound, x_ell_init, y_ell_init, a_init, b_init, theta_init,
                           s_init)
        fitting_theta = (fitting_theta + np.pi) % (2 * np.pi) - np.pi
        print("fitting_theta", fitting_theta)
        rotated_points = self.rotate_points(np.array(lidar_points), -fitting_theta)

        max_x_bound = np.max(rotated_points[:, 0]) - np.min(rotated_points[:, 0]) + 0.1
        max_y_bound = np.max(rotated_points[:, 1]) - np.min(rotated_points[:, 1]) + 0.1
        max_x_bound = max(max_x_bound, bound_init)
        max_y_bound = max(max_y_bound, bound_init)
        print("max_x_bound", max_x_bound)
        print("max_y_bound", max_y_bound)
        fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s = \
            self.solve_nop(lidar_points, max_x_bound, max_y_bound, x_ell_init, y_ell_init, a_init, b_init, theta_init,
                           s_init)

        return fitting_center, fitting_width, fitting_height, fitting_theta, fitting_s

import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import datetime

from NF.controller import NFController

import os
import sys

sys.path.append(os.getcwd())

import numpy as np
import math

from NF.utils import distance
from NF.navigation import NavigationFunction


class NFController(object):
    def __init__(self):
        pass

    def cartesian_to_egocentric(self, start_pose, goal_pose):
        rho = ((start_pose[0] - goal_pose[0]) ** 2 + (start_pose[1] - goal_pose[1]) ** 2) ** 0.5
        if rho == 0:
            alpha = start_pose[2]
        else:
            alpha = np.arctan2(goal_pose[1] - start_pose[1], goal_pose[0] - start_pose[0])
        phi = (goal_pose[2] - alpha + np.pi) % (2 * np.pi) - np.pi
        delta = (start_pose[2] - alpha + np.pi) % (2 * np.pi) - np.pi
        return rho, phi, delta

    def ego_controller(self, start_pose, goal_pose, k_1=1.0, k_2=1.0):
        velocity = 0.8 * np.tanh(distance(start_pose[0:2], goal_pose[0:2]))
        rho, theta, delta = self.cartesian_to_egocentric(start_pose, goal_pose)
        # velocity = 0.1
        yaw_velocity = (- velocity / rho) * (k_2 * (delta - np.arctan(- k_2 * theta)
                                                    ) + (1 + k_1 / (1 + (k_1 * theta) ** 2)) * np.sin(delta))
        return velocity, yaw_velocity

fig = plt.figure(figsize=(18, 6))

ax = fig.add_subplot(111)

max_step = 1000

start_pose = np.array([0.3297726158982113, 0.5648712446932808, -0.3882745676982333])
goal_pose = np.array([0.56997068, 0.92615038, 0.5199157])

print("dis", math.sqrt((start_pose[0] - goal_pose[0]) ** 2 + (start_pose[1] - goal_pose[1]) ** 2))


nf_controller = NFController()

current_pose = np.array([start_pose[0], start_pose[1], start_pose[2]])
dt = 0.1


traj_x = [current_pose[0]]
traj_y = [current_pose[1]]


def update_plot(i):
    ax.clear()
    ax.plot(start_pose[0], start_pose[1], '*', c='red')
    ax.plot(goal_pose[0], goal_pose[1], 'o', c='blue')

    ax.quiver(start_pose[0], start_pose[1], np.cos(start_pose[2]), np.sin(start_pose[2]), units='xy', width=0.05,
                    headwidth=3.3, scale=1 / 0.5, color='red', zorder=2)
    ax.quiver(goal_pose[0], goal_pose[1], np.cos(goal_pose[2]), np.sin(goal_pose[2]), units='xy', width=0.05,
                    headwidth=3.3, scale=1 / 0.5, color='red', zorder=2)

    ax.set_xlim(0, 7)
    ax.set_ylim(0, 4)
    ax.set_aspect('equal')

    velocity, yaw_velocity = nf_controller.ego_controller(current_pose, goal_pose)
    print("velocity yaw_velocity", velocity, yaw_velocity)
    current_pose[0] += velocity * np.cos(current_pose[2]) * dt
    current_pose[1] += velocity * np.sin(current_pose[2]) * dt
    current_pose[2] += yaw_velocity * dt
    # print("current_pose", current_pose)
    traj_x.append(current_pose[0])
    traj_y.append(current_pose[1])

    ax.plot(traj_x, traj_y, '-', color='red', linewidth=2.5)
    ax.plot(current_pose[0], current_pose[1], 'o', c='red')


    return



ani = FuncAnimation(fig, update_plot, frames=range(max_step), interval=10, repeat=False)

plt.show()

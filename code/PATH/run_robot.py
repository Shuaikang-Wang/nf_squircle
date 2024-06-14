'''
run_robot.py
用于让机器人根据寻路算法返回的path进行运动
'''
import math

import matplotlib.pyplot as plt

from ROBOT.vis import plot_robot

from ENV.vis import robot_sensing, plot_extended_squircle

import numpy as np


def run_bot(ax2, ax3, world, robot, path, extension = 0.2):
    for point in path:
        robot.pose = np.array([point[0], point[1], 0])  # x,y,rads
        robot_sensing(ax2, world, robot)
        plot_extended_squircle(ax3, world, robot, extension)


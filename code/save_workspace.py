import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
import numpy as np
import copy
from ENV.geometry import RealWorld, RobotWorld, InitWorld
from EXEC.execution import Execution
from ENV.vis import plot_world
from ROBOT.robot import Robot


real_world_config = './CONFIG/compare_world.yaml'


real_world = RealWorld(real_world_config)


fig = plt.figure(figsize=(9, 6))

ax = fig.add_subplot(221)

plot_world(ax, real_world)
file_path = './RESULT/workspace/'

file_name = "compare_world.png"
# print("path", os.path.join(folder_path, file_name))
plt.savefig(os.path.join(file_path, file_name), dpi=1200)
print("=============workspace is saved==============")

import os
import sys

sys.path.append(os.getcwd())

import numpy as np
import copy

import pickle
import datetime

import time


p_1 = [0.4, 0.4, -np.pi / 2]
p_2 = [2.2, 3.5, -0.2]
d_1 = [0.9, 3.5, -np.pi]
d_2 = [6.3, 0.5, -np.pi / 2]
d_3 = [6.4, 3.0, -np.pi / 2]
u_1 = [4.6, 0.5, -np.pi]
goal_pose_list = [p_2, d_1, u_1, d_2, d_3]

max_step = 10000

file_path = 'DATA/pickle_data/'
current_date_execution = "execution_2024_07_06"
folder_path_execution = os.path.join(file_path, current_date_execution)
if not os.path.exists(folder_path_execution):
    os.makedirs(folder_path_execution)

# load the saved pickle
with open(folder_path_execution + '/execution.pickle', 'rb') as f:
    execution_data = pickle.load(f)
current_frame = len(execution_data) - 1
print("all pickle frame", current_frame)

current_frame = 259

execution_data = execution_data[:current_frame + 1]

print("initial frame", current_frame)
main_execute = execution_data[current_frame]

main_execute.robot.goal_list = goal_pose_list
print("goal_index", main_execute.goal_index)

for frame in range(current_frame, max_step):
    current_frame = frame
    print("=============current frame is: " + str(current_frame) + "==============")

    # move to next goal
    if np.linalg.norm(np.asarray(goal_pose_list[-1][:2]) - main_execute.robot.pose[:2]) <= 0.1:
        main_execute.goal_index = -1
        main_execute.robot.set_goal(goal_pose_list[main_execute.goal_index])
    else:
        if np.linalg.norm(np.asarray(goal_pose_list[main_execute.goal_index][:2]) - main_execute.robot.pose[:2]) <= 0.1:
            main_execute.goal_index += 1
            main_execute.robot.set_goal(goal_pose_list[main_execute.goal_index])

    # save data every 20 step
    if current_frame % 20 == 0:
        with open(folder_path_execution + '/execution.pickle', 'wb') as f:
            pickle.dump(execution_data, f)

    if np.linalg.norm(np.asarray(goal_pose_list[-1][:2]) - main_execute.robot.pose[:2]) <= 0.1:
        main_execute.trajectory[0].append(goal_pose_list[-1][0])
        main_execute.trajectory[1].append(goal_pose_list[-1][1])
        with open(folder_path_execution + '/execution.pickle', 'wb') as f:
            pickle.dump(execution_data, f)
        break

    try:
        main_execute.one_step_forward(current_frame)
    except:
        print("************Error and Stop************")
        with open(folder_path_execution + '/execution.pickle', 'wb') as f:
            pickle.dump(execution_data, f)

        main_execute.one_step_forward(current_frame)
        break
    else:
        # add data to data_list in order to plot in the end.

        main_execute_copy = copy.deepcopy(main_execute)
        execution_data.append(main_execute_copy)
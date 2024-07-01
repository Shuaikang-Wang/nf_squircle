import os
import sys
sys.path.append(os.getcwd())

import time
import matplotlib.pyplot as plt
import numpy as np


from NF.static_NF.geometry import World
from NF.static_NF.navigation import NavigationFunction
from NF.static_NF.vis import plot_path_on_contour, plot_complexity, compute_total_cost

if __name__ == '__main__':
    # plot the navigation contour and navigation path

    goal_pose = np.array([0.6,2.2,3.17]) #1.5,1.9,2.5
    # [1.8, 3.5, np.pi / 2]
    # [3.5, 1.4, 0]
    # 1.052263705591528,2.800589744164332,1.2432342404042798
    start_pose = np.array([1.1075406863075947,2.719831148144997,-2.066782172437256])
    world_config = '../complex_world/auto_config/tsp_world.yaml'
    # world_config = '../easy_world/easy_world_1.yaml'
    # world_config = '../complex_world/selection_config/selection_world_0.yaml'
    # world_config = '../complex_world/auto_config/squire_world.yaml'

    world = World(world_config)
    # print(world.workspace)

    point = np.array([1.5, 2.5])

    nf = NavigationFunction(world, goal_pose, nf_lambda = 1e1, nf_mu = [1e20, 1e20, 1e20, 1e6])
    # inflated_world_4.yaml:nf_lambda = 2e2, nf_mu = [1e43, 1e30, 1e15]
    # inflated_world_2.yaml: nf_lambda = 5e1, nf_mu = [1e28, 1e20, 1e15]
    # inflated_world_1.yaml: nf_lambda = 8e1, nf_mu = [1e25, 1e25, 1e5]
    # nf_lambda = 8e2, nf_mu = [0]
    # nf_lambda = 1e2, nf_mu = [1e20, 1e20, 1e10] 1e13
    # gradient_point = nf.compute_gradient_point(point)

    # print("transformed point", nf.transformaton(point), "\n")

    compute_total_cost('../complex_world/auto_results/hardware/hard_trajectory_data.txt')

    fig = plt.figure()
    plot_path_on_contour(nf, start_pose, fig)



    # fig = plt.figure(figsize=(10,5))
    # plot_complexity(fig)

    # plt.savefig("../static_NF/vector_field_5.png", dpi=1200)
    # plt.savefig("../easy_world/static_results/update_2_2.png", dpi=1200)
    # plt.savefig("../complex_world/auto_results/compare_2.png", dpi=1200)
    # plt.savefig("../static_NF/complexity_0.png", dpi=1200, bbox_inches = 'tight')

    plt.show()
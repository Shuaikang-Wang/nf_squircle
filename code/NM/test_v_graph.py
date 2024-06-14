import os
import sys

sys.path.append(os.getcwd())

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from NM.navigation_map import NavigationMap

nm = NavigationMap(planner_type='vis', inflated_size=0.2)

fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim([-0.5, 7.5])
ax.set_ylim([-0.5, 4.5])

polygons = [[[-1.0, -1.0], [-1.0, 10.0], [10.0, 10.0], [10.0, -1.0]],
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]],
            [[4.0, 4.0], [7.0, 4.0], [7.0, 7.0], [4.0, 7.0]],
            [[1.0, 3.0], [3.0, 3.0], [3.0, 5.0], [1.0, 5.0]],
            [[6.0, 0.0], [7.0, 0.0], [7.0, 2.0], [9.0, 2.0], [9.0, 3.0], [6.0, 3.0]]]

polygons = [[
    (0.9800011999872371, -5.119997600047999),
    [0.9800011999872371, 0.619997600048049],
    [1.4199940003005915, 0.619997600048049],
    (1.4199940003005915, -5.119997600047999)
],
[
    (3.1800017142615546, 1.4199990769303075),
    (4.119998285738375, 1.419999400006),
    (4.119999400005799, 0.9800059997000152),
    (3.9999994000057995, 0.9800059997000152),
    (3.9999994000057995, 1.2999994000059998),
    (3.3000005999941644, 1.2999994000059998),
    (3.3000005999941644, -5.119999076930178),
    (3.1800005999941643, -5.119999076930178)
],
[
    (12.119997600047999, 1.4199994000060652),
    (12.119997600047999, 1.299999400006065),
    (6.5000005999946495, 1.299999400006065),
    (6.5000005999946495, 1.1000005999940552),
    (11.919829952617347, 1.1000005999940552),
    (11.919829952617347, 0.9800005999940551),
    (6.380004002082337, 0.9800005999940551),
    (6.38000059999465, 1.419994000300115)
],
[
    (1.299999400005935, 1.519994000299985),
    (1.419999400005935, 1.519994000299985),
    (1.419999400005935, 1.0800059996998947),
    (0.05956800686358976, 1.08000059999394),
    (0.05956800686358976, 1.20000059999394),
    (1.299999400005935, 1.20000059999394)
],
[
    (2.71999940000587, 2.1199990000083333),
    (2.71999940000587, 0.6800009999915867),
    (1.8800019999668671, 0.6800005999940002),
    (1.8800005999940999, 1.384759496087541),
    (2.0000005999940997, 1.384759496087541),
    (2.0000005999940997, 0.8000005999940001),
    (2.5999994000058697, 0.8000005999940001),
    (2.5999994000058697, 2.1199990000083333)
],
[
    (3.3800011116357735, 1.9800005999937893),
    (3.380000599993825, 2.381203523278948),
    (3.5000005999938253, 2.381203523278948),
    (3.5000005999938253, 2.1000005999937894),
    (4.589504326567969, 2.1000005999937894),
    (4.589504326567969, 2.4079729509611942),
    (4.709504326567968, 2.4079729509611942),
    (4.709504326567968, 1.966738718610656),
    (4.589504326567969, 1.966738718610656),
    (4.589504326567969, 1.9800005999937893)
],
[
    (4.700000599994235, 1.2999994000059998),
    (4.700000599994235, 0.980005999700015),
    (4.580000599994236, 0.980005999700015),
    (4.580000599994236, 1.419994000300115),
    (5.919998909100246, 1.419999400006),
    (5.919999400005709, 0.9800059997000152),
    (5.79999940000571, 0.9800059997000152),
    (5.79999940000571, 1.2999994000059998)
],
[
    (5.280000599993732, 2.381203523278948),
    (5.4000005999937315, 2.381203523278948),
    (5.4000005999937315, 2.100000599993895),
    (6.62422424122987, 2.100000599993895),
    (6.62422424122987, 1.980000599993895),
    (5.280001086724338, 1.980000599993895)
],
[
    (5.399999400006003, 0.7304169730715695),
    (5.399999400006003, 1.1181144061243),
    (5.519999400006002, 1.1181144061243),
    (5.519999400006002, 0.7304169730715695)
]]



print(len(polygons))

start_point = [0.0, 0.0]
# start_point = [2.3, 0.2]
# start_point = [1.1660107857701658, -0.7194644503557111]
goal_point = [6.5, 0.5]

# round_num = 5
# for i, polygon_i in enumerate(polygons):
#     for j, vertex_j in enumerate(polygon_i):
#         polygons[i][j] = np.round(polygons[i][j], round_num)
#
# # print(polygons)
#
# for i, coord_i in enumerate(start_point):
#     start_point[i] = np.round(start_point[i], round_num)
#
# for i, coord_i in enumerate(goal_point):
#     goal_point[i] = np.round(goal_point[i], round_num)

# nm.construct_planner_no_ws(start_point, goal_point, polygons)

nm.plot_polygons(ax, polygons)
# nm.plot_inflated_polygons(ax, polygons)
# nm.plot_path(ax)

plt.show()
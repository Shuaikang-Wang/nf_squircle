import os
import sys

sys.path.append(os.getcwd())


from ENV.polygon_partition import PolygonPartition

import numpy as np

polygon = [[5.749998499949523, 2.449998499932461], [5.749998499950643, 2.9499984999402775], [5.250001500095558, 2.9499984999402775], [5.250001500094007, 1.961019825086204], [7.309999062505859, 1.9610198250864548], [7.309999062505859, 2.4499984999326947]]


polygon = [[5.75, 2.45], [5.75, 2.95],
           [5.25, 2.95], [5.25, 1.95],
           [7.28, 1.95]]

poly_part = PolygonPartition(polygon)

result = poly_part.polygon_partition()

print("result", result)


import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_axis_off()
ax.set_xlim([4.5, 7.5])
ax.set_ylim([0.5, 4.5])


def plot_polygon(ax, polygon):
    for index in range(len(polygon) - 1):
        ax.plot([polygon[index][0], polygon[index + 1][0]],
                [polygon[index][1], polygon[index + 1][1]], color='blue', linewidth=3.0, zorder=1)
    vertex_x_list = [vertex[0] for vertex in polygon]
    vertex_y_list = [vertex[1] for vertex in polygon]
    plt.scatter(vertex_x_list, vertex_y_list, s=20, c='blue', marker='o')


def plot_squircle(ax, squircle):
    center = squircle.center
    width = squircle.width
    height = squircle.height
    polygon = [[center[0] - width / 2, center[1] - height / 2],
               [center[0] - width / 2, center[1] + height / 2],
               [center[0] + width / 2, center[1] + height / 2],
               [center[0] + width / 2, center[1] - height / 2]]
    poly = Polygon(polygon, edgecolor='red', linewidth=2.0, alpha=0.5, fill=False, zorder=2)
    ax.add_patch(poly)


for squircle in result:
    print("squircle", squircle.center, squircle.width, squircle.height)
    plot_squircle(ax, squircle)

plot_polygon(ax, polygon)


def plot_polygon_1(ax, squircle):
    # 绘制多边形
    center = squircle[0]
    width = squircle[1]
    height = squircle[2]
    polygon = [[center[0] - width / 2, center[1] - height / 2],
               [center[0] - width / 2, center[1] + height / 2],
               [center[0] + width / 2, center[1] + height / 2],
               [center[0] + width / 2, center[1] - height / 2]]
    poly = Polygon(polygon, edgecolor='red', alpha=0.7, fill=False, zorder=2)
    ax.add_patch(poly)

# squircle = [[[4.05,       1.92500075], 1.399997272751956, 0.15],
#  [[3.42500075, 2.1       ], 0.15, 0.49998500075038255],
#  [[4.67499925, 2.53196019], 0.15, 1.363917560369726],
#  [[4.5,        3.13891963], 0.49998500075086305, 0.15],
#  [[4.32500075, 2.76209904], 0.15, 0.9036377062883676],
#  [[3.79670234, 2.27499925], 0.8933996225056333, 0.15]]
#
# for squ in squircle:
#     plot_polygon_1(ax, squ)


plt.show()

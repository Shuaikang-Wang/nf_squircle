"""
=========test===========
"""

from ENV.polygon_partition import PolygonPartition

# polygon_vertices = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 3.0], [0.0, 3.0]]
# poly_part = PolygonPartition(polygon_vertices)
#
# squircle_list = poly_part.polygon_partition()
#
# print(squircle_list)

from shapely.ops import unary_union
from shapely.geometry import box


def rectangles_union(obs_group):
    rectangles = []
    for rect in obs_group:
        rectangles.append((rect[0], rect[1],
                           rect[2], rect[3]))

    boxes = [box(x - w / 2, y - h / 2, x + w / 2, y + h / 2) for x, y, w, h in rectangles]

    union_polygon = unary_union(boxes)

    return list(union_polygon.exterior.coords)[:-1]


import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def plot_rectangle(ax, center, length, width):
    # center = (x_center, y_center)
    x_center, y_center = center

    # Calculate the coordinates of the lower-left corner
    x_ll = x_center - length / 2
    y_ll = y_center - width / 2

    # Create a Rectangle patch
    rect = Rectangle((x_ll, y_ll), length, width, linewidth=2, edgecolor='r', facecolor='none')

    # Add the Rectangle patch to the axis
    ax.add_patch(rect)


polygon_list = [[1.0, 0.5, 2.0, 1.0],
                [1.5, 1.0, 1.0, 2.0]
                ]

new_polygon = rectangles_union(polygon_list)

# print("new polygon", new_polygon)

# Create a figure and axis
fig, ax = plt.subplots()

for rect in polygon_list:
    plot_rectangle(ax, [rect[0], rect[1]], rect[2], rect[3])
# Set axis limits
ax.set_xlim(-1.0, 5.0)
ax.set_ylim(-1.0, 5.0)

# Equal aspect ratio
ax.set_aspect('equal', 'box')

point1 = [0.0, 0.0]
point2 = [1.0, 0.0]
point3 = [2.0, 0.0]

# line = Line([[0.0, 0.0], [1.0, 0.0]])
# print(poly_part.point_on_line(line, point1))

# result = poly_part.select_farthest_points(point1, point2, point3)
# result = poly_part.point_in_polygon(point1, polygon_vertices)
# print(result)

from shapely.geometry import Polygon as Polygon_spl

# 定义多个多边形
polygon1 = Polygon_spl([(0, 0), (0, 1), (1, 1), (1, 0)])
polygon2 = Polygon_spl([(0.5, 0.5), (0.5, 1.5), (1.5, 1.5), (1.5, 0.5)])
polygon3 = Polygon_spl([(4, 4), (6, 4), (6, 6), (4, 6)])

# 计算多个多边形的并集
union_polygon = polygon1.union(polygon2).union(polygon3)


print("union_polygon", union_polygon)

for polygon_i in union_polygon.geoms:
    print("polygon_i", list(polygon_i.exterior.coords))


# Show plot
plt.grid(True)
plt.show()



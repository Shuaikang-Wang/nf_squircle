def find_rectilinear_polygon(segments):
    # 提取所有水平线段和垂直线段的端点
    points = set()
    for segment in segments:
        points.add(segment[0])
        points.add(segment[1])

    # 水平线段按照 y 坐标排序，垂直线段按照 x 坐标排序
    horizontal_segments = sorted([segment for segment in segments if segment[0][1] == segment[1][1]])
    vertical_segments = sorted([segment for segment in segments if segment[0][0] == segment[1][0]])

    # 从左下角开始扫描线
    current_point = min(points)
    polygon_vertices = []
    for segment in vertical_segments:
        if segment[0] == current_point:
            polygon_vertices.append(current_point)
            current_point = segment[1]
    polygon_vertices.append(current_point)

    for segment in horizontal_segments:
        if segment[0] == current_point:
            polygon_vertices.append(current_point)
            current_point = segment[1]
    polygon_vertices.append(current_point)

    return polygon_vertices


# 示例用法
segments = [((0, 0), (0, 2)), ((0, 2), (2, 2)), ((2, 2), (2, 0)), ((2, 0), (0, 0))]
result_polygon = find_rectilinear_polygon(segments)

print(result_polygon)

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim([0.5, 1.6])  # 根据您的数据进行调整
ax.set_ylim([-0.5, 1.0])  # 根据您的数据进行调整


def plot_polygon(ax, polygon, facecolor='gray'):
    # 绘制多边形
    poly = Polygon(polygon, facecolor=facecolor, edgecolor='black', alpha=0.7)
    ax.add_patch(poly)


plot_polygon(ax, result_polygon)

for point in result_polygon:
    plt.plot(*point, 'red', marker='.')

plt.show()

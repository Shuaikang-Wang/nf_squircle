import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Point, Polygon

def find_and_plot_intersections(rect_coords, circle_center, radius):
    # 创建矩形和圆形对象
    rectangle = Polygon(rect_coords)
    circle = Point(circle_center).buffer(radius)

    # 计算交点
    intersection = rectangle.intersection(circle)

    # 提取交点坐标
    if intersection.is_empty:
        intersections = []
    else:
        intersections = list(intersection.exterior.coords) if intersection.geom_type == 'Polygon' else list(intersection.coords)

    # 绘制图形
    fig, ax = plt.subplots()

    # 绘制矩形
    rect_patch = patches.Polygon(rect_coords, closed=True, edgecolor='blue', facecolor='none')
    ax.add_patch(rect_patch)

    # 绘制圆形
    circle_patch = patches.Circle(circle_center, radius, edgecolor='red', facecolor='none')
    ax.add_patch(circle_patch)

    # 绘制交点
    for (x, y) in intersections:
        ax.plot(x, y, 'go') # 绿色点表示交点

    # 设置显示范围
    all_x = [x for x, y in rect_coords] + [circle_center[0] - radius, circle_center[0] + radius]
    all_y = [y for x, y in rect_coords] + [circle_center[1] - radius, circle_center[1] + radius]
    ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
    ax.set_ylim(min(all_y) - 1, max(all_y) + 1)

    ax.set_aspect('equal')
    plt.show()

    return intersections

# 定义矩形和圆形
rectangle_coords = [(1, 2), (6, 2), (6, 5), (1, 5)] # 矩形的顶点坐标
circle_center = (4, 4) # 圆心
circle_radius = 3 # 半径

# 查找并绘制交点
intersections = find_and_plot_intersections(rectangle_coords, circle_center, circle_radius)
print("Intersections:", intersections)
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

def complement_of_rectangles(big_rectangle, small_rectangles):
    # 将大矩形转换为Shapely的box对象

    big_box = box(big_rectangle[0] + big_rectangle[2] / 2, big_rectangle[1] - big_rectangle[3] / 2,
                  big_rectangle[0] - big_rectangle[2] / 2, big_rectangle[1] - big_rectangle[3] / 2 - 1.0)

    # big_box = box(big_rectangle[0] - big_rectangle[2] / 2, big_rectangle[1] - big_rectangle[3] / 2,
    #               big_rectangle[0] + big_rectangle[2] / 2, big_rectangle[1] + big_rectangle[3] / 2)


    # 将所有小矩形转换为Shapely的box对象
    small_boxes = [box(x - w / 2, y - h / 2, x + w / 2, y + h / 2) for x, y, w, h in small_rectangles]
    small_boxes.append(big_box)

    # 求小矩形的并集
    small_union = unary_union(small_boxes)

    # 求小矩形相对于大矩形的补集
    # result_polygon = big_box.difference(small_union)
    result_polygon = small_union


    # 如果补集是一个多边形，则返回多边形的顶点
    if isinstance(result_polygon, Polygon):
        return list(result_polygon.exterior.coords)[:-1]  # 最后一个顶点是重复的第一个顶点，所以要去掉
    else:
        return None

# 测试示例
big_rectangle = (4, 4, 8, 8)
small_rectangles = [(2, 2, 1, 6), (4, 4.5, 4, 1), (6, 2, 1, 6)]
result_polygon = complement_of_rectangles(big_rectangle, small_rectangles)
print(result_polygon)


import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim([0, 10])  # 根据您的数据进行调整
ax.set_ylim([0, 10])  # 根据您的数据进行调整

def plot_polygon(ax, polygon, facecolor='gray'):
    # 绘制多边形
    poly = Polygon(polygon, facecolor=facecolor, edgecolor='black', alpha=0.7)
    ax.add_patch(poly)


# for rect in rectangles:
#     x, y, w, h = rect
#     vertices = [[x - w / 2, y - h / 2],
#                 [x + w / 2, y - h / 2],
#                 [x + w / 2, y + h / 2],
#                 [x - w / 2, y + h / 2]]
#     plot_polygon(ax, vertices, facecolor='blue')

plot_polygon(ax, result_polygon)

plt.show()


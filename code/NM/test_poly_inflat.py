from shapely.geometry import Point as Point_spl
from shapely.geometry import Polygon as Polygon_spl

def is_point_inside_polygon(point, polygon):
    point = Point_spl(point)
    polygon = Polygon_spl(polygon)
    return polygon.contains(point)


polygon = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]

point = [1e-5, 0]

print(is_point_inside_polygon(point, polygon))

import numpy as np


def normalize(vector):
    """单位化向量"""
    norm = np.linalg.norm(vector)
    if norm == 0:
        return vector
    return vector / norm




import numpy as np
import matplotlib.pyplot as plt

def project_point_to_segment(point, segment):
    """将点投影到线段上"""
    p1, p2 = segment
    # 计算线段向量
    segment_vector = np.array(p2) - np.array(p1)
    # 计算点到线段起点的向量
    point_vector = np.array(point) - np.array(p1)
    # 计算点在线段上的投影点的参数 t
    t = np.dot(point_vector, segment_vector) / np.dot(segment_vector, segment_vector)
    # 如果 t 小于 0，则投影点为线段起点
    if t < 0:
        projection = p1
    # 如果 t 大于 1，则投影点为线段终点
    elif t > 1:
        projection = p2
    # 否则，投影点在线段内部
    else:
        projection = p1 + t * segment_vector
    return projection

def symmetric_point(line_segment, point):
    """计算点相对于线段的对称点"""
    # 将点投影到线段上
    projection = project_point_to_segment(point, line_segment)
    # 计算投影点与给定点之间的向量
    vector_to_projection = np.array(projection) - np.array(point)
    # 计算对称点
    symmetric_point = np.array(projection) + vector_to_projection
    return symmetric_point

# 给定线段的两个端点
line_segment = [(1, 1), (4, 4)]

# 给定点的坐标
point = (3, 1)

# 计算对称点的坐标
symmetric_point_coords = symmetric_point(line_segment, point)
print("Symmetric Point Coordinates:", symmetric_point_coords)

# 绘制线段和点
plt.plot([line_segment[0][0], line_segment[1][0]], [line_segment[0][1], line_segment[1][1]], 'b-', label='Line Segment')
plt.plot(point[0], point[1], 'ro', label='Point')
plt.text(point[0], point[1], '   Point', verticalalignment='bottom')
plt.plot(symmetric_point_coords[0], symmetric_point_coords[1], 'go', label='Symmetric Point')
plt.text(symmetric_point_coords[0], symmetric_point_coords[1], '   Symmetric Point', verticalalignment='bottom')

# 设置图形属性
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Symmetric Point')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()


import matplotlib.pyplot as plt
import numpy as np

# 定义一组示例三角形顶点
triangles = [
    [[0, 0], [2, 4], [5, 1]],
    [[2, 4], [5, 1], [3, 6]],
    [[5, 1], [3, 6], [7, 3]],
    [[3, 6], [7, 3], [8, 5]],
    [[7, 3], [8, 5], [0, 0]],
]

triangles = [[[0., 0.], [1., 1.], [1., 2.]],
             [[1., 1.], [0., 0.], [3., 0.]],
             [[0., 3.], [1., 2.], [2., 2.]],
             [[1., 2.], [0., 3.], [0., 0.]],
             [[2., 1.], [3., 0.], [3., 3.]],
             [[3., 0.], [2., 1.], [1., 1.]],
             [[2., 2.], [3., 3.], [0., 3.]],
             [[3., 3.], [2., 2.], [2., 1.]]]


# 存储共享边的中点
shared_edge_midpoints = []

# 找到共享边的中点
for i, triangle_a in enumerate(triangles):
    for j, triangle_b in enumerate(triangles):
        if i != j:  # 确保不是同一个三角形
            shared_edges = set(map(tuple, triangle_a)) & set(map(tuple, triangle_b))
            if len(shared_edges) == 2:  # 如果两个三角形有共享边
                shared_edge_vertices = np.array(list(shared_edges))
                midpoint = np.mean(shared_edge_vertices, axis=0)
                shared_edge_midpoints.append(midpoint)

shared_edge_midpoints = np.array(shared_edge_midpoints)

# 绘制三角形
for triangle in triangles:
    for i in range(3):
        plt.plot([triangle[i][0], triangle[(i + 1) % 3][0]],
                 [triangle[i][1], triangle[(i + 1) % 3][1]], 'b-')

# 绘制共享边中点
plt.scatter(shared_edge_midpoints[:, 0], shared_edge_midpoints[:, 1], color='red', label='Shared Edge Midpoints')
plt.legend()
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Shared Edge Midpoints with Triangles')
plt.show()

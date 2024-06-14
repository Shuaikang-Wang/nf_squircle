import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

# 定义一组示例三角形顶点
triangle_list = [[[0., 0.], [1., 1.], [1., 2.]],
             [[1., 1.], [0., 0.], [3., 0.]],
             [[0., 3.], [1., 2.], [2., 2.]],
             [[1., 2.], [0., 3.], [0., 0.]],
             [[2., 1.], [3., 0.], [3., 3.]],
             [[3., 0.], [2., 1.], [1., 1.]],
             [[2., 2.], [3., 3.], [0., 3.]],
             [[3., 3.], [2., 2.], [2., 1.]]]


def is_point_in_triangle(A, B, C, P):
    def cross_product(u, v):
        return u[0] * v[1] - u[1] * v[0]

    AP = [P[0] - A[0], P[1] - A[1]]
    BP = [P[0] - B[0], P[1] - B[1]]
    CP = [P[0] - C[0], P[1] - C[1]]

    cross1 = cross_product([B[0] - A[0], B[1] - A[1]], AP)
    cross2 = cross_product([C[0] - B[0], C[1] - B[1]], BP)
    cross3 = cross_product([A[0] - C[0], A[1] - C[1]], CP)

    if (cross1 >= 0 and cross2 >= 0 and cross3 >= 0) or (cross1 <= 0 and cross2 <= 0 and cross3 <= 0):
        return True
    else:
        return False


def find_shared_edges(triangle_list):
    shared_edges = {}
    for triangle in triangle_list:
        A, B, C = triangle

        edges = [(A, B), (B, C), (C, A)]

        for edge in edges:
            mid_point = ((edge[0][0] + edge[1][0]) / 2, (edge[0][1] + edge[1][1]) / 2)
            if mid_point not in shared_edges:
                shared_edges[mid_point] = []
            shared_edges[mid_point].append(triangle)
    keys_to_remove = []
    for key_i in shared_edges.keys():
        if len(shared_edges[key_i]) < 2:
            keys_to_remove.append(key_i)

    for key_i in keys_to_remove:
        del shared_edges[key_i]

    start_point = [1.0, 0.5]
    goal_point = [2.0, 2.5]

    start_point = tuple(start_point)
    goal_point = tuple(goal_point)

    G = nx.Graph()
    G.add_node(start_point)

    start_triangle = None
    for key_i in shared_edges.keys():
        for triangle_i in shared_edges[key_i]:
            A, B, C = triangle_i
            is_start_in_triangle = is_point_in_triangle(A, B, C, start_point)
            if is_start_in_triangle:
                start_triangle = triangle_i
            break

    current_triangle = start_triangle
    current_mid_point = start_point
    G = recursive_traversal(G, shared_edges, current_triangle, current_mid_point, goal_point)
    # print(G)
    shortest_path = nx.shortest_path(G, source=start_point, target=goal_point, weight='weight')
    # print(shortest_path)
    return shortest_path


def recursive_traversal(G, shared_edges, current_triangle, current_mid_point, goal_point):
    keys_in_current_triangle = [key for key, value in shared_edges.items() if current_triangle in value]
    # print("current_mid_point", current_mid_point)
    # print("current_triangle", current_triangle)
    # print("shared_edges", shared_edges)
    # print("keys_in_current_triangle", keys_in_current_triangle)
    # print("G", G.nodes())

    for key_i in keys_in_current_triangle:
        G.add_node(key_i)
        dis_node = distance(current_mid_point, key_i)
        G.add_edge(current_mid_point, key_i, weight=dis_node)
        shared_edges[key_i].remove(current_triangle)

    is_goal_in_triangle = is_point_in_triangle(current_triangle[0], current_triangle[1], current_triangle[2],
                                               goal_point)
    if is_goal_in_triangle:
        G.add_node(goal_point)
        dis_start_to_goal = distance(current_mid_point, goal_point)
        G.add_edge(current_mid_point, goal_point, weight=dis_start_to_goal)

    keys_to_remove = []
    for key_i in shared_edges.keys():
        if len(shared_edges[key_i]) == 0:
            keys_to_remove.append(key_i)

    for key_i in keys_to_remove:
        del shared_edges[key_i]

    for key_i in keys_in_current_triangle:
        if key_i in keys_to_remove:
            continue
        if len(shared_edges) == 0:
            return G
        for triangle_i in shared_edges[key_i]:
            recursive_traversal(G, shared_edges, triangle_i, key_i, goal_point)


def distance(point1, point2):
    return ((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2) ** 0.5

path = find_shared_edges(triangle_list)


def plot_triangles(triangle_list):
    for triangle in triangle_list:
        points = np.array(triangle + [triangle[0]])
        plt.plot(points[:, 0], points[:, 1], 'b-')

def plot_points(points, marker, color):
    for point in points:
        plt.plot(point[0], point[1], marker, color=color)



# 假设给定的出发点、目标点和路径点
start_point = (1.0, 0.5)
goal_point = (2.0, 2.5)

plt.figure(figsize=(8, 6))

plot_triangles(triangle_list)
plot_points([start_point], marker='o', color='red')  # 出发点
plot_points([goal_point], marker='o', color='blue')  # 目标点
if path is not None:
    for path_i in path:
        print(path_i)
        plot_points([path_i], color='k', marker='*')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Triangles and Path')
plt.grid()
plt.show()



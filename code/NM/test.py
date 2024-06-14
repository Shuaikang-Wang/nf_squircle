import matplotlib.pyplot as plt
import numpy as np
import triangle as tr

from NM.delaunay import DelaunayTriangulation

# box = {'vertices': np.array([[0., 0.], [0., 3.], [3., 0.], [3., 3.], [1., 1.], [1., 2.], [2., 1.], [2., 2.]]),
#        'segments': np.array([[0, 1], [4, 6], [6, 7], [7, 5], [5, 4]]),
#        'holes': np.array([[1.5, 1.5]])
#        }
#
# t = tr.triangulate(box, 'pc')
#
#
# delaunay_triangles = []
#
# for index in t['triangles']:
#     triangle_i = [t['vertices'][index[0]], t['vertices'][index[1]], t['vertices'][index[2]]]
#     delaunay_triangles.append(triangle_i)
#
#
# tr.compare(plt, box, t)
# plt.show()

vertices_set = [np.array([0., 0.]), np.array([0., 3.]), np.array([3., 0.]), np.array([3., 3.]), np.array([1., 1.]), np.array([1., 2.]), np.array([2., 1.]), np.array([2., 2.])]

segments_set = [np.array([0, 1]), np.array([4, 6]), np.array([6, 7]), np.array([7, 5]), np.array([5, 4])]

delaunay = DelaunayTriangulation(vertices_set, segments_set)

start_point = [1.0, 0.5]
goal_point = [2.0, 2.5]
path = delaunay.generate_navigation_map(start_point, goal_point)
print(path)


def is_point_in_triangle(A, B, C, P):
    def cross_product(u, v):
        return u[0] * v[1] - u[1] * v[0]

    AP = [P[0] - A[0], P[1] - A[1]]
    BP = [P[0] - B[0], P[1] - B[1]]
    CP = [P[0] - C[0], P[1] - C[1]]

    cross1 = cross_product([B[0] - A[0], B[1] - A[1]], AP)
    cross2 = cross_product([C[0] - B[0], C[1] - B[1]], BP)
    cross3 = cross_product([A[0] - C[0], A[1] - C[1]], CP)

    # print("cross1", cross1)
    # print("cross2", cross2)
    # print("cross3", cross3)

    if (cross1 >= 0 and cross2 >= 0 and cross3 >= 0) or (cross1 <= 0 and cross2 <= 0 and cross3 <= 0):
        return True
    else:
        return False

A = [0, 0]
B = [2, 0]
C = [0, 2]
P = [1, 0]
print(is_point_in_triangle(A, B, C, P))

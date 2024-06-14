import numpy as np


def line_intersection(line1, line2):
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]

    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if det == 0:
        return None

    intersection_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det
    intersection_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det

    intersection_point = [intersection_x, intersection_y]
    return intersection_point


def intersect_ray(ray1, ray2):
    def cross_product(v1, v2):
        return v1[0] * v2[1] - v1[1] * v2[0]

    def subtract(v1, v2):
        return [v1[0] - v2[0], v1[1] - v2[1]]

    p1, v1 = ray1
    p2, v2 = ray2

    determinant = cross_product(v1, v2)

    if determinant == 0:
        return False

    t = cross_product(subtract(p2, p1), v2) / determinant
    u = cross_product(subtract(p2, p1), v1) / determinant

    if t >= 0 and u >= 0:
        return True
    else:
        return False

# Example usage
ray1 = ((0, 1), (1, 0))  # Ray starting at (1, 1) with direction (1, 0)
ray2 = ((1, 0), (0, 1))  # Ray starting at (2, 0) with direction (0, 1)

result = intersect_ray(ray1, ray2)
print("result", result)

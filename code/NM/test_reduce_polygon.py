import math


def remove_polygon_redundant_vertices(polygon_vertices):
    new_vertices = []

    i = 0
    # print("polygon vertices", polygon_vertices)
    while i < len(polygon_vertices):
        new_vertices.append(polygon_vertices[i])
        if i == len(polygon_vertices) - 1:
            break
        for j in range(i + 1, len(polygon_vertices)):
            # print("j", j)
            if math.sqrt((polygon_vertices[i][0] - polygon_vertices[j][0]) ** 2 +
                         (polygon_vertices[i][1] - polygon_vertices[j][1]) ** 2) < 1e-2:
                if j == len(polygon_vertices) - 1:
                    return new_vertices
                continue
            else:
                i = j
                break
    if math.sqrt((new_vertices[0][0] - new_vertices[-1][0]) ** 2 +
                 (new_vertices[0][1] - new_vertices[-1][1]) ** 2) < 1e-2:
        new_vertices = new_vertices[0:-1]
    return new_vertices


polygon_list = [(-20.119999076930178, 1.20000059999394), (1.299999400005935, 1.20000059999394), (1.299999400005935, 1.7911836777083892), (1.419999400005935, 1.7911836777083892), (1.419999400005935, 1.08000254670959), (1.4199990769300475, 1.08000254670959), (1.4199990769300475, 1.08000059999394), (-20.119999076930178, 1.08000059999394), (-20.119999076930178, 1.20000059999394)]

print("before", len(polygon_list))

polygon_list = remove_polygon_redundant_vertices(polygon_list)

print("after", len(polygon_list))

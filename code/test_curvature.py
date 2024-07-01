import numpy as np
import math

point_1 = [4.53778681, 2.67660663]
point_2 = [4.02176439, 3.00140024]

theta = np.arctan2(point_2[1] - point_1[1], point_2[0] - point_1[0])

if theta < 0:
    theta += np.pi

theta = np.pi - theta

dis = math.sqrt((point_1[0] - point_2[0])**2 + (point_1[1] - point_2[1])**2)
dis = dis * np.cos(0.5 + theta)

print("dis", dis)
print("theta", theta)
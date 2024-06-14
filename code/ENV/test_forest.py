import numpy as np
import matplotlib.pyplot as plt


def generate_random_points(num_points, x_range=(0, 10), y_range=(0, 10)):
    return np.random.uniform(low=[x_range[0], y_range[0]], high=[x_range[1], y_range[1]], size=(num_points, 2))


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def find_cluster(point, clusters, distance_threshold):
    for cluster in clusters:
        for cluster_point in cluster:
            if distance(point, cluster_point) < distance_threshold:
                return cluster
    return None


def connect_points(points, distance_threshold):
    clusters = []

    for point in points:
        connected_cluster = find_cluster(point, clusters, distance_threshold)
        if connected_cluster is not None:
            connected_cluster.append(point)
        else:
            clusters.append([point])

    # Merge clusters that share common points
    merged_clusters = []
    for cluster1 in clusters:
        merge_flag = 0
        for cluster2 in merged_clusters:
            if any(distance(point1, point2) < distance_threshold for point1 in cluster1 for point2 in cluster2):
                cluster2.extend(cluster1)
                break
        if merge_flag:
            merged_clusters.append(cluster1)

    return merged_clusters


# 生成100个随机点
points_1 = generate_random_points(10, x_range=(10,30), y_range=(10,30))

points_2 = generate_random_points(10, x_range=(70,90), y_range=(70,90))

point_3 = generate_random_points(1, x_range=(40,60), y_range=(40,60))

points = list(points_1) + list(points_2) + list(point_3)

# 定义距离阈值
distance_threshold = 40.0

# 连接点
clusters = connect_points(points, distance_threshold)

# 绘制结果
colors = plt.cm.rainbow(np.linspace(0, 1, len(clusters)))

for i, cluster in enumerate(clusters):
    cluster = np.array(cluster)
    plt.scatter(cluster[:, 0], cluster[:, 1], color=colors[i], marker='o')

plt.xlim([0.0, 100.0])
plt.ylim([0.0, 100.0])
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Connecting Random Points with Consideration of Merging Clusters')
plt.show()

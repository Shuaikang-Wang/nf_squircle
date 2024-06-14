import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN


def generate_points_on_line(start, end, num_points, noise_level):
    t = np.linspace(0, 1, num_points)
    points = np.outer(1 - t, start) + np.outer(t, end)
    noise = np.random.randn(num_points, 2) * noise_level
    noisy_points = points + noise
    return noisy_points


def calculate_slopes(points):
    slopes = []
    n_points = len(points)
    for i in range(n_points):
        for j in range(i + 1, n_points):
            if points[j, 0] != points[i, 0]:  # 避免除以零
                slope = (points[j, 1] - points[i, 1]) / (points[j, 0] - points[i, 0])
                slopes.append((i, j, slope))
    return slopes


def cluster_points_by_slope(slopes, eps=0.1, min_samples=2):
    slope_values = np.array([s[2] for s in slopes]).reshape(-1, 1)
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(slope_values)
    labels = db.labels_
    return labels


def assign_clusters_to_points(slopes, labels, n_points):
    clusters = [[] for _ in range(n_points)]
    for (i, j, _), label in zip(slopes, labels):
        if label != -1:  # -1 表示噪声点
            clusters[i].append(label)
            clusters[j].append(label)
    return clusters


def plot_clusters(points, clusters):
    unique_labels = set(label for cluster in clusters for label in cluster if label != -1)
    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

    for i, point in enumerate(points):
        for label in clusters[i]:
            if label != -1:
                col = colors[label % len(colors)]
                plt.plot(point[0], point[1], 'o', markerfacecolor=tuple(col), markeredgecolor='k', markersize=6)

    plt.title('Clustering points by slope')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()


def main():
    # 定义线段的端点
    start = np.array([0, 0])
    end = np.array([10, 10])

    # 在线段上生成随机点并添加噪声
    num_points = 20
    noise_level = 0.1
    points = generate_points_on_line(start, end, num_points, noise_level)

    start = np.array([10, 0])
    end = np.array([0, 10])

    points_1 = generate_points_on_line(start, end, num_points, noise_level)

    points = np.vstack((points, points_1))

    # 计算点对之间的斜率
    slopes = calculate_slopes(points)

    # 聚类斜率
    labels = cluster_points_by_slope(slopes)

    # 将聚类结果分配给点
    clusters = assign_clusters_to_points(slopes, labels, len(points))

    # 绘制结果
    plot_clusters(points, clusters)


if __name__ == "__main__":
    main()

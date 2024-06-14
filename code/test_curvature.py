import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from sklearn import linear_model

# 生成示例数据
np.random.seed(42)


def generate_noisy_line_segment(p1, p2, num_points, noise_level):
    """
    生成带有噪声的线段点。
    """
    t = np.linspace(0, 1, num_points)
    x = p1[0] + t * (p2[0] - p1[0]) + np.random.normal(scale=noise_level, size=num_points)
    y = p1[1] + t * (p2[1] - p1[1]) + np.random.normal(scale=noise_level, size=num_points)
    return np.vstack((x, y)).T


# 生成三条线段
points1 = generate_noisy_line_segment([0, 0], [5, 5], 50, 0.3)
points2 = generate_noisy_line_segment([5, 5], [10, 0], 50, 0.3)
points3 = generate_noisy_line_segment([10, 0], [15, 5], 50, 0.3)

# 合并点云
points = np.vstack((points1, points2, points3))

# 绘制点云
plt.figure(figsize=(10, 6))
plt.scatter(points[:, 0], points[:, 1], color='gray', label='Point Cloud')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original Point Cloud')
plt.legend()
plt.grid(True)
plt.show()

# 使用DBSCAN聚类
db = DBSCAN(eps=1.0, min_samples=10).fit(points)
labels = db.labels_

# 绘制聚类结果
plt.figure(figsize=(10, 6))
plt.scatter(points[:, 0], points[:, 1], c=labels, cmap='viridis', label='Point Cloud')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('DBSCAN Clustering of Point Cloud')
plt.colorbar(label='Cluster Label')
plt.grid(True)
plt.show()


def fit_line_ransac(points, max_trials=1000, residual_threshold=0.5):
    """
    使用RANSAC进行线段拟合。
    """
    if len(points) < 2:
        return None, None, None  # 不足够点进行拟合

    ransac = linear_model.RANSACRegressor(
        linear_model.LinearRegression(),
        max_trials=max_trials,
        residual_threshold=residual_threshold
    )
    X = points[:, 0].reshape(-1, 1)
    y = points[:, 1]
    ransac.fit(X, y)
    inlier_mask = ransac.inlier_mask_
    line_X = np.arange(X.min(), X.max())[:, np.newaxis]
    line_y_ransac = ransac.predict(line_X)

    return line_X, line_y_ransac, inlier_mask


# 绘制拟合结果
plt.figure(figsize=(10, 6))
plt.scatter(points[:, 0], points[:, 1], color='gray', alpha=0.6, label='Data Points')

unique_labels = set(labels)
for k in unique_labels:
    if k == -1:
        continue  # 噪声点
    class_points = points[labels == k]
    line_X, line_y_ransac, inlier_mask = fit_line_ransac(class_points)
    if line_X is not None:
        plt.plot(line_X, line_y_ransac, label=f'Cluster {k} Fitted Line')
        plt.scatter(class_points[inlier_mask, 0], class_points[inlier_mask, 1], edgecolor='k')

plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('Line Segments Fitting with RANSAC')
plt.grid(True)
plt.show()


def iterative_fit_optimization(points, labels, max_iter=5, residual_threshold=0.5):
    """
    通过迭代去除异常点进行优化拟合。
    """
    for _ in range(max_iter):
        new_labels = labels.copy()
        for k in unique_labels:
            if k == -1:
                continue
            class_points = points[labels == k]
            line_X, line_y_ransac, inlier_mask = fit_line_ransac(class_points, residual_threshold=residual_threshold)
            if line_X is None:
                continue
            outliers = ~inlier_mask
            # 标记为噪声点
            new_labels[labels == k] = -1
            new_labels[labels == k][outliers] = -1

        # 重新聚类
        new_db = DBSCAN(eps=1.0, min_samples=10).fit(points[new_labels != -1])
        new_labels[new_labels != -1] = new_db.labels_
        labels = new_labels.copy()

    return labels


# 迭代优化
labels = iterative_fit_optimization(points, labels)

# 绘制优化后的结果
plt.figure(figsize=(10, 6))
plt.scatter(points[:, 0], points[:, 1], color='gray', alpha=0.6, label='Data Points')

unique_labels = set(labels)
for k in unique_labels:
    if k == -1:
        continue  # 噪声点
    class_points = points[labels == k]
    line_X, line_y_ransac, inlier_mask = fit_line_ransac(class_points)
    if line_X is not None:
        plt.plot(line_X, line_y_ransac, label=f'Cluster {k} Fitted Line')
        plt.scatter(class_points[inlier_mask, 0], class_points[inlier_mask, 1], edgecolor='k')

plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('Optimized Line Segments Fitting')
plt.grid(True)
plt.show()

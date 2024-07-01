import copy
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import distance_matrix
from scipy.sparse.csgraph import minimum_spanning_tree
from scipy.sparse.csgraph import depth_first_order
from scipy.sparse import csr_matrix

COLORS = [plt.cm.Spectral(each) for each in np.linspace(0, 1, 30)]
np.random.shuffle(COLORS)


class SklearnCluster(object):
    def __init__(self, eps=0.5, min_samples=5):
        self.eps = eps
        self.min_samples = min_samples
        self.unique_labels = None
        self.labels = None

    def cluster(self, points):
        points_xy = points[:, :2]
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points_xy)
        self.labels = db.labels_
        self.unique_labels = set(self.labels)
        cluster_points = []
        for k in range(len(self.unique_labels)):
            if k == -1:
                continue
            class_member_mask = (self.labels == k)
            point = points[class_member_mask]
            if len(point) < 10:

                continue
            cluster_points.append(point)
        # print("test cluster_points", cluster_points)
        return cluster_points

    def find_nearest_path_with_matrix(self, dist_matrix, start_index):
        n = len(dist_matrix)
        dist_matrix = copy.deepcopy(dist_matrix)
        path = [start_index]
        total_distance = 0.0

        current_index = start_index

        for _ in range(1, n - 1):
            distances = dist_matrix[current_index, :]
            next_index = np.argmin(distances)
            total_distance += distances[next_index]
            # print("distances[next_index]", distances[next_index])
            path.append(next_index)
            # print("dist_matrix[current_index, next_index]", dist_matrix[current_index, next_index])
            dist_matrix[current_index, :] = np.inf
            dist_matrix[:, current_index] = np.inf
            current_index = next_index

        return total_distance, path

    def find_index_with_min_total_distance(self, dist_matrix):
        min_total_distance = float('inf')
        best_start_index = -1
        best_path = []

        for i in range(len(dist_matrix)):
            total_distance, path = self.find_nearest_path_with_matrix(dist_matrix, i)
            if total_distance < min_total_distance:
                min_total_distance = total_distance
                best_start_index = i
                best_path = path

        return best_start_index, best_path

    def sort_cluster_points(self, cluster_points):
        # print("cluster_points", cluster_points)
        if len(cluster_points[0]) == 0:
            return cluster_points
        # print("cluster_points", cluster_points)
        sorted_cluster_points = []
        for cluster_i in cluster_points:
            # print("cluster_i", cluster_i)
            if len(cluster_i) == 0:
                continue
            else:
                # print("cluster_i", cluster_i)
                dist_matrix = distance_matrix(cluster_i[:, :2], cluster_i[:, :2])
                for i in range(len(dist_matrix)):
                    dist_matrix[i, i] = np.inf
                _, best_path = self.find_index_with_min_total_distance(dist_matrix)
                # print("best_path", best_path)
                sorted_cluster_i = cluster_i[best_path]
                # sorted_cluster_i = cluster_i
                sorted_cluster_points.append(sorted_cluster_i)
        # print("sorted_cluster_points", sorted_cluster_points)
        return sorted_cluster_points

    def draw_results(self, ax, cluster_points):
        for points, col in zip(cluster_points, COLORS):
            xy = points
            ax.plot(xy[0, 0], xy[0, 1], 'o', markerfacecolor='b', markeredgecolor='b', markersize=6,
                    zorder=35)
            ax.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor=tuple(col), markersize=4, zorder=35)


if __name__ == "__main__":
    np.random.seed(0)
    points = np.random.rand(20, 2)
    points = np.array(
        [[3.3595595, 1.58091464], [3.3146241, 1.52334538], [3.26825386, 1.49332458], [3.22440975, 1.4612752],
         [3.17994333, 1.43683709], [3.13722279, 1.41038052], [3.09633757, 1.3820185], [3.05275949, 1.3607415],
         [3.01029928, 1.33731309],
         [2.96906306, 1.31179183], [2.92915392, 1.28424151], [2.88438985, 1.26251173], [2.84038674, 1.23824816],
         [2.79728692, 1.21147335],
         [2.74785841, 1.18897244], [2.70666181, 1.15689764], [2.65078777, 1.13438047], [2.60354218, 1.10229841],
         [2.54885348, 1.07220652],
         [2.48618047, 1.04283512], [2.42415415, 1.00858206], [2.36310807, 0.96944302], [2.28453318, 0.93213471],
         [2.20698723, 0.88809532],
         [2.12123941, 0.83973405]])
    print("points", points)
    sk_clu = SklearnCluster()
    cluster_points = sk_clu.cluster(points)
    print("cluster_points", cluster_points)

    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111)

    sk_clu.draw_results(ax, points)

    plt.show()

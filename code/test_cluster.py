import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN


class SklearnCluster(object):
    def __init__(self, eps=0.5, min_samples=5):
        self.eps = eps
        self.min_samples = min_samples
        self.unique_labels = None
        self.labels = None

    def cluster(self, points):
        db = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        self.labels = db.labels_
        self.unique_labels = set(self.labels)
        cluster_points = []
        for k in range(len(self.unique_labels)):
            if k == -1:
                continue
            class_member_mask = (self.labels == k)
            xy = points[class_member_mask]
            cluster_points.append(xy)
        return cluster_points

    def draw_results(self, ax, points):
        colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(self.unique_labels))]

        for k, col in zip(self.unique_labels, colors):
            if k == -1:
                col = [0, 0, 0, 1]

            class_member_mask = (self.labels == k)
            xy = points[class_member_mask]
            ax.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col), markeredgecolor=tuple(col), markersize=6)


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

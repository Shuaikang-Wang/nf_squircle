from ENV.cluster_split import ClusterSplit
import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance_matrix

cluster_split = ClusterSplit(5, 5)


def plot_segments(segments):
    colors = plt.cm.jet(np.linspace(0, 1, len(segments)))  # 使用颜色映射S

    plt.figure(figsize=(10, 6))
    for i, segment in enumerate(segments):
        segment = np.array(segment)
        plt.plot(segment[:, 0], segment[:, 1], 'o-', color=colors[i], label=f'Segment {i + 1}')

    x_smooth, y_smooth = cluster_split.smooth_points(points)
    plt.plot(x_smooth, y_smooth, '-', c='red')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Point Segmentation Based on Curvature')
    plt.legend()
    plt.grid(True)
    plt.show()


# 示例点
points = np.array([[9.50311915e-01, 2.72053499e-02],
                   [9.50340958e-01, -8.22986557e-03],
                   [9.50579370e-01, 2.68712072e-01],
                   [9.50631942e-01, -4.86627862e-02],
                   [9.50656419e-01, 2.73517592e-02],
                   [9.50656419e-01, 2.27351759e-01],
                   [9.50703686e-01, 1.94758027e-01],
                   [9.50744904e-01, 2.02629807e-01],
                   [9.50749410e-01, 3.22224527e-01],
                   [9.50791538e-01, 1.50757539e-01],
                   [9.50791538e-01, 2.50757539e-01],
                   [9.51075019e-01, 4.21338377e-03],
                   [9.51191097e-01, 1.29674127e-01],
                   [9.51290902e-01, 3.29825532e-01],
                   [9.51355199e-01, 8.43332363e-02],
                   [9.51577770e-01, 2.49533184e-01],
                   [9.51614954e-01, 2.63052888e-01],
                   [9.51759269e-01, 2.86975513e-01],
                   [9.52027703e-01, -2.84898258e-02],
                   [9.52027703e-01, 7.15101742e-02],
                   [9.52058911e-01, 2.94838343e-01],
                   [9.52240323e-01, 1.21318314e-01],
                   [9.52240323e-01, 3.21318314e-01],
                   [9.52254546e-01, 7.11157094e-02],
                   [9.52254546e-01, 1.71115709e-01],
                   [9.52267918e-01, 2.12877237e-01],
                   [9.52326290e-01, 3.14536523e-01],
                   [9.52364001e-01, 3.48308830e-01],
                   [9.52373062e-01, 3.67588379e-01],
                   [9.52441804e-01, 3.57926867e-01],
                   [9.52452942e-01, 2.71126453e-01],
                   [9.52882475e-01, 2.12773680e-01],
                   [9.52882475e-01, 3.12773680e-01],
                   [9.53548656e-01, 4.84541916e-02],
                   [9.53706039e-01, 1.30709017e-01],
                   [9.53877150e-01, 3.90894205e-01],
                   [9.53895815e-01, 4.17804388e-01],
                   [9.54207807e-01, 3.14159128e-01],
                   [9.54207807e-01, 4.14159128e-01],
                   [9.54421865e-01, 3.98123175e-01],
                   [9.54464431e-01, 3.16800434e-01],
                   [9.54587399e-01, 3.32040910e-01],
                   [9.54752934e-01, 3.65958644e-01],
                   [9.54965915e-01, 4.18694619e-01],
                   [9.54970875e-01, 4.20415980e-01],
                   [9.55056568e-01, 3.97183917e-01],
                   [9.55113176e-01, 3.73746672e-01],
                   [9.55263006e-01, 3.96616266e-01],
                   [9.56080886e-01, 3.67277638e-01],
                   [9.57219824e-01, 4.15294436e-01],
                   [9.57671333e-01, 4.36440967e-01],
                   [9.58664468e-01, 4.42477781e-01],
                   [9.58892980e-01, 4.33510686e-01],
                   [9.59245977e-01, 4.24557815e-01],
                   [9.62366328e-01, 4.47240577e-01],
                   [9.63228949e-01, 4.52499365e-01],
                   [9.65482478e-01, 4.50816124e-01],
                   [9.66353980e-01, 4.52318717e-01],
                   [9.69434609e-01, 4.58258133e-01],
                   [9.73357496e-01, 4.60532534e-01],
                   [9.74962667e-01, 4.58647255e-01],
                   [9.79493992e-01, 4.62242603e-01],
                   [9.80378918e-01, 4.59869958e-01],
                   [9.82344148e-01, 4.63775551e-01],
                   [9.82998004e-01, 4.62808587e-01],
                   [9.84298646e-01, 4.60883259e-01],
                   [9.85729606e-01, 4.63518472e-01],
                   [9.86821003e-01, 4.65389124e-01],
                   [9.90837204e-01, 4.63275011e-01],
                   [9.91471765e-01, 4.62675872e-01],
                   [9.94610931e-01, 4.65033383e-01],
                   [9.98327530e-01, 4.65010360e-01],
                   [9.98616839e-01, 4.66764239e-01],
                   [1.00515405e+00, 4.64984277e-01],
                   [1.00619587e+00, 4.64999098e-01],
                   [1.01107125e+00, 4.64766215e-01],
                   [1.01181236e+00, 4.65517778e-01],
                   [1.02174973e+00, 4.67093023e-01],
                   [1.02344857e+00, 4.67845504e-01],
                   [1.02400957e+00, 4.64905755e-01],
                   [1.02517917e+00, 4.67368897e-01],
                   [1.02764798e+00, 4.68484076e-01],
                   [1.02789359e+00, 4.67102839e-01],
                   [1.02994194e+00, 4.68450553e-01],
                   [1.03097248e+00, 4.67243867e-01],
                   [1.03180092e+00, 4.67645137e-01],
                   [1.03464726e+00, 4.65947338e-01],
                   [1.03806206e+00, 4.66668395e-01],
                   [1.03971271e+00, 4.65750602e-01],
                   [1.04188412e+00, 4.67155939e-01],
                   [1.05057937e+00, 4.68712072e-01],
                   [1.05121365e+00, 4.67744413e-01],
                   [1.05128003e+00, 4.68674716e-01],
                   [1.05480677e+00, 4.65756871e-01],
                   [1.05697340e+00, 4.66775710e-01],
                   [1.06207286e+00, 4.69324430e-01],
                   [1.06286883e+00, 4.67434006e-01],
                   [1.06373787e+00, 4.69672368e-01],
                   [1.07027750e+00, 4.67481802e-01],
                   [1.07154257e+00, 4.68033434e-01],
                   [1.07291794e+00, 4.67711270e-01],
                   [1.07575965e+00, 4.69496173e-01],
                   [1.07936066e+00, 4.67067251e-01],
                   [1.08206544e+00, 4.67720850e-01],
                   [1.08485941e+00, 4.65851715e-01],
                   [1.08511629e+00, 4.67246086e-01],
                   [1.08561237e+00, 4.67559563e-01],
                   [1.08587819e+00, 4.69937316e-01],
                   [1.08797499e+00, 4.67540965e-01],
                   [1.08869989e+00, 4.66837209e-01],
                   [1.09461093e+00, 4.65033383e-01],
                   [1.09562364e+00, 4.67184621e-01],
                   [1.09832753e+00, 4.65010360e-01],
                   [1.10033107e+00, 4.67835470e-01],
                   [1.10789779e+00, 4.68630037e-01],
                   [1.10802817e+00, 4.69075662e-01],
                   [1.11181236e+00, 4.65517778e-01],
                   [1.12517917e+00, 4.67368897e-01],
                   [1.12646088e+00, 4.69263634e-01],
                   [1.12764798e+00, 4.68484076e-01],
                   [1.12957317e+00, 4.67107219e-01],
                   [1.12994194e+00, 4.68450553e-01],
                   [1.13110391e+00, 4.69475040e-01],
                   [1.13307479e+00, 4.68396047e-01],
                   [1.13527275e+00, 4.66288998e-01],
                   [1.13806206e+00, 4.66668395e-01],
                   [1.13971271e+00, 4.65750602e-01],
                   [1.13999840e+00, 4.68375313e-01],
                   [1.14188412e+00, 4.67155939e-01],
                   [1.14275976e+00, 4.67698910e-01],
                   [1.15237306e+00, 4.67588379e-01],
                   [1.15480677e+00, 4.65756871e-01],
                   [1.15697340e+00, 4.66775710e-01],
                   [1.15844320e+00, 4.65885599e-01],
                   [1.16286883e+00, 4.67434006e-01],
                   [1.16298273e+00, 4.64605424e-01],
                   [1.16335369e+00, 4.67266403e-01],
                   [1.16591229e+00, 4.68769833e-01],
                   [1.17027750e+00, 4.67481802e-01],
                   [1.17154257e+00, 4.68033434e-01],
                   [1.17370329e+00, 4.68841474e-01],
                   [1.17764954e+00, 4.64867099e-01],
                   [1.18206544e+00, 4.67720850e-01],
                   [1.18485941e+00, 4.65851715e-01],
                   [1.18561237e+00, 4.67559563e-01],
                   [1.18827465e+00, 4.67220930e-01],
                   [1.18869989e+00, 4.66837209e-01],
                   [1.19262925e+00, 4.66873404e-01],
                   [1.19337207e+00, 4.65041057e-01],
                   [1.19832753e+00, 4.65010360e-01],
                   [1.20450463e+00, 4.65082042e-01],
                   [1.20515405e+00, 4.64984277e-01],
                   [1.21100608e+00, 4.65559255e-01],
                   [1.21104992e+00, 4.65428682e-01],
                   [1.21107125e+00, 4.64766215e-01],
                   [1.21181236e+00, 4.65517778e-01],
                   [1.22155827e+00, 4.60547876e-01],
                   [1.22615103e+00, 4.60597453e-01],
                   [1.22704429e+00, 4.57544371e-01],
                   [1.22766162e+00, 4.59069303e-01],
                   [1.22801041e+00, 4.57832165e-01],
                   [1.23048461e+00, 4.58894834e-01],
                   [1.23081032e+00, 4.55280307e-01],
                   [1.23131302e+00, 4.55051910e-01],
                   [1.23338046e+00, 4.57408663e-01],
                   [1.23587213e+00, 4.53413450e-01],
                   [1.23647502e+00, 4.49506366e-01],
                   [1.23660092e+00, 4.53855399e-01],
                   [1.23768723e+00, 4.47301809e-01],
                   [1.23956988e+00, 4.42422822e-01],
                   [1.24080236e+00, 4.40366851e-01],
                   [1.24082191e+00, 4.26648252e-01],
                   [1.24163702e+00, 4.43960822e-01],
                   [1.24207393e+00, 4.10873683e-01],
                   [1.24228467e+00, 4.05449792e-01],
                   [1.24247441e+00, 4.03534612e-01],
                   [1.24306728e+00, 4.22558922e-01],
                   [1.24310149e+00, 4.01980314e-01],
                   [1.24332595e+00, 3.95099113e-01],
                   [1.24379848e+00, 4.32281608e-01],
                   [1.24409104e+00, 3.53063866e-01],
                   [1.24414646e+00, 3.66702373e-01],
                   [1.24442385e+00, 3.73881674e-01],
                   [1.24457798e+00, 3.79324544e-01],
                   [1.24468008e+00, 4.28711627e-01],
                   [1.24556999e+00, 3.41049158e-01],
                   [1.24560485e+00, 3.79895682e-01],
                   [1.24576130e+00, 3.80687711e-01],
                   [1.24578043e+00, 3.25204425e-01],
                   [1.24581851e+00, 3.29870135e-01],
                   [1.24583000e+00, 3.63549975e-01],
                   [1.24589741e+00, 3.00677940e-01],
                   [1.24615237e+00, 2.17619502e-01],
                   [1.24626648e+00, 2.16537665e-01],
                   [1.24632260e+00, 7.16994763e-02],
                   [1.24639375e+00, 3.22151414e-01],
                   [1.24672293e+00, 1.28282828e-01],
                   [1.24689033e+00, 3.04205475e-01],
                   [1.24696298e+00, 9.20261715e-02],
                   [1.24703244e+00, 9.13266525e-02],
                   [1.24708202e+00, 3.47775405e-01],
                   [1.24713700e+00, 2.76527176e-01],
                   [1.24741122e+00, 3.27509756e-01],
                   [1.24745164e+00, -4.78276003e-03],
                   [1.24749055e+00, 2.93037896e-01],
                   [1.24751568e+00, 5.11384772e-02],
                   [1.24755422e+00, 3.55569107e-01],
                   [1.24767165e+00, 2.83156566e-01],
                   [1.24770378e+00, 2.48292015e-01],
                   [1.24782441e+00, 1.80787263e-01],
                   [1.24792283e+00, 1.66433413e-01],
                   [1.24801915e+00, 2.29811511e-01],
                   [1.24801937e+00, 2.27723239e-01],
                   [1.24803849e+00, 2.80118002e-01],
                   [1.24807950e+00, 7.96055789e-02],
                   [1.24829145e+00, -2.27022810e-02],
                   [1.24834721e+00, 2.08631039e-01],
                   [1.24835503e+00, 3.60505121e-01],
                   [1.24849357e+00, 2.27849927e-01],
                   [1.24873498e+00, 1.55606508e-01],
                   [1.24898590e+00, -2.21937218e-02],
                   [1.24914840e+00, 1.51241002e-01],
                   [1.24916260e+00, 2.56205657e-01],
                   [1.24921759e+00, 2.75123830e-01],
                   [1.24935739e+00, 8.50962251e-04],
                   [1.24940689e+00, 3.37149838e-02],
                   [1.24943147e+00, 3.10950153e-02],
                   [1.24953136e+00, 1.52839048e-01],
                   [1.24955336e+00, 2.62720675e-01],
                   [1.24970031e+00, 1.56437774e-01],
                   [1.24985015e+00, 1.79172679e-01],
                   [1.24992197e+00, 7.35849813e-02],
                   [1.24995158e+00, 1.44049457e-01]])


# 计算所有点之间的距离矩阵
def compute_distance_matrix(points):
    return distance_matrix(points, points)


# 使用距离矩阵计算最短路径
def find_nearest_path_with_matrix(dist_matrix, start_index):
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
    # print("dist_matrix", dist_matrix)
    # print("start_index", start_index, total_distance)
    return total_distance, path


# 主函数，寻找端点
def find_endpoint(dist_matrix):
    min_total_distance = float('inf')
    best_start_index = -1
    best_path = []

    for i in range(len(dist_matrix)):
        total_distance, path = find_nearest_path_with_matrix(dist_matrix, i)
        if total_distance < min_total_distance:
            min_total_distance = total_distance
            best_start_index = i
            best_path = path

    return best_start_index, min_total_distance, best_path


dist_matrix = compute_distance_matrix(points)
for i in range(len(dist_matrix)):
    dist_matrix[i, i] = np.inf

best_start_index, min_total_distance, best_path = find_endpoint(dist_matrix)

points = points[best_path]

segments = cluster_split.split_points_by_curvature(points)

# 绘制分割结果
plot_segments(segments)

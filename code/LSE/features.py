import numpy as np


class FeaturesDetection:
    def __init__(self):
        # variables for storing things
        self.LASERPOINTS = np.zeros(0) #雷达扫到的点（替换成我们的雷达） (x,y),(x,y)，可以考虑传入的仅是障碍物上的点
        # self.SEED_SEGMENTS = [] #存储种子线段的列表 has deleted
        self.LINE_SEGMENTS = []#存储线段的列表
        self.LINE_PARAMS = None #存储线段参数 a b c
        self.FEATURES = [] #存储的是一条线段的[线段方程，array(起点，终点)] new_rep（在test_LSE.py中存放当前时刻扫到的所有线段信息)
        self.All_features = [] #用于存储所有的历史线段信息
        self.LANDMARKS = [] #存储地标信息列表

#---------------    pygame  ------------------------------------------------------------------------------------------------
        # variables in Pixel units
        # self.EPSILON = 10  # 最大正交距离 maximum orthogonal distance of a point to its line
        # self.DELTA = 50  # 最大非正交距离 maximum non-orthogonal distance of a point to its line
        # self.Kappa = 2 * self.EPSILON  # this variable is necessary, because we delete measurements without obstacles
        # self.LMIN = 20  # 最小线段长度 minimum length of a line segment (originally 20)
        #
        # # variables for counting points
        # self.NP = len(self.LASERPOINTS) - 1  # number of scanned points扫描点的数量
        # self.SNUM = 6  # number of points to start segment detection with用于开始检测线段的点数量，点达到多少才开始检测线段
        # self.PMIN = 10  # number of points a line segment has to have at least生成线段至少需要的点数
        # self.STEP = min(self.SNUM, self.PMIN - self.SNUM)  # step size in segment detection线段检测中的步长

#--------------------   nf test in env without disturbance-----------------------------------------------------------------------------------------------
        # # variables in Pixel units
        # self.EPSILON = 10  # 预测点到 估计直线的距离
        # self.DELTA = 0.01  # 预测点到 原来laser点的距离
        # # self.Kappa = 1 # this variable is necessary, because we delete measurements without obstacles
        # self.Kappa = 0.5 #相邻laser点之间的距离
        # self.LMIN = 0.01  # 最小线段长度 minimum length of a line segment (originally 20)

        # # variables for counting points
        # self.NP = len(self.LASERPOINTS) - 1  # number of scanned points扫描点的数量
        # self.SNUM = 3  # number of points to start segment detection with用于开始检测线段的点数量，点达到多少才开始检测线段
        # self.PMIN = 3  # number of points a line segment has to have at least生成线段至少需要的点数
        # # self.STEP = min(self.SNUM, self.PMIN - self.SNUM)  # step size in segment detection线段检测中的步长
        # self.STEP = 1

#--------------------   nf test in env with lidar random disturbance-----------------------------------------------------------------------------------------------
        # variables in Pixel units
        self.EPSILON = 10  # 预测点到 估计直线的距离 #预测的点与真实的点距离 小于DELTA  && 预测点和直线距离小于epsilon
        self.DELTA = 0.01  # 预测点到 原来laser点的距离
        # self.Kappa = 1 # this variable is necessary, because we delete measurements without obstacles
        self.Kappa = 0.2 #相邻laser点之间的距离
        self.LMIN = 0.01  # 最小线段长度 minimum length of a line segment (originally 20)

        # variables for counting points
        self.NP = len(self.LASERPOINTS) - 1  # number of scanned points扫描点的数量
        self.SNUM = 3  # number of points to start segment detection with用于开始检测线段的点数量，点达到多少才开始检测线段
        self.PMIN = 3  # number of points a line segment has to have at least生成线段至少需要的点数
        # self.STEP = min(self.SNUM, self.PMIN - self.SNUM)  # step size in segment detection线段检测中的步长
        self.STEP = 1


    # distance point to line written in the general form
    @staticmethod
    def dist_point2line(params, point):#计算点到直线的距离
        '''
        :param params: a b c
        :param point: 点
        :return: 点到直线的最短距离
        '''
        a, b, c = params
        w = np.array([a, b])  # orthogonal vector to the line
        return np.abs(np.dot(point, w) + c) / np.linalg.norm(w)

    @staticmethod
    def projection_point2line(params, point):#计算点在直线上的投影 仅在 test中使用 本类无关联函数
        a, b, c = params
        w = np.array([a, b])
        # print("point", point)
        # print("projection",point - w * (np.dot(point, w) + c) / (np.linalg.norm(w) ** 2))
        return point - w * (np.dot(point, w) + c) / (np.linalg.norm(w) ** 2)

    @staticmethod  # Angles, Distances to position
    def AD2pos(distances, angles, robotPosition):#通过lidar角度 距离 转换成点的位置  无关！
        return robotPosition + np.expand_dims(distances, 1) * np.append(np.expand_dims(np.cos(angles), 1),
                                                                        np.expand_dims(np.sin(angles), 1), axis=1)

    @staticmethod
    def intersection2lines(line_params, laserpoint, robotpos):
        '''
        :param line_params:  输入的是由雷达点云 奇异值分解 预测的直线 参数 a b c
        :param laserpoint:
        :param robotpos:
        :return: 雷达点（与机器人的连线）与预测直线的交点
        '''
        a, b, c = line_params
        v_rot = np.array([a, b])  # v_rot 是预测的线的 法向向量 90° rotated directional vector of the line
        ba = np.asarray(laserpoint) - np.asarray(robotpos)  # ba 是雷达点云和机器人未知的方向向量 它是一个list=[[x,y],[x,y]..]每个子元素是一个方向向量 directional vector of the laser scan
        # two parallel lines intersect in infinity

        # print("v_rot",v_rot)
        # print("ba",ba)
        # print("intersection2lines", robotpos + ba * (-c - np.dot(robotpos, v_rot)) /np.expand_dims(np.dot(ba, v_rot), len(np.asarray(laserpoint).shape) - 1),)
        # print("intersection",robotpos + ba * (-c - np.dot(robotpos, v_rot)) / np.expand_dims(np.dot(ba, v_rot), len(laserpoint.shape) - 1)  if np.all(np.dot(ba, v_rot) != 0) else np.full(laserpoint.shape, np.inf))
        return robotpos + ba * (-c - np.dot(robotpos, v_rot)) / \
            np.expand_dims(np.dot(ba, v_rot), len(np.asarray(laserpoint).shape) - 1) \
            if np.all(np.dot(ba, v_rot) != 0) else np.full(laserpoint.shape, np.inf)

    @staticmethod
    def odr_fit(data):  # orthogonal distance regression; data is an (n x d) matrix
        '''
        :function : 拟合一堆二维点坐标 通过奇异值分解得到一条直线，
        :param data:
        :return:直线的参数方程 ax+by+c=0
        '''
        data_mean = np.mean(data, axis=0)
        _, _, V = np.linalg.svd(data - data_mean)  # singular value decomposition
        # print("odr_fit",-V[1, 0] * -1e4, V[1, 1] * 1e4, np.dot(data_mean, V[1]) * -1e4)
        return -V[1, 0] * -1e4, V[1, 1] * 1e4, np.dot(data_mean, V[1]) * -1e4  # numbers a, b, c mustn't get too small


    def laser_points_set(self, data):#获得雷达的点云 无关 可替换！
        self.LASERPOINTS = []
        if not data:
            pass
        else:  # convert distance, angle and position to pixel
            self.LASERPOINTS = np.array(self.AD2pos(data[0], data[1], data[2]), dtype=int)
        self.NP = len(self.LASERPOINTS) - 1

    def seed_segment_detection(self, robot_position, break_point_ind):
        self.NP = max(0, self.NP) #扫描点的数量
        # self.SEED_SEGMENTS = []
        # NP = Number (laser-) Points, PMIN = Min Number of Points a seed segment should have
        # print("break_point_ind, (self.NP - self.PMIN), self.STEP", break_point_ind, (self.NP - self.PMIN), self.STEP)
        # print("self.NP",self.NP)
        for i in range(break_point_ind, (self.NP - self.PMIN), self.STEP):
            # print("break_point_ind, (self.NP - self.PMIN), self.STEP",break_point_ind, (self.NP - self.PMIN), self.STEP)
            j = i + self.SNUM  # SNUM = Number of points in our seed segment
            #snum , pmin =2
            # print("self.LASERPOINTS[i:j-1] - self.LASERPOINTS[i+1:j]",self.LASERPOINTS[i:j-1] ,self.LASERPOINTS[i+1:j])
            # if not np.all(np.linalg.norm(self.LASERPOINTS[i:j-1] - self.LASERPOINTS[i+1:j], axis=1) < self.Kappa):
            if not np.all(np.linalg.norm(np.asarray(self.LASERPOINTS[i:j-1]) - np.asarray(self.LASERPOINTS[i+1:j]), axis=1) < self.Kappa):
                #当点与点距离过大就不拟合了
                # print("11111")
                continue  # some could be skipped
            params = self.odr_fit(self.LASERPOINTS[i:j])#对满足的点进行拟合

            predicted_points = self.intersection2lines(params, self.LASERPOINTS[i:j], robot_position)#获得在直线上的交点作为映射点
            # if the fitted line fulfills the epsilon and delta condition
            #预测的点与真实的点距离 小于DELTA  && 预测点和直线距离小于epsilon
            if np.all(np.linalg.norm(predicted_points - self.LASERPOINTS[i:j], axis=1) <= self.DELTA) \
                    and np.all(self.dist_point2line(params, predicted_points) <= self.EPSILON):
                self.LINE_PARAMS = params
                return [self.LASERPOINTS[i:j], (i, j)]#生成了一个子线段
            else:
                # print("33333333")
                pass
        return False

    def seed_segment_growing(self, indices, break_point): # FIXME: 在扩充点的时候出现问题
        line_eq = self.LINE_PARAMS
        i, j = indices
        # Beginning and Final points in the line segment
        PB, PF = i, j
        # print("self.LASERPOINTS[PF]",self.LASERPOINTS[PF],PF,len(self.LASERPOINTS))
        while self.dist_point2line(line_eq, self.LASERPOINTS[PF]) < self.EPSILON and \
                np.linalg.norm(self.LASERPOINTS[PF] - self.LASERPOINTS[PF - 1]) < self.Kappa:
            if PB <= PF <= self.NP - 1:
                line_eq = self.odr_fit(self.LASERPOINTS[PB:PF])
            else:
                break
            PF += 1
        PF -= 1

        while self.dist_point2line(line_eq, self.LASERPOINTS[PB]) < self.EPSILON and \
                np.linalg.norm(self.LASERPOINTS[PB] - self.LASERPOINTS[PB + 1]) < self.Kappa:
            if break_point <= PB <= PF:
                line_eq = self.odr_fit(self.LASERPOINTS[PB:PF])
            else:
                break
            PB -= 1
        PB += 1

        # if the line is long enough and contains enough points
        if (np.linalg.norm(self.LASERPOINTS[PB] - self.LASERPOINTS[PF]) >= self.LMIN) and (PF - PB > self.PMIN):
            self.LINE_PARAMS = line_eq
            self.LINE_SEGMENTS.append((self.LASERPOINTS[PB + 1], self.LASERPOINTS[PF - 1]))
            return [line_eq, PB, PF]
        else:
            return False

    def landmark_association(self, landmarks):
        thresh = 10
        for _landmark in landmarks:

            flag = False
            for i, Landmark in enumerate(self.LANDMARKS):
                # print("landmarks",self.LANDMARKS)
                dist = np.linalg.norm(_landmark[2] - np.array(Landmark[2]))
                if dist < thresh:
                    if not is_overlap(_landmark[1], Landmark[1]):
                        continue
                    else:
                        self.LANDMARKS.pop(i)
                        self.LANDMARKS.insert(i, _landmark)
                        flag = True

                        break
            if not flag:
                self.LANDMARKS.append(_landmark)


def is_overlap(seg1, seg2):
    length1 = np.linalg.norm(seg1[0] - seg1[1])
    length2 = np.linalg.norm(seg2[0] - seg2[1])
    return np.linalg.norm(np.sum(seg1 - seg2, axis=0) / 2) <= (length1 + length2) / 2

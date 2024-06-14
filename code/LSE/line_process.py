import math

import numpy as np
from shapely import LineString, Point
import matplotlib.pyplot as plt
from scipy import stats

step = 0
changed = False

angle_threshold = 30
line_distance_threshold = 0.3
ADD_CORNER_THRESHOLD = 0.3


class Line(object):
    def __init__(self):
        self.param = None  # 可以是元组(tuple)
        self.endpoint = None  # 可以是数组(array)
        self.direction = None  # 可以是数组(array)
        self.points_on_line = None  # 可以是数组(array)
        self.step = 0  # 更新的轮数
        self.changed = False

    def decompose_line(self):
        line = [self.param, self.endpoint, self.direction, self.points_on_line, self.step, self.changed]
        return line

    def generate_line(self, line):
        self.param = line[0]
        self.endpoint = line[1]
        self.direction = line[2]
        self.points_on_line = line[3]
        self.step = line[4]
        self.changed = line[5]


class Line_process():
    def __init__(self):
        self.all_line_segment_list = []  # 用于存储目前为止 探测生成的所有线段
        self.points_unprocessed = np.zeros(0)
        self.all_line_segment_class = []  # class Line list

    def decompose_line_process(self):
        line_list = []
        for line_class in self.all_line_segment_class:
            line = [[], [], [], [], [], []]
            line[0] = line_class.param
            line[1] = line_class.endpoint
            line[2] = line_class.direction
            line[3] = line_class.points_on_line
            line[4] = line_class.step
            line[5] = line_class.changed
            line_list.append([line[0], line[1], line[2], line[3], line[4], line[5]])
        self.all_line_segment_list = line_list

    def generate_line_process(self):
        line_class_list = []
        for line in self.all_line_segment_list:
            line_class = Line()
            line_class.param = line[0]
            line_class.endpoint = line[1]
            line_class.direction = line[2]
            line_class.points_on_line = line[3]
            line_class.step = line[4]
            line_class.changed = line[5]
            line_class_list.append(line_class)
        self.all_line_segment_class = line_class_list

    def whether_merge(self, line1, line2):  # 判断两条直线是否该合并
        # 若两线段斜率差距不大，且最短距离不大 则表示可以合并，否则不能合并
        a1, b1, c1 = line1[0]
        a2, b2, c2 = line2[0]
        endpoint1, endpoint2 = line1[1][0], line1[1][1]
        endpoint3, endpoint4 = line2[1][0], line2[1][1]

        segment1 = LineString([endpoint1, endpoint2])
        segment2 = LineString([endpoint3, endpoint4])
        distance = segment1.distance(segment2)
        # print("aaa")
        if b1 == 0 and b2 != 0:
            slope2 = -a2 / b2
            angle2 = math.degrees(math.atan(slope2))
            angle1 = 90
            difference = abs(angle1 - angle2)  # 两条直线的夹角
            if difference >= 90:
                difference = 180 - difference
            if difference < angle_threshold and distance <= line_distance_threshold:
                return True
            else:
                return False

            return False
        if b1 != 0 and b2 == 0:
            slope1 = -a1 / b1
            angle1 = math.degrees(math.atan(slope1))
            angle2 = 90
            difference = abs(angle1 - angle2)  # 两条直线的夹角
            if difference >= 90:
                difference = 180 - difference
            if difference < angle_threshold and distance <= line_distance_threshold:
                return True
            else:
                return False
            return False

        if b1 == 0 and b2 == 0 and distance <= line_distance_threshold:
            return True

        slope1 = -a1 / (b1 + 1e-5)
        angle1 = math.degrees(math.atan(slope1))
        slope2 = -a2 / (b2 + 1e-5)
        angle2 = math.degrees(math.atan(slope2))
        difference = abs(angle1 - angle2)  # 两条直线的夹角
        if difference >= 90:
            difference = 180 - difference

        # print("111",line1[0],angle1,line2[0],angle2,difference,distance)
        # 判断斜率和距离
        if difference <= angle_threshold and distance <= line_distance_threshold:
            # print("正确", "角度",line1[0],angle1,line2[0],angle2,difference,distance,"True")
            # print("")
            # 此时认为是可以合并的
            return True
        else:
            # print("错误","角度",line1[0],angle1,line2[0],angle2,difference,distance,"False")
            # print("")
            return False

    # 定义一个函数，根据直线的参数，返回交点的坐标
    def get_intersection_point(self, Line1, Line2):
        # 判断两条直线是否平行，如果平行，就没有交点
        a1, b1, c1 = Line1
        a2, b2, c2 = Line2
        if a1 * b2 == a2 * b1 and a1 != 0 and b1 != 0 and a2 != 0 and b2 != 0:
            return None
        # 否则，用克拉默法则求出x和y的值
        elif a1 == a2 and a1 == 0 and b1 != b2:
            return None
        elif b1 == b2 and b1 == 0 and a1 != a2:
            return None
        else:
            x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1 + 1e-5)
            y = (a2 * c1 - a1 * c2) / (a1 * b2 - a2 * b1 + 1e-5)
            # 返回交点的坐标
            return [x, y]

    def whether_add_corner(self, robot):
        length = len(self.all_line_segment_list)
        for i in range(0, length - 1):
            # line : param,endpoint,normal,segment(all points)
            Line1 = LineString(self.all_line_segment_list[i][1])
            Line1_param = self.all_line_segment_list[i][0]

            for j in range(i + 1, length):
                Line2 = LineString(self.all_line_segment_list[j][1])
                Line2_param = self.all_line_segment_list[j][0]
                intersection_points = self.get_intersection_point(Line1_param, Line2_param)
                position = np.array([robot.pose[0], robot.pose[1]])
                # print("abc")
                if intersection_points == None:
                    # 若两条直线无交点，或者交点还没有进入robot的扫描范围就跳过
                    # print("def")
                    continue
                # elif Line1.intersects(Point(intersection_points)) == True and Line2.intersects(Point(intersection_points)) == True :
                #     #如果 这两条直线已经同时 交在一个点上了  就不需要作后续处理
                #     #实际上在 无扰动的情况下是正确的， 然而在有扰动的情况下 交点不一定正好是 原map中两条直线的真实交点
                #     # print("fgh")
                #     j = j + 1
                elif Line1.distance(Point(intersection_points)) <= ADD_CORNER_THRESHOLD and Line2.distance(
                        Point(intersection_points)) <= ADD_CORNER_THRESHOLD:
                    #  or np.linalg.norm(position-np.array(intersection_points)) < robot.lidar.radius + :
                    # 需要把corner加入线段中
                    # 加入的时候 需要将 corner点作为固定的一个点
                    points1 = [self.all_line_segment_list[i][1][0], self.all_line_segment_list[i][1][1], intersection_points]
                    points2 = [self.all_line_segment_list[j][1][0], self.all_line_segment_list[j][1][1], intersection_points]
                    new_line1 = self.compute_max_distance_point(points1)
                    new_line2 = self.compute_max_distance_point(points2)
                    self.all_line_segment_list[i][1] = new_line1
                    self.all_line_segment_list[j][1] = new_line2

    def compute_max_distance_point(self, points):
        max_distance = 0
        max_index = 0
        for i in range(0, 3):
            distance = np.linalg.norm(np.array(points[i]) - np.array(points[2]))
            if distance > max_distance:
                max_index = i
                max_distance = distance
        return np.array([points[max_index], points[2]])

    def compute_slope_difference(self, line1, line2):
        '''
        :param line1:
        :param line2:
        :return:    True :means
        '''
        # 若两线段斜率差距不大，且最短距离不大 则表示可以合并，否则不能合并
        a1, b1, c1 = line1[0]
        a2, b2, c2 = line2[0]
        endpoint1, endpoint2 = line1[1][0], line1[1][1]
        endpoint3, endpoint4 = line2[1][0], line2[1][1]
        segment1 = LineString([endpoint1, endpoint2])
        segment2 = LineString([endpoint3, endpoint4])
        distance = segment1.distance(segment2)
        # print("aaa")
        if b1 == 0 and b2 != 0:
            slope2 = -a2 / b2
            angle2 = math.degrees(math.atan(slope2))
            angle1 = 90
            difference = abs(angle1 - angle2)  # 两条直线的夹角
            if difference >= 90:
                difference = 180 - difference
            if difference < angle_threshold and distance <= line_distance_threshold:
                return True
            else:
                return False

            return False
        if b1 != 0 and b2 == 0:
            slope1 = -a1 / b1
            angle1 = math.degrees(math.atan(slope1))
            angle2 = 90
            difference = abs(angle1 - angle2)  # 两条直线的夹角
            if difference >= 90:
                difference = 180 - difference
            if difference < angle_threshold and distance <= line_distance_threshold:
                return True
            else:
                return False
            return False

        if b1 == 0 and b2 == 0 and distance <= line_distance_threshold:
            return True

        slope1 = -a1 / b1
        angle1 = math.degrees(math.atan(slope1))
        slope2 = -a2 / b2
        angle2 = math.degrees(math.atan(slope2))
        difference = abs(angle1 - angle2)  # 两条直线的夹角
        if difference >= 90:
            difference = 180 - difference

        if difference <= angle_threshold and distance <= line_distance_threshold:

            return True
        else:

            return False

    def merge_2line(self, line1, line2):  # 用于合并两条线段，方法：投影方法
        # TODO:choose which one merges to another
        '''
        :function merge line2 to line1
        :param line1: old line  line1 = [params, endpoints, direction, history_points]
        :param line2: new_line line2 = [params, endpoints, direction, new_points]
        :return:
        '''
        # line = [params, endpoints, direction, history_points]
        # hypothesis let line2 merge to line1 ,form a new line 1
        if self.whether_merge(line1, line2) == False:
            # print("mmm", line1)
            # print("nnn", line2)
            # print("lll", "False不合并")
            return
        else:  # 表示可以合并 考虑将line2合并到line1上去
            # print("type(line1[3]",type(line1[3]),line1[3])
            # print("type(line2[3]", type(line2[3]), line2[3])
            # 获取line1[3]和line2[3]中不重复的值
            whole_points = np.concatenate((line1[3], line2[3]))
            line1[3] = np.unique(whole_points, axis=0)
            # slope, intercept, r_value, p_value, std_err = stats.linregress(line1[3][:,0], line1[3][:,1])

            endpoint = self.projection_line2line(line1, line2)  # self.projection_line2line() 返回的是投影的两个端点
            point1 = line1[1][0]
            point2 = line1[1][1]
            point3 = endpoint[0]
            point4 = endpoint[1]
            Final_Endpoint = self.max_distance([point1, point2, point3, point4])
            line_old = line1
            line_new = line2
            if math.sqrt((line_old[2][0])**2 + (line_old[2][1])**2) < 1e-5:
                line_old[2] = line_new[2]

            NEW_LINE = [line1[0], Final_Endpoint, line_old[2], line1[3], step, changed]
            # print("mmm", line1[1],line1[2])
            # print("nnn", line2[1],line2[2])
            # print("lll", NEW_LINE)
            return NEW_LINE

    def merge_myline(self):
        '''
        :param all_linesegment: self.all_line_sgement
        :return:
        '''
        new_all_line_segment_list = []
        for i in range(0, len(self.all_line_segment_list)):

            flag = 0
            if len(new_all_line_segment_list) == 0:
                # 第一次时候 new_all_line_segment为空,则直接把self.all_line_segement加入
                new_all_line_segment_list.append(self.all_line_segment_list[i])
            for j in range(0, len(new_all_line_segment_list)):
                new_line = self.merge_2line(self.all_line_segment_list[i], new_all_line_segment_list[j])
                if new_line != None:
                    # 说明这两条线段能够合并
                    flag = 1
                    new_all_line_segment_list[j] = new_line
                    break
            if flag == 0:  # 则说明遍历完了还是发现没法合并，则是一个新的无法合并线段
                new_all_line_segment_list.append(self.all_line_segment_list[i])
        self.all_line_segment_list = new_all_line_segment_list

    def projection_line2line(self, line1, line2):  # 计算点在直线上的投影 仅在 test中使用 本类无关联函数
        '''
        :param line1: line1 =[(a,b,c),array(endpoints:p1,p2)]
        :param line2:
        :return:
        '''
        a, b, c = line1[0]
        endpoint1 = line2[1][0]  # line2 's endpoint1
        endpoint2 = line2[1][1]
        w = np.array([a, b])
        projection1 = endpoint1 - w * (np.dot(endpoint1, w) + c) / (np.linalg.norm(w) ** 2)
        projection2 = endpoint2 - w * (np.dot(endpoint2, w) + c) / (np.linalg.norm(w) ** 2)
        return np.array([projection1, projection2])

    def max_distance(self, points):
        max_dist = 0
        point1, point2 = None, None

        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dist = math.sqrt((points[j][0] - points[i][0]) ** 2 + (points[j][1] - points[i][1]) ** 2)
                if dist > max_dist:
                    max_dist = dist
                    point1, point2 = points[i], points[j]

        return np.array([point1, point2])

    def force_mapping(self):
        for i in range(0, len(self.all_line_segment_list)):
            if self.all_line_segment_list[i][0][0] != 0 and self.all_line_segment_list[i][0][1] != 0:
                if abs(self.all_line_segment_list[i][0][0]) > abs(self.all_line_segment_list[i][0][1]):
                    self.all_line_segment_list[i][0][1] = 0

                else:
                    self.all_line_segment_list[i][0][0] = 0
import yaml
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import LineString

from ENV.line_to_squircle import LineToSquircle


class RealWorld(object):
    def __init__(self, config_file='./CONFIG/simple_world.yaml'):
        self.config_file = config_file
        self.workspace = None  # contains a lot of polygonobs
        self.obstacles = None  # contains a lot of polygonobs
        self.config = None

        self.x_limits = None
        self.y_limits = None
        self.width = None
        self.height = None

        self.load_world_config()
        self.construct_world()
        self.get_workspace_size()

    def load_world_config(self):
        # with open(self.config_file, "r") as stream:
        with open(self.config_file, "rb") as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def construct_world(self):
        self.obstacles = []
        # add obstacles by param
        all_obs = self.config['obstacles']
        if all_obs is None:
            pass
        else:
            for obs in all_obs:
                obs_type = obs['type']
                obs_vertices = []
                for vertex in obs['vertices']:
                    obs_vertices.append(np.array(vertex))
                polygon = PolygonObs(obs_type, obs_vertices)
                self.obstacles.append(polygon)

        self.workspace = []
        # add workspace by param
        all_ws = self.config['workspace']
        for ws in all_ws:
            ws_type = ws['type']
            ws_vertices = []
            for vertex in ws['vertices']:
                ws_vertices.append(np.array(vertex))
            polygon = PolygonObs(ws_type, ws_vertices)
            self.workspace.append(polygon)

    def get_workspace_size(self):
        ws = self.workspace[0]
        left_bottom_corner = ws.vertices[0]
        right_top_corner = ws.vertices[2]
        self.x_limits = [left_bottom_corner[0], right_top_corner[0]]
        self.y_limits = [left_bottom_corner[1], right_top_corner[1]]
        self.width = self.x_limits[1] - self.x_limits[0]
        self.height = self.y_limits[1] - self.y_limits[0]

    def check_point_in_free_space(self, q):
        for ws in self.workspace:
            if not ws.check_point_inside(q):
                return False
        for obs in self.obstacles:
            if obs.check_point_inside(q):
                return False
        return True

    def check_point_in_obs_free_space(self, q):
        for obs in self.obstacles:
            if obs.check_point_inside(q):
                return False
        return True

    def check_point_distance_of_obstacle(self, q, radius):  # 可变相将机器人膨胀
        for ws in self.workspace:
            if not ws.check_point_inside_inflation(q, radius):
                return False
        for obs in self.obstacles:
            if obs.check_point_inside_inflation(q, radius):
                return False
        return True


class InitWorld(RealWorld):
    def __init__(self, config_file='./CONFIG/simple_world.yaml'):
        super().__init__(config_file)


class Line(object):
    def __init__(self, endpoint):
        self.endpoint = endpoint


class PolygonObs(object):
    def __init__(self, type_, vertices):
        self.type = type_
        self.vertices = vertices
        self.sides = None

        self.construct_sides()

    def construct_sides(self):
        self.sides = []
        for index in range(len(self.vertices) - 1):
            end_points = [self.vertices[index], self.vertices[index + 1]]
            side = Line(end_points)
            self.sides.append(side)
        end_points = [self.vertices[-1], self.vertices[0]]
        side = Line(end_points)
        self.sides.append(side)

    def check_point_inside(self, q):
        vertex_list = [tuple(vertex) for vertex in self.vertices]
        polygon = Polygon(vertex_list)
        point = Point(q[0], q[1])
        return polygon.contains(point)

    def check_point_inside_inflation(self, q, radius):
        vertex_list = [tuple(vertex) for vertex in self.vertices]
        polygon = Polygon(vertex_list)
        point = Point(q[0], q[1])
        circle = point.buffer(radius)
        return polygon.intersects(circle)


class ForestWorld(object):
    def __init__(self, workspace, obstacles):
        self.workspace = workspace
        self.obstacles = obstacles


class RobotWorld(object):
    """
    function: use to update the robot world
        ① merge the points and lines in each obstacle and workspace(update point and line)
        ② update the squircle
        ③ update forest
    input: the new classified points and new classified lines in each obs and workspace
    """

    def __init__(self):
        self.workspace = []  # list = [ [], [], []....] which sub_list contains polygon
        self.workspace.append(Squircle('workspace', np.array([3.5, 2.0]), 7.0, 4.0))
        self.obstacles = []  # list = [ [], [], []....]
        self.forest_world = None

        self.points_ws = []  # list = [ [], [], []....] which sub_list contains points(history)
        self.points_obs = []  # list = [ [], [], []....] which sub_list contains points(history)

        self.lidar_point_set = []
        self.line_ws = []
        self.line_obs = []

        self.obstacles_points = []  # 历史点云
        self.all_line_class = []
        self.now_point = []  # 当前点云
        self.all_line_list = []
        self.all_squircles = []

    def check_point_in_free_space(self, q: np.ndarray, threshold=0.0):
        if self.workspace[0][0].check_point_inside(q, threshold):
            return False
        for ws in self.workspace:
            for ws_i in ws[1:]:
                if ws_i.check_point_inside(q, threshold):
                    return False
        for obs in self.obstacles:
            for obs_i in obs:
                if obs_i.check_point_inside(q, threshold):
                    return False
        return True

    def on_same_line(self, points):  # 判断points中的点是否在同一直线上
        if len(points) < 3:
            return True

        (x0, y0), (x1, y1) = points[0], points[1]
        if x1 - x0 != 0:
            slope = (y1 - y0) / (x1 - x0)
            # print("slope", slope)
        else:
            slope = None

        for i in range(2, len(points)):
            x, y = points[i]
            # print(points[i])
            if x == x0 and y == y0:  # 如果当前点与第一个点坐标相同，则跳过判断
                continue
            if x - x0 != 0:
                new_slope = (y - y0) / (x - x0)
                # print(new_slope)
            else:
                new_slope = None

            if new_slope != slope:
                return False

        return True

    def farthest_points_np(self, points):  # just return two points,not list
        '''
        :param points:
        :return: the farthest_points in points,when all points are on the same line
        '''
        # print("farthest_points_np_points",points)
        points = np.array(points)
        dists = np.sqrt(np.sum((points[:, None] - points) ** 2, axis=-1))
        i, j = np.unravel_index(dists.argmax(), dists.shape)
        return list(points[i]), list(points[j])

    def update_points(self, point_in_obs):

        if not self.points_obs:
            self.points_obs = point_in_obs
        else:
            self.obstacles_points.extend(points for points in point_in_obs)

        self.now_point = point_in_obs

    def merge_myline(self):

        remove_index = []
        # merge obs first
        for i in range(0, len(self.line_obs)):  # i means which obs/ws
            if self.line_obs[i] == [] or self.line_obs[i] == [[]]:
                continue
            for j in range(0, len(self.line_obs[i]) - 1):
                for k in range(j + 1, len(self.line_obs[i])):
                    points = [self.line_obs[i][j][0], self.line_obs[i][j][1], self.line_obs[i][k][0],
                              self.line_obs[i][k][1]]

                    line1 = LineString([self.line_obs[i][j][0], self.line_obs[i][j][1]])
                    line2 = LineString([self.line_obs[i][k][0], self.line_obs[i][k][1]])
                    # print("online",self.on_same_line(points))
                    if self.on_same_line(points) == True and line1.distance(line2) == 0:
                        # 表明存在需要合并的点
                        # print("pointaaaaaaaaa",[self.line_obs[i][j][0],self.line_obs[i][j][1],self.line_obs[i][k][0],self.line_obs[i][k][1]])
                        self.line_obs[i][j] = [self.farthest_points_np(points)]
                        remove_index.append(k)
            # print("remove_index",remove_index)
            self.line_obs[i] = [value for index, value in enumerate(self.line_obs[i]) if index not in remove_index]

        # merge ws second

    def update_line(self, points_to_line):
        # TODO: maybe need to add function merge_myself
        """
        :general_method : when distance(line1,line2) == 0 and on_same_line(line1,line2) ,then merge it
                            else add line to the line_ws/obs (* in the final we need to merge line_ws/obs itself
        :param points_to_line: class PointToLine  just use its line_in_ws/obs
        :return: new_line_ws/obs
        """
        flag = 0  # if flag =0 means no need to merge
        if not self.line_obs:
            self.line_obs = points_to_line.line_in_obs
        else:
            # means we need to iterate the line
            obs_length = len(points_to_line.line_in_obs)
            # pick out each new line in obs to compare with the old line
            for i in range(0, obs_length):  # i for polygonobs_index
                flag = 0
                new_line_list = points_to_line.line_in_obs[i]
                # print("new_line_list", new_line_list)
                if new_line_list == [[]]:
                    continue
                old_line_list = self.line_obs[i]
                for j in range(0, len(new_line_list)):  # j for new_line
                    new_line = new_line_list[j]
                    flag = 0
                    for k in range(0, len(old_line_list)):
                        old_line = old_line_list[k]
                        p1 = old_line[0]
                        p2 = old_line[1]
                        p3 = new_line[0]
                        p4 = new_line[1]
                        line1 = LineString([p1, p2])
                        line2 = LineString([p3, p4])
                        point_set = [p1, p2, p3, p4]
                        if self.on_same_line(point_set) and line1.distance(
                                line2) == 0:  # on the same line and overlapping then merge
                            merged_line = list(self.farthest_points_np(point_set))
                            self.line_obs[i][k] = merged_line
                            flag = 1
                            break
                    if flag == 0:  # if flag =0 means no need to merge
                        self.line_obs[i].append(new_line)
        # print("final_self.line_obs", self.line_obs)
        self.merge_myline()
        # print("final_self.line_obs", self.line_obs)

        flag = 0  # if flag =0 means no need to merge
        if not self.line_ws:
            self.line_ws = points_to_line.line_in_ws
        else:
            # means we need to iterate the line
            ws_length = len(points_to_line.line_in_ws)
            # pick out each new line in ws to compare with the old line
            for i in range(0, ws_length):  # i for polygonobs_index
                flag = 0
                new_line_list = points_to_line.line_in_ws[i]
                # print("new_line_list", new_line_list)
                if new_line_list == [[]]:
                    continue
                old_line_list = self.line_ws[i]
                for j in range(0, len(new_line_list)):  # j for new_line
                    new_line = new_line_list[j]
                    flag = 0
                    for k in range(0, len(old_line_list)):
                        old_line = old_line_list[k]
                        p1 = old_line[0]
                        p2 = old_line[1]
                        p3 = new_line[0]
                        p4 = new_line[1]
                        line1 = LineString([p1, p2])
                        line2 = LineString([p3, p4])
                        point_set = [p1, p2, p3, p4]
                        if self.on_same_line(point_set) and line1.distance(
                                line2) == 0:  # on the same line and overlapping then merge
                            merged_line = list(self.farthest_points_np(point_set))
                            self.line_ws[i][k] = merged_line
                            flag = 1
                            break
                    if flag == 0:  # if flag =0 means no need to merge
                        self.line_ws[i].append(new_line)
        # print("final_self.line_ws", self.line_ws)


class Squircle(object):
    def __init__(self, type_, center, width, height, theta=0.0):
        self.type = type_
        self.center = center
        self.width = width
        self.height = height
        self.theta = theta
        self.outer_line = None
        self.ori_line = None
        self.vector = None
        self.extension = None
        if self.type == 'obstacle':
            self.radius = 0.3 * min(width, height)
        else:
            self.radius = 2.0 * max(width, height)

    def potential(self, q, s=1.0):
        x, y, x_0, y_0, a, b = q[0], q[1], self.center[0], self.center[1], self.width / 2, self.height / 2
        if self.type == 'obstacle':
            return ((x - x_0) ** 2 + (y - y_0) ** 2 + (((x - x_0) ** 2 -
                                                        (y - y_0) ** 2 + b ** 2 - a ** 2) ** 2 + (1 - s ** 2) * (
                                                               a ** 2 + b ** 2)) ** 0.5) - (a ** 2 + b ** 2)
        else:
            return (a ** 2 + b ** 2) - ((x - x_0) ** 2 + (y - y_0) ** 2 + (((x - x_0) ** 2 -
                                                                            (y - y_0) ** 2 + b ** 2 - a ** 2) ** 2 + (
                                                                                   1 - s ** 2) * (
                                                                                   a ** 2 + b ** 2)) ** 0.5)

    def x_limits(self, threshold=0.2):
        x_min = self.center[0] - self.width / 2 - threshold
        x_max = self.center[0] + self.width / 2 + threshold
        return x_min, x_max

    def y_limits(self, threshold=0.2):
        y_min = self.center[1] - self.height / 2 - threshold
        y_max = self.center[1] + self.height / 2 + threshold
        return y_min, y_max

    def check_point_inside_limits(self, q, threshold=0.0):
        x_min = self.center[0] - self.width / 2 + threshold
        x_max = self.center[0] + self.width / 2 - threshold
        y_min = self.center[1] - self.height / 2 + threshold
        y_max = self.center[1] + self.height / 2 - threshold
        if x_min <= q[0] <= x_max and y_min <= q[1] <= y_max:
            return True
        else:
            return False

    def check_point_inside(self, q, threshold=0.0, s=1.0):
        x, y, x_0, y_0, a, b = q[0], q[1], self.center[0], self.center[
            1], self.width / 2 + threshold, self.height / 2 + threshold
        if self.type == 'obstacle':
            potential_point = (x - x_0) ** 2 + (y - y_0) ** 2 + (((x - x_0) ** 2 -
                                                                  (y - y_0) ** 2 + b ** 2 - a ** 2) ** 2 + (
                                                                             1 - s ** 2) * (
                                                                         a ** 2 + b ** 2)) ** 0.5 - (a ** 2 + b ** 2)
        else:
            potential_point = (a ** 2 + b ** 2) - ((x - x_0) ** 2 + (y - y_0) ** 2 + (((x - x_0) ** 2 -
                                                                                       (
                                                                                               y - y_0) ** 2 + b ** 2 - a ** 2) ** 2 + (
                                                                                              1 - s ** 2) * (
                                                                                              a ** 2 + b ** 2)) ** 0.5)
        if potential_point <= 0.0:
            return True
        return False

    def workspace_meshgrid(self, resolution=0.05, threshold=0.0):
        # 0.05
        x_min, x_max = self.x_limits()
        y_min, y_max = self.y_limits()
        x = np.arange(x_min - threshold, x_max + threshold, resolution)
        y = np.arange(y_min - threshold, y_max + threshold, resolution)
        xx, yy = np.meshgrid(x, y)
        return xx, yy

    def compute_v(self, q: np.ndarray, beta: float) -> float:
        maxVal = (self.width / 2) ** 2 + (self.height / 2) ** 2
        if self.type == 'obstacle':
            if np.linalg.norm(q - self.center) < 1.0e-3:
                return self.radius * (1.0 + beta / maxVal) * 1e3
            else:
                return self.radius * (1.0 + beta / maxVal) / np.linalg.norm(q - self.center)
        else:
            if np.linalg.norm(q - self.center) < 1.0e-3:
                return self.radius * (1 - beta / maxVal) * 1.0e3
            else:
                return self.radius * (1 - beta / maxVal) / np.linalg.norm(q - self.center)

    def compute_T(self, q: np.ndarray, beta: float) -> np.ndarray:
        return self.compute_v(q, beta) * (q - self.center) + self.center


def on_same_line(points):  # 判断points中的点是否在同一直线上
    if len(points) < 3:
        return True

    (x0, y0), (x1, y1) = points[0], points[1]
    if x1 - x0 != 0:
        slope = (y1 - y0) / (x1 - x0)
        # print("slope", slope)
    else:
        slope = None

    for i in range(2, len(points)):
        x, y = points[i]
        # print(points[i])
        if x == x0 and y == y0:  # 如果当前点与第一个点坐标相同，则跳过判断
            continue
        if x - x0 != 0:
            new_slope = (y - y0) / (x - x0)
            # print(new_slope)
        else:
            new_slope = None

        if new_slope != slope:
            return False

    return True


points = [[2.0, 2.0], [3.9126970440230977, 2.0], [2.0, 2.0], [3.2126970440230975, 2.0]]
# print("1",on_same_line(points))
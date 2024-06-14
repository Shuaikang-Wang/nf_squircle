"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math

import matplotlib.pyplot as plt

import numpy as np

show_animation = True

STEP = 0.1
WEIGHT = STEP
OB_MINX = 0
OB_MINY = 0
OB_MAXX = 7
OB_MAXY = 4
X_WIDTH = 70
Y_WIDTH = 40
class AStarPlanner:

    def __init__(self, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = OB_MINX, OB_MINY
        self.max_x, self.max_y = OB_MAXX, OB_MAXY
        self.obstacle_map = None
        self.x_width, self.y_width = X_WIDTH, Y_WIDTH
        self.motion = self.get_motion_model()
        # self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self,  start, goal, world):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        sx = start[0]
        sy = start[1]
        gx = goal[0]
        gy = goal[1]

        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)
        print('startnode1', start_node.x, start_node.y)
        print("start_node.x",start_node.x)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        print('startnode2', start_node.x,start_node.y)

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break
            else:
                print('time')

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # # show graph
            # if show_animation:  # pragma: no cover
                # plt.plot(self.calc_grid_position(current.x, self.min_x),
                #          self.calc_grid_position(current.y, self.min_y), "oc")
                # for stopping simulation with the esc key.
                # plt.gcf().canvas.mpl_connect('key_release_event',
                #                              lambda event: [exit(
                #                                  0) if event.key == 'escape' else None])
                # if len(closed_set.keys()) % 10 == 0:
                #     plt.pause(0.001)

            if math.sqrt((current.x - goal_node.x)**2 + (current.y - goal_node.y)**2) < STEP:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node, world): # avoid collision
                    print('碰撞了')
                    continue

                if n_id in closed_set:
                    print('测过了')
                    continue
                print('n_id', n_id)
                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                    print('next_node', node.x, node.y)
                    # ax.plot(node.x, node.y, '.b',markersize=3)
                    # self.astar_robot_sensing(ax, world, robot, node)
                else:
                    # self.astar_robot_sensing(ax, world, robot, node)
                    # ax.plot(node.x, node.y, '.b', markersize=3)
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node


        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry
    def astar_robot_sensing(self, ax, world, robot, node):
        robot.pose = np.array([node.x, node.y, 0])


    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = WEIGHT  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node, world):
        # px = self.calc_grid_position(node.x, self.min_x)
        # py = self.calc_grid_position(node.y, self.min_y)
        obstacles = world.obstacles
        workspace = world.workspace
        point = (node.x, node.y)
        # if px < self.min_x:
        #     return False
        # elif py < self.min_y:
        #     return False
        # elif px >= self.max_x:
        #     return False
        # elif py >= self.max_y:
        #     return False

        # collision check
        # if self.obstacle_map[node.x][node.y]:
        #     return False
        if world.check_point_in_free_space(point) == False:
            return False

        # for obs in obstacles:
        #     for obs_i in obs:
        #         if obs_i.check_point_inside(point, threshold):
        #             print('falsenode',node.x,node.y)
        #             return False
        # for ws in workspace:
        #     for ws_i in ws[1:]:
        #         if ws_i.check_point_inside(point, threshold):
        #             print('falsenode', node.x, node.y)
        #             return False
        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        step = STEP
        motion = [[step, 0, step],
                  [0, step, step],
                  [-step, 0, step],
                  [0, -step, step]]
        # motion = [[step, 0, step],
        #           [0, step, step],
        #           [-step, 0, step],
        #           [0, -step, step],
        #           [-step, -step, step * math.sqrt(2)],
        #           [-step, step, step * math.sqrt(2)],
        #           [step, -step, step * math.sqrt(2)],
        #           [step, step, step * math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

def astar_test( robot, start, goal, world, threshold):
    '''
    :param robot: car robot
    :param start: startpoint (x,y)
    :param goal: goalpoint (x,y)
    :return:
    '''
    # start and goal position
    sx = start[0] # [m]
    sy = start[1]  # [m]
    gx = goal[0]  # [m]
    gy = goal[1]  # [m]
    grid_size = robot.size  # [m]
    robot_radius = robot.size  # [m]

    # set obstacle positions
    # for robot, the whole map is an unknown area
    ox = []
    oy = []

    # if show_animation:  # pragma: no cover
    #     ax.plot(sx, sy, "or", markersize=3)
    #     ax.plot(gx, gy, "or", markersize=3)

    a_star = AStarPlanner(ox, oy, 1, robot_radius)
    rx, ry = a_star.planning(start, goal, world, robot, threshold)

    lengthrx = len(rx)
    print('lengthrx',lengthrx)
    # i = len(rx)-1
    # while i >= 0:
    #     print(rx[i], ry[i])
    #     i = i-1
    # if show_animation:  # pragma: no cover
    #     ax.plot(rx, ry, "-r", markersize=3)
    print('finish')


# if __name__ == '__main__':
#     main()
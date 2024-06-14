# from __future__ import print_function
import heapq
import matplotlib.pyplot as plt
import unittest

INFLATION_RADIUS = 0.15

class cell(object):
    def __init__(self, x, y, reachable: bool) -> None:
        self.x = x
        self.y = y
        self.isReachable = reachable
        self.g = 0
        self.h = 0
        self.f = self.g + self.h
        self.parent = None
        self.weight = 0

    def __lt__(self, other):
        return self.f < other.f


class AStar(object):
    def __init__(self) -> None:
        '''
        初始化一些数据结构：堆、集合帮助实现算法
        '''
        self.closed = set()
        self.openlist = []
        heapq.heapify(self.openlist)
        self.cells = []

    def init_grid(self, width, height, real_world):
        '''
        初始化地图，导入cells
        '''
        self.grid_width = width
        self.grid_height = height

        for x in range(0, self.grid_width):
            for y in range(0, self.grid_height):
                if not real_world.check_point_in_free_space([x/10, y/10], 0.2):
                    reachable = False
                else:
                    reachable = True
                self.cells.append(cell(x, y, reachable))

    def get_cell(self, x, y):
        '''
        因为输入cells信息时为一维信息，这里需要通过width和height检索到相应位置的cell
        '''
        return self.cells[int(x * self.grid_height + y)]

    def caculate_one_way(self, start, end):
        '''
        在地图确定不变的情况下，每次传入不同的起点和终点
        '''
        self.start = self.get_cell(*start)
        self.end = self.get_cell(*end)
        # print("self.end",self.end)

    def caculate_heuristic(self, cell):
        '''
        计算启发式距离h值，这里采用曼哈顿距离
        '''
        return 10 * (abs(self.end.x - cell.x) + abs(self.end.y - cell.y))

    def get_adjacent_cell(self, cell):
        '''
        返回cell周围的cell，这里的周围指八个方向
        '''
        adj_cells = []
        for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, -1), (-1, 1), (-1, -1), (1, 1)]:
            x2 = cell.x + dx
            y2 = cell.y + dy
            if x2 > 0 and x2 < self.grid_width and y2 > 0 and y2 < self.grid_height:
                adj_cells.append(self.get_cell(x2, y2))
        return adj_cells

    def get_updated(self, adj, cell):
        '''
        用于每次更新cell信息
        '''
        adj.g = cell.g + 10
        adj.parent = cell
        adj.h = self.caculate_heuristic(adj)
        adj.f = adj.g + adj.h

    def save_path(self):
        '''
        保存计算路径
        '''
        cell = self.end
        path = [[cell.x, cell.y]]
        while cell.parent is not self.start:
            cell = cell.parent
            path.append([cell.x, cell.y])
        path.append([self.start.x, self.start.y])
        path.reverse()
        return path

    def solve(self):
        '''
        代码核心，实现逻辑
        '''
        heapq.heappush(self.openlist, (self.start.f, self.start))
        while len(self.openlist):
            f, cell = heapq.heappop(self.openlist)
            self.closed.add(cell)
            if cell is self.end:
                return self.save_path()

            adj_cells = self.get_adjacent_cell(cell)
            for adj_cell in adj_cells:
                if adj_cell.isReachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.openlist:
                        if adj_cell.g > cell.g + 10:
                            self.get_updated(adj_cell, cell)
                    else:
                        self.get_updated(adj_cell, cell)
                        heapq.heappush(self.openlist, (adj_cell.f, adj_cell))

        raise RuntimeError("A* failed to find a solution")


def draw_result(result_path, walls, start, end):
    plt.plot([v[0] for v in result_path], [v[1] for v in result_path])
    plt.plot([v[0] for v in result_path], [v[1] for v in result_path], 'o', color='lightblue')
    plt.plot([start[0], end[0]], [start[1], end[1]], 'o', color='red')
    plt.plot([barrier[0] for barrier in walls], [barrier[1] for barrier in walls], 's', color='m')
    plt.xlim(-1, 8)
    plt.ylim(-1, 8)
    plt.show()


# if __name__ == '__main__':
#     a = AStar()
#     walls = ((2, 5), (2, 6), (3, 6), (4, 6), (5, 6), (5, 5),
#              (5, 4), (5, 3), (5, 2), (4, 2), (3, 2), (7, 1), (6, 4), (1, 5), (7, 6))
#     a.init_grid(8, 8, real_world)
#     a.caculate_one_way((0, 0), (7, 7))
#     path = a.solve()
#     print(path)
#     draw_result(path, walls, (0, 0), (7, 7))

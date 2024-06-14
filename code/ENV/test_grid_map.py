import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
import cmaps
from matplotlib.colors import ListedColormap
from skimage.draw import line


def draw_occupancy_grid(grid):

    # 创建一个新的图形
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(221)

    # 定义颜色映射，0为灰色，1为白色
    newcolors = (['white', 'grey'])
    newcmap = ListedColormap(newcolors[::1]) # 重构为新的colormap

    # 绘制栅格地图
    plt.imshow(grid, cmap=newcmap, origin='lower', extent=[0, 7, 0, 4])

    # 显示绘图
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_aspect('equal')
    # ax.set_axis_off()
    plt.title('Occupancy Grid Map')
    plt.grid(True)
    plt.show()


# fig = plt.figure(figsize=(9, 6))
# ax = fig.add_subplot(221)

# 创建一个示例的占有栅格地图
grid_resolution = 0.1
grid_size_x = int(7 / grid_resolution)
grid_size_y = int(4 / grid_resolution)

# 随机生成一个占有栅格地图，值为0或1
grid = np.zeros((grid_size_y, grid_size_x))
print(grid.shape)

line_start = [0, 0]
line_end = [2, 10]
line_x = np.linspace(int(line_start[0] / grid_resolution), int(line_end[0] / grid_resolution), int(10 / grid_resolution))
line_y = np.linspace(int(line_start[1] / grid_resolution), int(line_end[1] / grid_resolution), int(10 / grid_resolution))


rr, cc = line(line_start[1], line_start[0], line_end[1], line_end[0])

line_x = [int(i) for i in line_x]
line_y = [int(i) for i in line_y]

print(line_x)
print(line_y)

grid[rr, cc] = 1
# grid[20, 60] = 1

# 绘制占有栅格地图
draw_occupancy_grid(grid)

plt.show()
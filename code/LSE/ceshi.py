import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 创建一个新的图
fig, ax = plt.subplots()

# 绘制workspace
workspace = patches.Polygon([[0.0, 0.0], [7.0, 0.0], [7.0, 4.0], [0.0, 4.0]], closed=True, fill=None, edgecolor='b')
ax.add_patch(workspace)

# 绘制obstacles
obstacles = [
    patches.Polygon([[2.0, 1.0], [4.0, 1.0], [4.0, 2.0], [2.0, 2.0]], closed=True, fill=None, edgecolor='r'),
    patches.Polygon([[5.0, 2.0], [6.0, 2.0], [6.0, 3.0], [5.0, 3.0]], closed=True, fill=None, edgecolor='r')
]

for obstacle in obstacles:
    ax.add_patch(obstacle)

# 绘制其他形状
shapes = [
    patches.Polygon([[5.38, 2.0], [7.18, 2.0], [7.18, 2.2], [5.38, 2.2]], closed=True, fill=None, edgecolor='g'),
    # 其他形状...
]

for shape in shapes:
    ax.add_patch(shape)

# 设置坐标轴范围
plt.xlim(-1, 8)
plt.ylim(-1, 5)

# 显示图形
plt.show()
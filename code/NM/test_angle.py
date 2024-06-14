import numpy as np
import matplotlib.pyplot as plt
import random

# 生成随机角度
angle1 = random.uniform(0, 2*np.pi)
angle2 = random.uniform(0, 2*np.pi)

if angle1 < 0:
    angle1 = angle1 + 2 * np.pi
if angle2 < 0:
    angle2 = angle2 + 2 * np.pi

theta_diff = (angle2 - angle1) / 2
middle_angle = theta_diff + angle1

if abs(angle2 - angle1) > np.pi:
    middle_angle = middle_angle - np.pi
middle_angle = (middle_angle + np.pi) % (2 * np.pi) - np.pi

# 绘制角度图
angles = [0, angle1, angle2, middle_angle]

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_direction(-1)  # 逆时针方向
ax.set_theta_zero_location('N')  # 设置0度角在北方

theta = np.linspace(0, 2*np.pi, 100)
r = np.ones_like(theta)
ax.plot(theta, r, color='gray')  # 绘制一个灰色圆圈作为参考

ax.plot([0, angle1], [0, 1], 'red')
ax.plot([0, angle2], [0, 1], 'red')
ax.plot([0, middle_angle], [0, 1], 'blue')

plt.show()

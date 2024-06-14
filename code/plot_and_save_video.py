import pickle
import os
import sys

sys.path.append(os.getcwd())

import matplotlib.pyplot as plt
import datetime
from ENV.vis import plot_four_ax

file_path = 'DATA/pickle_data/'
current_date_execution = datetime.datetime.now().strftime("execution_%Y_%m_%d")

current_date_execution = 'execution_2024_04_06'

folder_path_execution = os.path.join(file_path, current_date_execution)

with open(folder_path_execution + '/execution.pickle', 'rb') as f:
    execution_data = pickle.load(f)

fig = plt.figure(figsize=(9, 6))

ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(222)
ax3 = fig.add_subplot(223)
ax4 = fig.add_subplot(224)
print("overall frame: ", len(execution_data))

plot_step = len(execution_data)

# from matplotlib.patches import Polygon
#
# all_squircle = execution_data[-1].all_squircles
# for squircle_i in all_squircle:
#     center = squircle_i.center
#     width = squircle_i.width
#     height = squircle_i.height
#     polygon = [[center[0] - width / 2, center[1] - height / 2],
#                [center[0] - width / 2, center[1] + height / 2],
#                [center[0] + width / 2, center[1] + height / 2],
#                [center[0] + width / 2, center[1] - height / 2]]
#     poly = Polygon(polygon, edgecolor='red', alpha=0.7, fill=False, zorder=2)
#     ax4.add_patch(poly)
#     if squircle_i.ori_line is not None:
#         ax4.plot((squircle_i.ori_line[0][0], squircle_i.ori_line[1][0]),
#                  (squircle_i.ori_line[0][1], squircle_i.ori_line[1][1]), color='blue',
#                  linewidth=2.0, alpha=0.8, zorder=6)
# ax4.set_xlim([- 0.5, 7.5])
# ax4.set_ylim([- 0.5, 4.5])
# plt.show()d

for i in range(len(execution_data) - plot_step, len(execution_data)):
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()
    plot_four_ax(ax1, ax2, ax3, ax4, execution_data[i])
    file_path = './DATA/figure_data'
    current_date = datetime.datetime.now().strftime("snap_%Y_%m_%d")
    folder_path = os.path.join(file_path, current_date)

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    file_name = f"{i}.png"
    # print("path", os.path.join(folder_path, file_name))
    plt.savefig(os.path.join(folder_path, file_name), dpi=200)
    print("=============frame " + str(i) + " is saved==============")




import os
import imageio
from PIL import Image
import datetime

from moviepy.editor import ImageSequenceClip

# 设置生成的视频文件名和路径
snap_path = "DATA/figure_data/"
current_snap = datetime.datetime.now().strftime("snap_%Y_%m_%d")
current_snap = "snap_2024_04_06"
folder_snap_path = os.path.join(snap_path, current_snap)


name_path = "DATA/gif_data/"
current_gif = datetime.datetime.now().strftime("video_%Y_%m_%d")
folder_name_path = os.path.join(name_path, current_gif)


def make_video(image_folder, output_vid, fps=24):
    # 获取文件夹内所有图片的路径
    images = []
    files = os.listdir(image_folder)
    files.sort(key=lambda x: int(x[:-4]))
    for file_name in files:
        images.append(os.path.join(image_folder, file_name))
    # 创建视频剪辑
    clip = ImageSequenceClip(images, fps=fps)
    # 输出到文件
    clip.write_videofile(output_vid)


# 使用示例
make_video(folder_snap_path, folder_name_path + ".mp4", fps=12)
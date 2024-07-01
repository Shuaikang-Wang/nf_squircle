import os
import imageio
from PIL import Image
import datetime

from moviepy.editor import ImageSequenceClip

# 设置生成的视频文件名和路径
snap_path = "RESULT/results/"


name_path = "RESULT"
current_gif = "ani"
folder_name_path = os.path.join(name_path, current_gif)


def make_video(image_folder, output_vid, fps=12):
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
make_video(snap_path, folder_name_path + ".mp4", fps=4)

# # 读取所有 PNG 图片
# images = []
# folder = folder_snap_path
# files = os.listdir(folder)
# files.sort(key=lambda x: int(x[:-4]))
# for file_name in files:
#     if file_name.endswith('.png'):
#         print(file_name)
#         images.append(Image.open(file_name))
#
# # 将图片转换为视频
# fps = 5  # 每秒钟30帧
# with imageio.get_writer(folder_name_path, fps=fps) as video:
#     for image in images:
#         frame = image.convert('RGB')
#         video.append_data(frame)

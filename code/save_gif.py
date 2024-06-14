import os
import datetime
from os.path import join
import imageio.v2 as imageio


def generate_gif(name, folder_path):
    fig_files = []
    folder = folder_path
    files = os.listdir(folder)
    files.sort(key=lambda x: int(x[:-4]))
    for file_name in files:
        if file_name.endswith('.png'):
            fig_files.append(join(folder, file_name))
    # print("========== add figure files ==========")
    save_name = join('%s.gif' % name)
    with imageio.get_writer(save_name, mode='I', duration=0.2) as writer:
        for fig in fig_files:
            image = imageio.imread(fig)
            writer.append_data(image)


snap_path = "DATA/figure_data/"
current_snap = datetime.datetime.now().strftime("snap_%Y_%m_%d")
current_snap = "snap_2024_03_16"
folder_snap_path = os.path.join(snap_path, current_snap)


name_path = "DATA/gif_data/"
current_gif = datetime.datetime.now().strftime("gif_%Y_%m_%d")
current_gif = "gif_2024_03_18"
folder_name_path = os.path.join(name_path, current_gif)


generate_gif(folder_name_path, folder_snap_path)
print("==========gif has been saved ==========")


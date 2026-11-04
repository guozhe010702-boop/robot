import os
import os.path as osp
import shutil
import glob


def extract_image():
    input_folder = "orbbec"

    output_color_folder = "img/color_img"
    output_depth_folder = "img/deep_img"

    os.makedirs(output_color_folder, exist_ok=True)
    os.makedirs(output_depth_folder, exist_ok=True)

    for subfolder in os.listdir(input_folder):
        subfolder_path = osp.join(input_folder, subfolder)
        if osp.isdir(subfolder_path):
            # color_img
            color_images = glob.glob(osp.join(subfolder_path, '*_color.jpg'))
            for color_image in color_images:
                shutil.copy(color_image, output_color_folder)

            # depth_img
            depth_imgs = glob.glob(osp.join(subfolder_path, '*_depth.png'))
            for depth_img in depth_imgs:
                shutil.copy(depth_img, output_depth_folder)

if __name__ == '__main__':
    extract_image()
    
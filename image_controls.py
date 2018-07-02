import logging
logging.basicConfig(level=logging.INFO)

import os

def clearTestImages():

    depth_folder = './frames/depth/'
    color_folder = './frames/color/'
    gray_folder = './frames/gray/'

    dirs = [depth_folder, color_folder, gray_folder]
    for folder in dirs:
        for file in os.listdir(folder):
            file_path = os.path.join(folder, file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(e)



import cv2
import os


depth_folder = './frames/depth/'
color_folder = './frames/color/'
gray_folder = './frames/gray/'
cleaned_image_folder = './alan_test_images/'


def clearImagesWithPrompt():
    print('Do you want to clear alan_test_images directory? (y/n)')
    if raw_input() == 'y':
        for file in os.listdir(cleaned_image_folder):
            file_path = os.path.join(cleaned_image_folder, file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(e)

def clearImages():
    for file in os.listdir(cleaned_image_folder):
        file_path = os.path.join(cleaned_image_folder, file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)


def depthImageSegmentation():
    print('Processing images...')
    for file_name in os.listdir(color_folder):
        imageRGB = cv2.imread(color_folder + file_name, 1)
        imageDepth = cv2.imread(depth_folder + file_name, 2)

        for i in range(len(imageRGB)):
            for j in range(len(imageRGB[0])):
                if imageRGB[i][j][0] < 50 and imageRGB[i][j][1] < 50 and imageRGB[i][j][2] < 50:
                    imageRGB[i][j] = [0, 0, 0]
                    imageDepth[i][j] = 0

        cv2.imwrite(cleaned_image_folder + 'color_' + file_name, imageRGB)
        cv2.imwrite(cleaned_image_folder + 'depth_' + file_name, imageDepth)


    print('All images processed')

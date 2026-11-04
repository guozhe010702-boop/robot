import os
import glob
import json
import yaml
import cv2
import numpy as np
from constants import SIZE


def undistort():
    # 未矫正图片列表
    imgList = os.listdir('./img/color_img/')
    imgList.sort(key=lambda x: int(x.split('.')[0]))
    color_imgs = ['img/color_img/' + item for item in imgList]
    # 矫正后的图片保存路径
    undistorted_img_save_path = 'img/undistorted_img'
    os.makedirs(undistorted_img_save_path, exist_ok=True)

    # 读取原始内参矩阵和畸变系数
    with open('result/rectified.json', 'r') as f:
        data = json.load(f)

    cameraMatrix = np.array(data['rectified']['K'])
    distCoeffs = np.array(data['rectified']['D'])

    # 读取kinect相机原始内参矩阵和畸变系数
    # intrinsics_path='calib_k4a_000003404412_1024.yaml'
    # with open(intrinsics_path, 'r') as f:
    #     data = yaml.safe_load(f)

    # cameraMatrix = np.array(data['color_camera']['intrinsics']['K'])
    # distCoeffs = np.array(data['color_camera']['intrinsics']['D'])

    print(f'\n cameraMatrix:\n{cameraMatrix}\n')
    print(f'distCoeffs:\n{distCoeffs}\n')

    # 新的内参矩阵
    newCameraMatrix, validPixROI = cv2.getOptimalNewCameraMatrix(
        cameraMatrix, distCoeffs, SIZE, 0, SIZE)
    print(f'newCameraMatrix:\n{newCameraMatrix}\n')

    out = {
        'undistorted': {
            'newCameraMatrix': newCameraMatrix.tolist(),
            'validPixROI': validPixROI
        }
    }

    undistorted_path = 'result'
    os.makedirs(undistorted_path, exist_ok=True)
    json.dump(out, open(os.path.join(undistorted_path,
                                     'undistorted.json'), 'w'), indent=2)

    # 畸变矫正映射表
    mapx, mapy = cv2.initUndistortRectifyMap(
        cameraMatrix, distCoeffs, None, newCameraMatrix, SIZE, cv2.CV_32FC1)

    # 矫正畸变并保存
    for i, img_file in enumerate(color_imgs):
        img = cv2.imread(img_file)
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        cv2.imwrite(os.path.join(undistorted_img_save_path,
                    os.path.basename(img_file)), undistorted_img)
        print(
            f'id={i+1} ,{os.path.basename(img_file)} distortion correction completed')


if __name__=="__main__":
    undistort()
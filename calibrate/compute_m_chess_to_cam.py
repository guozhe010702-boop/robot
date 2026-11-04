import os
import glob
import json
import cv2
import numpy as np
from utils import find_chessboard_corners
from constants import CB_W, CB_H, SQUARE_SIZE, NUM
from scipy.spatial.transform import Rotation as R

def compute_m_chess_to_cam():
    # img_files = sorted(glob.glob('img/undistorted_img/*.png'))
    imgList = os.listdir('./img/undistorted_img/')
    imgList.sort(key=lambda x: int(x.split('.')[0]))
    img_files = ['img/undistorted_img/' + item for item in imgList]
    save_path = 'img/undistorted_img_corners'
    os.makedirs(save_path, exist_ok=True)

    objectPoints, imagePoints = find_chessboard_corners(
        img_files, save_path, CB_W, CB_H, SQUARE_SIZE)

    objectPoints = np.array(objectPoints, dtype=np.float64)
    imagePoints = np.array(imagePoints, dtype=np.float64)
    # imagePoints = imagePoints.squeeze(2)
    print(f'objectPoints_shape:{objectPoints.shape}')
    print(f'imagePoints_shape:{imagePoints.shape}')
    print(f'objectPoints:\n{objectPoints}\n')
    print(f'imagePoints:\n{imagePoints}\n')
    # imagePoints = imagePoints.squeeze(2)

    # 此处内参已完成畸变矫正
    # 内参矩阵(畸变矫正后的)
    with open('result/undistorted.json') as f:
        data = json.load(f)
    newCameraMatrix=data['undistorted']['newCameraMatrix']
    # with open('result/rectified.json') as f:
    #     data = json.load(f)
    # newCameraMatrix = data['rectified']['K']
    # newCameraMatrix = np.array([[998.7793, 0, 483.6675], [0, 1106.98, 320.9552], [0, 0, 1]])
    # newCameraMatrix[0,:]=newCameraMatrix[0,:]*(1920/960)
    # newCameraMatrix[1,:]=newCameraMatrix[1,:]*(1080/600)
    print(f'newCameraMatrix:\n{newCameraMatrix}\n')

    # 畸变系数(畸变矫正后distCoeffs为零)
    distCoeffs = np.zeros((5, 1))
    print(f'distCoeffs:\n{distCoeffs}\n')

    # 估计相机的外部参数
    M_chess_to_cam_all = []
    for i in range(NUM):
        success, rvec, tvec = cv2.solvePnP(
            objectPoints[i], imagePoints[i], np.float64(newCameraMatrix), np.float64(distCoeffs))
        if success:
            # print(rvec)
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            # print(rotation_matrix)
            RT = np.hstack((rotation_matrix, tvec))
            M_chess_to_cam = np.vstack((RT, [0, 0, 0, 1])).reshape(4, 4)
            np.set_printoptions(suppress=True)
            print(f'M_chess_to_cam{i+1}:\n{M_chess_to_cam}')
            M_chess_to_cam_all.append(M_chess_to_cam)

    M_chess_to_cam_all = np.array(M_chess_to_cam_all)
    np.save('result/M_chess_to_cam_all.npy', M_chess_to_cam_all)
    return M_chess_to_cam_all
if __name__=="__main__":
    m=compute_m_chess_to_cam()

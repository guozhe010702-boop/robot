import math
import cv2
import os
import numpy as np


def RPY2Rotation_matrix(x, y, z):
    """
    func:
        根据欧拉角计算旋转矩阵
    Args:
        x, y, z: 欧拉角
    return:
        R: 旋转矩阵
    """
    Rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)],
                   [0, math.sin(x), math.cos(x)]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [
        0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rz = np.array([[math.cos(z), -math.sin(z), 0],
                   [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R


def get_m_end_to_base(pos_ori):
    """
    func:
        计算机械臂末端到基座的变换矩阵
    Args:
        pos_ori=[Tx, Ty, Tz, rx, ry, rz] 
        unit: mm/rad
    return:
        M_end_to_base
    """
    # rx = math.radians(pos_ori[3])  # 将 pos_ori[3]（表示绕 x 轴的旋转角度，单位是度）转换为弧度
    # ry = math.radians(pos_ori[4])
    # rz = math.radians(pos_ori[5])
    R = RPY2Rotation_matrix(pos_ori[3], pos_ori[4], pos_ori[5])  # 根据绕 x, y, z 轴的旋转角度（即 rx, ry, rz）生成一个旋转矩阵 R
    t = np.array([[pos_ori[0]], [pos_ori[1]], [pos_ori[2]]])
    RT = np.column_stack([R, t])
    M_end_to_base = np.vstack((RT, np.array([0, 0, 0, 1])))
    return M_end_to_base
# [ R11, R12, R13, t1 ]
# [ R21, R22, R23, t2 ]
# [ R31, R32, R33, t3 ]
# [  0 ,  0 ,  0 ,  1 ]


def find_chessboard_corners(img_files, save_path, CB_W, CB_H, SQUARE_SIZE):
    """
    Args:
        img_files: 待检测图片列表
        save_path: 绘制角点后的图片保存路径
    return:
        objectPointsAll: all 3D points
        imagePointsAll: all 2D points
    """

    objectPointsAll = []
    imagePointsAll = []

    objectPoints = np.zeros((CB_W * CB_H, 3), dtype=np.float32)
    objectPoints[:, :2] = np.mgrid[0:CB_W, 0:CB_H].T.reshape(-1, 2) * SQUARE_SIZE

    for id, img_file in enumerate(img_files):
        print(f'detect img_{id + 1}: {img_file}')
        color_img = cv2.imread(img_file)
        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (CB_W, CB_H), None)
        if ret:
            objectPointsAll.append(objectPoints)
            criteria = (cv2.TERM_CRITERIA_MAX_ITER |
                        cv2.TERM_CRITERIA_EPS, 30, 0.001)
            corners2 = cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1), criteria)
            if [corners2]:
                imagePoint = corners2
            else:
                imagePoint = corners
            imagePointsAll.append(imagePoint)

        color_img_copy = color_img.copy()
        cv2.drawChessboardCorners(
            color_img_copy, (CB_W, CB_H), imagePoint, ret)

        img_save_path = os.path.join(save_path, os.path.basename(img_file))
        cv2.imwrite(img_save_path, color_img_copy)

    return objectPointsAll, imagePointsAll

import json
import glob
import os
import random
import statistics
import matplotlib
import matplotlib.pyplot as plt
import cv2
import numpy as np
import open3d as o3d
from utils import get_m_end_to_base
from constants import EYEINHAND_MODE, EYETOHAND_MODE, CB_W, SQUARE_SIZE, CB_H, NUM
from scipy.spatial.transform import Rotation as R

matplotlib.use('TkAgg')


# 定义一个函数，将矩阵转换为点云
def matrix_to_points(matrix):
    points = []
    rows, cols = matrix.shape
    for i in range(rows):
        for j in range(cols):
            # 使用矩阵的元素作为 z 坐标，行号作为 x 坐标，列号作为 y 坐标
            points.append([i, j, matrix[i, j]])
    return np.array(points)


def eyetohand_calibrate(calib_imgs, track_points):
    eye_hand_matrix_save_path = 'result/eye_hand_matrix.json'
    imgs = sorted(glob.glob(calib_imgs))

    R_all_base_to_end = []
    T_all_base_to_end = []

    R_all_chessboard_to_cam = []
    T_all_chessboard_to_cam = []

    assert len(track_points) == len(imgs)

    for id, point in enumerate(track_points):
        np.set_printoptions(suppress=True)
        print(f'point_{id}:\n{point}')
        print(f'img{id}:\n{imgs[id]}')

        M_chessboard_to_cam = np.load('result/M_chess_to_cam_all.npy')[id]

        if M_chessboard_to_cam is None:
            print("current point is not found chessboard corners")
            continue

        R_all_chessboard_to_cam.append(M_chessboard_to_cam[:3, :3])
        T_all_chessboard_to_cam.append(
            M_chessboard_to_cam[:3, 3].reshape((3, 1)))

        M_end_to_base = get_m_end_to_base(point)
        M_base_to_end = np.linalg.inv(M_end_to_base)
        R_all_base_to_end.append(M_base_to_end[:3, :3])
        T_all_base_to_end.append(M_base_to_end[:3, 3].reshape((3, 1)))

    # 手眼标定变换矩阵cam_to_base
    R, T = cv2.calibrateHandEye(R_all_base_to_end, T_all_base_to_end, R_all_chessboard_to_cam,
                                T_all_chessboard_to_cam, method=cv2.CALIB_HAND_EYE_TSAI)
    RT = np.column_stack((R, T))
    M_cam_to_base = np.vstack((RT, np.array([0, 0, 0, 1])))

    out = {
        'eye_hand_matrix': {
            'K': M_cam_to_base.tolist()
        }
    }
    json.dump(out, open(eye_hand_matrix_save_path, 'w'), indent=2)

    # 结果验证，原则上来说，每次结果相差较小
    for i in range(len(track_points)):
        RT_base_to_end = np.column_stack(
            (R_all_base_to_end[i], T_all_base_to_end[i]))
        RT_base_to_end = np.vstack((RT_base_to_end, np.array([0, 0, 0, 1])))

        RT_chess_to_cam = np.column_stack(
            (R_all_chessboard_to_cam[i], T_all_chessboard_to_cam[i]))
        RT_chess_to_cam = np.vstack(
            (RT_chess_to_cam, np.array([0, 0, 0, 1])))

        RT_cam_to_base = np.column_stack((R, T))
        RT_cam_to_base = np.vstack((RT_cam_to_base, np.array([0, 0, 0, 1])))

        RT_chessboard_to_end = RT_base_to_end @ RT_cam_to_base @ RT_chess_to_cam
        # RT_chess_to_end = np.linalg.inv(RT_chess_to_end)
        np.set_printoptions(suppress=True, linewidth=1000)
        print(f'第{i + 1}组：')
        print('RT_chessboard_to_end')
        print(RT_chessboard_to_end[:3, :])

    return M_cam_to_base


def eyeinhand_calibrate(calib_imgs, track_points):
    eye_hand_matrix_save_path = 'result/eye_hand_matrix.json'
    # imgs = sorted(glob.glob(calib_imgs))
    imgList = os.listdir(calib_imgs)
    imgList.sort(key=lambda x: int(x.split('.')[0]))
    imgs = ['img/undistorted_img/' + item for item in imgList]

    R_all_end_to_base = []
    T_all_end_to_base = []

    R_all_chessboard_to_cam = []
    T_all_chessboard_to_cam = []

    assert len(track_points) == len(imgs)

    for id, point in enumerate(track_points):
        np.set_printoptions(suppress=True)  # 确保输出时不显示小数的科学计数法，而是直接以常规小数形式显示
        print(f'point_{id}:\n{point}')
        print(f'img{id}:\n{imgs[id]}')

        M_chessboard_to_cam = np.load('result/M_chess_to_cam_all.npy')[id]

        if M_chessboard_to_cam is None:
            print("current point is not found chessboard corners")
            continue
        # 用来存储每个棋盘格相对于相机坐标系的旋转矩阵和位移向量
        R_all_chessboard_to_cam.append(M_chessboard_to_cam[:3, :3])
        T_all_chessboard_to_cam.append(
            M_chessboard_to_cam[:3, 3].reshape((3, 1)))
        # 获取当前点的机器人末端执行器到基坐标系的变换矩阵
        M_end_to_base = get_m_end_to_base(point)
        R_all_end_to_base.append(M_end_to_base[:3, :3])
        T_all_end_to_base.append(M_end_to_base[:3, 3].reshape((3, 1)))

    # 手眼标定变换矩阵cam_to_end,表示从相机坐标系到末端执行器坐标系的变换
    R, T = cv2.calibrateHandEye(R_all_end_to_base, T_all_end_to_base, R_all_chessboard_to_cam,
                                T_all_chessboard_to_cam, method=cv2.CALIB_HAND_EYE_TSAI)
    RT = np.column_stack((R, T))
    M_cam_to_end = np.vstack((RT, np.array([0, 0, 0, 1])))

    out = {
        'eye_hand_matrix': {
            'K': M_cam_to_end.tolist()
        }
    }
    json.dump(out, open(eye_hand_matrix_save_path, 'w'), indent=2)
    RT_val = []
    # 结果验证，原则上来说，每次结果相差较小
    for i in range(len(track_points)):
        # 末端执行器在基坐标系中的旋转矩阵和位移向量
        RT_end_to_base = np.column_stack(
            (R_all_end_to_base[i], T_all_end_to_base[i]))
        RT_end_to_base = np.vstack((RT_end_to_base, np.array([0, 0, 0, 1])))
        # 相机坐标系的旋转矩阵和位移向量组合成齐次变换矩阵
        RT_chess_to_cam = np.column_stack(
            (R_all_chessboard_to_cam[i], T_all_chessboard_to_cam[i]))
        RT_chess_to_cam = np.vstack(
            (RT_chess_to_cam, np.array([0, 0, 0, 1])))
        # 通过cv2.calibrateHandEye()函数得到的相机到末端执行器的旋转矩阵和平移向量
        RT_cam_to_end = np.column_stack((R, T))
        RT_cam_to_end = np.vstack((RT_cam_to_end, np.array([0, 0, 0, 1])))
        # @运算符是矩阵乘法的运算符
        # 即为固定的棋盘格相对于机器人基坐标系位姿
        RT_chessboard_to_base = RT_end_to_base @ RT_cam_to_end @ RT_chess_to_cam
        np.set_printoptions(suppress=True)
        print(f'第{i + 1}组：')
        print('RT_chessboard_to_base')
        print(RT_chessboard_to_base[:3, :])
        RT_val.append(RT_chessboard_to_base[:3, :])
    # # 计算均值
    # avg_matrix = np.mean(np.array(RT_val), axis=0)
    # print(avg_matrix)
    # # 计算均方误差
    # mse = 0
    # for i in range(len(RT_val)):
    #     mse = np.mean((np.array(RT_val[i]) - avg_matrix) ** 2) + mse
    # print(mse)
    np.save('result/RT_val.npy', RT_val)
    return M_cam_to_end, RT_val


def validate_RT(RT_val):
    # 计算均值
    # avg_matrix = np.mean(np.array(RT_val), axis=0)
    # # print(avg_matrix)
    # T_avg=avg_matrix[:,3]
    # R_avg=avg_matrix[:,:3]
    R_mat = []
    T_mat = []
    # print(T_mat)
    for RT in RT_val:
        R_mat.append(RT[:, :3])
        T_mat.append(RT[:, 3])
    # print(R_mat)
    # print(T_mat)
    # 从旋转矩阵创建一个 Rotation 对象
    rotation = R.from_matrix(R_mat)
    # 将旋转表示为旋转向量
    rotvec = rotation.as_rotvec()
    # print(rotvec)
    R_avg = np.mean(rotvec, axis=0)
    R_data = rotvec - R_avg
    # print(R_data)
    covariance_R = np.cov(R_data.T)
    print('旋转矩阵协方差')
    print(covariance_R)
    T_avg = np.mean(T_mat, axis=0)
    T_data = T_mat - T_avg
    # print(T_data)
    covariance_T = np.cov(T_data.T)
    print('平移矩阵协方差')
    print(covariance_T)
    # 计算均方误差
    # mse = 0
    # for i in range(len(RT_val)):
    #     mse = np.mean((np.array(RT_val[i]) - avg_matrix) ** 2) + mse
    # print(mse)


def point_clouds(RT_val):
    objectPoints = np.zeros((CB_W * CB_H, 3), dtype=np.float32)
    objectPoints[:, :2] = np.mgrid[0:CB_W, 0:CB_H].T.reshape(-1, 2) * SQUARE_SIZE
    print("objectPoints:",objectPoints)
    # 将点云数据转换为NumPy数组
    points = np.array(objectPoints)
    # 将棋盘格点云转换为齐次坐标
    points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
    # print(points_homogeneous)
    pcd_all = []
    # colors = [(random.random(), random.random(), random.random()) for _ in range(22)]
    colors = [[(i * 0.05 + j * 0.1) % 1.0 for j in range(3)] for i in range(NUM)]
    # pcd = o3d.geometry.PointCloud()
    transformedAlls = []
    for i in range(len(RT_val)):
        # 应用旋转平移矩阵
        transformed_points_homogeneous = np.dot(RT_val[i], points_homogeneous.T).T
        # print('transformed_points_homogeneous',transformed_points_homogeneous)
        # 将齐次坐标转换回3D坐标
        transformed_points = transformed_points_homogeneous[:, :3]
        transformedAlls.append(transformed_points)
        # print('乘旋转矩阵后')
        # print(transformed_points)
        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(transformed_points)
        pcd.paint_uniform_color(colors[i])
        pcd_all.append(pcd)

        # 绘制点云
    # o3d.visualization.draw_geometries(pcd_all,window_name="Multiple Point Clouds")
    # 创建Open3D点云对象
    # 创建坐标轴对象，size参数设置坐标轴的大小
    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd_all.append(pcd)
    # 绘制点云
    o3d.visualization.draw_geometries(pcd_all, window_name="Multiple Point Clouds", point_show_normal=True)
    # o3d.visualization.draw_geometries([pcd])
    print(pcd_all)
    # 合并多个点云
    merged_pcd = pcd_all[0]
    for item in pcd_all[1:]:
        merged_pcd += item
    # 保存点云到PCD文件
    o3d.io.write_point_cloud("test_pcd.pcd", merged_pcd)
    print("Saved point cloud to test_pcd.pcd.")
    return transformedAlls


def compute_distance(transformedAlls):
    disAll = []
    hist = []
    matrix = []
    for i in range(transformedAlls.shape[1]):
        extracted_elements = transformedAlls[:, i, :]
        matrix.append(extracted_elements)

    print(matrix)
    for point in matrix:
        distances = []
        # 计算每对点的距离
        for i in range(point.shape[0]):
            for j in range(i + 1, point.shape[0]):
                distance = np.linalg.norm(point[i] - point[j])  # 计算欧氏距离
                # print(point[i], point[j])
                distances.append(distance)
                hist.append(distance)
        average_distance = np.mean(distances)
        disAll.append(average_distance)
    print(np.mean(disAll))
    variance = statistics.variance(hist)
    plt.figure(figsize=(8, 6))
    plt.hist(hist, bins=30, edgecolor='black')
    plt.title('Variance:{:.3f} Mean:{:.3f}'.format(variance, np.mean(disAll)))
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.show()


def handeye_calibrate():
    path = 'img'
    # calib_imgs = os.path.join(path, "undistorted_img/*.png")
    calib_imgs = os.path.join(path, "undistorted_img/")
    track_points = np.load('result/point_robot.npy')
    if EYETOHAND_MODE:
        M_cam_to_base = eyetohand_calibrate(calib_imgs=calib_imgs,
                                            track_points=track_points)
        print(f'\n M_cam_to_base:\n {M_cam_to_base}')
    elif EYEINHAND_MODE:
        M_cam_to_end, RT_val = eyeinhand_calibrate(calib_imgs=calib_imgs,
                                                   track_points=track_points)
        print(f'\n M_cam_to_end:\n {M_cam_to_end}')
        validate_RT(RT_val)
        transformedAlls = point_clouds(RT_val)
        compute_distance(np.array(transformedAlls))


if __name__ == "__main__":
    handeye_calibrate()

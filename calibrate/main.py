from calibrate_camera import calibrate_camera
from undistort import undistort
from compute_m_chess_to_cam import compute_m_chess_to_cam
from extract_pose import extract_pose_main
from handeye_calibrate import handeye_calibrate

if __name__ == '__main__':
    calibrate_camera()  # 计算内参
    undistort()  # 去畸变
    compute_m_chess_to_cam()  # 计算标定板到相机的变换矩阵
    extract_pose_main()  # 提取位姿
    handeye_calibrate()  # 手眼标定，计算相机到机械臂末端的变换矩阵

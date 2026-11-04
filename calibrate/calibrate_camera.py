import os
import json
import cv2
from constants import CB_W, CB_H, SQUARE_SIZE, SIZE
from utils import find_chessboard_corners


def calibrate_camera():
    imgList = os.listdir('./img/color_img/')
    imgList.sort(key=lambda x: int(x.split('.')[0]))
    img_files = ['img/color_img/'+item for item in imgList]
    save_path = "img/color_img_corners"
    os.makedirs(save_path, exist_ok=True)

    objectPointsAll, imagePointsAll = find_chessboard_corners(
        img_files, save_path, CB_W, CB_H, SQUARE_SIZE)
    print(objectPointsAll)
    print(imagePointsAll)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objectPointsAll, imagePointsAll, SIZE, None, None)

    out = {
        'rectified': {
            'K': mtx.tolist(),
            'D': dist.tolist(),
        },
        'rms': ret,
    }

    rectified_save_path = 'result'
    os.makedirs(rectified_save_path, exist_ok=True)
    json.dump(out, open(os.path.join(
        rectified_save_path, 'rectified.json'), 'w'), indent=2)


if __name__ == "__main__":
    calibrate_camera()
    print("Calibrate camera done.")


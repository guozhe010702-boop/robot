import json
import os
import numpy as np
from constants import NUM


def extract_pose(save_path):
    point = []
    for i in range(1, NUM+1):
        file = os.path.join(save_path, f'result{i}.npy')
        coordinate = np.load(file)
        # coordinate[3:] = coordinate[3:]*180/np.pi
        # coordinate[3:] = coordinate[3:]
        np.set_printoptions(suppress=True)
        print(f'the coordinate of point{i}:\n{coordinate}')
        point.append(coordinate)

    point = np.array(point)
    np.save(f'result/{save_path.split("/")[0]}.npy', point)


def extract_pose_main():
    # pose_path_tool = 'point_3D/tool'
    # path_tool_flange = 'point_3D/flange'
    pose_path='point_robot/'
    extract_pose(pose_path)
    # extract_pose(path_tool_flange, 1)


if __name__ == '__main__':
    extract_pose_main()

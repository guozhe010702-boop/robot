import numpy as np


def transform_point(point, transform_matrix):
      """
      将点与变换矩阵相乘
      :param point: 一个点，表示为一个 NumPy 数组，例如 [x, y, z, 1]（齐次坐标）
      :param transform_matrix: 一个 4x4 的变换矩阵
      :return: 变换后的点
      """
      # 将点转换为齐次坐标
      homogeneous_point = np.array([point[0], point[1], point[2], 1])

      # 将点与变换矩阵相乘
      transformed_point = np.dot(transform_matrix, homogeneous_point)

      # 返回变换后的点（非齐次坐标）
      return transformed_point[:3]


def calculate_distance(point1, point2):
      """
      计算两个点之间的欧几里得距离
      :param point1: 第一个点的坐标，表示为一个 NumPy 数组 [x1, y1, z1]
      :param point2: 第二个点的坐标，表示为一个 NumPy 数组 [x2, y2, z2]
      :return: 两点之间的距离
      """
      # 计算两点之间的差值
      delta = point2 - point1

      # 计算欧几里得距离
      distance = np.linalg.norm(delta)

      return distance


def calculate_distance_manual(point1, point2):
      """
      手动计算两个点之间的欧几里得距离
      :param point1: 第一个点的坐标，表示为一个列表 [x1, y1, z1]
      :param point2: 第二个点的坐标，表示为一个列表 [x2, y2, z2]
      :return: 两点之间的距离
      """
      # 计算两点之间的差值
      delta_x = point2[0] - point1[0]
      delta_y = point2[1] - point1[1]
      delta_z = point2[2] - point1[2]

      # 计算欧几里得距离
      distance = (delta_x ** 2 + delta_y ** 2 + delta_z ** 2) ** 0.5

      return distance

mat = np.load('result/RT_val.npy')
print(mat.shape)
# 示例：定义一个点
point = np.array([150., 25., 0.])
truth = np.array([-66.29, -382.94, 968.26])
result = []
for i in range(mat.shape[0]):
    transformed_point = transform_point(point, mat[i])
    print("变换后的点:", transformed_point)
    sqrt_res = calculate_distance(transformed_point, truth)
    result.append(sqrt_res)
# print("原始点:", point)
# print("变换后的点:", transformed_point)
print(result)
print(sum(result)/len(result))
# print("<UNK>:", sqrt_res)
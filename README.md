# EC66 视觉机械臂标定与控制

本项目用于艾利特（ELITE）EC66 机械臂与奥比中光 Gemini 215 相机的视觉集成实验，包含：

- 相机内参、棋盘格位姿和手眼标定流程；
- 机械臂末端工具尖点的三点固定点 TCP 标定；
- 基于 8055/TCP JSON-RPC 的 EC66 Python SDK；
- 给定基座坐标系下 TCP 位姿的运动预检和控制示例。

## 目录

```text
calibrate/       相机标定、棋盘格位姿、手眼标定和采样数据
threePoint/      四姿态固定点 TCP 标定（前三姿态求解，第四姿态验证）
elite-ec-sdk/    可被其他 Python 程序导入的 EC Socket SDK
move/            EC66 指定位姿运动的安全预检和示例
.vscode/         项目编辑器配置
```

项目中保留了实验图像、位姿、标定矩阵、日志、测试数据以及已使用的 Python 环境文件，便于复现实验。运行代码时建议使用独立虚拟环境，不要直接修改已保存的结果文件。

## 环境与网络

- Python 3.10 或更高版本；
- EC66 控制器的 8055/TCP 服务；
- 本项目实验控制器地址为 `192.168.1.200`，端口为 `8055`。如果网络设置变化，应替换为现场实际地址；
- 运动程序要求机械臂处于 REMOTE 模式、非急停、已同步并已使能，且坐标模式和工具号与现场配置一致；
- 机械臂运动前必须确认工作空间、工具安装、负载、速度和急停条件。程序不会自动切换模式、使能伺服、清除报警或替代现场急停。

先验证网络和只读状态：

```powershell
python .\elite-ec-sdk\examples\read_only.py
```

## 安装依赖

安装可复用的 EC Socket SDK：

```powershell
python -m pip install -e .\elite-ec-sdk
```

安装三点 TCP 标定依赖：

```powershell
python -m pip install -r .\threePoint\requirements.txt
```

传统手眼标定脚本还使用 OpenCV、SciPy、Open3D 和 Matplotlib。请根据当前 Python 环境安装与版本兼容的依赖；`calibrate` 目录中的脚本依赖其相对数据目录结构运行。

## 三点 TCP 标定

三点标定用于求工具尖点相对法兰坐标系的偏移。让工具尖点四次接触同一个固定参考点：前三个姿态用于求解，第四个姿态作为独立验证，不参与拟合。前三个姿态应有足够的旋转变化；仅靠固定点不能确定工具轴方向。

在线采集（程序只读取反馈，不发送运动指令）：

```powershell
python .\threePoint\three_point.py capture `
  --host 192.168.1.200 --port 8055 --expected-version V3.14.2
```

按提示在示教器上调整姿态、接触固定点并停稳，然后按回车。默认每个姿态读取 5 次，并检查静止稳定性。

离线复算已有日志：

```powershell
python .\threePoint\three_point.py file .\threePoint\calibration.log
```

输出文件：

- `threePoint/output.json`：TCP 法兰偏移、固定点、残差和第四姿态验证结果；
- `threePoint/calibration.log`：带时间、运行编号、原始姿态和计算细节的追加日志。

结果只有在 `output.json` 的 `status` 为 `calibrated` 且第四姿态验证通过时，才应供下游程序使用。长度单位为 mm；角度和姿态约定必须与控制器反馈一致。

离线测试：

```powershell
python -B -m unittest discover -s .\threePoint\tests -v -b
```

## 相机标定与手眼标定

`calibrate` 目录保存基于棋盘格图像的实验流程，主要步骤为：

1. 使用棋盘格图像求相机内参和畸变参数；
2. 对图像去畸变并检测棋盘格角点；
3. 求每幅图像中棋盘格到相机的变换；
4. 读取同一编号对应的机械臂法兰位姿；
5. 使用 OpenCV 手眼标定求相机到末端（eye-in-hand）或相机到基座（eye-to-hand）变换；
6. 用各组数据重建棋盘格在基座坐标系中的位姿，检查结果一致性。

默认配置在 `calibrate/constants.py` 中：棋盘格内角点为 `11 x 8`，方格边长为 `25 mm`，样本数为 `20`，默认采用 eye-in-hand。图像编号、机器人位姿编号和矩阵数组下标必须一一对应。

主要结果位于 `calibrate/result/`，包括相机标定参数、棋盘格变换、手眼矩阵和验证数据。请先确认控制器版本、法兰/TCP 坐标系、单位以及相机安装方式，再解释矩阵方向；不能把法兰位姿、TCP 位姿和相机位姿混用。

## EC Socket SDK

SDK 是纯 Python 标准库实现的 8055/TCP JSON-RPC 客户端，导入不会自动连接或运动：

```python
from elite_ec_sdk import ECClient

with ECClient("192.168.1.200", port=8055, timeout=3) as robot:
    print(robot.get_soft_version())
    print(robot.get_robot_mode())       # 0 示教 / 1 自动 / 2 远程
    print(robot.get_robot_state())      # 0 停止 / 1 暂停 / 2 急停 / 3 运行 / 4 报警 / 5 碰撞
    print(robot.get_base_flange_pose(unit_type=1))
```

只读方法和控制方法由 SDK 分开保护。非白名单方法需要显式允许控制并按手册填写精确的方法名、参数名和单位。SDK 依据《EC_SDK_Socket 手册 Ver3.22.2》实现，但实机此前反馈版本为 `V3.14.2`，使用前必须先读取并核对实际固件版本。

SDK 测试使用模拟 Socket，不连接真实机械臂：

```powershell
python -m unittest discover -s .\elite-ec-sdk\tests -v
```

## 指定位姿运动示例

`move/move_to_pose.py` 接收基座坐标系下当前工具 TCP 的六维目标位姿：

```text
(x, y, z, rx, ry, rz)
```

其中位置单位为 mm，旋转单位为 rad。程序先读取状态、当前 TCP 位姿并进行距离/姿态变化限制，再调用逆运动学得到关节角，最后以 `moveByJoint` 发送关节目标。默认只做预检，不运动：

```powershell
python .\move\run_move_test.py --pose 100 300 500 3 1 3
```

只有在现场完成安全核对并显式提供执行参数后才会发送一次运动指令：

```powershell
python .\move\run_move_test.py `
  --host 192.168.1.200 --pose 100 300 500 3 1 3 `
  --expected-version V3.14.2 --expected-tool 0 `
  --confirm-base-tcp-rad --execute
```

示例数值仅用于说明命令格式，不代表安全或可达目标。运动前应以示教器低速确认路径、工具坐标、负载、限位和急停。程序超时后的普通 `stop` 不能替代现场急停，也不会自动重发运动。

## 常见问题

- **连接超时**：检查网线、电脑与控制器是否在同一网段，确认端口 `8055` 可达，再检查控制器版本和服务状态。
- **REMOTE/伺服检查失败**：程序不会自动切换模式或使能，请在示教器上完成后重新运行只读预检。
- **手眼结果不稳定**：检查图像与位姿编号是否对应、棋盘格是否完整可见、姿态变化是否充分、内参和单位是否正确。
- **TCP 验证失败**：检查四次是否接触同一个固定点、法兰位姿是否稳定、工具尖点是否松动，以及是否误把 TCP 位姿作为法兰位姿输入。
- **接口不支持**：以实机 `get_soft_version()` 返回值和控制器手册为准，不要直接把其他 EC/CS 系列接口当作 EC66 当前固件接口。

## 许可与资料

本项目代码为实验项目代码。EC66 通信接口以用户提供的《EC_SDK_Socket 手册 Ver3.22.2》和实际控制器反馈为准；Gemini 215 相机 SDK、驱动和 Viewer 请从奥比中光官方资料获取。

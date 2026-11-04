# EC Socket SDK（Python）

这个独立文件夹提供可供其他 Python 程序导入的艾利特 EC/EA 控制器 **8055/TCP JSON-RPC** 客户端。依据用户提供的《EC_SDK_Socket手册_Ver3.22.2》2.1、2.2 节实现通信协议，并为常用的状态、位姿和坐标运算接口提供便捷方法。纯 Python 标准库，导入模块不会连接或操作机械臂。

## 安装与调用

在本文件夹运行：

```powershell
python -m pip install -e .
```

然后从任意 Python 项目导入：

```python
from elite_ec_sdk import ECClient, RPCError

try:
    with ECClient("192.168.1.200", timeout=3) as robot:
        print("软件版本:", robot.get_soft_version())
        print("模式:", robot.get_robot_mode())       # 0 示教 / 1 自动 / 2 远程
        print("状态:", robot.get_robot_state())     # 0 停止 / 1 暂停 / 2 急停 / 3 运行 / 4 报警 / 5 碰撞
        print("法兰基座位姿:", robot.get_base_flange_pose(unit_type=1))
except RPCError as exc:
    print(f"控制器不支持或拒绝该方法: {exc.code} {exc.message}")
```

也可运行 `examples/read_only.py`。该示例只读状态，不使能伺服、不清报警、不运动。这里的地址是本项目已知的控制器地址；若网络设置变化，应改为实际地址。

## API 范围

- `ECClient(host, port=8055, timeout=3)`；`connect()`、`close()`、上下文管理器；`call(精确方法名, 参数)` 是通用调用入口，按手册中的大小写和参数名传值，覆盖控制器 **8055 服务**支持的方法。
- 已核对且默认放行的只读方法列在 `READ_ONLY_METHODS`；其中 `get_robot_mode()`、`get_robot_state()`、`get_joint_pos()`、`get_tcp_pose()`、`get_base_flange_pose()`、`pose_mul()`、`pose_inv()` 等有参数或返回值校验。其他只读方法可直接 `call()`，但未逐项封装。
- 非白名单方法一律视为可能改变机械臂状态，必须同时设置构造参数 `allow_control=True` 和单次调用参数 `control=True`。这**只是软件防误触护栏**，并不验证安全区域、运动轨迹、远程模式、伺服状态或人员安全。调用前必须逐项核对手册、当前固件版本和现场安全条件。本包不自动重试指令，也不把 RPC 成功当成运动完成。
- 8056 监控流和 8058/8059 日志服务**未在此版本实现**；这不是对整个 PDF 所有协议的完整实现。

例如，手册 2.2.4 的精确方法名可以通过 `robot.call("poseMul", {"pose1": p1, "pose2": p2, "unit_type": 1})` 调用。不同方法的参数必须按原手册填写，客户端不会猜测或自动转换单位。

## 标定项目注意

- 手册适用于控制器软件 **Ver3.22.2**；本项目实机此前显示 **V3.14.2**。连接后先用 `get_soft_version()` 核对；如返回 JSON-RPC `-32601` 等不支持错误，应按实机版本调整，不能直接假定新接口可用。
- `get_base_flange_pose()` 返回基座坐标系下的**法兰**位姿，前三项为 mm；`unit_type=1` 时后三项为 rad。`get_tcp_pose()` 返回工具/TCP 位姿，会受工具号和坐标号影响，不能与法兰位姿混用。
- `get_joint_pos()` 对应手册 2.2.2.3，但该条未明确返回角度单位。本包保留原值，不擅自换算；用于运动学或标定前应在实机上与示教器读数核对。
- 手眼标定采样应保存机械臂位姿、相机图像及时间/序号对应关系；TCP、法兰和相机坐标变换要明确命名。网络读取成功不意味着这些坐标系已标定。

## 测试

在本文件夹运行 `python -m unittest discover -s tests -v`。测试使用模拟 Socket，不连接真实机械臂。

来源：用户提供的《EC_SDK_Socket手册_Ver3.22.2.pdf》。

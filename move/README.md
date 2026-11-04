# EC66 指定位姿运动试验

`move_to_pose.py` 将基座坐标系下当前工具 TCP 的六维目标位姿 `(x, y, z, rx, ry, rz)` 送给 EC 控制器。`x/y/z` 单位是 **mm**，`rx/ry/rz` 单位是 **rad**。用户已确认这一目标定义。该程序使用相邻的 `elite-ec-sdk` 包，无需另装第三方依赖。

实机软件为 V3.14.2，已反馈 `moveByJoint` 拒绝 `target_pose`（JSON-RPC -32603）。因此本程序不再将 TCP 位姿直接发送给运动接口，而是先调用 `inverseKinematic(targetPose=目标TCP, referencePos=当前关节, unit_type=1)`，再以 `moveByJoint(targetPos=逆解关节角)` 发送关节运动。**不能仅将 target_pose 改名为 targetPos 并保留 XYZ/RX/RY/RZ 数值**；后者必须是真实关节角，单位为度。

接口依据：《EC_SDK_Socket手册_Ver3.22.2.pdf》2.2.3.1（印刷页 68–69 / PDF 80–81）的 `moveByJoint.targetPos`；2.2.4.1–2（印刷页 88–89 / PDF 100–101）的逆解与正解。新版手册虽列出 `target_pose`，实机反馈证明该参数不能直接用于当前控制器。程序通过当前关节正解与当前 TCP 比对核实单位/工具坐标一致性，并对目标逆解做正解复核；接口不支持或复核失败时终止。运动路径为关节路径，末端不保证走直线。

## 使用顺序

新增手动测试入口 `run_move_test.py`，默认 IP 为 `192.168.1.200`、8055/TCP；导入不联网，直接运行只查询当前状态/位姿：

```powershell
python .\test\run_move_test.py
```

它会调用 `move_to_pose.py`，保留其全部参数检查与执行确认。添加 `--pose x y z rx ry rz` 做只读预检，添加 `--offline` 仅检查参数。实际运动还需 `--expected-version V3.14.2 --expected-tool 0 --confirm-base-tcp-rad --execute`，并按原程序提示确认；目标须填写现场核对的基座 TCP 位姿。可用 `--host`、`--port` 覆盖测试入口默认连接参数。运行 `--help` 查看完整参数。

在 `C:\Users\Administrator\Desktop\Robot` 下用 PowerShell 运行，以下 IP 是你此前给出的实机地址，使用前核对：

```powershell
python .\test\move_to_pose.py --host 192.168.1.200 --show-current
```

上面只读状态和当前基座下 TCP 位姿。按显示的当前位姿拟定目标，先离线验证参数（以下六个数只是格式演示，**不是推荐实机目标**）：

`--show-current` 仅显示反馈，不要求伺服同步/使能、REMOTE 或示教器选中基座坐标模式；返回的 TCP 位姿仍明确指定基座坐标。`伺服=False` 表示尚未使能，`同步=True` 表示同步已完成，两者是独立状态。坐标模式 `4` 表示圆柱模式。目标预检与实际运动继续检查同步、使能、REMOTE 和基座模式，不会自动使能或切换坐标。

```powershell
python .\test\move_to_pose.py --host 192.168.1.200 --pose 100 200 300 0 0 0 --offline
```

去掉 `--offline` 后，程序会连接机械臂做只读预检、求解目标关节角并打印拟发送的 `targetPos`，**仍不会运动**。只有同时给出 `--execute`、实机查询版本、预期工具号、位姿约定确认，并在终端再次输入 `MOVE`，才发送**一次** `moveByJoint`。确认后会重查当前状态并复核关节解；发生变化则取消。示例中的目标数值必须替换为经核对且可达、路径无障碍的实际数值：

```powershell
python .\test\move_to_pose.py --host 192.168.1.200 --pose 100 200 300 0 0 0 --expected-version V3.14.2 --expected-tool 0 --confirm-base-tcp-rad --execute
```

当前文件配置的目标平移上限为 500 mm、姿态各分量变化上限为 2 rad（保留本地已有设置）；`speed=5`，`acc=dec=10`。这些只是软件门槛，不代表轨迹安全。可以通过 `--max-step-mm`、`--max-angle-rad`、`--speed`、`--acc`、`--dec` 显式调整，但应先用示教器低速确认实际空间、关节路径、工具安装、负载和急停。**禁止把法兰位姿或未配置到当前 TCP 的枪尖位姿直接当成 TCP 目标输入。**

程序要求 REMOTE、停止、非急停、伺服已同步/使能、当前坐标模式为基座（1），并核对远程模式工具号；不会自动切模式、同步、使能或清报警。`moveByJoint` 返回 `true` 后仍有界轮询机器人状态和基座 TCP 位姿，只有停止且进入容差才报告到位。超时时会在现有连接上尽力发送普通 `stop`，但**普通 stop、程序退出或网络断开均不能代替现场急停**，也不能保证已停机。指令超时后不会自动重发。

## 测试

在 `test` 目录运行：

```powershell
python -B -m unittest discover -s tests -v
```

测试使用模拟控制器，不连接真实机械臂；实机功能、目标可达性和路径安全须现场验证。

兼容性回归模拟 V3.14.2 拒绝直接 TCP 运动参数，检查运动请求仅发送逆解关节 `targetPos`，并覆盖无效逆解、正解不一致时禁止运动及 RPC 失败不重试。已完成离线验证，未在本次修复中运行实机运动。

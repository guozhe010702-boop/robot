# 使用当前 SDK 采集工具位姿

实际目录为 `C:\Users\Administrator\Desktop\Robot\calibrate\pose`。

在 Robot 目录运行（核对 IP）：

```powershell
python .\calibrate\pose\robot_control.py --host 192.168.1.200
```

每运行一次，采集一次并追加一行到本目录 `output.txt`。从任意工作目录启动均默认读取 `Robot/threePoint/output.json`、保存到脚本旁的 `output.txt`；也可用 `--calibration`、`--output` 显式指定路径。可增加 `--expected-version V3.14.2` 核对控制器版本。

运行前须完成工具标定和第四姿态验证，使 `threePoint/output.json` 的 `status=calibrated`，并含有有限的 `tcp_offset_flange_mm`。pending 或 validation_failed 文件无法用于换算，程序会在连接前报错。原始位姿和标定详情保存在 `threePoint/calibration.log`。

程序使用相邻 `elite-ec-sdk` 的 `ECClient`，通过 8055 读取版本、停止状态、**当前实际法兰位姿**，再调用只读坐标运算 `poseMul`：

`T_base_tool = T_base_flange × Translation(tcp_offset_flange_mm)`

等价位置公式为 `p_base_tool = R_base_flange × tcp_offset_flange_mm + p_base_flange`。不能将 XYZ 偏移直接加到基座 XYZ，也不能读取已包含 TCP 的位姿后再重复补偿。所有长度为 mm，不再使用旧代码的米到毫米乘 1000。

`output.txt` 每行仍为 `x y z rx ry rz`，前三项为工具尖点在基座中的 XYZ（mm），后三项为其姿态（rad），由控制器运算返回。三姿态标定只确定 TCP 位置，故这里**明确将工具坐标轴定义为平行法兰坐标轴**，姿态与法兰相同；此约定不表示工具方向已标定。保存的是工具相对基座的位姿，尖点在自身工具系中的坐标始终是零。

采集前后都要求状态为停止；无运动、使能或工具参数写入。程序保留原有 output.txt 历史行，不会重写历史数据的单位或来源；本次代码修改也未向其中追加实机样本。

将 TXT 转为 NumPy 文件：

```powershell
python .\calibrate\pose\save.py
```

`save.py` 使用脚本目录下的 `output.txt` 和 `result`，先校验所有行均为六个有限数值，再依次保存为 `result1.npy` 等文件。导入脚本不会执行采集或转换。

接口依据：EC SDK Socket Ver3.22.2，2.2.2.38（印刷页 30 / PDF 42，`get_base_flange_pose`，注明 v2.14.4+），2.2.4.5（印刷页 90–91 / PDF 102–103，`poseMul`，unit_type=1 为弧度）。本地此前记录实机 V3.14.2，运行时重新查询；不支持接口时保留错误并结束，不猜测控制器欧拉角顺序。下游 `calibrate/utils.py` 的欧拉角解释仍须与实际控制器约定一致。

离线测试（模拟 SDK，不连接机械臂）：

```powershell
python -B -m unittest discover -s .\calibrate\pose\tests -v
```

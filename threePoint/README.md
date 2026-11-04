# 工具 TCP 标定：前三姿态求解，第四姿态验证

用四组基座坐标系下的法兰位姿完成标定：前三组求 TCP 相对法兰的 XYZ 偏移（mm），第四组独立验证，不参与拟合。四次均使同一工具尖点接触同一固定点；前三姿态绕不同轴充分变化，第四次选不同姿态。工具轴方向不能由定点法确定。

## 运行

在 `C:\Users\Administrator\Desktop\Robot` 运行：

```powershell
python .\threePoint\three_point.py capture --host 192.168.1.200 --port 8055 --expected-version V3.14.2
```

按提示用示教器调整四个姿态，每次接触参考点并停稳后按回车。程序只读取反馈及做坐标运算，不发送运动指令。每姿态默认读取 5 次，位置最大点间距不超过 0.2 mm、旋转变化不超过 0.1° 才接受。

依赖 Python 3.10+ 和 NumPy：

```powershell
python -m pip install -r .\threePoint\requirements.txt
```

## 文件保存

**`output.json` 只保存可供下游读取的标定结果**：

```json
{
  "schema_version": 3,
  "status": "calibrated",
  "calibration_type": "three_pose_fixed_point_tcp",
  "length_unit": "mm",
  "input_frame": "robot_base_flange",
  "tcp_offset_flange_mm": [10.0, 20.0, 100.0],
  "tool_orientation": null
}
```

以上偏移仅为格式示例。`status=pending` 表示未标定；第四姿态验证通过为 `calibrated`，失败为 `validation_failed`。下游仅接受 calibrated。每次完整计算原子替换结果；输入错误、前三姿态求解失败或中途取消保留旧结果。

**`calibration.log` 保存其余全部过程数据**，每行一个 UTF-8 JSON 对象，包含时间、run_id、事件及数据，按运行追加：

- `run_started`：参数。
- `controller_version`：实机版本。
- `captured_poses`：阶段采样快照，包含原来的 captured_poses.json 数据、原始六维位姿、采样矩阵、稳定性、用途和采样完整性。
- `manual_pose`：手动输入位姿及转换结果。
- `calibration_details`：完整计算详情，包括固定点、残差、奇异值、条件数及第四姿态验证误差。
- `result_saved`、`run_finished` 或 `run_failed`：保存结果或失败原因。
- `legacy_archive`：改版前文件的完整归档。

不再单独生成 `captured_poses.json`，也不再生成旧拼写 `outout.json`。已有文件迁移时先完整写入日志再移除旧文件，标定结果保留为 output.json。采集程序 `calibrate/pose/robot_control.py` 已同步读取新路径。

可使用 `--log-file` 自定义日志位置，结果文件仍固定在本脚本目录。

## 离线输入及复算

日志最后一次运行包含完整四姿态记录时，可以从日志复算，不连接机械臂：

```powershell
python .\threePoint\three_point.py file .\threePoint\calibration.log
```

即使输入与默认输出日志相同，也会先读取之前运行的记录，再追加本次计算详情。若最后一次运行只有部分采样则报错，不会悄悄采用更早数据；可将目标完整运行的日志行复制到独立 `.log` 文件再指定它。

旧 JSON 输入仍可读取：须声明 `length_unit=mm`、`input_frame=robot_base_flange`，`T_base_flange` 按顺序包含四个 4×4 法兰到基座变换。手动输入四组六维法兰位姿：

```powershell
python .\threePoint\three_point.py manual --angle-unit deg --rotation-convention rpy-xyz
```

仅在已确认输入姿态采用 `R=Rz(rz) Ry(ry) Rx(rx)` 时使用此模式；输入为 mm 和指定的 deg/rad，不能将 TCP 位姿当成法兰位姿。

## 求解及验证

前三姿态满足 `Ri @ t + pi = c`。去均值后用 SVD 检查可观测性，最小二乘求解 TCP 法兰坐标 t 和固定点基座坐标 c。第四次误差 `e4=R4 @ t+p4-c`，验证距离为其模长；结果及判定保存到日志。第四次数据不参与重新求解。

当前 CLI 保留本地设置，第四姿态最大误差默认 10 mm，可用 `--max-validation-error-mm` 明确调整，例如 0.5 mm：

```powershell
python .\threePoint\three_point.py capture --host 192.168.1.200 --max-validation-error-mm 0.5
```

前三组最大拟合残差默认 1 mm、最小奇异值 0.05、条件数上限 100；分别用 `--max-residual-mm`、`--min-singular-value`、`--max-condition` 调整。阈值只是验收门槛，不代表工作空间的定位精度。验证失败也保存带无效状态的简要结果和详细日志，退出码为 1。

## 控制器兼容性与测试

V3.14.2 已反馈不支持 pose_to_matrix。默认 `--rotation-source controller` 用 poseMul 将三个局部基向量变换后构造旋转矩阵，不猜测欧拉角顺序；pose-mul 是同义选项。新固件可明确选择 matrix 调用原生矩阵接口；rpy-xyz 仅供已经核实该旋转约定的设备。

依据 EC SDK Socket Ver3.22.2：§2.2.2.38（印刷页30 / PDF42）为法兰反馈；§2.2.4.5（印刷页90–91 / PDF102–103）为 poseMul，unit_type=1 对应 mm/rad。poseMul 支持情况以实机返回为准，不支持时保留错误并结束。

```powershell
python -B -m unittest discover -s .\threePoint\tests -v -b
```

测试使用合成数据和模拟控制器，不连接实机。

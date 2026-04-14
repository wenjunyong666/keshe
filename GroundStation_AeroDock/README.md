# AeroDock 四轴无人机地面站

这是给当前 `REMOTE_F401_V3.2` 遥控器和 `DRONE_F401-V4.4_同学基础适配版` 飞控适配的新版地面站。

## 适配说明

- 通信逻辑沿用参考工程：支持串口 / USB CDC 和 USB HID 入口，默认使用串口 `115200`。
- 协议逻辑沿用 ANO V4.5：解析 `STATUS`、`SENSOR`、`POWER`、`PID1~PID6`、`ACK` 等帧。
- 参数逻辑沿用参考工程：支持读取 PID、写入 PID、从 Flash 重载 PID、保存 PID 到 Flash。
- 界面逻辑重新设计：左侧为深绿色链路控制坞，右侧为浅色飞行仪表舱和参数仓，和原版地面站拉开视觉差异。

## 运行方式

```powershell
cd D:\keshe2\GroundStation_AeroDock
pip install -r requirements.txt
python .\ground_station\main.py
```

## 验证方式

```powershell
python -m compileall D:\keshe2\GroundStation_AeroDock\ground_station
```

最近一次检查已通过 Python 语法编译。

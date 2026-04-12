# 小四轴遥控器与飞控适配工程

本仓库包含当前交付版本的两个 Keil 工程：

- `REMOTE_F401_V3.2`：遥控器工程。
- `DRONE_F401-V4.4_同学基础适配版`：基于同学原始飞控工程继续适配后的飞控工程。

## 当前功能重点

- 遥控器保留原工程主框架和同学的部分命名，同时重做了 OLED 中文界面、菜单、锁定/解锁、摇杆校准、微调和无线重置/配对入口。
- 遥控器休眠为软休眠：上锁且无操作达到设置时间后只关闭 OLED 和 LED，MCU、SysTick 和 NRF 继续运行，通信不中断。
- 飞控休眠为软休眠：收到遥控器“上锁且无操作”状态后开始计时，到时关闭电机输出和 LED 状态，但保留 NRF 通信。
- 遥控器和飞控的休眠时间都会写入 Flash；飞控侧会在启动时从 Flash 恢复休眠时间。
- 飞控的配对参数和休眠参数共用同一 Flash 扇区保存，写入时会一起保存，避免互相覆盖。

## 编译方式

遥控器：

```powershell
& 'D:\KEIL\KEILV5\UV4\UV4.exe' -b 'D:\keshe2\REMOTE_F401_V3.2\MDK-ARM\REMOTE_F401_V3.2.uvprojx' -j0
```

飞控：

```powershell
& 'D:\KEIL\KEILV5\UV4\UV4.exe' -b 'D:\keshe2\DRONE_F401-V4.4_同学基础适配版\MDK-ARM\DRONE_F401-V4.2.uvprojx' -j0
```

最近一次验证结果：两个工程均为 `0 Error(s), 0 Warning(s)`。

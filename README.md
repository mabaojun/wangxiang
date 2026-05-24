# 升降网箱模型控制系统

基于 Python + Tkinter 的升降网箱模型控制系统，通过 Modbus RTU 协议控制5个独立压载舱的姿态，实现网箱的升降与自动稳定。

## 硬件架构

- **控制端**: Windows PC + USB转RS485适配器
- **执行端**: 5个 HY-IOS N2S 两路IO控制器 + 2个薄膜电机（排水/注水）每个舱
- **感知端**: 3轴 IMU（俯仰/横滚/偏航）
- **通信**: Modbus RTU (9600, N, 8, 1) via RS485总线

## 软件模块

| 文件 | 说明 |
|------|------|
| [config.py](config.py) | 全局配置（串口参数/设备地址/PID参数） |
| [modbus_client.py](modbus_client.py) | Modbus RTU通信客户端 |
| [tank_controller.py](tank_controller.py) | 压载舱控制器 |
| [imu_reader.py](imu_reader.py) | IMU数据采集 |
| [attitude_controller.py](attitude_controller.py) | PID姿态控制器 |
| [keyboard_handler.py](keyboard_handler.py) | 键盘输入处理 |
| [cli.py](cli.py) | 命令行界面 |
| [gui.py](gui.py) | GUI界面（Tkinter） |
| [main.py](main.py) | CLI程序入口 |
| [gui_main.py](gui_main.py) | GUI程序入口 |

## 运行方式

### 依赖安装
```bash
pip install pyserial
```

### 命令行模式
```bash
python main.py [COM端口]
# 例如: python main.py COM5
```

### GUI模式
```bash
python gui_main.py [COM端口]
```

## 键盘控制

| 按键 | 功能 |
|------|------|
| W | 前倾（北降南升） |
| S | 后仰（南降北升） |
| A | 左倾（西降东升） |
| D | 右倾（东降西升） |
| Q | 全部排水（上浮） |
| Z | 全部注水（下潜） |
| 空格 | 紧急停止 |

## 系统架构图

```
┌────────────────────────────┐
│        Windows PC          │
│  ┌────────────────────────┐ │
│  │  CLI / GUI (Tkinter)   │ │
│  │  PID姿态控制算法       │ │
│  └──────────┬─────────────┘ │
│             │               │
│  ┌──────────▼─────────────┐ │
│  │   Modbus RTU Client    │ │
│  │   (pyserial + CRC16)   │ │
│  └──────────┬─────────────┘ │
│  USB转RS485 │               │
└─────────────┼───────────────┘
              │ RS485总线
  ┌───────────┼───────────┐
  │           │           │
 IO1         IO2        IMU
 中心        北/南      3轴
 舱          东西舱     姿态
```

## 许可证

MIT License
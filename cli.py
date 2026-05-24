import os
import time
import threading
import logging
from typing import Optional

from config import SystemConfig, TankId, PumpAction
from modbus_client import ModbusRTUClient
from tank_controller import TankController
from imu_reader import IMUReader
from attitude_controller import AttitudeController
from keyboard_handler import KeyboardHandler

logger = logging.getLogger(__name__)

BANNER = r"""
╔══════════════════════════════════════════════════╗
║         升降网箱模型控制系统 v1.0                  ║
║         Lifting Cage Control System               ║
╚══════════════════════════════════════════════════╝
"""

HELP_TEXT = """
╔══════════════════════════════════════════════════╗
║  键盘控制:                                        ║
║    W - 前倾(北降南升)    S - 后仰(南降北升)        ║
║    A - 左倾(西降东升)    D - 右倾(东降西升)        ║
║    Q - 全部排水(上浮)    Z - 全部注水(下潜)        ║
║    空格 - 紧急停止                                  ║
║                                                    ║
║  命令行指令:                                       ║
║    status    - 显示系统状态                         ║
║    imu       - 显示IMU数据                         ║
║    auto      - 切换自动姿态稳定                     ║
║    drain <舱> - 指定舱排水 (center/north/south/    ║
║                 east/west 或 all)                   ║
║    fill <舱>  - 指定舱注水                          ║
║    stop <舱>  - 指定舱停止                          ║
║    stopall   - 全部停止                             ║
║    pid <轴> <P> <I> <D> - 设置PID参数(轴=pitch/roll)║
║    target <轴> <角度>   - 设置目标角度              ║
║    help      - 显示帮助                             ║
║    quit      - 退出程序                             ║
╚══════════════════════════════════════════════════╝
"""

TANK_ALIAS = {
    "center": TankId.CENTER,
    "c": TankId.CENTER,
    "north": TankId.NORTH,
    "n": TankId.NORTH,
    "south": TankId.SOUTH,
    "s": TankId.SOUTH,
    "east": TankId.EAST,
    "e": TankId.EAST,
    "west": TankId.WEST,
    "w": TankId.WEST,
}


class CLI:
    def __init__(self, config: SystemConfig):
        self._config = config
        self._client = ModbusRTUClient(config.serial)
        self._tank = TankController(self._client, config)
        self._imu = IMUReader(self._client, config)
        self._attitude = AttitudeController(config.attitude, self._imu)
        self._keyboard = KeyboardHandler()
        self._running = False
        self._display_thread: Optional[threading.Thread] = None
        self._control_thread: Optional[threading.Thread] = None

    def start(self):
        print(BANNER)
        print("正在连接串口...")
        if not self._client.connect():
            print(f"错误: 无法连接串口 {self._config.serial.port}")
            print("请检查串口配置后重试。")
            return

        print(f"串口已连接: {self._config.serial.port} @ {self._config.serial.baudrate}")
        print(HELP_TEXT)

        self._imu.start_polling()
        self._keyboard.start(self._on_key)
        self._running = True

        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._display_thread.start()

        try:
            while self._running:
                try:
                    cmd = input("> ").strip()
                    if cmd:
                        self._process_command(cmd)
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break
        finally:
            self.shutdown()

    def shutdown(self):
        self._running = False
        self._keyboard.stop()
        self._imu.stop_polling()
        self._tank.stop_all()
        self._client.disconnect()
        print("\n系统已安全关闭。")

    def _on_key(self, key: str):
        if key in ("q", "z", "w", "a", "s", "d", " "):
            self._tank.apply_keyboard_action(key)
        elif key == "e":
            self._attitude.toggle()
            state = "启用" if self._attitude.enabled else "禁用"
            print(f"\n[自动姿态稳定: {state}]")

    def _control_loop(self):
        while self._running:
            if self._attitude.enabled:
                pitch_out, roll_out = self._attitude.compute()
                self._tank.apply_attitude_output(pitch_out, roll_out)
            time.sleep(self._config.attitude.control_interval)

    def _display_loop(self):
        while self._running:
            time.sleep(1.0)

    def _process_command(self, cmd: str):
        parts = cmd.lower().split()
        if not parts:
            return

        action = parts[0]

        if action == "quit" or action == "exit":
            self._running = False
        elif action == "help" or action == "h":
            print(HELP_TEXT)
        elif action == "status":
            self._print_status()
        elif action == "imu":
            print(self._imu.get_status_string())
        elif action == "auto":
            self._attitude.toggle()
            state = "启用" if self._attitude.enabled else "禁用"
            print(f"自动姿态稳定: {state}")
        elif action == "drain":
            self._cmd_pump(parts, PumpAction.DRAIN)
        elif action == "fill":
            self._cmd_pump(parts, PumpAction.FILL)
        elif action == "stop":
            if len(parts) > 1 and parts[1] != "all":
                tid = TANK_ALIAS.get(parts[1])
                if tid is not None:
                    self._tank.set_pump(tid, PumpAction.STOP)
                else:
                    print(f"未知舱位: {parts[1]}")
            else:
                self._tank.stop_all()
                print("所有泵已停止")
        elif action == "stopall":
            self._tank.stop_all()
            print("所有泵已停止")
        elif action == "pid":
            self._cmd_pid(parts)
        elif action == "target":
            self._cmd_target(parts)
        else:
            print(f"未知命令: {action}，输入 help 查看帮助")

    def _cmd_pump(self, parts: list, action: PumpAction):
        if len(parts) < 2:
            print("用法: drain/fill <舱位> (center/north/south/east/west/all)")
            return
        target = parts[1]
        if target == "all":
            if action == PumpAction.DRAIN:
                self._tank.all_drain()
                print("全部排水")
            else:
                self._tank.all_fill()
                print("全部注水")
        else:
            tid = TANK_ALIAS.get(target)
            if tid is not None:
                self._tank.set_pump(tid, action)
                from tank_controller import TANK_NAMES
                action_name = "排水" if action == PumpAction.DRAIN else "注水"
                print(f"{TANK_NAMES[tid]}舱{action_name}")
            else:
                print(f"未知舱位: {target}")

    def _cmd_pid(self, parts: list):
        if len(parts) < 5:
            print("用法: pid <pitch/roll> <Kp> <Ki> <Kd>")
            return
        axis = parts[1]
        try:
            kp = float(parts[2])
            ki = float(parts[3])
            kd = float(parts[4])
        except ValueError:
            print("错误: PID参数必须为数字")
            return

        if axis == "pitch":
            self._config.attitude.pitch_pid.kp = kp
            self._config.attitude.pitch_pid.ki = ki
            self._config.attitude.pitch_pid.kd = kd
            self._attitude._pitch_pid = type(self._attitude._pitch_pid)(
                self._config.attitude.pitch_pid
            )
            print(f"俯仰PID已更新: Kp={kp}, Ki={ki}, Kd={kd}")
        elif axis == "roll":
            self._config.attitude.roll_pid.kp = kp
            self._config.attitude.roll_pid.ki = ki
            self._config.attitude.roll_pid.kd = kd
            self._attitude._roll_pid = type(self._attitude._roll_pid)(
                self._config.attitude.roll_pid
            )
            print(f"横滚PID已更新: Kp={kp}, Ki={ki}, Kd={kd}")
        else:
            print("错误: 轴名必须为 pitch 或 roll")

    def _cmd_target(self, parts: list):
        if len(parts) < 3:
            print("用法: target <pitch/roll> <角度>")
            return
        axis = parts[1]
        try:
            angle = float(parts[2])
        except ValueError:
            print("错误: 角度必须为数字")
            return

        if axis == "pitch":
            self._config.attitude.target_pitch = angle
            print(f"俯仰目标角度: {angle}°")
        elif axis == "roll":
            self._config.attitude.target_roll = angle
            print(f"横滚目标角度: {angle}°")
        else:
            print("错误: 轴名必须为 pitch 或 roll")

    def _print_status(self):
        print("=" * 50)
        print("【系统状态】")
        print("-" * 50)
        print("压载舱:")
        print(self._tank.get_status_string())
        print("-" * 50)
        print("IMU姿态:")
        print(self._imu.get_status_string())
        print("-" * 50)
        print("姿态控制:")
        print(self._attitude.get_status_string())
        print("=" * 50)

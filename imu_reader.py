import logging
import threading
import time
from typing import Optional, Tuple

from config import SystemConfig, IMUConfig
from modbus_client import ModbusRTUClient

logger = logging.getLogger(__name__)


class IMUReader:
    def __init__(self, client: ModbusRTUClient, config: SystemConfig):
        self._client = client
        self._imu_config: IMUConfig = config.imu
        self._poll_interval = config.imu_poll_interval
        self._pitch: float = 0.0
        self._roll: float = 0.0
        self._yaw: float = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_update: float = 0.0
        self._error_count: int = 0

    @property
    def pitch(self) -> float:
        with self._lock:
            return self._pitch

    @property
    def roll(self) -> float:
        with self._lock:
            return self._roll

    @property
    def yaw(self) -> float:
        with self._lock:
            return self._yaw

    @property
    def last_update(self) -> float:
        return self._last_update

    @property
    def is_alive(self) -> bool:
        return self._running and self._thread is not None and self._thread.is_alive()

    def get_angles(self) -> Tuple[float, float, float]:
        with self._lock:
            return self._pitch, self._roll, self._yaw

    def read_once(self) -> Tuple[float, float, float]:
        cfg = self._imu_config
        regs = self._client.read_holding_registers(
            cfg.address,
            cfg.pitch_reg,
            count=3,
        )
        if len(regs) >= 3:
            raw_pitch = regs[0]
            raw_roll = regs[1]
            raw_yaw = regs[2]

            if raw_pitch > 32767:
                raw_pitch -= 65536
            if raw_roll > 32767:
                raw_roll -= 65536
            if raw_yaw > 32767:
                raw_yaw -= 65536

            pitch = raw_pitch * cfg.scale_factor
            roll = raw_roll * cfg.scale_factor
            yaw = raw_yaw * cfg.scale_factor

            with self._lock:
                self._pitch = pitch
                self._roll = roll
                self._yaw = yaw

            self._last_update = time.time()
            self._error_count = 0
            return pitch, roll, yaw
        else:
            self._error_count += 1
            if self._error_count > 10:
                logger.warning(f"IMU连续读取失败 {self._error_count} 次")
            with self._lock:
                return self._pitch, self._roll, self._yaw

    def start_polling(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()
        logger.info("IMU轮询已启动")

    def stop_polling(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("IMU轮询已停止")

    def _poll_loop(self):
        while self._running:
            try:
                self.read_once()
            except Exception as e:
                logger.error(f"IMU轮询异常: {e}")
            time.sleep(self._poll_interval)

    def get_status_string(self) -> str:
        p, r, y = self.get_angles()
        age = time.time() - self._last_update if self._last_update > 0 else -1
        return (
            f"  俯仰(Pitch): {p:+7.2f}°\n"
            f"  横滚(Roll):  {r:+7.2f}°\n"
            f"  偏航(Yaw):   {y:+7.2f}°\n"
            f"  数据年龄:    {age:.1f}s"
        )

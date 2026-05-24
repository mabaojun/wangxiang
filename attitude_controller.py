import logging
import time
from dataclasses import dataclass

from config import PIDParams, AttitudeConfig
from imu_reader import IMUReader

logger = logging.getLogger(__name__)


@dataclass
class PIDState:
    integral: float = 0.0
    prev_error: float = 0.0
    prev_time: float = 0.0
    output: float = 0.0


class PIDController:
    def __init__(self, params: PIDParams):
        self._params = params
        self._state = PIDState()

    def reset(self):
        self._state = PIDState()

    def update(self, error: float, dt: float = None) -> float:
        now = time.time()
        if dt is None:
            if self._state.prev_time > 0:
                dt = now - self._state.prev_time
            else:
                dt = 0.01
        self._state.prev_time = now

        if dt <= 0:
            dt = 0.001

        p = self._params.kp * error

        self._state.integral += error * dt
        if self._params.integral_limit > 0:
            self._state.integral = max(
                -self._params.integral_limit,
                min(self._params.integral_limit, self._state.integral),
            )
        i = self._params.ki * self._state.integral

        derivative = (error - self._state.prev_error) / dt
        d = self._params.kd * derivative
        self._state.prev_error = error

        output = p + i + d
        output = max(self._params.output_min, min(self._params.output_max, output))
        self._state.output = output
        return output

    @property
    def output(self) -> float:
        return self._state.output


class AttitudeController:
    def __init__(self, config: AttitudeConfig, imu: IMUReader):
        self._config = config
        self._imu = imu
        self._pitch_pid = PIDController(config.pitch_pid)
        self._roll_pid = PIDController(config.roll_pid)
        self._enabled = False
        self._last_control_time: float = 0.0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self):
        self._enabled = True
        self._pitch_pid.reset()
        self._roll_pid.reset()
        logger.info("姿态自动控制已启用")

    def disable(self):
        self._enabled = False
        logger.info("姿态自动控制已禁用")

    def toggle(self) -> bool:
        if self._enabled:
            self.disable()
        else:
            self.enable()
        return self._enabled

    def compute(self) -> tuple:
        if not self._enabled:
            return 0.0, 0.0

        now = time.time()
        if now - self._last_control_time < self._config.control_interval:
            return self._pitch_pid.output, self._roll_pid.output
        self._last_control_time = now

        pitch, roll, _ = self._imu.get_angles()

        pitch_error = self._config.target_pitch - pitch
        roll_error = self._config.target_roll - roll

        if (abs(pitch_error) < self._config.angle_threshold and
                abs(roll_error) < self._config.angle_threshold):
            return 0.0, 0.0

        pitch_output = self._pitch_pid.update(pitch_error)
        roll_output = self._roll_pid.update(roll_error)

        return pitch_output, roll_output

    def get_status_string(self) -> str:
        pitch, roll, _ = self._imu.get_angles()
        p_out = self._pitch_pid.output
        r_out = self._roll_pid.output
        return (
            f"  自动稳定: {'启用' if self._enabled else '禁用'}\n"
            f"  目标角度: Pitch={self._config.target_pitch:.1f}° Roll={self._config.target_roll:.1f}°\n"
            f"  当前误差: Pitch={pitch - self._config.target_pitch:+.2f}° Roll={roll - self._config.target_roll:+.2f}°\n"
            f"  PID输出:  Pitch={p_out:+.3f} Roll={r_out:+.3f}"
        )

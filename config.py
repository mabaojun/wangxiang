from dataclasses import dataclass, field
from enum import IntEnum


class TankId(IntEnum):
    CENTER = 0
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4


class PumpAction(IntEnum):
    STOP = 0
    DRAIN = 1
    FILL = 2


@dataclass
class SerialConfig:
    port: str = "COM3"
    baudrate: int = 9600
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 1
    timeout: float = 0.5


@dataclass
class DeviceConfig:
    address: int = 1
    drain_coil: int = 0x0000
    fill_coil: int = 0x0001


@dataclass
class IMUConfig:
    address: int = 6
    pitch_reg: int = 0x0000
    roll_reg: int = 0x0001
    yaw_reg: int = 0x0002
    data_format: str = "int16"
    scale_factor: float = 0.01


@dataclass
class PIDParams:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.5
    output_min: float = -1.0
    output_max: float = 1.0
    integral_limit: float = 0.5


@dataclass
class AttitudeConfig:
    pitch_pid: PIDParams = field(default_factory=lambda: PIDParams(kp=1.2, ki=0.05, kd=0.8))
    roll_pid: PIDParams = field(default_factory=lambda: PIDParams(kp=1.2, ki=0.05, kd=0.8))
    target_pitch: float = 0.0
    target_roll: float = 0.0
    angle_threshold: float = 1.0
    control_interval: float = 0.1


@dataclass
class SystemConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    tanks: dict = field(default_factory=lambda: {
        TankId.CENTER: DeviceConfig(address=1),
        TankId.NORTH: DeviceConfig(address=2),
        TankId.SOUTH: DeviceConfig(address=3),
        TankId.EAST: DeviceConfig(address=4),
        TankId.WEST: DeviceConfig(address=5),
    })
    imu: IMUConfig = field(default_factory=IMUConfig)
    attitude: AttitudeConfig = field(default_factory=AttitudeConfig)
    imu_poll_interval: float = 0.2
    command_retry_count: int = 3
    command_retry_delay: float = 0.1

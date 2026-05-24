import struct
import time
import threading
import logging
from typing import Optional

import serial

from config import SerialConfig

logger = logging.getLogger(__name__)

CRC16_TABLE = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
    return crc


def _build_frame(slave_addr: int, func_code: int, data: bytes) -> bytes:
    payload = bytes([slave_addr, func_code]) + data
    crc = crc16(payload)
    return payload + struct.pack("<H", crc)


def _parse_response(data: bytes, expected_func: int) -> bytes:
    if len(data) < 5:
        raise ModbusError(f"响应帧过短: {len(data)} 字节")
    crc_received = struct.unpack("<H", data[-2:])[0]
    crc_calc = crc16(data[:-2])
    if crc_received != crc_calc:
        raise ModbusError(f"CRC校验失败: 接收=0x{crc_received:04X}, 计算=0x{crc_calc:04X}")
    slave_addr = data[0]
    func_code = data[1]
    if func_code & 0x80:
        exception_code = data[2] if len(data) > 2 else 0
        raise ModbusError(f"从站{slave_addr}异常响应, 功能码=0x{func_code:02X}, 异常码={exception_code}")
    if func_code != expected_func:
        raise ModbusError(f"功能码不匹配: 期望=0x{expected_func:02X}, 实际=0x{func_code:02X}")
    return data[2:-2]


class ModbusError(Exception):
    pass


class ModbusRTUClient:
    def __init__(self, config: SerialConfig):
        self._config = config
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._connected = False
        self._log_callback = None

    def set_log_callback(self, callback):
        self._log_callback = callback

    def _emit_log(self, direction: str, message: str):
        if self._log_callback:
            try:
                self._log_callback(direction, message)
            except Exception:
                pass

    @property
    def connected(self) -> bool:
        return self._connected and self._serial is not None and self._serial.is_open

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._config.port,
                baudrate=self._config.baudrate,
                bytesize=self._config.bytesize,
                parity=self._config.parity,
                stopbits=self._config.stopbits,
                timeout=self._config.timeout,
            )
            self._connected = True
            logger.info(f"串口已连接: {self._config.port} @ {self._config.baudrate}")
            return True
        except serial.SerialException as e:
            logger.error(f"串口连接失败: {e}")
            self._connected = False
            return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._connected = False
            logger.info("串口已断开")

    def _send_and_receive(self, frame: bytes, expected_func: int, retry: int = 3) -> bytes:
        if not self._serial or not self._serial.is_open:
            raise ModbusError("串口未连接")
        with self._lock:
            for attempt in range(retry):
                try:
                    self._serial.reset_input_buffer()
                    self._serial.reset_output_buffer()
                    self._serial.write(frame)
                    send_hex = frame.hex(' ').upper()
                    logger.debug(f"发送: {send_hex}")
                    self._emit_log("TX", send_hex)
                    response = self._serial.read(256)
                    if not response:
                        raise ModbusError("从站无响应(超时)")
                    recv_hex = response.hex(' ').upper()
                    logger.debug(f"接收: {recv_hex}")
                    self._emit_log("RX", recv_hex)
                    return _parse_response(response, expected_func)
                except ModbusError as e:
                    self._emit_log("ERR", str(e))
                    if attempt < retry - 1:
                        logger.warning(f"通信重试 {attempt + 1}/{retry}: {e}")
                        time.sleep(0.05)
                    else:
                        raise

    def write_single_coil(self, slave_addr: int, coil_addr: int, value: bool, retry: int = 3) -> bool:
        func_code = 0x05
        coil_value = 0xFF00 if value else 0x0000
        data = struct.pack(">HH", coil_addr, coil_value)
        frame = _build_frame(slave_addr, func_code, data)
        try:
            self._send_and_receive(frame, func_code, retry)
            return True
        except ModbusError as e:
            logger.error(f"写线圈失败(从站={slave_addr}, 地址={coil_addr}): {e}")
            return False

    def write_multiple_coils(self, slave_addr: int, start_addr: int, values: list, retry: int = 3) -> bool:
        func_code = 0x0F
        num_coils = len(values)
        byte_count = (num_coils + 7) // 8
        coil_bytes = bytearray(byte_count)
        for i, v in enumerate(values):
            if v:
                coil_bytes[i // 8] |= 1 << (i % 8)
        data = struct.pack(">HHB", start_addr, num_coils, byte_count) + bytes(coil_bytes)
        frame = _build_frame(slave_addr, func_code, data)
        try:
            self._send_and_receive(frame, func_code, retry)
            return True
        except ModbusError as e:
            logger.error(f"写多线圈失败(从站={slave_addr}, 起始地址={start_addr}): {e}")
            return False

    def read_holding_registers(self, slave_addr: int, start_addr: int, count: int = 1, retry: int = 3) -> list:
        func_code = 0x03
        data = struct.pack(">HH", start_addr, count)
        frame = _build_frame(slave_addr, func_code, data)
        try:
            resp_data = self._send_and_receive(frame, func_code, retry)
            byte_count = resp_data[0]
            reg_values = []
            for i in range(count):
                offset = 1 + i * 2
                if offset + 1 < len(resp_data):
                    val = struct.unpack(">H", resp_data[offset:offset + 2])[0]
                    reg_values.append(val)
            return reg_values
        except ModbusError as e:
            logger.error(f"读保持寄存器失败(从站={slave_addr}, 起始地址={start_addr}): {e}")
            return []

    def read_input_registers(self, slave_addr: int, start_addr: int, count: int = 1, retry: int = 3) -> list:
        func_code = 0x04
        data = struct.pack(">HH", start_addr, count)
        frame = _build_frame(slave_addr, func_code, data)
        try:
            resp_data = self._send_and_receive(frame, func_code, retry)
            byte_count = resp_data[0]
            reg_values = []
            for i in range(count):
                offset = 1 + i * 2
                if offset + 1 < len(resp_data):
                    val = struct.unpack(">H", resp_data[offset:offset + 2])[0]
                    reg_values.append(val)
            return reg_values
        except ModbusError as e:
            logger.error(f"读输入寄存器失败(从站={slave_addr}, 起始地址={start_addr}): {e}")
            return []

    def write_single_register(self, slave_addr: int, reg_addr: int, value: int, retry: int = 3) -> bool:
        func_code = 0x06
        data = struct.pack(">HH", reg_addr, value)
        frame = _build_frame(slave_addr, func_code, data)
        try:
            self._send_and_receive(frame, func_code, retry)
            return True
        except ModbusError as e:
            logger.error(f"写保持寄存器失败(从站={slave_addr}, 地址={reg_addr}): {e}")
            return False

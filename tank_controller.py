import logging
from typing import Dict

from config import SystemConfig, TankId, PumpAction, DeviceConfig
from modbus_client import ModbusRTUClient

logger = logging.getLogger(__name__)

TANK_NAMES = {
    TankId.CENTER: "中心",
    TankId.NORTH: "北",
    TankId.SOUTH: "南",
    TankId.EAST: "东",
    TankId.WEST: "西",
}


class TankController:
    def __init__(self, client: ModbusRTUClient, config: SystemConfig):
        self._client = client
        self._config = config
        self._states: Dict[TankId, PumpAction] = {
            tid: PumpAction.STOP for tid in TankId
        }

    def get_state(self, tank_id: TankId) -> PumpAction:
        return self._states[tank_id]

    def get_all_states(self) -> Dict[TankId, PumpAction]:
        return dict(self._states)

    def set_pump(self, tank_id: TankId, action: PumpAction) -> bool:
        dev: DeviceConfig = self._config.tanks[tank_id]
        success = True

        if action == PumpAction.STOP:
            drain_ok = self._client.write_single_coil(dev.address, dev.drain_coil, False)
            fill_ok = self._client.write_single_coil(dev.address, dev.fill_coil, False)
            success = drain_ok and fill_ok
        elif action == PumpAction.DRAIN:
            fill_ok = self._client.write_single_coil(dev.address, dev.fill_coil, False)
            drain_ok = self._client.write_single_coil(dev.address, dev.drain_coil, True)
            success = fill_ok and drain_ok
        elif action == PumpAction.FILL:
            drain_ok = self._client.write_single_coil(dev.address, dev.drain_coil, False)
            fill_ok = self._client.write_single_coil(dev.address, dev.fill_coil, True)
            success = drain_ok and fill_ok

        if success:
            self._states[tank_id] = action
            logger.info(f"{TANK_NAMES[tank_id]}舱 → {action.name}")
        else:
            logger.error(f"{TANK_NAMES[tank_id]}舱控制失败")

        return success

    def stop_all(self) -> bool:
        all_ok = True
        for tid in TankId:
            if not self.set_pump(tid, PumpAction.STOP):
                all_ok = False
        return all_ok

    def all_drain(self) -> bool:
        all_ok = True
        for tid in TankId:
            if not self.set_pump(tid, PumpAction.DRAIN):
                all_ok = False
        return all_ok

    def all_fill(self) -> bool:
        all_ok = True
        for tid in TankId:
            if not self.set_pump(tid, PumpAction.FILL):
                all_ok = False
        return all_ok

    def apply_attitude_output(self, pitch_output: float, roll_output: float):
        if pitch_output > 0:
            self.set_pump(TankId.NORTH, PumpAction.FILL)
            self.set_pump(TankId.SOUTH, PumpAction.DRAIN)
        elif pitch_output < 0:
            self.set_pump(TankId.NORTH, PumpAction.DRAIN)
            self.set_pump(TankId.SOUTH, PumpAction.FILL)
        else:
            self.set_pump(TankId.NORTH, PumpAction.STOP)
            self.set_pump(TankId.SOUTH, PumpAction.STOP)

        if roll_output > 0:
            self.set_pump(TankId.EAST, PumpAction.FILL)
            self.set_pump(TankId.WEST, PumpAction.DRAIN)
        elif roll_output < 0:
            self.set_pump(TankId.EAST, PumpAction.DRAIN)
            self.set_pump(TankId.WEST, PumpAction.FILL)
        else:
            self.set_pump(TankId.EAST, PumpAction.STOP)
            self.set_pump(TankId.WEST, PumpAction.STOP)

    def apply_keyboard_action(self, key: str) -> bool:
        key = key.lower()
        if key == "q":
            return self.all_drain()
        elif key == "z":
            return self.all_fill()
        elif key == " ":
            return self.stop_all()
        elif key == "w":
            self.set_pump(TankId.NORTH, PumpAction.FILL)
            self.set_pump(TankId.SOUTH, PumpAction.DRAIN)
            self.set_pump(TankId.EAST, PumpAction.STOP)
            self.set_pump(TankId.WEST, PumpAction.STOP)
            return True
        elif key == "s":
            self.set_pump(TankId.NORTH, PumpAction.DRAIN)
            self.set_pump(TankId.SOUTH, PumpAction.FILL)
            self.set_pump(TankId.EAST, PumpAction.STOP)
            self.set_pump(TankId.WEST, PumpAction.STOP)
            return True
        elif key == "a":
            self.set_pump(TankId.EAST, PumpAction.DRAIN)
            self.set_pump(TankId.WEST, PumpAction.FILL)
            self.set_pump(TankId.NORTH, PumpAction.STOP)
            self.set_pump(TankId.SOUTH, PumpAction.STOP)
            return True
        elif key == "d":
            self.set_pump(TankId.EAST, PumpAction.FILL)
            self.set_pump(TankId.WEST, PumpAction.DRAIN)
            self.set_pump(TankId.NORTH, PumpAction.STOP)
            self.set_pump(TankId.SOUTH, PumpAction.STOP)
            return True
        return False

    def get_status_string(self) -> str:
        lines = []
        for tid in TankId:
            state = self._states[tid]
            name = TANK_NAMES[tid]
            lines.append(f"  {name}舱: {state.name}")
        return "\n".join(lines)

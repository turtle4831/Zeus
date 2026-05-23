"""Read game controller input on the laptop driver station."""

from __future__ import annotations

import time

from RobotSide.Networking.protocol import ControlMessage


def rescale_value(value, original_min, original_max, target_min, target_max):
    if original_max - original_min == 0:
        return target_min + (target_max - target_min) / 2
    scaled_to_01 = (value - original_min) / (original_max - original_min)
    return target_min + scaled_to_01 * (target_max - target_min)


class ControllerInput:
    def __init__(self):
        self._controller = None
        self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def start(self) -> bool:
        try:
            from pydualsense import pydualsense

            self._controller = pydualsense()
            self._controller.init()
            self._available = True
            return True
        except Exception:
            self._controller = None
            self._available = False
            return False

    def stop(self) -> None:
        if self._controller is not None:
            try:
                self._controller.close()
            except Exception:
                pass
        self._controller = None
        self._available = False

    def read(self, enabled: bool = True) -> ControlMessage:
        if not self._available or self._controller is None:
            return ControlMessage(enabled=False, timestamp=time.time())

        state = self._controller.state
        buttons = {
            "cross": bool(getattr(state, "cross", False)),
            "circle": bool(getattr(state, "circle", False)),
            "square": bool(getattr(state, "square", False)),
            "triangle": bool(getattr(state, "triangle", False)),
            "l1": bool(getattr(state, "L1", False)),
            "r1": bool(getattr(state, "R1", False)),
            "l2": bool(getattr(state, "L2", False)),
            "r2": bool(getattr(state, "R2", False)),
        }
        return ControlMessage(
            enabled=enabled,
            lx=rescale_value(state.LX, -128, 127, -1, 1),
            ly=rescale_value(state.LY, -128, 127, -1, 1),
            rx=rescale_value(state.RX, -128, 127, -1, 1),
            ry=rescale_value(getattr(state, "RY", 0), -128, 127, -1, 1),
            buttons=buttons,
            timestamp=time.time(),
        )

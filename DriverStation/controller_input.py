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
        self._pygame = None
        self._joystick = None
        self._available = False
        self._backend = "none"
        self._name = "none"
        self._last_error = ""

    @property
    def available(self) -> bool:
        return self._available

    @property
    def backend(self) -> str:
        return self._backend

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def name(self) -> str:
        return self._name

    def start(self) -> bool:
        if self._available:
            return True

        if self._start_pygame():
            return True

        if self._start_pydualsense():
            return True

        return False

    def _start_pydualsense(self) -> bool:
        try:
            from pydualsense import pydualsense

            self._controller = pydualsense()
            self._controller.init()
            self._available = True
            self._backend = "pydualsense"
            self._name = "DualSense"
            return True
        except Exception as exc:
            self._controller = None
            self._available = False
            self._name = "none"
            self._last_error = f"pydualsense unavailable: {exc}"
            return False

    def _start_pygame(self) -> bool:
        try:
            import pygame

            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                self._last_error = "pygame found no joysticks"
                return False

            self._pygame = pygame
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._available = True
            self._backend = "pygame"
            self._name = self._joystick.get_name()
            return True
        except Exception as exc:
            self._pygame = None
            self._joystick = None
            self._available = False
            self._name = "none"
            self._last_error = f"pygame unavailable: {exc}"
            return False

    def stop(self) -> None:
        if self._controller is not None:
            try:
                self._controller.close()
            except Exception:
                pass
        if self._pygame is not None:
            try:
                self._pygame.joystick.quit()
                self._pygame.quit()
            except Exception:
                pass
        self._controller = None
        self._pygame = None
        self._joystick = None
        self._available = False
        self._backend = "none"
        self._name = "none"

    def read(self, enabled: bool = True) -> ControlMessage:
        if not self._available or self._controller is None:
            if self._backend == "pygame" and self._joystick is not None:
                return self._read_pygame(enabled)
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

    def _read_pygame(self, enabled: bool = True) -> ControlMessage:
        self._pygame.event.pump()

        def axis(index: int, default: float = 0.0) -> float:
            if self._joystick.get_numaxes() <= index:
                return default
            value = self._joystick.get_axis(index)
            return 0.0 if abs(value) < 0.08 else value

        def button(index: int) -> bool:
            return self._joystick.get_numbuttons() > index and bool(self._joystick.get_button(index))

        return ControlMessage(
            enabled=enabled,
            lx=axis(0),
            ly=-axis(1),
            rx=axis(3),
            ry=-axis(4),
            buttons={
                "cross": button(0),
                "circle": button(1),
                "square": button(2),
                "triangle": button(3),
                "l1": button(4),
                "r1": button(5),
            },
            timestamp=time.time(),
        )

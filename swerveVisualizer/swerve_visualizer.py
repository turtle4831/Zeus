#!/usr/bin/env python3
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

from PyQt6.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt6.QtGui import QBrush, QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QWidget

ZEUS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ZEUS_ROOT))

from DriverStation.controller_input import ControllerInput
from RobotSide.shooting.positionPrediction import PositionPrediction
from RobotSide.swerve.swerveKinematics import SwerveKinematics


FIELD_SIZE_METERS = 9.0
ROBOT_LENGTH_METERS = 0.8
ROBOT_WIDTH_METERS = 0.8
MAX_LINEAR_SPEED = 3.0
MAX_ANGULAR_SPEED = 180.0
TIME_OF_FLIGHT = 0.5


class SwerveVisualizer(QWidget):
    def __init__(self):
        super().__init__()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.kinematics = SwerveKinematics(ROBOT_LENGTH_METERS, ROBOT_WIDTH_METERS)
        self.predictor = PositionPrediction()
        self.controller = ControllerInput()
        self.controller_available = self.controller.start()

        self.pose = [FIELD_SIZE_METERS / 2, FIELD_SIZE_METERS / 2, 0.0]
        self.velocity = (0.0, 0.0)
        self.module_states = [(0.0, 0.0)] * 4
        self.keys_pressed: set[int] = set()
        self.last_update = time.time()

        self.status = QLabel(self)
        self.status.setStyleSheet("background: rgba(255, 255, 255, 180); padding: 4px;")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(20)

    def update_simulation(self):
        now = time.time()
        dt = now - self.last_update
        self.last_update = now

        field_vx, field_vy, omega = self._read_controls()
        self.pose[2] = self._wrap_degrees(self.pose[2] + omega * dt)

        heading = math.radians(self.pose[2])
        robot_vx = field_vx * math.cos(heading) + field_vy * math.sin(heading)
        robot_vy = -field_vx * math.sin(heading) + field_vy * math.cos(heading)

        speeds, angles = self.kinematics.toSwerveModuleStates(robot_vx, robot_vy, math.radians(omega))
        speeds, angles = self.kinematics.normalize((speeds, angles), MAX_LINEAR_SPEED)
        self.module_states = list(zip(speeds, angles))

        self.velocity = (field_vx, field_vy)

        self.pose[0] = self._clamp(self.pose[0] + field_vx * dt, 0, FIELD_SIZE_METERS)
        self.pose[1] = self._clamp(self.pose[1] + field_vy * dt, 0, FIELD_SIZE_METERS)

        self.status.setText(
            f"Controller: {self.controller.backend if self.controller_available else 'keyboard fallback'} | "
            f"Pose: x={self.pose[0]:.2f} y={self.pose[1]:.2f} heading={self.pose[2]:.1f} | "
            f"Velocity: vx={field_vx:.2f} vy={field_vy:.2f} | Ghost TOF={TIME_OF_FLIGHT:.1f}s"
        )
        self.status.adjustSize()
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(34, 38, 44))

        field = self._field_rect()
        painter.setPen(QPen(QColor(220, 220, 220), 2))
        painter.setBrush(QBrush(QColor(25, 80, 35)))
        painter.drawRect(field)

        self._draw_grid(painter, field)

        ghost_position = self.predictor.predictPosition(
            (self.pose[0], self.pose[1]), self.velocity, TIME_OF_FLIGHT
        )
        ghost_pose = (
            self._clamp(ghost_position[0], 0, FIELD_SIZE_METERS),
            self._clamp(ghost_position[1], 0, FIELD_SIZE_METERS),
            self.pose[2],
        )

        self._draw_robot(painter, ghost_pose, QColor(80, 170, 255, 90), QColor(80, 170, 255), ghost=True)
        self._draw_robot(painter, tuple(self.pose), QColor(240, 185, 60), QColor(255, 255, 255), ghost=False)

    def keyPressEvent(self, event):
        self.keys_pressed.add(event.key())

    def keyReleaseEvent(self, event):
        self.keys_pressed.discard(event.key())

    def closeEvent(self, event):
        self.controller.stop()
        super().closeEvent(event)

    def _read_controls(self):
        if self.controller_available:
            controls = self.controller.read(enabled=True)
            return (
                controls.lx * MAX_LINEAR_SPEED,
                controls.ly * MAX_LINEAR_SPEED,
                controls.rx * MAX_ANGULAR_SPEED,
            )

        vx = 0.0
        vy = 0.0
        omega = 0.0
        if Qt.Key.Key_W in self.keys_pressed:
            vy += MAX_LINEAR_SPEED
        if Qt.Key.Key_S in self.keys_pressed:
            vy -= MAX_LINEAR_SPEED
        if Qt.Key.Key_D in self.keys_pressed:
            vx += MAX_LINEAR_SPEED
        if Qt.Key.Key_A in self.keys_pressed:
            vx -= MAX_LINEAR_SPEED
        if Qt.Key.Key_E in self.keys_pressed:
            omega += MAX_ANGULAR_SPEED
        if Qt.Key.Key_Q in self.keys_pressed:
            omega -= MAX_ANGULAR_SPEED
        return vx, vy, omega

    def _field_rect(self):
        margin = 40
        size = min(self.width(), self.height()) - margin * 2
        left = (self.width() - size) / 2
        top = (self.height() - size) / 2
        return QRectF(left, top, size, size)

    def _draw_grid(self, painter, field):
        painter.setPen(QPen(QColor(255, 255, 255, 70), 1))
        for meter in range(10):
            offset = field.width() * meter / FIELD_SIZE_METERS
            painter.drawLine(QPointF(field.left() + offset, field.top()), QPointF(field.left() + offset, field.bottom()))
            painter.drawLine(QPointF(field.left(), field.top() + offset), QPointF(field.right(), field.top() + offset))

    def _draw_robot(self, painter, pose, fill_color, pen_color, ghost=False):
        x, y, heading = pose
        center = self._to_screen(x, y)
        scale = self._field_rect().width() / FIELD_SIZE_METERS
        half_length = ROBOT_LENGTH_METERS * scale / 2
        half_width = ROBOT_WIDTH_METERS * scale / 2
        heading_rad = math.radians(heading)

        corners = [
            self._rotate_point(half_length, half_width, heading_rad, center),
            self._rotate_point(half_length, -half_width, heading_rad, center),
            self._rotate_point(-half_length, -half_width, heading_rad, center),
            self._rotate_point(-half_length, half_width, heading_rad, center),
        ]

        painter.setPen(QPen(pen_color, 2, Qt.PenStyle.DashLine if ghost else Qt.PenStyle.SolidLine))
        painter.setBrush(QBrush(fill_color))
        painter.drawPolygon(QPolygonF(corners))

        heading_end = self._rotate_point(half_length, 0, heading_rad, center)
        painter.drawLine(center, heading_end)

        if not ghost:
            self._draw_modules(painter, center, heading_rad, scale)

    def _draw_modules(self, painter, center, heading_rad, scale):
        painter.setPen(QPen(QColor(20, 20, 20), 4))
        for (module_x, module_y), (speed, angle) in zip(self.kinematics.moduleLocations, self.module_states):
            module_center = self._rotate_point(module_x * scale, -module_y * scale, heading_rad, center)
            wheel_angle = heading_rad + math.radians(angle)
            length = 0.25 * scale * (0.5 + min(abs(speed) / MAX_LINEAR_SPEED, 1.0))
            start = self._rotate_point(-length / 2, 0, wheel_angle, module_center)
            end = self._rotate_point(length / 2, 0, wheel_angle, module_center)
            painter.drawLine(start, end)

    def _to_screen(self, x, y):
        field = self._field_rect()
        return QPointF(
            field.left() + x / FIELD_SIZE_METERS * field.width(),
            field.bottom() - y / FIELD_SIZE_METERS * field.height(),
        )

    def _rotate_point(self, x, y, angle, center):
        return QPointF(
            center.x() + x * math.cos(angle) - y * math.sin(angle),
            center.y() + x * math.sin(angle) + y * math.cos(angle),
        )

    def _clamp(self, value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def _wrap_degrees(self, angle):
        return ((angle + 180) % 360) - 180


class SwerveVisualizerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Zeus Swerve Visualizer")
        self.resize(900, 900)
        self.setCentralWidget(SwerveVisualizer())


def main():
    app = QApplication(sys.argv)
    window = SwerveVisualizerWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

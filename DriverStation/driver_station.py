#!/usr/bin/env python3
"""PyQt6 driver station for Zeus robot control over Wi-Fi hotspot."""

from __future__ import annotations

import sys
from pathlib import Path

from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

ZEUS_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ZEUS_ROOT))

from DriverStation.controller_input import ControllerInput
from DriverStation.network_client import DriverStationClient
from RobotSide.Networking.protocol import (
    DEFAULT_PORT,
    INITIALIZE_ESC_BUTTON,
    ControlMessage,
    TelemetryMessage,
)


class DriverStationWindow(QMainWindow):
    telemetry_received = pyqtSignal(object)
    connection_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Zeus Driver Station")
        self.resize(720, 520)

        self._controller = ControllerInput()
        self._client: DriverStationClient | None = None
        self._connected = False
        self._latest_telemetry: TelemetryMessage | None = None
        self._enabled = True
        self._initialize_esc_requested = False

        self._build_ui()
        self._start_controller()
        self.telemetry_received.connect(self._on_telemetry_ui)
        self.connection_changed.connect(self._on_connection_ui)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_control_labels)
        self._ui_timer.start(50)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        connection_box = QGroupBox("Connection")
        connection_layout = QHBoxLayout(connection_box)
        self._host_input = QLineEdit("192.168.4.1")
        self._port_input = QLineEdit(str(DEFAULT_PORT))
        self._connect_button = QPushButton("Connect")
        self._connect_button.clicked.connect(self._toggle_connection)
        connection_layout.addWidget(QLabel("Robot IP:"))
        connection_layout.addWidget(self._host_input)
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self._port_input)
        connection_layout.addWidget(self._connect_button)
        layout.addWidget(connection_box)

        status_box = QGroupBox("Status")
        status_form = QFormLayout(status_box)
        self._connection_label = QLabel("Disconnected")
        self._controller_label = QLabel("Not initialized")
        self._robot_state_label = QLabel("-")
        self._battery_label = QLabel("-")
        status_form.addRow("Link:", self._connection_label)
        status_form.addRow("Controller:", self._controller_label)
        status_form.addRow("Robot state:", self._robot_state_label)
        status_form.addRow("Battery:", self._battery_label)
        layout.addWidget(status_box)

        controls_box = QGroupBox("Controller Output")
        controls_layout = QGridLayout(controls_box)
        self._lx_label = QLabel("0.00")
        self._ly_label = QLabel("0.00")
        self._rx_label = QLabel("0.00")
        self._ry_label = QLabel("0.00")
        self._enabled_label = QLabel("false")
        controls_layout.addWidget(QLabel("LX"), 0, 0)
        controls_layout.addWidget(self._lx_label, 0, 1)
        controls_layout.addWidget(QLabel("LY"), 0, 2)
        controls_layout.addWidget(self._ly_label, 0, 3)
        controls_layout.addWidget(QLabel("RX"), 1, 0)
        controls_layout.addWidget(self._rx_label, 1, 1)
        controls_layout.addWidget(QLabel("RY"), 1, 2)
        controls_layout.addWidget(self._ry_label, 1, 3)
        controls_layout.addWidget(QLabel("Enabled"), 2, 0)
        controls_layout.addWidget(self._enabled_label, 2, 1)
        layout.addWidget(controls_box)

        telemetry_box = QGroupBox("Robot Telemetry")
        telemetry_layout = QVBoxLayout(telemetry_box)
        self._subsystems_label = QLabel("No telemetry yet")
        self._subsystems_label.setWordWrap(True)
        self._subsystems_label.setAlignment(Qt.AlignmentFlag.AlignTop)
        telemetry_layout.addWidget(self._subsystems_label)
        layout.addWidget(telemetry_box)

        self._enable_button = QPushButton("Disable Controls")
        self._enable_button.clicked.connect(self._toggle_enabled)
        layout.addWidget(self._enable_button)

        self._initialize_esc_button = QPushButton("Initialize ESCs")
        self._initialize_esc_button.clicked.connect(self._request_initialize_esc)
        layout.addWidget(self._initialize_esc_button)

        self.statusBar().showMessage("Ready")

    def _start_controller(self) -> None:
        if self._controller.start():
            self._controller_label.setText(
                f"{self._controller.name} ({self._controller.backend})"
            )
            self.statusBar().showMessage(f"Controller detected: {self._controller.name}")
        else:
            self._controller_label.setText(f"Not found: {self._controller.last_error}")
            self.statusBar().showMessage("Controller not found")

    def _toggle_connection(self) -> None:
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        if not self._controller.available:
            self._start_controller()

        host = self._host_input.text().strip()
        port = int(self._port_input.text().strip())
        self._client = DriverStationClient(host=host, port=port)
        try:
            self._client.connect(
                control_provider=self._read_control,
                on_telemetry=self._handle_telemetry,
                on_connection_changed=self._handle_connection_changed,
            )
        except OSError as exc:
            self._client = None
            self.statusBar().showMessage(f"Connection failed: {exc}")
            return

        self._connected = True
        self._connect_button.setText("Disconnect")
        self._host_input.setEnabled(False)
        self._port_input.setEnabled(False)
        self.statusBar().showMessage(f"Connected to {host}:{port}")

    def _disconnect(self) -> None:
        if self._client:
            self._client.disconnect()
            self._client = None
        self._connected = False
        self._connect_button.setText("Connect")
        self._host_input.setEnabled(True)
        self._port_input.setEnabled(True)
        self.statusBar().showMessage("Disconnected")

    def _toggle_enabled(self) -> None:
        self._enabled = not self._enabled
        self._enable_button.setText("Disable Controls" if self._enabled else "Enable Controls")

    def _request_initialize_esc(self) -> None:
        self._initialize_esc_requested = True
        self.statusBar().showMessage("ESC initialize requested (requires disabled controls on robot)")

    def _read_control(self):
        control = self._controller.read(enabled=self._enabled and self._connected)
        buttons = dict(control.buttons)
        if self._initialize_esc_requested:
            buttons[INITIALIZE_ESC_BUTTON] = True
            self._initialize_esc_requested = False
        return ControlMessage(
            enabled=control.enabled,
            lx=control.lx,
            ly=control.ly,
            rx=control.rx,
            ry=control.ry,
            buttons=buttons,
            timestamp=control.timestamp,
        )

    def _handle_telemetry(self, telemetry: TelemetryMessage) -> None:
        self.telemetry_received.emit(telemetry)

    def _handle_connection_changed(self, connected: bool) -> None:
        self.connection_changed.emit(connected)

    def _on_telemetry_ui(self, telemetry: TelemetryMessage) -> None:
        self._latest_telemetry = telemetry
        self._robot_state_label.setText(telemetry.robot_state)
        if telemetry.battery_voltage is not None:
            self._battery_label.setText(f"{telemetry.battery_voltage:.2f} V")
        self._subsystems_label.setText(
            "\n".join(f"{name}: {value}" for name, value in telemetry.subsystems.items())
            or "No subsystem data"
        )

    def _on_connection_ui(self, connected: bool) -> None:
        self._connection_label.setText("Connected" if connected else "Disconnected")
        if not connected and self._connected:
            self._disconnect()

    def _refresh_control_labels(self) -> None:
        control = self._read_control()
        self._lx_label.setText(f"{control.lx:.2f}")
        self._ly_label.setText(f"{control.ly:.2f}")
        self._rx_label.setText(f"{control.rx:.2f}")
        self._ry_label.setText(f"{control.ry:.2f}")
        self._enabled_label.setText(str(control.enabled).lower())

    def closeEvent(self, event) -> None:
        self._disconnect()
        self._controller.stop()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = DriverStationWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

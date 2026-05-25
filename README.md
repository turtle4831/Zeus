# Zeus Robot

This repo contains the robot-side code and a PyQt6 driver station for controlling the robot from a laptop over the robot's Wi-Fi hotspot.

## Setup

Use a virtual environment before installing dependencies.

### Linux

```bash
cd Zeus
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### Windows

From PowerShell:

```powershell
cd Zeus
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

If the virtual environment already exists, just activate it before running commands:

Linux:

```bash
cd Zeus
source .venv/bin/activate
```

Windows PowerShell:

```powershell
cd Zeus
.\.venv\Scripts\Activate.ps1
```

`requirements.txt` is platform-aware. Windows installs the driver station, controller, test, and shared RobotPy dependencies, but skips Linux-only robot hardware packages such as `gpiozero` and `smbus`. Those hardware packages install on Linux and on the robot.

## Start The Robot

Run this on the robot after it has created its hotspot:

```bash
cd Zeus
source .venv/bin/activate
python robot.py
```

The robot starts a driver station server on TCP port `5800`. It listens for controller inputs from the laptop and sends telemetry back to the driver station.

## Start The Driver Station

Connect the laptop to the robot's hotspot, then run:

```bash
cd Zeus
source .venv/bin/activate
python DriverStation/driver_station.py
```

In the driver station window:

1. Set `Robot IP` to the robot hotspot IP address.
2. Leave `Port` as `5800` unless the robot code was changed.
3. Click `Connect`.

The default robot IP in the UI is `192.168.4.1`. Change it if your hotspot uses a different address.

## Controls And Telemetry

The driver station reads a DualSense controller through `pygame` first, then falls back to `pydualsense`, and sends control packets to the robot at about `50 Hz`.

The robot sends telemetry back at about `10 Hz`, including connection status, robot state, subsystem state text, and battery voltage if the robot code provides it.

If the robot stops receiving control packets for more than `0.25` seconds, it falls back to disabled zeroed controls.

## Run Tests

After activating the virtual environment, run:

```bash
cd Zeus
source .venv/bin/activate
python -m pytest
```

On Windows PowerShell:

```powershell
cd Zeus
.\.venv\Scripts\Activate.ps1
python -m pytest
```

ROS can add broken pytest plugins to the Python path on some Linux machines. If pytest fails while loading unrelated ROS plugins, run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest
```

## Troubleshooting

If the driver station cannot connect:

- Make sure the laptop is connected to the robot hotspot.
- Confirm the robot process is running with `python robot.py`.
- Check that the driver station IP matches the robot hotspot IP.
- Make sure port `5800` is not blocked by a firewall.

If the controller does not work:

- Make sure the DualSense controller is connected to the laptop.
- Restart the driver station after connecting the controller.
- Check that `pygame` can see the controller in the active virtual environment.
- On Windows, confirm the controller appears in Bluetooth devices or Game Controllers before starting the driver station.

## Fast Deploy To Robot

For day-to-day development, you can use your laptop to deploy to the robot over SSH.

### One-Time Setup

On the robot, create the project directory and virtual environment once:

```bash
mkdir -p ~/ZeusRobot/Zeus
cd ~/ZeusRobot/Zeus
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On your laptop, make sure SSH works to the robot hotspot:

```bash
ssh robot@192.168.4.1
```

Set defaults so deploy commands are shorter:

```bash
export ZEUS_ROBOT_HOST=robot@192.168.4.1
export ZEUS_ROBOT_PATH=~/ZeusRobot/Zeus
```

### Deploy Code

From the laptop:

```bash
cd Zeus
python scripts/deploy_robot.py
```

This syncs changed files with `rsync`, then restarts `robot.py` on the robot.

Useful options:

```bash
# Preview what would be copied
python scripts/deploy_robot.py --dry-run

# Copy files without restarting robot.py
python scripts/deploy_robot.py --sync-only

# Copy files and reinstall dependencies on the robot
python scripts/deploy_robot.py --install-deps
```

You can also pass values directly:

```bash
python scripts/deploy_robot.py --host robot@192.168.4.1 --remote-path ~/ZeusRobot/Zeus
```

### Verify Deploy

After deploying, SSH into the robot and check:

```bash
pgrep -af robot.py
tail -n 50 ~/ZeusRobot/Zeus/robot.log
```

### Deploy Troubleshooting

If deploy fails:

- Confirm the laptop is connected to the robot hotspot.
- Confirm SSH works: `ssh $ZEUS_ROBOT_HOST`
- Confirm the remote path exists and contains `.venv`.
- If dependencies changed, rerun with `--install-deps`.
- Use `--dry-run` first to verify the rsync target before copying files.

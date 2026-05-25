# Swerve Visualizer

PyQt6 visualizer for the Zeus swerve drive simulation.

## Run

From `Zeus`:

```bash
source .venv/bin/activate
python swerveVisualizer/swerve_visualizer.py
```

The visualizer draws a `9m x 9m` field, the current swerve pose, wheel vectors, and a blue ghost pose predicted `0.5s` into the future using `PositionPrediction`.

## Controls

The app tries to use the same DualSense controller path as the driver station. Translation is field-relative, so forward always means toward the top of the field even after the robot rotates.

If no DualSense is available, use the keyboard fallback:

- `W` / `S`: forward and backward
- `A` / `D`: strafe left and right
- `Q` / `E`: rotate left and right

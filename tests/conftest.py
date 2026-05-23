import os
import sys
from pathlib import Path

# Avoid broken ROS pytest plugins on systems with /opt/ros sourced.
os.environ.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")

ZEUS_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ZEUS_ROOT.parent

sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(ZEUS_ROOT))

#!/usr/bin/env python3
"""接手基线快速校验脚本。"""

from __future__ import annotations

import importlib
import inspect
import math
import os
import sys
import tempfile
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_DIR / "src"
MUJOCO_SIM_DIR = PROJECT_DIR / "mujoco_sim"
MUJOCO_SRC_DIR = MUJOCO_SIM_DIR / "src"

for path in (SRC_DIR, MUJOCO_SRC_DIR, PROJECT_DIR):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def check(name: str, condition: bool, detail: str = "") -> bool:
    status = "PASS" if condition else "FAIL"
    suffix = f" - {detail}" if detail else ""
    print(f"[{status}] {name}{suffix}")
    return condition


def main() -> int:
    ok = True

    required_modules = ["mujoco", "PyQt6", "scipy", "serial", "numpy"]
    for mod_name in required_modules:
        try:
            importlib.import_module(mod_name)
            ok &= check(f"Import {mod_name}", True)
        except Exception as exc:
            ok &= check(f"Import {mod_name}", False, str(exc))

    from main import MainWindowWithMuJoCo
    from core.robot import DummyRobot
    from core.teach_mode import TeachMode, TrajectoryPoint
    from gui.main_window_with_mujoco import SimPlaybackThread
    from utils.config import JOINT_LIMITS, JointLimitChecker
    from qt_integration import firmware_to_urdf

    ok &= check(
        "Main entry points to MuJoCo window",
        MainWindowWithMuJoCo.__name__ == "MainWindowWithMuJoCo",
    )
    ok &= check(
        "J3 hard minimum is 35 deg",
        JOINT_LIMITS[2] == (35, 180),
        f"found {JOINT_LIMITS[2]}",
    )
    ok &= check(
        "Joint clamp enforces J3 minimum",
        JointLimitChecker.clamp_angles([0, 0, 0, 0, 0, 0])[2] == 35,
    )

    mapped = firmware_to_urdf([10, -20, 90, 30, 40, -50])
    expected = [
        -math.radians(10),
        math.radians(20),
        0.0,
        -math.radians(30),
        math.radians(40),
        math.radians(50),
    ]
    ok &= check(
        "firmware_to_urdf mapping matches baseline",
        all(abs(a - b) < 1e-9 for a, b in zip(mapped, expected)),
    )

    robot_source = inspect.getsource(DummyRobot)
    for label, needle in [
        ("Teach mode uses !DISABLE", "!DISABLE"),
        ("Teach exit re-enables with !START", "!START"),
        ("Position query uses #GETJPOS", "#GETJPOS"),
    ]:
        ok &= check(label, needle in robot_source)

    sim_source = inspect.getsource(SimPlaybackThread.run)
    ok &= check("Sim playback keeps >=20ms spacing", "max(0.02, dt)" in sim_source)

    teach = TeachMode(DummyRobot())
    sample = [
        TrajectoryPoint(timestamp=0.0, angles=[0, -75, 180, 0, 0, 0]),
        TrajectoryPoint(timestamp=0.1, angles=[0, -60, 150, 0, 10, 0]),
    ]
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir) / "traj.json"
        teach.save_trajectory(sample, str(tmp_path), format="json", metadata={"source": "verify"})
        loaded = teach.load_trajectory(str(tmp_path), validate_limits=True)
        ok &= check("Trajectory JSON save/load works", loaded is not None and len(loaded) == 2)

    print()
    print("Summary:", "all baseline checks passed" if ok else "baseline check failed")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

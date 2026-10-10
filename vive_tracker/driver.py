"""Device-level Vive Tracker API independent of ROS 2."""

from __future__ import annotations

import ctypes
import ctypes.util
import logging
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

logger = logging.getLogger(__name__)


def _libsurvive_root() -> Path:
    configured = os.environ.get("LIBSURVIVE_HOME")
    if configured:
        return Path(configured).expanduser()
    return Path(__file__).resolve().parent.parent / "third_party" / "libsurvive"


def _load_pysurvive():
    bindings = _libsurvive_root() / "bindings" / "python"
    if bindings.is_dir() and str(bindings) not in sys.path:
        sys.path.insert(0, str(bindings))
    try:
        import pysurvive
    except ImportError as exc:
        raise RuntimeError("pysurvive is unavailable; build libsurvive first") from exc
    return pysurvive


@dataclass(frozen=True)
class Pose:
    """Latest Tracker pose. Quaternion order is x, y, z, w."""

    device_name: str
    serial_number: Optional[str]
    timestamp: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    raw_position: tuple[float, float, float]
    raw_orientation: tuple[float, float, float, float]


def _quaternion_matrix(q: Iterable[float]) -> np.ndarray:
    x, y, z, w = q
    values = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(values))
    if norm == 0:
        raise ValueError("orientation quaternion must not be zero")
    x, y, z, w = values / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
        [0, 0, 0, 1],
    ], dtype=float)


def _matrix_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    rotation = matrix[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        s = 2 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = 2 * np.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            x, y, z, w = 0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s, (rotation[0, 2] + rotation[2, 0]) / s, (rotation[2, 1] - rotation[1, 2]) / s
        elif index == 1:
            s = 2 * np.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            x, y, z, w = (rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s, (rotation[1, 2] + rotation[2, 1]) / s, (rotation[0, 2] - rotation[2, 0]) / s
        else:
            s = 2 * np.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            x, y, z, w = (rotation[0, 2] + rotation[2, 0]) / s, (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s, (rotation[1, 0] - rotation[0, 1]) / s
    values = np.array([x, y, z, w], dtype=float)
    values /= np.linalg.norm(values)
    return tuple(float(value) for value in values)


def _transform_matrix(translation: Iterable[float], rpy_degrees: Iterable[float]) -> np.ndarray:
    tx, ty, tz = translation
    roll, pitch, yaw = np.deg2rad(list(rpy_degrees))
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = [tx, ty, tz]
    return result


class ViveTracker:
    """Read poses from one or more Trackers through libsurvive."""

    def __init__(self, serial_numbers: Optional[Iterable[str]] = None,
                 translation=(0.0, 0.0, 0.0), rpy_degrees=(0.0, 0.0, 0.0),
                 config_path: Optional[str] = None, lh_config: Optional[str] = None,
                 args: Optional[Iterable[str]] = None):
        self.serial_numbers = set(serial_numbers or [])
        self.transform = _transform_matrix(translation, rpy_degrees)
        self.config_path, self.lh_config = config_path, lh_config
        self.args = list(args or [])
        self.context = None
        self.running = False
        self._poses: dict[str, Pose] = {}
        self._serial_to_name: dict[str, str] = {}
        self._devices: dict[str, dict] = {}
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._libsurvive = self._load_library()

    def _load_library(self):
        root = _libsurvive_root()
        candidates = [root / "bin" / "libsurvive.so", root / "bin" / "libsurvive.so.0"]
        found = ctypes.util.find_library("survive")
        if found:
            candidates.insert(0, Path(found))
        for candidate in candidates:
            if candidate.exists() or str(candidate) == found:
                try:
                    library = ctypes.CDLL(str(candidate))
                    library.survive_simple_serial_number.argtypes = [ctypes.c_void_p]
                    library.survive_simple_serial_number.restype = ctypes.c_char_p
                    return library
                except OSError:
                    pass
        raise RuntimeError(f"libsurvive.so not found under {root}")

    @staticmethod
    def _name(device) -> str:
        value = device.Name()
        return value.decode() if isinstance(value, bytes) else str(value)

    def _serial(self, device) -> Optional[str]:
        try:
            pointer = device.ptr
            if isinstance(pointer, int):
                pointer = ctypes.c_void_p(pointer)
            else:
                pointer = ctypes.cast(pointer, ctypes.c_void_p)
            value = self._libsurvive.survive_simple_serial_number(pointer)
            return value.decode() if value else None
        except (AttributeError, TypeError, ValueError):
            return None

    def connect(self) -> None:
        if self.running:
            return
        pysurvive = _load_pysurvive()
        survive_args = [sys.argv[0]]
        if self.config_path:
            survive_args += ["--config", self.config_path]
        if self.lh_config:
            survive_args += ["--lh", self.lh_config]
        survive_args += self.args
        self.context = pysurvive.SimpleContext(survive_args)
        if not self.context:
            raise RuntimeError("failed to initialize pysurvive context")
        self.running = True
        self._thread = threading.Thread(target=self._collect, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self.running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.context = None

    def _collect(self) -> None:
        while self.running and self.context and self.context.Running():
            updated = self.context.NextUpdated()
            if not updated:
                continue
            try:
                name, serial = self._name(updated), self._serial(updated)
                pose_data, timestamp = updated.Pose()
                raw_position = tuple(float(value) for value in pose_data.Pos[:3])
                raw_orientation = tuple(float(value) for value in (pose_data.Rot[1], pose_data.Rot[2], pose_data.Rot[3], pose_data.Rot[0]))
                matrix = _quaternion_matrix(raw_orientation)
                matrix[:3, 3] = raw_position
                transformed = matrix @ self.transform
                pose = Pose(name, serial, float(timestamp), tuple(float(value) for value in transformed[:3, 3]), _matrix_quaternion(transformed), raw_position, raw_orientation)
                with self._lock:
                    self._poses[name] = pose
                    self._devices[name] = {"serial": serial, "updates": self._devices.get(name, {}).get("updates", 0) + 1}
                    if serial:
                        self._serial_to_name[serial] = name
            except Exception:
                logger.exception("failed to process Tracker update")

    def get_pose(self, serial_number: Optional[str] = None):
        with self._lock:
            if serial_number is None:
                return dict(self._poses)
            name = self._serial_to_name.get(serial_number)
            return self._poses.get(name) if name else None

    def get_devices(self) -> dict[str, dict]:
        with self._lock:
            return {name: dict(info) for name, info in self._devices.items()}

    def wait_for_serial(self, serial_number: str, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_pose(serial_number) is not None:
                return True
            time.sleep(0.05)
        return False

"""Historical three-point mapping reference; test-only, never installed.

The extraction matches ``pico_manager_thread_server._process_3pt_pose`` without
SciPy.  Calibration intentionally targets GearSonic's policy keypoints first,
then subtracts the configured body offsets to produce URDF link origins.  The
existing GearSonicInterface can therefore apply its offsets exactly once.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


BODY_SHAPE = (24, 7)
KEYPOINT_COUNT = 3

# [left wrist, right wrist, torso/head] policy keypoints in the pelvis frame.
GEAR_SONIC_DEFAULT_KEYPOINT_POSITIONS = np.asarray(
    [
        [0.0903, 0.1615, -0.2411],
        [0.1280, -0.1522, -0.2461],
        [0.0241, -0.0081, 0.4028],
    ],
    dtype=np.float32,
)
# Same values as GearSonicInterface, converted from wxyz to xyzw.
GEAR_SONIC_DEFAULT_ORIENTATIONS_XYZW = np.asarray(
    [
        [0.3145, 0.5533, -0.2506, 0.7295],
        [-0.2639, 0.5395, 0.3217, 0.7320],
        [0.0110, 0.0402, -0.0002, 0.9991],
    ],
    dtype=np.float32,
)
GEAR_SONIC_BODY_OFFSETS = np.asarray(
    [
        [0.18, -0.025, 0.0],
        [0.18, 0.025, 0.0],
        [0.0, 0.0, 0.35],
    ],
    dtype=np.float32,
)

_UNITY_TO_ROBOT = np.asarray(
    [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
_KEYPOINT_JOINTS = (0, 22, 23, 12)  # pelvis, left wrist, right wrist, neck


class BodyMappingError(ValueError):
    """Raised when body input or calibration is invalid."""


def _owned_finite(value: Any, shape: tuple[int, ...], *, name: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise BodyMappingError(f"{name} must be numeric") from exc
    if result.shape != shape:
        raise BodyMappingError(f"{name} must have shape {shape}, got {result.shape}")
    if not np.all(np.isfinite(result)):
        raise BodyMappingError(f"{name} contains NaN or infinity")
    return np.array(result, dtype=np.float64, copy=True)


def _normalize_xyzw(quaternion: np.ndarray, *, name: str) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm <= 1e-8:
        raise BodyMappingError(f"{name} has a zero quaternion")
    return quaternion / norm


def _matrix_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_xyzw(quaternion, name="orientation")
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _xyzw_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to a normalized xyzw quaternion."""

    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.asarray(
            [
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
                0.25 * s,
            ]
        )
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = np.asarray(
                [
                    0.25 * s,
                    (m[0, 1] + m[1, 0]) / s,
                    (m[0, 2] + m[2, 0]) / s,
                    (m[2, 1] - m[1, 2]) / s,
                ]
            )
        elif i == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = np.asarray(
                [
                    (m[0, 1] + m[1, 0]) / s,
                    0.25 * s,
                    (m[1, 2] + m[2, 1]) / s,
                    (m[0, 2] - m[2, 0]) / s,
                ]
            )
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = np.asarray(
                [
                    (m[0, 2] + m[2, 0]) / s,
                    (m[1, 2] + m[2, 1]) / s,
                    0.25 * s,
                    (m[1, 0] - m[0, 1]) / s,
                ]
            )
    q = _normalize_xyzw(q, name="converted orientation")
    # A unique sign keeps calibration and tests deterministic (q and -q rotate equally).
    return -q if q[3] < 0.0 else q


def _rotation_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotation_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.asarray([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _extrinsic_xyz_degrees(x: float, y: float, z: float) -> np.ndarray:
    x_rad, y_rad, z_rad = np.deg2rad([x, y, z])
    return _rotation_z(z_rad) @ _rotation_y(y_rad) @ _rotation_x(x_rad)


_JOINT_ROTATION_OFFSETS = (
    _extrinsic_xyz_degrees(0.0, 0.0, -90.0),
    _extrinsic_xyz_degrees(90.0, 0.0, 0.0),
    _extrinsic_xyz_degrees(-90.0, 0.0, 180.0),
    _extrinsic_xyz_degrees(0.0, 0.0, -90.0),
)


def process_3pt_pose(body: Any) -> np.ndarray:
    """Return source-compatible ``[L wrist, R wrist, neck]`` xyz+wxyz.

    Input is 24 SMPL poses in Unity xyz+xyzw.  Output positions and
    orientations are relative to the offset-adjusted pelvis frame.
    """

    poses = _owned_finite(body, BODY_SHAPE, name="body")
    positions_robot = poses[:, :3] @ _UNITY_TO_ROBOT.T
    rotations_robot = np.empty((BODY_SHAPE[0], 3, 3), dtype=np.float64)
    for index in range(BODY_SHAPE[0]):
        rotation_unity = _matrix_from_xyzw(poses[index, 3:])
        rotations_robot[index] = _UNITY_TO_ROBOT @ rotation_unity @ _UNITY_TO_ROBOT.T

    key_positions = positions_robot[np.asarray(_KEYPOINT_JOINTS)]
    key_rotations = np.stack(
        [
            rotations_robot[joint] @ _JOINT_ROTATION_OFFSETS[index]
            for index, joint in enumerate(_KEYPOINT_JOINTS)
        ]
    )
    root_position = key_positions[0]
    root_inverse = key_rotations[0].T

    result = np.empty((KEYPOINT_COUNT, 7), dtype=np.float32)
    for output_index in range(KEYPOINT_COUNT):
        source_index = output_index + 1
        result[output_index, :3] = root_inverse @ (key_positions[source_index] - root_position)
        relative_rotation = root_inverse @ key_rotations[source_index]
        xyzw = _xyzw_from_matrix(relative_rotation)
        result[output_index, 3:] = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
    return result


@dataclass(frozen=True)
class BodyTargets:
    """Calibrated policy keypoints and link-origin poses, all in pelvis frame."""

    keypoint_poses_xyzw: np.ndarray
    link_poses_xyzw: np.ndarray

    def __post_init__(self) -> None:
        for field_name in ("keypoint_poses_xyzw", "link_poses_xyzw"):
            value = _owned_finite(getattr(self, field_name), (KEYPOINT_COUNT, 7), name=field_name)
            value = value.astype(np.float32)
            value.setflags(write=False)
            object.__setattr__(self, field_name, value)


def gear_sonic_keypoints_from_links(link_poses_xyzw: Any) -> np.ndarray:
    """Apply GearSonic's body offsets, useful for boundary verification."""

    links = _owned_finite(link_poses_xyzw, (KEYPOINT_COUNT, 7), name="link_poses_xyzw")
    result = links.copy()
    for index in range(KEYPOINT_COUNT):
        rotation = _matrix_from_xyzw(links[index, 3:])
        result[index, :3] += rotation @ GEAR_SONIC_BODY_OFFSETS[index]
    return result.astype(np.float32)


class BodyMapper:
    """First-frame-neutral calibrated mapping to GearSonic link targets."""

    def __init__(self) -> None:
        self._neck_inverse: np.ndarray | None = None
        self._position_offsets: np.ndarray | None = None
        self._rotation_offsets: np.ndarray | None = None

    @property
    def is_calibrated(self) -> bool:
        return self._neck_inverse is not None

    def reset(self) -> None:
        self._neck_inverse = None
        self._position_offsets = None
        self._rotation_offsets = None

    @staticmethod
    def _raw_components(raw_wxyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        positions = raw_wxyz[:, :3].astype(np.float64)
        rotations = np.stack(
            [_matrix_from_xyzw(row[[4, 5, 6, 3]]) for row in raw_wxyz]
        )
        return positions, rotations

    def calibrate(self, neutral_body: Any) -> None:
        """Capture a neutral frame and align it to GearSonic defaults."""

        raw = process_3pt_pose(neutral_body)
        positions, rotations = self._raw_components(raw)
        neck_inverse = rotations[2].T
        corrected_positions = (neck_inverse @ positions.T).T
        corrected_rotations = np.stack([neck_inverse @ rotation for rotation in rotations])

        target_rotations = np.stack(
            [_matrix_from_xyzw(value) for value in GEAR_SONIC_DEFAULT_ORIENTATIONS_XYZW]
        )
        self._neck_inverse = neck_inverse
        self._position_offsets = corrected_positions - GEAR_SONIC_DEFAULT_KEYPOINT_POSITIONS
        self._rotation_offsets = np.stack(
            [
                target_rotations[index] @ corrected_rotations[index].T
                for index in range(KEYPOINT_COUNT)
            ]
        )

    def map(self, body: Any) -> BodyTargets:
        """Map one body frame, using the first call as neutral calibration."""

        if not self.is_calibrated:
            self.calibrate(body)
        raw = process_3pt_pose(body)
        positions, rotations = self._raw_components(raw)
        assert self._neck_inverse is not None
        assert self._position_offsets is not None
        assert self._rotation_offsets is not None

        corrected_positions = (self._neck_inverse @ positions.T).T
        corrected_rotations = np.stack([self._neck_inverse @ rotation for rotation in rotations])
        keypoint_positions = corrected_positions - self._position_offsets
        keypoint_rotations = np.stack(
            [
                self._rotation_offsets[index] @ corrected_rotations[index]
                for index in range(KEYPOINT_COUNT)
            ]
        )

        keypoint_poses = np.empty((KEYPOINT_COUNT, 7), dtype=np.float32)
        link_poses = np.empty((KEYPOINT_COUNT, 7), dtype=np.float32)
        for index in range(KEYPOINT_COUNT):
            orientation = _xyzw_from_matrix(keypoint_rotations[index])
            keypoint_poses[index, :3] = keypoint_positions[index]
            keypoint_poses[index, 3:] = orientation
            rotated_offset = (
                keypoint_rotations[index] @ GEAR_SONIC_BODY_OFFSETS[index]
            )
            link_poses[index, :3] = keypoint_positions[index] - rotated_offset
            link_poses[index, 3:] = orientation
        return BodyTargets(keypoint_poses_xyzw=keypoint_poses, link_poses_xyzw=link_poses)

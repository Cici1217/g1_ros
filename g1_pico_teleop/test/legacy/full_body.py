"""Historical full-body mapping reference; test-only, never installed.

The math mirrors the full-body ``PoseStreamer`` path in
``GR00T-WholeBodyControl``.  It is implemented with NumPy/SciPy so the ROS 2
adapter does not import the reference project or its PyTorch environment at
runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation


BODY_SHAPE = (24, 7)

# XRoboToolkit body hierarchy used by pico_manager_thread_server.PoseStreamer.
_XRT_PARENTS = np.asarray(
    [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 22],
    dtype=np.int64,
)

# The first 22 SMPL-H joints plus the two thumb-tip chains used by
# compute_human_joints(..., use_thumb_joints=True).  These rest positions are
# from gear_sonic/data/human/human_joints_info.pkl in the reference project.
_SMPLH_PARENTS_22 = np.asarray(
    [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19],
    dtype=np.int64,
)
_SMPLH_REST_22 = np.asarray(
    [
        [0.0031232606, -0.3514074683, 0.0120365508],
        [0.0613126531, -0.4441709518, -0.0139646353],
        [-0.0601442158, -0.4553154707, -0.0092138201],
        [0.0003605621, -0.2415168583, -0.0155810807],
        [0.1160081103, -0.8229243755, -0.0233606994],
        [-0.1043541729, -0.8176955581, -0.0260377023],
        [0.0098082609, -0.1096636057, -0.0215210654],
        [0.0725546628, -1.2259838581, -0.0552366450],
        [-0.0889373645, -1.2284233570, -0.0462299734],
        [-0.0015221529, -0.0574284494, 0.0069258320],
        [0.1198119670, -1.2839812040, 0.0629796833],
        [-0.1277497709, -1.2867517471, 0.0728190243],
        [-0.0136866113, 0.1077386066, -0.0246895105],
        [0.0448420048, 0.0275152735, -0.0002946509],
        [-0.0492170788, 0.0269102231, -0.0064740698],
        [0.0110968733, 0.2681904137, -0.0039522452],
        [0.1640810370, 0.0852432996, -0.0157555901],
        [-0.1517948210, 0.0804346725, -0.0191425979],
        [0.4182038903, 0.0130927814, -0.0582144447],
        [-0.4229443669, 0.0439421907, -0.0456096828],
        [0.6701906323, 0.0363140106, -0.0606865250],
        [-0.6722118258, 0.0394096449, -0.0609348677],
    ],
    dtype=np.float64,
)
_THUMB_CHAINS = (
    (
        20,
        np.asarray(
            [
                [0.7108263969, 0.0183372851, -0.0350756459],
                [0.7278420925, 0.0193130989, -0.0100975055],
                [0.7483652234, 0.0141535439, 0.0054255710],
            ],
            dtype=np.float64,
        ),
    ),
    (
        21,
        np.asarray(
            [
                [-0.7108249664, 0.0183352213, -0.0350735225],
                [-0.7278403044, 0.0193113182, -0.0100959428],
                [-0.7483659387, 0.0141541166, 0.0054256050],
            ],
            dtype=np.float64,
        ),
    ),
)


class FullBodyMappingError(ValueError):
    """Raised when a full-body frame cannot be converted safely."""


def _finite_body(value: Any) -> np.ndarray:
    try:
        body = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise FullBodyMappingError("body must be numeric") from exc
    if body.shape != BODY_SHAPE:
        raise FullBodyMappingError(f"body must have shape {BODY_SHAPE}, got {body.shape}")
    if not np.all(np.isfinite(body)):
        raise FullBodyMappingError("body contains NaN or infinity")
    norms = np.linalg.norm(body[:, 3:], axis=1)
    if np.any(norms <= 1e-8):
        raise FullBodyMappingError("body contains a zero quaternion")
    result = np.array(body, dtype=np.float64, copy=True)
    result[:, 3:] /= norms[:, None]
    return result


def _smpl_local_rotations(body: np.ndarray) -> tuple[np.ndarray, Rotation]:
    global_rotations = Rotation.from_quat(body[:, 3:]) * Rotation.from_euler(
        "y", 180.0, degrees=True
    )
    global_matrices = global_rotations.as_matrix()
    local_matrices = np.empty_like(global_matrices)
    for index, parent in enumerate(_XRT_PARENTS):
        if parent < 0:
            local_matrices[index] = global_matrices[index]
        else:
            local_matrices[index] = global_matrices[parent].T @ global_matrices[index]
    local_rotvec = Rotation.from_matrix(local_matrices).as_rotvec()
    return local_rotvec, global_rotations


def _forward_smpl_joints(
    smpl_pose: np.ndarray, root_rotation: Rotation
) -> np.ndarray:
    local_matrices = np.empty((22, 3, 3), dtype=np.float64)
    local_matrices[0] = root_rotation.as_matrix()
    local_matrices[1:] = Rotation.from_rotvec(smpl_pose.reshape(21, 3)).as_matrix()

    world_rotations = np.empty_like(local_matrices)
    world_positions = np.empty((22, 3), dtype=np.float64)
    for index, parent in enumerate(_SMPLH_PARENTS_22):
        if parent < 0:
            world_rotations[index] = local_matrices[index]
            world_positions[index] = _SMPLH_REST_22[index]
            continue
        offset = _SMPLH_REST_22[index] - _SMPLH_REST_22[parent]
        world_rotations[index] = world_rotations[parent] @ local_matrices[index]
        world_positions[index] = world_positions[parent] + world_rotations[parent] @ offset

    output = [world_positions[index] for index in range(22)]
    for wrist_index, rest_chain in _THUMB_CHAINS:
        parent_position = world_positions[wrist_index]
        parent_rotation = world_rotations[wrist_index]
        previous_rest = _SMPLH_REST_22[wrist_index]
        for rest_position in rest_chain:
            parent_position = parent_position + parent_rotation @ (rest_position - previous_rest)
            previous_rest = rest_position
        output.append(parent_position)
    return np.asarray(output, dtype=np.float64)


def _swing_euler_xyz(rotation_aa: np.ndarray) -> np.ndarray:
    rotation = Rotation.from_rotvec(rotation_aa)
    quaternion_xyzw = rotation.as_quat()
    quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]]
    twist_wxyz = np.asarray([quaternion_wxyz[0], 0.0, quaternion_wxyz[2], 0.0])
    norm = float(np.linalg.norm(twist_wxyz))
    if norm <= 1e-8:
        twist_wxyz = np.asarray([1.0, 0.0, 0.0, 0.0])
    else:
        twist_wxyz /= norm
    twist = Rotation.from_quat(twist_wxyz[[1, 2, 3, 0]])
    swing = twist.inv() * rotation
    return swing.as_euler("XYZ", degrees=False)


def _joint_position_hints(smpl_pose: np.ndarray) -> np.ndarray:
    body_pose = smpl_pose.reshape(21, 3)
    left_elbow_swing = _swing_euler_xyz(body_pose[17])
    right_elbow_swing = _swing_euler_xyz(body_pose[18])
    left_wrist = Rotation.from_rotvec(body_pose[19]).as_euler("XYZ", degrees=False)
    right_wrist = Rotation.from_rotvec(body_pose[20]).as_euler("XYZ", degrees=False)

    result = np.zeros(29, dtype=np.float64)
    result[23] = left_elbow_swing[0] + left_wrist[0]
    result[25] = left_wrist[1]
    result[27] = left_elbow_swing[2] + left_wrist[2]
    result[24] = -(right_elbow_swing[0] + right_wrist[0])
    result[26] = -right_wrist[1]
    result[28] = right_elbow_swing[2] + right_wrist[2]
    return result


@dataclass(frozen=True)
class FullBodyTarget:
    """One processed frame matching ``gear_sonic_interfaces/SmplMotion``."""

    smpl_pose: np.ndarray
    smpl_joints: np.ndarray
    body_quat_xyzw: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray


def map_full_body(body: Any) -> FullBodyTarget:
    """Convert one finite XR body frame using the reference SONIC algorithm."""

    source = _finite_body(body)
    local_rotvec, _ = _smpl_local_rotations(source)
    smpl_pose = local_rotvec[1:22].reshape(63)

    root_y_up = Rotation.from_rotvec(local_rotvec[0])
    root_z_up = Rotation.from_rotvec([np.pi / 2.0, 0.0, 0.0]) * root_y_up
    joints_world = _forward_smpl_joints(smpl_pose, root_z_up)

    # remove_smpl_base_rot(): q_out = q_z_up * conjugate([.5,.5,.5,.5] wxyz)
    base_inverse = Rotation.from_quat([-0.5, -0.5, -0.5, 0.5])
    body_rotation = root_z_up * base_inverse
    joints_local = body_rotation.inv().apply(joints_world)

    return FullBodyTarget(
        smpl_pose=smpl_pose.astype(np.float64),
        smpl_joints=joints_local.reshape(72).astype(np.float64),
        body_quat_xyzw=body_rotation.as_quat().astype(np.float64),
        joint_pos=_joint_position_hints(smpl_pose),
        joint_vel=np.zeros(29, dtype=np.float64),
    )

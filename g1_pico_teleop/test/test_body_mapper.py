import numpy as np

from legacy.body_mapper import (
    BodyMapper,
    GEAR_SONIC_BODY_OFFSETS,
    GEAR_SONIC_DEFAULT_KEYPOINT_POSITIONS,
    gear_sonic_keypoints_from_links,
    process_3pt_pose,
)


def neutral_body() -> np.ndarray:
    body = np.zeros((24, 7), dtype=np.float32)
    body[:, 6] = 1.0
    body[0, :3] = [0.1, 1.0, 0.2]
    body[22, :3] = [-0.25, 1.15, 0.45]
    body[23, :3] = [0.30, 1.15, 0.42]
    body[12, :3] = [0.10, 1.55, 0.25]
    return body


def test_neutral_calibration_maps_to_defaults_and_inverts_body_offset() -> None:
    mapper = BodyMapper()
    targets = mapper.map(neutral_body())

    assert mapper.is_calibrated
    np.testing.assert_allclose(
        targets.keypoint_poses_xyzw[:, :3],
        GEAR_SONIC_DEFAULT_KEYPOINT_POSITIONS,
        atol=2e-6,
    )
    reconstructed = gear_sonic_keypoints_from_links(targets.link_poses_xyzw)
    np.testing.assert_allclose(reconstructed, targets.keypoint_poses_xyzw, atol=2e-6)
    difference = targets.link_poses_xyzw[:, :3] - targets.keypoint_poses_xyzw[:, :3]
    assert np.max(np.abs(difference)) > 0.1
    np.testing.assert_allclose(np.linalg.norm(GEAR_SONIC_BODY_OFFSETS[2]), 0.35)


def test_pelvis_relative_mapping_ignores_global_unity_translation() -> None:
    mapper = BodyMapper()
    neutral = neutral_body()
    first = mapper.map(neutral)
    translated = neutral.copy()
    translated[:, :3] += [4.0, -3.0, 2.0]
    second = mapper.map(translated)
    np.testing.assert_allclose(second.keypoint_poses_xyzw, first.keypoint_poses_xyzw, atol=3e-6)


def test_process_3pt_pose_is_defensive_and_source_layout_is_wxyz() -> None:
    body = neutral_body()
    before = body.copy()
    raw = process_3pt_pose(body)
    assert raw.shape == (3, 7)
    np.testing.assert_array_equal(body, before)
    np.testing.assert_allclose(np.linalg.norm(raw[:, 3:], axis=1), 1.0, atol=1e-6)

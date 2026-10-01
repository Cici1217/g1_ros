import numpy as np
import pytest

from legacy.full_body import FullBodyMappingError, map_full_body


def neutral_body() -> np.ndarray:
    body = np.zeros((24, 7), dtype=np.float32)
    body[:, 6] = 1.0
    return body


def test_neutral_frame_matches_reference_smpl_conversion() -> None:
    target = map_full_body(neutral_body())

    np.testing.assert_allclose(target.smpl_pose, 0.0, atol=1e-7)
    np.testing.assert_allclose(target.joint_pos, 0.0, atol=1e-7)
    np.testing.assert_allclose(target.joint_vel, 0.0, atol=1e-7)
    np.testing.assert_allclose(
        target.body_quat_xyzw,
        [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
        atol=1e-7,
    )
    expected_selected_joints = np.asarray(
        [
            [-0.351407468, -0.003123261, 0.012036551],
            [-0.377408654, 0.055066132, -0.080726933],
            [-0.372657839, -0.066390737, -0.091871452],
            [-0.300464336, 0.113565446, -0.920537185],
            [-0.290624995, -0.133996292, -0.923307728],
            [-0.424130544, 0.663944111, 0.399758030],
            [-0.424378887, -0.678458347, 0.402853664],
            [-0.358018448, 0.742118702, 0.377597563],
            [-0.358018414, -0.754612460, 0.377598136],
        ]
    )
    selected = target.smpl_joints.reshape(24, 3)[[0, 1, 2, 10, 11, 20, 21, 22, 23]]
    np.testing.assert_allclose(selected, expected_selected_joints, atol=2e-6)


def test_full_body_mapping_does_not_mutate_input() -> None:
    body = neutral_body()
    body[:, :3] = np.arange(72, dtype=np.float32).reshape(24, 3)
    before = body.copy()

    target = map_full_body(body)

    np.testing.assert_array_equal(body, before)
    assert target.smpl_pose.shape == (63,)
    assert target.smpl_joints.shape == (72,)
    assert target.body_quat_xyzw.shape == (4,)
    assert target.joint_pos.shape == (29,)
    assert target.joint_vel.shape == (29,)


def test_full_body_mapping_rejects_invalid_quaternion() -> None:
    body = neutral_body()
    body[7, 3:] = 0.0
    with pytest.raises(FullBodyMappingError, match="zero quaternion"):
        map_full_body(body)

import numpy as np

from drone_vo_lab.benchmark import perturb
from drone_vo_lab.scene import Sequence


def fixture_sequence():
    count = 20
    return Sequence(
        np.arange(count, dtype=np.uint8)[:, None, None, None]
        * np.ones((count, 8, 8, 3), dtype=np.uint8),
        np.arange(count, dtype=np.float32)[:, None, None]
        * np.ones((count, 8, 8), dtype=np.float32),
        np.arange(count) / 15,
        np.repeat(np.eye(4)[None], count, axis=0),
        np.eye(3),
        "warehouse",
        0,
    )


def test_frame_drop_preserves_rgb_depth_timestamp_correspondence():
    sequence = fixture_sequence()
    indices, images = perturb(sequence, "frame_drop", 4)
    np.testing.assert_array_equal(indices, [0, 4, 8, 12, 16])
    np.testing.assert_array_equal(images[:, 0, 0, 0], sequence.depth[indices, 0, 0])
    np.testing.assert_allclose(np.diff(sequence.timestamps[indices]), 4 / 15)


def test_blackout_does_not_modify_source_sequence_or_depth():
    sequence = fixture_sequence()
    original = sequence.rgb.copy()
    indices, images = perturb(sequence, "blackout", 2)
    np.testing.assert_array_equal(sequence.rgb, original)
    assert not images[8:11].any()
    np.testing.assert_array_equal(images[11:], original[11:])
    np.testing.assert_array_equal(sequence.depth[:, 0, 0], np.arange(20))
    np.testing.assert_array_equal(indices, np.arange(20))

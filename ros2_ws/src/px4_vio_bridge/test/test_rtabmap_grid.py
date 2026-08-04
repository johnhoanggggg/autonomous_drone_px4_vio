import numpy as np
import pytest
from builtin_interfaces.msg import Time

from px4_vio_bridge.rtabmap_grid import decode_grid_image, make_occupancy_grid


def test_display_encoding_is_inverted_and_rows_are_unflipped():
    # Top display row becomes the last/high-Y ROS row.
    grid = decode_grid_image(np.array([[89, 178], [0, 200]], dtype=np.uint8))
    assert grid.tolist() == [[100, 0], [-1, 0]]


def test_probability_grays_are_inverted():
    # Approximate display grays produced from occupancy probabilities 75 and 25.
    grid = decode_grid_image(np.array([[44, 133]], dtype=np.uint8))
    assert grid.tolist() == [[75, 25]]


@pytest.mark.parametrize("values", [np.array([[179]]), np.array([[255]]), np.zeros((2, 2, 2))])
def test_invalid_grid_is_rejected(values):
    with pytest.raises(ValueError):
        decode_grid_image(values)


def test_ros_message_geometry_and_row_major_data():
    stamp = Time(sec=12, nanosec=34)
    msg = make_occupancy_grid(
        np.array([[89, 178], [0, 200]], dtype=np.uint8),
        min_x=-1.5,
        min_y=2.0,
        resolution=0.1,
        stamp=stamp,
    )
    assert msg.header.frame_id == "world"
    assert msg.info.width == 2
    assert msg.info.height == 2
    assert msg.info.origin.position.x == -1.5
    assert msg.info.origin.position.y == 2.0
    assert msg.info.origin.orientation.w == 1.0
    assert list(msg.data) == [100, 0, -1, 0]

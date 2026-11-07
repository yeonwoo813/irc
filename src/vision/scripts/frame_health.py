"""Small completion reports; no image copies or object-detection requirement."""

import time


def frame_health(images, received_monotonic, received_ros_ns, reset_token):
    """Report original image age including conversion/inference/publication.

    All vision and decision processes run on this host. ROS time is used only
    to measure image age on entry; elapsed work and report transport use the
    host's monotonic clock, even if wall time changes during inference.
    """
    completed = time.monotonic()
    stamps = [
        int(image.header.stamp.sec) * 1_000_000_000
        + int(image.header.stamp.nanosec)
        for image in images
    ]
    return {
        'reset_token': reset_token,
        'frame_stamps_ns': stamps,
        'frame_ages_sec': [
            (received_ros_ns - stamp) / 1_000_000_000
            + completed - received_monotonic
            for stamp in stamps
        ],
        'received_monotonic': received_monotonic,
        'completed_monotonic': completed,
    }

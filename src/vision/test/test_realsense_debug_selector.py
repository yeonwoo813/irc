from pathlib import Path
import sys
from types import SimpleNamespace

from std_msgs.msg import Bool


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from realsense_debug_selector import RealSenseDebugSelector  # noqa: E402


def test_ball_to_hoop_switch_clears_stale_ball_image_and_shows_hoop():
    stale_ball_image = object()
    hoop_image = object()
    published = []
    harness = SimpleNamespace(
        ball_enabled=True,
        hoop_enabled=False,
        latest_ball_image=stale_ball_image,
        latest_hoop_image=None,
        _publish_and_show=published.append,
    )
    harness._active_source = lambda: RealSenseDebugSelector._active_source(
        harness
    )

    RealSenseDebugSelector.cb_ball_active(harness, Bool(data=False))
    RealSenseDebugSelector.cb_hoop_active(harness, Bool(data=True))
    RealSenseDebugSelector.cb_hoop_image(harness, hoop_image)

    assert harness.latest_ball_image is None
    assert harness.latest_hoop_image is hoop_image
    assert published == [hoop_image]


def test_default_mode_does_not_let_an_old_ball_image_block_hoop():
    stale_ball_image = object()
    hoop_image = object()
    published = []
    harness = SimpleNamespace(
        latest_ball_image=stale_ball_image,
        latest_hoop_image=None,
        _active_source=lambda: "default",
        _publish_and_show=published.append,
    )

    RealSenseDebugSelector.cb_hoop_image(harness, hoop_image)

    assert published == [hoop_image]

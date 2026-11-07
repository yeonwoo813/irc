"""Require continuous fresh webcam and RGB/depth inference before starting."""

import math
from dataclasses import dataclass


@dataclass
class _Stream:
    stamps: tuple = ()
    completed: float = None
    arrival: float = None
    valid_until: float = None


class VisionStartupGate:
    """Watch completion evidence from the same host, independently of ROS time."""

    IMAGE_COUNTS = {'webcam': 1, 'realsense': 2}

    def __init__(self, stable_sec=3.0, max_age_sec=0.5):
        for value in (stable_sec, max_age_sec):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Startup vision durations must be positive and finite')
        self.stable_sec = stable_sec
        self.max_age_sec = max_age_sec
        self.reset('', 0.0)

    def reset(self, token, now):
        """Discard evidence collected before the current preparation finished."""
        self.token = token
        self.not_before = now
        self.streams = {name: _Stream() for name in self.IMAGE_COUNTS}
        self.stable_since = None

    def invalidate(self, source):
        self.streams[source].valid_until = None
        self.stable_since = None

    def _all_fresh(self, now):
        return all(
            stream.valid_until is not None and now < stream.valid_until
            for stream in self.streams.values()
        )

    def ready(self, now):
        if not self._all_fresh(now):
            self.stable_since = None
        return (
            self.stable_since is not None
            and now - self.stable_since >= self.stable_sec
        )

    def observe(self, source, report, now):
        """Accept only new successful inferences of recent original images."""
        self.ready(now)  # A late callback must not hide an intervening gap.
        try:
            if report['reset_token'] != self.token:
                return False
            stamps = tuple(report['frame_stamps_ns'])
            ages = tuple(float(age) for age in report['frame_ages_sec'])
            received = float(report['received_monotonic'])
            completed = float(report['completed_monotonic'])
            if (
                len(stamps) != self.IMAGE_COUNTS[source]
                or len(ages) != len(stamps)
                or any(type(stamp) is not int or stamp <= 0 for stamp in stamps)
                or not all(math.isfinite(value) for value in (*ages, received, completed))
            ):
                raise ValueError('Invalid frame health report')
        except (KeyError, TypeError, ValueError, OverflowError):
            self.invalidate(source)
            return False

        # In-flight reports from before the reset ACK cannot satisfy this gate.
        if received < self.not_before:
            return False

        stream = self.streams[source]
        advancing = (
            not stream.stamps
            or all(stamp > old for stamp, old in zip(stamps, stream.stamps))
        )
        ordered = stream.completed is None or completed > stream.completed
        if (
            stream.completed is not None
            and (completed - stream.completed >= self.max_age_sec
                 or now - stream.arrival >= self.max_age_sec)
        ):
            self.stable_since = None
        # Rebaseline image stamps after a camera/clock restart; still require
        # a completely new continuous window after this invalid report.
        stream.stamps = stamps
        stream.completed = completed
        stream.arrival = now
        transport_age = now - completed
        if (
            not advancing or not ordered
            or not self.not_before <= received <= completed <= now
            or completed - received >= self.max_age_sec
            or transport_age >= self.max_age_sec
            or any(age < -0.05 or age + transport_age >= self.max_age_sec
                   for age in ages)
        ):
            self.invalidate(source)
            return False

        stream.valid_until = completed + self.max_age_sec - max(0.0, *ages)
        if self.stable_since is None and self._all_fresh(now):
            self.stable_since = now
        return True

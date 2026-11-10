"""Ignore new hurdles for 30 seconds from a hurdle approach's start state."""

from collections import OrderedDict


HURDLE_APPROACH_MOTIONS = frozenset((26, 42, 43, 44))
HURDLE_DETECTION_BLOCK_SEC = 30.0


class HurdleDetectionBlock:
    def __init__(self):
        self.deadline = 0.0
        self._started_requests = OrderedDict()

    def observe(self, state, received_at):
        """Return true only for the first running state of an approach."""
        motion_id = getattr(state, 'motion_id', -1)
        request_id = getattr(state, 'request_id', '')
        if (
            motion_id not in HURDLE_APPROACH_MOTIONS
            or state.motion_end
            or not state.motion_ready
            or not request_id
        ):
            return False
        key = (motion_id, request_id)
        if key in self._started_requests:
            return False
        self._started_requests[key] = None
        if len(self._started_requests) > 64:
            self._started_requests.popitem(last=False)
        self.deadline = received_at + HURDLE_DETECTION_BLOCK_SEC
        return True

    def active(self, now):
        return now < self.deadline

    def remaining(self, now):
        return max(0.0, self.deadline - now)

    def reset(self):
        self.deadline = 0.0
        self._started_requests.clear()

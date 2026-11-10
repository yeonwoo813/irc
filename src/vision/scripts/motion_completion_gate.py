"""Correlate a vision phase with one execution, independent of topic order."""

from collections import OrderedDict


class MotionCompletionGate:
    def __init__(self, history_size=64):
        self._history_size = history_size
        self._completed = OrderedDict()
        self._commands = OrderedDict()
        self._expected = None

    @staticmethod
    def _key(motion_id, request_id):
        if not isinstance(request_id, str) or not request_id:
            return None
        return int(motion_id), request_id

    def _remember(self, history, key):
        history[key] = None
        history.move_to_end(key)
        while len(history) > self._history_size:
            history.popitem(last=False)

    def begin(self, command):
        """Arm once per request. An unidentified command cannot complete."""
        key = self._key(command.command, getattr(command, 'request_id', ''))
        if key is not None:
            if key in self._commands:
                return False
            self._remember(self._commands, key)
        self._expected = key
        return True

    def has_seen(self, command):
        return self._key(
            command.command, getattr(command, 'request_id', ''),
        ) in self._commands

    def observe(self, state):
        # Keep early completions: the command subscription may run later.
        # A matching terminal state is sufficient; observing the start first
        # is unnecessary and would introduce a new wait if that sample is lost.
        key = self._key(
            getattr(state, 'motion_id', -1), getattr(state, 'request_id', ''),
        )
        if key is not None and state.motion_end and state.motion_ready:
            self._remember(self._completed, key)

    def consume(self):
        if self._expected is None or self._expected not in self._completed:
            return False
        self._expected = None
        return True

    def cancel(self):
        self._expected = None

    def reset(self):
        self.cancel()
        self._completed.clear()
        self._commands.clear()

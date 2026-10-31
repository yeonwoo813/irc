"""Optional terminal polling evidence; does not read or consume input itself."""

from collections import deque
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import termios
import time


class InputLatencyTrace:
    """Keep recent poll timings in memory and write only terminal/Enter events."""

    WINDOW_SECONDS = 10.0

    def __init__(self, directory):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%dT%H%M%S_%f')
        self.path = directory / f'mission_input_{stamp}_{os.getpid()}.jsonl'
        self._file = self.path.open('x', encoding='utf-8')
        self._polls = deque(maxlen=512)
        self._last_poll_at = None
        self._last_empty_at = None

    def record(self, event, **fields):
        if self._file is None:
            return
        try:
            self._file.write(json.dumps({
                'event': event,
                'recorded_at': datetime.now().astimezone().isoformat(),
                'recorded_monotonic': time.monotonic(),
                **fields,
            }, ensure_ascii=False) + '\n')
            self._file.flush()
        except OSError:
            # An optional diagnostic must never prevent a mission request.
            self.close()

    @staticmethod
    def terminal_state(fd):
        try:
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            attrs = termios.tcgetattr(fd)
            foreground = os.tcgetpgrp(fd)
            return {
                'device': os.ttyname(fd),
                'nonblocking': bool(flags & os.O_NONBLOCK),
                'canonical': bool(attrs[3] & termios.ICANON),
                'software_flow_control': bool(attrs[0] & termios.IXON),
                'process_group': os.getpgrp(),
                'foreground_process_group': foreground,
                'in_foreground': foreground == os.getpgrp(),
            }
        except (OSError, termios.error) as exc:
            return {'error': str(exc)}

    def note_poll(self, started_at, finished_at, *, empty):
        gap = (
            started_at - self._last_poll_at
            if self._last_poll_at is not None else None
        )
        self._polls.append((started_at, finished_at, gap))
        self._last_poll_at = started_at
        if empty:
            self._last_empty_at = finished_at

    def input_snapshot(self, read_at):
        recent = [
            poll for poll in self._polls
            if poll[1] >= read_at - self.WINDOW_SECONDS
        ]
        gaps = [poll[2] for poll in recent if poll[2] is not None]
        last = recent[-1] if recent else None
        return {
            # This is a read timestamp, never the physical keypress timestamp.
            'read_monotonic': read_at,
            'window_seconds': self.WINDOW_SECONDS,
            'poll_count': len(recent),
            'last_empty_read_monotonic': self._last_empty_at,
            'last_empty_read_age_ms': (
                (read_at - self._last_empty_at) * 1000
                if self._last_empty_at is not None else None
            ),
            'last_poll_gap_ms': (
                last[2] * 1000 if last and last[2] is not None else None
            ),
            'max_poll_gap_ms': max(gaps) * 1000 if gaps else None,
            'read_duration_ms': (last[1] - last[0]) * 1000 if last else None,
            'max_read_duration_ms': (
                max(poll[1] - poll[0] for poll in recent) * 1000
                if recent else None
            ),
        }

    def close(self):
        if self._file is not None:
            try:
                self._file.close()
            except OSError:
                pass
            self._file = None

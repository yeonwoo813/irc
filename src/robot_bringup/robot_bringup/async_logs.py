"""Drain each node's output independently of launch/terminal logging speed.

The node keeps its PID, signals and stdin. A helper reads its stdout/stderr
into a bounded queue; only the helper's writer can block on output. ROS's
synchronous file and rosout sinks are disabled by quiet_process in this mode.
Launch still records forwarded output in launch.log when output='both'.
"""

import argparse
from collections import deque
import os
import selectors
import subprocess
import sys
import threading
import time


MAX_RECORDS = 512
MAX_BYTES = 1024 * 1024
MAX_LINE_BYTES = 64 * 1024
EXIT_FLUSH_SECONDS = 0.5
CONSOLE_ONLY_ROS_ARGS = [
    '--ros-args', '--disable-rosout-logs', '--disable-external-lib-logs',
]


class LogQueue:
    """Drop the oldest pending records instead of waiting for free space."""

    def __init__(self, max_records=MAX_RECORDS, max_bytes=MAX_BYTES):
        if max_records < 1 or max_bytes < 1:
            raise ValueError('Log queue limits must be positive')
        self.max_records = max_records
        self.max_bytes = max_bytes
        self.records = deque()
        self.byte_count = 0
        self.dropped = 0
        self.closed = False
        self.condition = threading.Condition()

    def put(self, stream, data):
        # Also bound a single oversized record. Normal lines are already
        # limited by LineBuffer, including streams which never send a newline.
        data = data[:self.max_bytes]
        with self.condition:
            if self.closed:
                return
            while self.records and (
                len(self.records) >= self.max_records
                or self.byte_count + len(data) > self.max_bytes
            ):
                _, old = self.records.popleft()
                self.byte_count -= len(old)
                self.dropped += 1
            self.records.append((stream, data))
            self.byte_count += len(data)
            self.condition.notify()

    def get(self):
        with self.condition:
            while not self.records and not self.closed:
                self.condition.wait()
            if not self.records:
                return None
            stream, data = self.records.popleft()
            self.byte_count -= len(data)
            return stream, data

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class LineBuffer:
    """Frame byte streams into records without unbounded partial lines."""

    def __init__(self, stream, queue):
        self.stream = stream
        self.queue = queue
        self.pending = bytearray()
        self.discarding = False

    def feed(self, chunk):
        for part in chunk.splitlines(keepends=True):
            ended = part.endswith((b'\n', b'\r'))
            if not self.discarding:
                remaining = MAX_LINE_BYTES - len(self.pending)
                self.pending.extend(part[:remaining])
                if len(part) > remaining:
                    self.pending.extend(b' [log line truncated]\n')
                    self.flush()
                    self.discarding = True
                elif ended:
                    self.flush()
            if ended:
                self.discarding = False

    def flush(self):
        if self.pending:
            self.queue.put(self.stream, bytes(self.pending))
            self.pending.clear()


def write_records(queue):
    """Never hold the queue lock during potentially blocking output I/O."""
    broken = set()
    while True:
        record = queue.get()
        if record is None:
            return
        stream, data = record
        if stream in broken:
            continue
        view = memoryview(data)
        try:
            while view:
                written = os.write(stream, view)
                if written == 0:
                    raise BrokenPipeError('Log output closed')
                view = view[written:]
        except OSError:
            # A closed terminal must not stop draining the robot's pipes.
            broken.add(stream)


def relay(stdout_fd, stderr_fd, parent_pid):
    queue = LogQueue()
    writer = threading.Thread(target=write_records, args=(queue,), daemon=True)
    writer.start()
    with selectors.DefaultSelector() as selector:
        for fd, stream in ((stdout_fd, 1), (stderr_fd, 2)):
            os.set_blocking(fd, False)
            selector.register(fd, selectors.EVENT_READ, LineBuffer(stream, queue))
        orphan_deadline = None
        try:
            while selector.get_map():
                # Descendants can inherit a pipe. Do not keep launch's output
                # descriptors alive indefinitely after the actual node exits.
                if os.getppid() != parent_pid:
                    if orphan_deadline is None:
                        orphan_deadline = time.monotonic() + EXIT_FLUSH_SECONDS
                    elif time.monotonic() >= orphan_deadline:
                        break
                for key, _ in selector.select(timeout=0.05):
                    try:
                        chunk = os.read(key.fd, 8192)
                    except BlockingIOError:
                        continue
                    if chunk:
                        key.data.feed(chunk)
                    else:
                        key.data.flush()
                        selector.unregister(key.fd)
                        os.close(key.fd)
        finally:
            for key in list(selector.get_map().values()):
                key.data.flush()
                selector.unregister(key.fd)
                os.close(key.fd)
            queue.close()
            # Even shutdown cannot wait indefinitely for a blocked terminal.
            writer.join(timeout=EXIT_FLUSH_SECONDS)


def start_relay():
    """Redirect output before exec/runpy, preserving the node's original PID."""
    stdout_read, stdout_write = os.pipe()
    stderr_read, stderr_write = os.pipe()
    try:
        subprocess.Popen(
            [sys.executable, '-m', 'robot_bringup.async_logs',
             '--stdout-fd', str(stdout_read), '--stderr-fd', str(stderr_read),
             '--parent-pid', str(os.getpid())],
            stdin=subprocess.DEVNULL,
            pass_fds=(stdout_read, stderr_read),
            # Helpers drain EOF and exit independently when launch signals the
            # original node. close_fds prevents inheriting the camera lock.
            start_new_session=True,
        )
        os.dup2(stdout_write, 1)
        os.dup2(stderr_write, 2)
    finally:
        for fd in (stdout_read, stdout_write, stderr_read, stderr_write):
            os.close(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stdout-fd', type=int, required=True)
    parser.add_argument('--stderr-fd', type=int, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    args = parser.parse_args()
    relay(args.stdout_fd, args.stderr_fd, args.parent_pid)


if __name__ == '__main__':
    main()

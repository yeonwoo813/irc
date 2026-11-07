"""Best-effort fixed-size checkpoints; never wait for the diagnostic reader.

No pixels, files, ROS calls, formatting or locks on the inference path. If the
reader dies or its socket fills, discard the checkpoint and count the loss.
"""

import os
import socket
import struct
import time


SOCKET_ENV = 'IRC_VISION_TRACE_SOCKET'
STAGES = (
    'model_load', 'model_ready', 'raw_color', 'raw_depth', 'callback',
    'predict', 'predicted', 'publish', 'preview', 'complete', 'error',
    'inactive', 'throttled', 'process_start',
)
SOURCES = ('webcam', 'realsense', 'webcam_driver', 'realsense_driver', 'decision', 'motion')
PACKET = struct.Struct('<4sIBB2xdqqQQ')
MAGIC = b'IVT1'


def image_stamp(image):
    return image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec


class TraceProbe:
    def __init__(self, source, address=None):
        self.address = os.environ.get(SOCKET_ENV, '') if address is None else address
        self.source = SOURCES.index(source)
        self.pid = os.getpid()
        self.sequence = 0
        self.dropped = 0
        self.stamps = (0, 0)
        self.socket = None
        if self.address:
            try:
                self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.socket.setblocking(False)
            except OSError:
                self.address = ''

    def mark(self, stage, stamp1=None, stamp2=0):
        if not self.address:
            return
        if stamp1 is not None:
            stamps = (stamp1, stamp2)
            # Raw callbacks must not overwrite the last synchronized pair.
            if stage == 'callback':
                self.stamps = stamps
        else:
            stamps = self.stamps
        self.sequence += 1
        try:
            packet = PACKET.pack(
                MAGIC, self.pid, self.source, STAGES.index(stage),
                time.monotonic(), *stamps, self.sequence, self.dropped,
            )
            self.socket.sendto(packet, self.address)
        except (OSError, ValueError, struct.error, AttributeError):
            self.dropped += 1

    def close(self):
        if self.socket is not None:
            self.socket.close()
        self.address = ''

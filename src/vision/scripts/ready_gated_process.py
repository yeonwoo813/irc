#!/usr/bin/env python3
"""Run a child process only while a ROS Bool readiness gate is open.

The child is stopped when the readiness publisher disappears or publishes
False.  Failed children are restarted with exponential backoff while the gate
remains open.  This is used to keep TensorRT engine initialization sequential
on memory-constrained Jetson systems.
"""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from typing import Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


TRUE_VALUES = {"1", "true", "yes", "on"}
FALSE_VALUES = {"0", "false", "no", "off"}


def parse_bool(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def next_backoff(current: float, maximum: float) -> float:
    return min(maximum, max(0.1, current * 2.0))


class ReadyGate(Node):
    def __init__(self, node_name: str, topic: str, enabled: bool) -> None:
        super().__init__(node_name)
        self.topic = topic
        self.enabled = enabled
        self.ready = not enabled
        self.publisher_count = 0 if enabled else 1

        if enabled:
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.subscription = self.create_subscription(
                Bool,
                topic,
                self._on_ready,
                qos,
            )
        else:
            self.subscription = None

    def _on_ready(self, message: Bool) -> None:
        self.ready = bool(message.data)

    def is_open(self) -> bool:
        if not self.enabled:
            return True

        self.publisher_count = self.count_publishers(self.topic)
        if self.publisher_count <= 0:
            # A TRANSIENT_LOCAL True sample must not keep the gate open after
            # the detector that published it has crashed.
            self.ready = False
        return self.ready and self.publisher_count > 0


def stop_child(
    child: subprocess.Popen,
    node: Node,
    graceful_seconds: float = 1.0,
) -> None:
    if child.poll() is not None:
        return

    node.get_logger().info(f"Stopping gated child pid={child.pid}")
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=max(0.1, graceful_seconds))
        return
    except subprocess.TimeoutExpired:
        pass

    child.terminate()
    try:
        child.wait(timeout=1.0)
        return
    except subprocess.TimeoutExpired:
        pass

    child.kill()
    child.wait(timeout=1.0)


def parse_arguments(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-name", default="ready_gated_process")
    parser.add_argument("--gate-topic", required=True)
    parser.add_argument("--gate-enabled", type=parse_bool, default=True)
    parser.add_argument("--source-enabled", type=parse_bool, default=True)
    parser.add_argument("--gate-open-delay-seconds", type=float, default=0.0)
    parser.add_argument("--initial-backoff-seconds", type=float, default=5.0)
    parser.add_argument("--max-backoff-seconds", type=float, default=30.0)
    parser.add_argument("--stable-reset-seconds", type=float, default=30.0)
    parser.add_argument(
        "--restart-on-clean-exit",
        type=parse_bool,
        default=False,
    )
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("a child command is required after --")
    return args


def run(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_arguments(argv)
    gate_enabled = bool(args.gate_enabled and args.source_enabled)
    initial_backoff = max(0.1, float(args.initial_backoff_seconds))
    maximum_backoff = max(initial_backoff, float(args.max_backoff_seconds))
    stable_reset = max(0.0, float(args.stable_reset_seconds))
    gate_open_delay = (
        max(0.0, float(args.gate_open_delay_seconds)) if gate_enabled else 0.0
    )

    rclpy.init(args=[])
    node = ReadyGate(args.node_name, args.gate_topic, gate_enabled)
    child: Optional[subprocess.Popen] = None
    child_started_at = 0.0
    next_start_at = 0.0
    backoff = initial_backoff
    gate_was_open = False
    gate_opened_at: Optional[float] = None

    try:
        if gate_enabled:
            node.get_logger().info(
                f"Waiting for {args.gate_topic}=true before starting child"
            )
        else:
            node.get_logger().info("Readiness gate bypassed")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            gate_open = node.is_open()

            if gate_open != gate_was_open:
                state = "OPEN" if gate_open else "CLOSED"
                node.get_logger().info(
                    f"Readiness gate {state}: {args.gate_topic}"
                )
                gate_opened_at = now if gate_open else None
                gate_was_open = gate_open

            if child is not None:
                return_code = child.poll()
                if return_code is not None:
                    runtime = max(0.0, now - child_started_at)
                    node.get_logger().warning(
                        "Gated child exited "
                        f"code={return_code} runtime={runtime:.1f}s"
                    )
                    child = None

                    if return_code == 0 and not args.restart_on_clean_exit:
                        return 0

                    if runtime >= stable_reset:
                        backoff = initial_backoff
                    next_start_at = now + backoff
                    node.get_logger().warning(
                        f"Retrying gated child in {backoff:.1f}s"
                    )
                    backoff = next_backoff(backoff, maximum_backoff)
                elif not gate_open:
                    stop_child(child, node)
                    child = None
                    next_start_at = 0.0
                    backoff = initial_backoff

            gate_delay_elapsed = (
                gate_opened_at is not None
                and now - gate_opened_at >= gate_open_delay
            )
            if (
                child is None
                and gate_open
                and gate_delay_elapsed
                and now >= next_start_at
            ):
                node.get_logger().info(
                    "Starting gated child: " + " ".join(args.command)
                )
                child = subprocess.Popen(args.command)
                child_started_at = time.monotonic()

        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if child is not None:
            stop_child(child, node, graceful_seconds=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()

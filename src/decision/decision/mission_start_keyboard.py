#!/usr/bin/env python3
"""Call MainDecision's mission-start service when Enter is pressed."""

import atexit
import os
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class MissionStartKeyboard(Node):
    def __init__(self):
        super().__init__('mission_start_keyboard')
        self.start_client = self.create_client(Trigger, '/mission/start')
        self.tty_fd = None
        self.original_terminal_settings = None
        self.pending_request = None
        self.mission_armed = False
        self.start_requested = False
        self.armed_screen_active = False
        self._open_terminal()
        self.poll_timer = self.create_timer(0.05, self._poll_keyboard)
        self.screen_timer = self.create_timer(0.25, self._refresh_armed_screen)
        armed_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.armed_sub = self.create_subscription(
            Bool,
            '/mission/armed',
            self._armed_callback,
            armed_qos,
        )
        atexit.register(self._restore_terminal)

    def _open_terminal(self):
        try:
            self.tty_fd = os.open(
                '/dev/tty',
                os.O_RDWR | os.O_NONBLOCK,
            )
            self.original_terminal_settings = termios.tcgetattr(self.tty_fd)
            tty.setcbreak(self.tty_fd, termios.TCSANOW)
        except (OSError, termios.error) as exc:
            self._restore_terminal()
            self.get_logger().error(
                'Cannot read launch-terminal keys from /dev/tty: '
                f'{exc}. Use: ros2 service call /mission/start '
                'std_srvs/srv/Trigger {}'
            )

    def _armed_callback(self, msg):
        self.mission_armed = bool(msg.data)
        if self.mission_armed and not self.start_requested:
            self._enter_armed_screen()
        else:
            self._leave_armed_screen()

    def _write_terminal(self, data):
        if self.tty_fd is None:
            return False
        try:
            os.write(self.tty_fd, data)
        except (BlockingIOError, OSError):
            return False
        return True

    def _enter_armed_screen(self):
        if self.tty_fd is None or self.armed_screen_active:
            return
        self.armed_screen_active = True
        self._write_terminal(b'\x1b[?1049h\x1b[?25l')
        self._draw_armed_prompt()

    def _draw_armed_prompt(self):
        if not self.armed_screen_active:
            return
        self._write_terminal(
            b'\x1b[2J\x1b[H[MISSION ARMED] Press Enter to start\r\n'
        )

    def _refresh_armed_screen(self):
        if (
            self.mission_armed
            and not self.start_requested
            and not self.armed_screen_active
        ):
            self._enter_armed_screen()
        elif self.armed_screen_active and not self.start_requested:
            self._draw_armed_prompt()

    def _leave_armed_screen(self):
        if not self.armed_screen_active:
            return
        self._write_terminal(b'\x1b[?25h\x1b[?1049l')
        self.armed_screen_active = False

    def _restore_terminal(self):
        self._leave_armed_screen()
        if (
            self.tty_fd is not None
            and self.original_terminal_settings is not None
        ):
            try:
                termios.tcsetattr(
                    self.tty_fd,
                    termios.TCSANOW,
                    self.original_terminal_settings,
                )
            except (OSError, termios.error):
                pass
        if self.tty_fd is not None:
            try:
                os.close(self.tty_fd)
            except OSError:
                pass
        self.tty_fd = None
        self.original_terminal_settings = None

    def _poll_keyboard(self):
        if self.tty_fd is None:
            return
        try:
            pressed = os.read(self.tty_fd, 64)
        except BlockingIOError:
            return
        except OSError as exc:
            self.get_logger().error(f'Keyboard read failed: {exc}')
            self._restore_terminal()
            return

        for key in pressed.decode(errors='ignore'):
            if key in ('\r', '\n'):
                self._request_mission_start()

    def _request_mission_start(self):
        if self.pending_request is not None and not self.pending_request.done():
            return
        if not self.start_client.service_is_ready():
            self.get_logger().warning(
                'Mission start service is not ready yet; press Enter again '
                'after [MISSION ARMED].'
            )
            return

        # 서비스가 응답하기 전에 원래 로그 화면으로 복귀시켜 MainDecision의
        # 시작 로그부터 빠짐없이 보이게 한다.
        self.start_requested = True
        self._leave_armed_screen()
        self.pending_request = self.start_client.call_async(Trigger.Request())
        self.pending_request.add_done_callback(self._start_response)

    def _start_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.start_requested = False
            self.get_logger().error(f'Mission start request failed: {exc}')
            if self.mission_armed:
                self._enter_armed_screen()
            return

        if response.success:
            self.mission_armed = False
            self.get_logger().info(f'[MISSION KEY] {response.message}')
        else:
            self.start_requested = False
            self.get_logger().warning(f'[MISSION KEY] {response.message}')
            if self.mission_armed:
                self._enter_armed_screen()

    def destroy_node(self):
        self._restore_terminal()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionStartKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

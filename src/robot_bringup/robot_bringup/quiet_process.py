"""Reversible logging suppression without changing robot startup or commands."""

import argparse
import logging
import os
from pathlib import Path
import runpy
import shlex
import sys


QUIET_ENV = 'IRC_QUIET_LOGS'
ROS_QUIET_ARGS = [
    '--ros-args', '--log-level', 'fatal',
    '--disable-stdout-logs', '--disable-rosout-logs',
    '--disable-external-lib-logs',
]


def quiet_enabled():
    return os.environ.get(QUIET_ENV, '').lower() in {'1', 'true', 'yes', 'on'}


def configure_launch_logging(context):
    from launch.substitutions import LaunchConfiguration

    value = LaunchConfiguration('quiet_logs', default='true').perform(context)
    quiet = value.lower() in {'1', 'true', 'yes', 'on'}
    context.environment[QUIET_ENV] = '1' if quiet else '0'
    # Launch itself also echoes every process start/stop and received log line.
    logging.disable(logging.CRITICAL if quiet else logging.NOTSET)
    return []


def process_prefix(kind):
    return [shlex.join([
        sys.executable, '-m', 'robot_bringup.quiet_process', kind, '--',
    ])]


def quiet_python_command(script, *args):
    return [
        sys.executable, '-m', 'robot_bringup.quiet_process', '--python', '--',
        sys.executable, script, *args,
    ]


def quiet_ros_args(args):
    return list(sys.argv if args is None else args) + ROS_QUIET_ARGS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--python', action='store_true')
    mode.add_argument('--ros', action='store_true')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    options = parser.parse_args(argv)
    command = options.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('process command required after --')

    if not quiet_enabled():
        os.execvp(command[0], command)

    # Native library prints and SDK stdout bypass ROS logging. Keep them out
    # of launch's pipes too. /dev/tty remains available for the Enter prompt.
    with open(os.devnull, 'wb') as sink:
        os.dup2(sink.fileno(), 1)
        os.dup2(sink.fileno(), 2)
    logging.disable(logging.CRITICAL)

    if options.ros:
        os.execvp(command[0], command + ROS_QUIET_ARGS)

    # Keep custom script arguments unchanged, including the readiness gate's
    # '-- child command'. Inject ROS options only when rclpy initializes.
    import rclpy

    original_init = rclpy.init

    def quiet_init(*, args=None, **kwargs):
        return original_init(args=quiet_ros_args(args), **kwargs)

    rclpy.init = quiet_init
    if Path(command[0]).name.startswith('python'):
        command = command[1:]
    script = command[0]
    sys.argv = command
    sys.path.insert(0, str(Path(script).resolve().parent))
    runpy.run_path(script, run_name='__main__')


if __name__ == '__main__':
    main()

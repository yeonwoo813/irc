#!/usr/bin/env python3
"""Control only neck ID 23. Stop robot_bringup/motion before using this port.

Examples (run from /home/rnd/irc):
    python3 tools/neck_motor.py --goal 2735 --duration 1.0
    python3 tools/neck_motor.py --hold
    python3 tools/neck_motor.py --read
    python3 tools/neck_motor.py --off

Successful goal/hold commands leave torque ON after exit. On interruption or
failure after a movement write, attempt to turn only neck torque OFF.
Raw register limits do not establish the robot's mechanical clearance.
Uses the port, baud rate and register layout in motion/include/dynamixel.hpp.
Position-limit register reference:
https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#minmax-position-limit52-48
"""

import argparse
import math
import sys
import time


PORT = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAKRLQ9-if00-port0'
BAUDRATE = 2000000
NECK_ID = 23
OPERATING_MODE = 11
MAX_POSITION = 48
MIN_POSITION = 52
TORQUE_ENABLE = 64
GOAL_POSITION = 116
PRESENT_POSITION = 132


def raw_position(value):
    raw = int(value)
    if not 0 <= raw <= 4095:
        raise argparse.ArgumentTypeError('raw position must be in 0..4095')
    return raw


def positive_duration(value):
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError('duration must be finite and > 0')
    return duration


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--goal', type=raw_position, help='target raw position; leave torque ON')
    action.add_argument('--hold', action='store_true', help='torque ON at current position')
    action.add_argument('--read', action='store_true', help='read position and torque; no writes')
    action.add_argument('--off', action='store_true', help='torque OFF for ID 23 only')
    parser.add_argument('--duration', type=positive_duration, default=1.0,
                        help='target interpolation time in seconds (default: 1.0)')
    parser.add_argument('--port', default=PORT)
    return parser.parse_args(argv)


def control_neck(packet, port, args):
    """All requests explicitly address ID 23; never use broadcast packets."""
    def check(result, error):
        if result != 0:  # DYNAMIXEL SDK COMM_SUCCESS
            raise RuntimeError(packet.getTxRxResult(result))
        if error:
            raise RuntimeError(packet.getRxPacketError(error))

    def read(address, size):
        method = packet.read1ByteTxRx if size == 1 else packet.read4ByteTxRx
        value, result, error = method(port, NECK_ID, address)
        check(result, error)
        return value

    def write(address, value, size):
        method = packet.write1ByteTxRx if size == 1 else packet.write4ByteTxRx
        check(*method(port, NECK_ID, address, value))

    if args.off:
        write(TORQUE_ENABLE, 0, 1)
        print('ID=23 torque=OFF')
        return

    present = read(PRESENT_POSITION, 4)
    if present >= 2**31:
        present -= 2**32
    if args.read:
        torque = read(TORQUE_ENABLE, 1)
        print(f'ID=23 present_raw={present} torque={"ON" if torque else "OFF"}')
        return

    mode = read(OPERATING_MODE, 1)
    if mode != 3:
        raise RuntimeError(f'ID=23 operating mode is {mode}; position mode 3 is required')
    minimum = read(MIN_POSITION, 4)
    maximum = read(MAX_POSITION, 4)
    goal = present if args.hold else args.goal
    if not 0 <= minimum <= maximum <= 4095:
        raise RuntimeError(f'Invalid position limits: {minimum}..{maximum}')
    for label, value in [('present', present), ('goal', goal)]:
        if not minimum <= value <= maximum:
            raise RuntimeError(
                f'{label} raw={value} is outside motor limits {minimum}..{maximum}; no writes sent')

    try:
        # Seed the current position before enabling torque, as StartNeckMotion does.
        write(GOAL_POSITION, present, 4)
        write(TORQUE_ENABLE, 1, 1)
        if goal != present:
            started = time.monotonic()
            while True:
                fraction = min((time.monotonic() - started) / args.duration, 1.0)
                blend = fraction**3 * (10.0 - 15.0 * fraction + 6.0 * fraction**2)
                target = round(present + (goal - present) * blend)
                write(GOAL_POSITION, target, 4)
                if fraction >= 1.0:
                    break
                time.sleep(0.01)
    except (Exception, KeyboardInterrupt):
        try:
            write(TORQUE_ENABLE, 0, 1)
        except Exception as cleanup_error:
            print(f'ID=23 torque OFF failed: {cleanup_error}', file=sys.stderr)
        raise
    print(f'ID=23 goal_raw={goal} sent; torque=ON (kept ON after exit)')


def main(argv=None):
    args = parse_args(argv)
    try:
        from dynamixel_sdk import PacketHandler, PortHandler
    except ImportError:
        print('dynamixel_sdk unavailable. Run: source /opt/ros/humble/setup.bash', file=sys.stderr)
        return 1

    port = PortHandler(args.port)
    try:
        # setBaudRate opens the port directly at the desired baud rate in the SDK.
        if not port.setBaudRate(BAUDRATE):
            raise RuntimeError(f'Cannot open {args.port} at {BAUDRATE} baud')
        control_neck(PacketHandler(2.0), port, args)
        return 0
    except KeyboardInterrupt:
        print('Interrupted.', file=sys.stderr)
        return 130
    except Exception as error:
        print(f'Neck command failed: {error}', file=sys.stderr)
        return 1
    finally:
        if port.is_open:
            port.closePort()


if __name__ == '__main__':
    sys.exit(main())

"""Small decision snapshots; no frame stream, disk I/O or history here.

Each v1 record is self-contained, including column names for its input rows.
``used`` holds [status, angle, follow_point] for line, [status, angle,
ball_in_hand, goal_distance_cm] for ball and [status, angle, ready] for hurdle.
Only active/nonzero state flags are stored. ``age_ms`` is time since reception
by decision, not camera capture age. Webcam stamps use the ROS clock; goal
frame/result stamps use the monotonic clock. Nonfinite measurements are null.
Timeout input rows describe unsuccessful attempts; their ``used`` is empty.
"""

from contextlib import contextmanager
import json
import math
import time

from std_msgs.msg import String


TOPIC = '/decision/motion_log'
MAX_RECORD_BYTES = 12 * 1024
FIELDS = {
    'line': ('sequence', 'age_ms', 'status', 'angle', 'follow_point',
             'point_count', 'decision_type', 'decision_angle', 'line_distance', 'curve_a'),
    'ball': ('sequence', 'age_ms', 'status', 'decision_angle', 'measured_angle',
             'x_distance_px', 'y_distance_px', 'ball_x_px', 'ball_y_px',
             'goal_x_px', 'goal_y_px', 'goal_distance_cm', 'ball_in_hand',
             'pre_shoot_verified', 'goal_raw_detected', 'goal_held_previous_detection',
             'goal_frame_stamp_sec', 'webcam_frame_stamp_sec',
             'webcam_ball_detected', 'result_stamp_sec'),
    'hurdle': ('age_ms', 'status', 'angle', 'hurdle_ready', 'ignored',
               'hurdle_intersection_distance_px'),
}
STATE_FIELDS = (
    'has_ball', 'pick_done', 'turn_after_pick', 'back_to_walk_after_pick',
    'turn_after_shoot', 'back_to_walk_after_shoot', 'hurdle_detected',
    'hurdle_step', 'hurdle_line_search_step', 'lost_step', 'lost_found_dir',
    'lost_left_line_seen', 'lost_right_line_seen', 'hurdle_left_line_seen',
    'hurdle_right_line_seen', 'goal_initial_pending', 'goal_initial_done',
    'pick_return_to_walk_pending', 'goal_return_to_walk_pending',
    'post_pick_failure_return_pending', 'startup_back_to_walk_pending',
    'turn_count', 'ball_count', 'goal_count', 'lost_body_turn_count',
    'hurdle_line_turn_count',
    'hurdle_initial_pose_done', 'hurdle_exception_deadline',
)


def scalar(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value[:48]
    return value


def rows(kind, details, now):
    """Column names appear once per group, not once per input."""
    fields = FIELDS[kind]
    values = []
    for detail in list(details)[-5:]:
        row = []
        for field in fields:
            if field == 'age_ms':
                received = detail.get('received_mono')
                value = round((now - received) * 1000, 1) if received is not None else None
            else:
                value = detail.get(field)
            row.append(scalar(value))
        values.append(row)
    return {'fields': fields, 'rows': values}


def vote_details(node, kind, count):
    """Align with the actual vote buffer, including synthetic status-only inputs."""
    statuses = list(getattr(node, kind + '_buffer', ()))[-count:]
    details = list(getattr(node, kind + '_vote_detail_buffer', ()))[-len(statuses):] if statuses else []
    details = [None] * (len(statuses) - len(details)) + details
    return [detail if detail is not None and detail.get('status') == status
            else {'status': status} for status, detail in zip(statuses, details)]


def snapshot(node, source, *, line=False, ball=0, hurdle=False, ball_details=None):
    """Copy selected scalar fields once, immediately before the decision."""
    now = time.monotonic()
    inputs, used = {}, {}
    if line:
        inputs['line'] = rows('line', vote_details(node, 'line', 3), now)
        used['line'] = [getattr(node, 'line_status', None),
                        scalar(getattr(node, 'angle', None)),
                        getattr(node, 'line_follow_point', None)]
    if ball or ball_details is not None:
        details = ball_details if ball_details is not None else vote_details(node, 'ball', ball)
        inputs['ball'] = rows('ball', details, now)
        used['ball'] = [getattr(node, 'ball_status', None),
                        scalar(getattr(node, 'ball_angle', None)),
                        getattr(node, 'ball_in_hand', None),
                        scalar(getattr(node, 'latest_goal_distance_cm', None))]
    if hurdle:
        inputs['hurdle'] = rows('hurdle', vote_details(node, 'hurdle', 5), now)
        used['hurdle'] = [getattr(node, 'hurdle_status', None),
                          scalar(getattr(node, 'hurdle_angle', None)),
                          getattr(node, 'hurdle_ready', None)]
    return {
        'source': source, 'decision_mono': now,
        'mode_before': getattr(node, 'current_mode', None),
        'inputs': inputs, 'used': {} if source.endswith('timeout') else used,
        'state': {name: getattr(node, name) for name in STATE_FIELDS
                  if getattr(node, name, False)},
    }


@contextmanager
def decision_scope(node, source, **kwargs):
    """Scope evidence to one decision; logging failure cannot block the motion."""
    if getattr(node, 'motion_log_pub', None) is None:
        yield
        return
    previous = getattr(node, '_motion_log_context', None)
    try:
        node._motion_log_context = snapshot(node, source, **kwargs)
    except Exception:
        node._motion_log_context = {'source': source, 'inputs': {}, 'used': {},
                                    'snapshot_error': True}
    try:
        yield
    finally:
        node._motion_log_context = previous


def emit_motion(node, command):
    """Called after command publication; diagnostics must never stop control."""
    publisher = getattr(node, 'motion_log_pub', None)
    if publisher is None:
        return
    try:
        node.motion_log_sequence = getattr(node, 'motion_log_sequence', 0) + 1
        context = getattr(node, '_motion_log_context', None)
        if context is None:
            # Startup, timed recovery and fixed sequences need not use vision.
            context = {'source': 'state', 'inputs': {}, 'used': {},
                       'state': {name: getattr(node, name) for name in STATE_FIELDS
                                 if getattr(node, name, False)}}
        record = {'type': 'motion_decision', 'v': 1,
                  'seq': node.motion_log_sequence, 'command': int(command),
                  'log_dropped': getattr(node, 'motion_log_dropped', 0),
                  'mode': getattr(node, 'current_mode', None),
                  'command_mono': time.monotonic(), 'command_wall_ns': time.time_ns(),
                  **context}
        payload = json.dumps(record, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
        if len(payload.encode()) > MAX_RECORD_BYTES:
            node.motion_log_dropped = getattr(node, 'motion_log_dropped', 0) + 1
            return
        publisher.publish(String(data=payload))
    except Exception:
        # A disconnected recorder or a logging failure must not prevent reset
        # of the decision cycle after the motor command was already sent.
        node.motion_log_dropped = getattr(node, 'motion_log_dropped', 0) + 1


def remember_hurdle(node, status, angle, ready, ignored, intersection_distance_px=None):
    buffer = getattr(node, 'hurdle_vote_detail_buffer', None)
    if buffer is not None and getattr(node, 'motion_log_pub', None) is not None:
        distance = scalar(intersection_distance_px)
        if ignored or distance is None or distance < 0.0:
            distance = None
        buffer.append({'received_mono': time.monotonic(), 'status': int(status),
                       'angle': angle, 'hurdle_ready': bool(ready), 'ignored': bool(ignored),
                       'hurdle_intersection_distance_px': distance})


def clear_details(node, kind):
    buffer = getattr(node, kind + '_vote_detail_buffer', None)
    if buffer is not None:
        buffer.clear()

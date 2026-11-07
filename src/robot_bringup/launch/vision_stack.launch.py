#!/usr/bin/env python3
"""IRC 전체 Vision 실행: webcam YOLO + RealSense ball/hoop YOLO.

배치 위치:
  ~/irc/src/robot_bringup/launch/vision_stack.launch.py

기본 Vision 스크립트 위치:
  ~/irc/src/vision/scripts

화면 없이 실행: ros2 launch robot_bringup vision_stack.launch.py
검출 화면 확인: ros2 launch robot_bringup vision_stack.launch.py show_preview:=true
"""

import glob
import os
import shlex
import stat
import sys
import tempfile
import time
import uuid

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

from robot_bringup.depth_settings import depth_parameters
from robot_bringup.realsense_guard import READY_MARKER
from robot_bringup.quiet_process import (
    configure_launch_logging, process_prefix, quiet_python_command,
)


def _make_webcam_node(context):
    start_webcam = LaunchConfiguration("start_webcam").perform(context)
    if start_webcam.strip().lower() not in {"1", "true", "yes", "on"}:
        return []

    device = LaunchConfiguration("webcam_device").perform(context)
    if device == "auto":
        c920_devices = sorted(
            glob.glob("/dev/v4l/by-id/*C920*video-index0")
        )
        if not c920_devices:
            raise RuntimeError(
                "C920 webcam not found under /dev/v4l/by-id/. "
                "Connect the webcam or set webcam_device explicitly."
            )
        device = c920_devices[0]
    try:
        if not stat.S_ISCHR(os.stat(device).st_mode):
            raise RuntimeError(f'Webcam is not a camera device: {device}')
    except OSError as error:
        raise RuntimeError(f'Webcam is unavailable: {device}: {error}') from error
    width = int(LaunchConfiguration("webcam_width").perform(context))
    height = int(LaunchConfiguration("webcam_height").perform(context))
    fps = int(LaunchConfiguration("webcam_fps").perform(context))
    if min(width, height, fps) <= 0:
        raise RuntimeError('Webcam width, height and fps must be positive.')

    return [
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="webcam",
            prefix=process_prefix('--ros'),
            output="both",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_webcam")),
            parameters=[
                {
                    "video_device": device,
                    "image_size": [width, height],
                    "time_per_frame": [1, fps],
                    # C920 영상은 픽셀 기반 검출에만 사용합니다. 유효한
                    # 미보정 CameraInfo를 지정해 드라이버의 기본 ~/.ros
                    # 캘리브레이션 파일 조회와 불필요한 오류를 막습니다.
                    "camera_info_url": (
                        "package://robot_bringup/config/"
                        "webcam_uncalibrated.yaml"
                    ),
                }
            ],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
        )
    ]


def _start_stack(context):
    # Resolve/validate the webcam before returning ANY process actions.  An
    # exception in a later OpaqueFunction used to leave the driver orphaned.
    webcam_nodes = _make_webcam_node(context)
    diagnostics = _diagnostics_actions(context)
    return diagnostics + _make_stack_actions(context, webcam_nodes)


def _diagnostics_actions(context):
    if not IfCondition(LaunchConfiguration('vision_diagnostics', default='true')).evaluate(context):
        context.environment.pop('IRC_VISION_TRACE_SOCKET', None)
        return []
    # A private per-launch address prevents packets from another run mixing in.
    # Creating it never opens a camera, GPU or robot device.
    try:
        socket_dir = tempfile.mkdtemp(prefix='irc-vdiag-')
    except OSError as exc:
        print(f'[VisionDiagnostics] Disabled: cannot create trace socket directory: {exc}', flush=True)
        context.environment.pop('IRC_VISION_TRACE_SOCKET', None)
        return []
    socket_path = os.path.join(socket_dir, 'trace.sock')
    run_id = 'run-' + time.strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8]
    root = LaunchConfiguration('vision_diagnostics_dir').perform(context)
    context.environment['IRC_VISION_TRACE_SOCKET'] = socket_path
    return [ExecuteProcess(
        name='vision_diagnostics',
        cmd=[sys.executable, '-m', 'robot_bringup.vision_diagnostics',
             '--socket', socket_path, '--directory', os.path.join(root, run_id),
             '--settings', LaunchConfiguration('settings_ini')],
        output='both', respawn=True, respawn_delay=2.0,
        sigterm_timeout='2.0', sigkill_timeout='1.0',
    )]


def _make_stack_actions(context, webcam_nodes):
    scripts_dir = LaunchConfiguration("scripts_dir")
    settings_ini = LaunchConfiguration("settings_ini")
    start_realsense = LaunchConfiguration("start_realsense")
    start_yolo = LaunchConfiguration("start_yolo")
    start_realsense_yolo = LaunchConfiguration("start_realsense_yolo")
    # Read the chosen INI on every launch, including headless operation.
    # Validate before returning any camera/process actions.
    depth_settings = (
        depth_parameters(settings_ini.perform(context))
        if IfCondition(start_realsense).evaluate(context) else {}
    )
    webcam_preview = (
        IfCondition(LaunchConfiguration("start_webcam_viewer")).evaluate(context)
        and IfCondition(LaunchConfiguration("start_webcam")).evaluate(context)
        and IfCondition(start_yolo).evaluate(context)
    )
    realsense_preview = (
        IfCondition(LaunchConfiguration("start_realsense_viewer")).evaluate(context)
        and IfCondition(start_realsense).evaluate(context)
        and IfCondition(start_realsense_yolo).evaluate(context)
    )
    realsense_view_topic = LaunchConfiguration("realsense_view_topic")
    start_ball = LaunchConfiguration("start_ball")
    start_hurdle = LaunchConfiguration("start_hurdle")
    start_monitor = LaunchConfiguration("start_monitor")

    yolo_script = PathJoinSubstitution([scripts_dir, "yolo_detector.py"])
    realsense_yolo_script = PathJoinSubstitution(
        [scripts_dir, "realsense_yolo_detector.py"]
    )
    ready_gate_script = PathJoinSubstitution(
        [scripts_dir, "ready_gated_process.py"]
    )
    ball_script = PathJoinSubstitution([scripts_dir, "ball_vision_fusion.py"])
    hurdle_script = PathJoinSubstitution(
        [scripts_dir, "hurdle_vision_fusion.py"]
    )
    monitor_script = PathJoinSubstitution(
        [scripts_dir, "vision_status_monitor.py"]
    )
    rgb_stabilizer_script = PathJoinSubstitution(
        [scripts_dir, "realsense_rgb_stabilizer.py"]
    )

    # RealSense color/depth는 하나의 YOLO 노드에서 공과 후프 검출에 사용한다.
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="",
        name="camera",
        output="both",
        emulate_tty=True,
        prefix=[
            shlex.quote(sys.executable)
            + ' -m robot_bringup.realsense_guard -- '
            + process_prefix('--ros')[0]
        ],
        sigterm_timeout='3.0',
        sigkill_timeout='2.0',
        on_exit=[Shutdown(reason='RealSense driver exited; see camera/guard log.')],
        condition=IfCondition(start_realsense),
        parameters=[
            {
                "camera_name": "camera",
                "enable_color": True,
                "enable_depth": True,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "rgb_camera.color_profile": "640,480,30",
                "depth_module.depth_profile": "640,480,30",
                "depth_module.hdr_enabled": False,
                **depth_settings,
                "enable_sync": True,
                "align_depth.enable": True,
                "pointcloud.enable": False,
                # A hardware reset disconnects the D435 for roughly
                # 6-7 seconds.
                # Normal startup and hot-plug reconnect remain supported by the
                # driver without forcing that reset on every launch.
                "initial_reset": False,
                # UVC power-line-frequency: 0=off, 1=50 Hz, 2=60 Hz,
                # 3=auto. 이 RealSense는 [0, 2]만 지원하므로 한국 실내
                # 조명에서는 auto(3)가 아니라 60 Hz(2)를 명시한다.
                "rgb_camera.power_line_frequency": 2,
                # 시작 직후에는 현장 조명에 적응시킨다. 아래 stabilizer가
                # 3~5초 뒤 수렴값을 읽고 자동 기능을 끈다.
                "rgb_camera.enable_auto_exposure": True,
                "rgb_camera.enable_auto_white_balance": True,
            }
        ],
    )

    rgb_stabilizer_process = ExecuteProcess(
        name="realsense_rgb_stabilizer_process",
        prefix=process_prefix('--python'),
        cmd=[
            sys.executable,
            rgb_stabilizer_script,
            "--camera-node",
            "/camera",
            "--warmup-seconds",
            LaunchConfiguration("realsense_rgb_warmup_seconds"),
            "--power-line-frequency",
            "2",
        ],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        condition=IfCondition(
            LaunchConfiguration("lock_realsense_rgb_after_warmup")
        ),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    yolo_process = ExecuteProcess(
        name="yolo_vision_process",
        prefix=process_prefix('--python'),
        cmd=[sys.executable, yolo_script, settings_ini, "--ros2",
             "--preview", "true" if webcam_preview else "false"],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        respawn=True,
        # Give CUDA/TensorRT time to release allocations after an
        # abnormal exit.
        respawn_delay=5.0,
        condition=IfCondition(start_yolo),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    # Keep the RealSense TensorRT process out of memory until webcam YOLO has
    # completed its first inference.  If webcam YOLO later crashes, its READY
    # publisher disappears and the supervisor stops RealSense YOLO so webcam
    # can reload alone.  Child failures use exponential retry backoff.
    realsense_yolo_process = ExecuteProcess(
        name="realsense_yolo_gate_process",
        prefix=process_prefix('--python'),
        cmd=[
            sys.executable,
            ready_gate_script,
            "--node-name",
            "realsense_yolo_gate",
            "--gate-topic",
            "/vision/webcam_yolo_ready",
            "--gate-enabled",
            start_yolo,
            "--source-enabled",
            LaunchConfiguration("start_webcam"),
            # The first detector has published READY, but allowing its CUDA
            # allocator to settle briefly avoids a transient NvMap OOM while
            # the second TensorRT runtime is created.
            "--gate-open-delay-seconds",
            "3.0",
            "--initial-backoff-seconds",
            "5.0",
            "--max-backoff-seconds",
            "30.0",
            "--restart-on-clean-exit",
            "true",
            "--",
            *quiet_python_command(
                realsense_yolo_script, settings_ini,
                "--preview", "true" if realsense_preview else "false",
            ),
        ],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        condition=IfCondition(start_realsense_yolo),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    # Both external viewers wait until engine initialization is finished.
    # In webcam-only mode there is no RealSense readiness publisher to wait for.
    wait_for_realsense = (
        IfCondition(start_realsense).evaluate(context)
        and IfCondition(start_realsense_yolo).evaluate(context)
    )
    preview_ready_topic = (
        "/vision/realsense_yolo_ready" if wait_for_realsense
        else "/vision/webcam_yolo_ready"
    )
    viewer_processes = []
    if (
        IfCondition(LaunchConfiguration("start_depth_viewer")).evaluate(context)
        and IfCondition(start_realsense).evaluate(context)
    ):
        viewer_processes.append(ExecuteProcess(
            name="realsense_depth_tuner",
            prefix=process_prefix('--python'),
            cmd=[sys.executable, ready_gate_script,
                 "--node-name", "depth_viewer_gate",
                 "--gate-topic", preview_ready_topic,
                 "--gate-enabled", "true" if wait_for_realsense else "false",
                 "--restart-on-clean-exit", "false",
                 "--", sys.executable, "-m", "robot_bringup.depth_tuner",
                 "--settings", settings_ini, "--camera-node", "/camera"],
            additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
            output="both",
        ))
    for name, topic, enabled in (
        ("webcam", "/line_tracker/debug_image", webcam_preview),
        ("realsense", realsense_view_topic, realsense_preview),
    ):
        if not enabled:
            continue
        viewer_processes.append(ExecuteProcess(
            name=f"{name}_image_view",
            prefix=process_prefix('--python'),
            cmd=[
                sys.executable, ready_gate_script,
                "--node-name", f"{name}_viewer_gate",
                "--waiting-message", "Preview is waiting for YOLO readiness.",
                "--gate-topic", preview_ready_topic,
                "--", "ros2", "run", "rqt_image_view", "rqt_image_view", topic,
            ],
            # Isolate the viewers from stale Fast DDS SHM ports; camera and
            # inference nodes retain their existing transport configuration.
            additional_env={"FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"},
            output="both",
        ))

    # 융합 노드는 YOLO JSON 상태만 처리하며 카메라 영상을 다시 구독하지 않는다.
    ball_process = ExecuteProcess(
        name="ball_vision_fusion_process",
        prefix=process_prefix('--python'),
        cmd=[sys.executable, ball_script],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        condition=IfCondition(start_ball),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    hurdle_process = ExecuteProcess(
        name="webcam_hurdle_publisher_process",
        prefix=process_prefix('--python'),
        cmd=[sys.executable, hurdle_script],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        condition=IfCondition(start_hurdle),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    monitor_process = ExecuteProcess(
        name="vision_status_monitor_process",
        prefix=process_prefix('--python'),
        cmd=[sys.executable, monitor_script],
        cwd=scripts_dir,
        output="both",
        emulate_tty=True,
        condition=IfCondition(start_monitor),
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    # Without a forced hardware reset the camera normally declares its dynamic
    # RGB parameters within a few seconds.  The stabilizer also waits for the
    # parameter services, so a small launch delay is sufficient.
    delayed_rgb_stabilizer = TimerAction(
        period=3.0,
        actions=[rgb_stabilizer_process],
    )

    delayed_vision = TimerAction(
        period=2.0,
        actions=[
            ball_process,
            hurdle_process,
            monitor_process,
        ],
    )

    startup_actions = [
        *webcam_nodes,
        yolo_process,
        realsense_yolo_process,
        *viewer_processes,
        delayed_vision,
    ]
    if not IfCondition(start_realsense).evaluate(context):
        return startup_actions
    startup_actions.insert(0, delayed_rgb_stabilizer)
    return _after_camera_ownership(context, realsense_node, startup_actions)


def _after_camera_ownership(context, camera_node, startup_actions):
    # Only the winner of the atomic lock may start the remaining stack.  This
    # also prevents a second launch from briefly opening the webcam/GPU.
    pending = bytearray()
    started = False

    def on_camera_output(event):
        nonlocal started
        if started or context.is_shutdown:
            return []
        pending.extend(event.text)
        if READY_MARKER.encode() in pending:
            started = True
            return startup_actions
        # Keep enough bytes to match a marker split across output events.
        del pending[:-len(READY_MARKER)]
        return []

    return [
        RegisterEventHandler(OnProcessIO(
            target_action=camera_node, on_stdout=on_camera_output,
        )),
        camera_node,
    ]


def generate_launch_description() -> LaunchDescription:
    declarations = [
        DeclareLaunchArgument(
            'quiet_logs', default_value='true',
            description='Hide runtime console logs; diagnostic warnings are still recorded.',
        ),
        DeclareLaunchArgument('vision_diagnostics', default_value='true',
                             description='Record bounded camera health metadata, without images.'),
        DeclareLaunchArgument('vision_diagnostics_dir', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), '.ros', 'vision_diagnostics'])),
        DeclareLaunchArgument(
            "scripts_dir",
            default_value=PathJoinSubstitution(
                [
                    EnvironmentVariable("HOME"),
                    "irc",
                    "src",
                    "vision",
                    "scripts",
                ]
            ),
            description="vision Python scripts/settings/model directory",
        ),
        DeclareLaunchArgument(
            "settings_ini",
            default_value=PathJoinSubstitution(
                [
                    EnvironmentVariable("HOME"),
                    "irc",
                    "src",
                    "vision",
                    "config",
                    "settings.ini",
                ]
            ),
        ),
        DeclareLaunchArgument("start_realsense", default_value="true"),
        DeclareLaunchArgument("start_webcam", default_value="true"),
        DeclareLaunchArgument("start_yolo", default_value="true"),
        DeclareLaunchArgument("start_realsense_yolo", default_value="true"),
        DeclareLaunchArgument(
            "show_preview", default_value="false",
            description="Open detection views and the depth exposure tuner; overrides INI preview settings.",
        ),
        DeclareLaunchArgument(
            "start_webcam_viewer",
            default_value=LaunchConfiguration("show_preview"),
            description="Open the webcam detection/status view.",
        ),
        DeclareLaunchArgument(
            "start_realsense_viewer",
            default_value=LaunchConfiguration("show_preview"),
            description="Open the RealSense detection/status view and enable its debug images.",
        ),
        DeclareLaunchArgument(
            "start_depth_viewer",
            default_value=LaunchConfiguration("show_preview"),
            description="Open aligned depth with live exposure controls and INI save; also works without YOLO.",
        ),
        DeclareLaunchArgument(
            "realsense_view_topic",
            default_value="/vision/realsense_combined_image",
            description="RealSense YOLO debug image topic shown by rqt.",
        ),
        DeclareLaunchArgument("start_ball", default_value="true"),
        DeclareLaunchArgument("start_hurdle", default_value="true"),
        DeclareLaunchArgument("start_monitor", default_value="true"),
        DeclareLaunchArgument(
            "webcam_device",
            default_value="auto",
            description=(
                "Webcam device path. 'auto' selects the C920 video-index0 "
                "device from /dev/v4l/by-id/."
            ),
        ),
        DeclareLaunchArgument("webcam_width", default_value="640"),
        DeclareLaunchArgument("webcam_height", default_value="480"),
        DeclareLaunchArgument("webcam_fps", default_value="30"),
        DeclareLaunchArgument(
            "lock_realsense_rgb_after_warmup",
            default_value="true",
            description=(
                "Warm up RealSense RGB auto exposure/WB, then lock the "
                "settled values for stable YOLO input."
            ),
        ),
        DeclareLaunchArgument(
            "realsense_rgb_warmup_seconds",
            default_value="5.0",
            description="Auto exposure/WB warmup time before locking.",
        ),
    ]
    return LaunchDescription(declarations + [
        OpaqueFunction(function=configure_launch_logging),
        OpaqueFunction(function=_start_stack),
    ])

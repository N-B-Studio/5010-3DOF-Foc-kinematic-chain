#!/usr/bin/env python3
"""Run the first deliberately small Cartesian trajectory on Ben's 3-DOF leg.

Default mode is offline preview: it never opens a serial port.  ``--execute``
adds an explicit ``RUN`` confirmation, then owns the PC-to-H723 UART link,
sends KEEP at 50 Hz, and always sends DISARM / POWER OFF on exit.

The trajectory is intentionally a staging test, not a gait or a jump:
  1. smooth zero -> crouch centre over 3 s,
  2. fade in either a sole ellipse or a pure vertical sole motion at 0.25 Hz,
  3. smooth return to the all-vertical ZERO pose,
  4. actively hold ZERO for one minute (Ctrl-C safely ends early).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import cos, degrees, pi, sin
import queue
import re
import threading
import time
from typing import Iterable

from leg_kinematics import (
    PHYSICAL_KNEE_BRANCH,
    controller_from_kinematic_deg,
    ik,
    within_h723_soft_limit,
)


HZ = 50.0
DT_S = 1.0 / HZ
DEFAULT_ENTRY_S = 3.0
FADE_IN_S = 1.0
DEFAULT_RETURN_S = 3.0
DEFAULT_ZERO_HOLD_S = 60.0
DEFAULT_ELLIPSE_PERIOD_S = 4.0
CENTRE_X_M = 0.000
CENTRE_Z_M = -0.250
DEFAULT_AMPLITUDE_X_M = 0.010
DEFAULT_AMPLITUDE_Z_M = 0.005

# H723 status reports its controller order [hip, knee, ankle].  RViz uses the
# mathematical URDF convention from leg_kinematics.py, therefore the knee sign
# is inverted before publishing JointState.
STATUS_Q_RE = re.compile(
    r"(?:^|\s)q=\[\s*([+\-0-9.eE]+)\s*,\s*([+\-0-9.eE]+)\s*,\s*([+\-0-9.eE]+)\s*\]"
)


@dataclass(frozen=True)
class TrajectoryConfig:
    cycles: int
    motion: str
    amplitude_x_m: float
    amplitude_z_m: float
    vertical_up_m: float
    vertical_down_m: float
    period_s: float
    entry_s: float = DEFAULT_ENTRY_S
    return_s: float = DEFAULT_RETURN_S


def smoothstep01(value: float) -> float:
    """Cubic position profile with zero velocity at both ends."""
    clamped = min(1.0, max(0.0, value))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def lerp(a: tuple[float, float, float], b: tuple[float, float, float], alpha: float) -> tuple[float, float, float]:
    return tuple(x + (y - x) * alpha for x, y in zip(a, b, strict=True))


def cartesian_target(time_s: float, config: TrajectoryConfig) -> tuple[float, float]:
    """Return desired sole x/z in metres for the execute phase.

    IK always receives foot_pitch=0 later, so the foot plate remains parallel
    to the ground.  ``vertical`` holds x=0 and only moves the sole in z.
    """
    ellipse_s = config.cycles * config.period_s
    if time_s < config.entry_s:
        alpha = smoothstep01(time_s / config.entry_s)
        return CENTRE_X_M * alpha, -0.265 + (CENTRE_Z_M + 0.265) * alpha

    ellipse_time_s = time_s - config.entry_s
    if ellipse_time_s < ellipse_s:
        amplitude = smoothstep01(ellipse_time_s / FADE_IN_S)
        phase = 2.0 * pi * ellipse_time_s / config.period_s
        if config.motion == "vertical":
            wave = sin(phase)
            vertical_amplitude = config.vertical_up_m if wave >= 0.0 else config.vertical_down_m
            return CENTRE_X_M, CENTRE_Z_M + amplitude * vertical_amplitude * wave
        return (
            CENTRE_X_M + amplitude * config.amplitude_x_m * cos(phase),
            CENTRE_Z_M + amplitude * config.amplitude_z_m * sin(phase),
        )

    # The caller uses this only to construct the smooth return segment.
    return CENTRE_X_M, CENTRE_Z_M


def h723_target_deg(sole_x_m: float, sole_z_m: float) -> tuple[float, float, float]:
    q_kin_deg = tuple(
        degrees(value)
        for value in ik(sole_x_m, sole_z_m, foot_pitch_rad=0.0, knee_branch=PHYSICAL_KNEE_BRANCH)
    )
    target = controller_from_kinematic_deg(q_kin_deg)
    if not within_h723_soft_limit(target):
        raise ValueError(f"trajectory target exceeds H723 soft limit: {target}")
    return target


def sample_targets(config: TrajectoryConfig) -> Iterable[tuple[str, tuple[float, float, float]]]:
    """Yield every H723 target, including entry and return, for preflight."""
    ellipse_s = config.cycles * config.period_s
    total_s = config.entry_s + ellipse_s
    steps = int(total_s * HZ) + 1
    for step in range(steps):
        t = step / HZ
        x, z = cartesian_target(t, config)
        yield "entry_or_ellipse", h723_target_deg(x, z)

    last = h723_target_deg(*cartesian_target(total_s - DT_S, config))
    for step in range(int(config.return_s * HZ) + 1):
        yield "return", lerp(last, (0.0, 0.0, 0.0), smoothstep01(step / (config.return_s * HZ)))


def print_preview(config: TrajectoryConfig) -> None:
    targets = list(sample_targets(config))
    values = list(zip(*(target for _, target in targets), strict=True))
    motion_description = (
        f"vertical: x fixed at 0.0 mm, z +{config.vertical_up_m * 1000:.1f}/-{config.vertical_down_m * 1000:.1f} mm"
        if config.motion == "vertical"
        else f"ellipse: x +/-{config.amplitude_x_m * 1000:.1f} mm, z +/-{config.amplitude_z_m * 1000:.1f} mm"
    )
    print("Offline only: no serial port will be opened.")
    print(
        f"physical branch: {PHYSICAL_KNEE_BRANCH}; rate: {HZ:.0f} Hz; cycles: {config.cycles}; "
        f"{motion_description}; period: {config.period_s:.2f} s"
    )
    print("H723 SET limits across complete trajectory:")
    for name, joint_values in zip(("hip", "knee", "ankle"), values, strict=True):
        print(f"  {name:5s}: {min(joint_values):7.2f} .. {max(joint_values):7.2f} deg")
    print("First crouch centre: SET 22.33 44.66 22.33")
    print("Run command: python run_leg_trajectory.py --port COM4 --execute")


def start_reader(port, received: queue.Queue[str]) -> None:
    def worker() -> None:
        while True:
            try:
                line = port.readline().decode("ascii", errors="replace").strip()
            except Exception:
                return
            if line:
                received.put(line)

    threading.Thread(target=worker, daemon=True).start()


def send(port, command: str) -> None:
    port.write((command + "\n").encode("ascii"))


class RosJointStatePublisher:
    """Publish measured H723 joint angles while this process owns the UART.

    rclpy is imported only when --ros2 is used, so the existing Windows-only
    trajectory workflow remains dependency-free.  The published topic is the
    standard /joint_states topic consumed by robot_state_publisher and RViz.
    """

    def __init__(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except ImportError as exc:
            raise SystemExit(
                "--ros2 requires a sourced ROS2 environment with rclpy and sensor_msgs. "
                "Run this mode on your Ubuntu ROS2 PC, not the Windows-only COM4 setup."
            ) from exc

        self._rclpy = rclpy
        self._joint_state_type = JointState
        rclpy.init()
        self._node = Node("h723_leg_trajectory")
        self._publisher = self._node.create_publisher(JointState, "/joint_states", 10)

    def publish_status_line(self, line: str) -> bool:
        match = STATUS_Q_RE.search(line)
        if not match:
            return False

        hip_deg, knee_h723_deg, ankle_deg = (float(value) for value in match.groups())
        message = self._joint_state_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.name = ["hip_joint", "knee_joint", "ankle_joint"]
        message.position = [
            hip_deg * pi / 180.0,
            -knee_h723_deg * pi / 180.0,
            ankle_deg * pi / 180.0,
        ]
        self._publisher.publish(message)
        return True

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def publish_received_statuses(
    received: queue.Queue[str], publisher: RosJointStatePublisher | None
) -> None:
    """Drain UART status without delaying the deterministic 50 Hz sender."""
    while True:
        try:
            line = received.get_nowait()
        except queue.Empty:
            return
        if publisher is not None:
            publisher.publish_status_line(line)


def wait_for_armed(received: queue.Queue[str], timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            line = received.get(timeout=0.1)
        except queue.Empty:
            continue
        if "mode=ARMED" in line and "fault=armed" in line:
            return True
        if "mode=FAULT" in line:
            print(f"H723 refused ARM: {line}")
            return False
    return False


def stream_target(port, target: tuple[float, float, float]) -> None:
    send(port, "SET {:.3f} {:.3f} {:.3f}".format(*target))
    send(port, "KEEP")


def hold_zero(port, received: queue.Queue[str], ros_publisher: "RosJointStatePublisher | None", duration_s: float) -> None:
    """Keep the leg actively at its captured ZERO pose for a finite duration."""
    print(f"Holding ZERO for {duration_s:.0f} s; press Ctrl-C to stop early.")
    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        tick_start = time.monotonic()
        publish_received_statuses(received, ros_publisher)
        stream_target(port, (0.0, 0.0, 0.0))
        remaining = DT_S - (time.monotonic() - tick_start)
        if remaining > 0.0:
            time.sleep(remaining)


def execute(args: argparse.Namespace, config: TrajectoryConfig) -> None:
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Missing dependency: python -m pip install pyserial") from exc

    confirmation = input(
        "Leg must be supported, unloaded, and exactly straight in its ZERO pose. "
        "Type RUN to enable motors: "
    )
    if confirmation != "RUN":
        print("Cancelled; no serial port opened.")
        return

    ros_publisher = RosJointStatePublisher() if args.ros2 else None
    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        received: queue.Queue[str] = queue.Queue()
        start_reader(port, received)
        try:
            # This is a consciously explicit state sequence.  ZERO captures the
            # all-vertical, natural-hang pose required by the kinematic model.
            send(port, "CLEAR")
            time.sleep(0.15)
            send(port, "ZERO")
            time.sleep(0.20)
            send(port, "POWER ON")
            time.sleep(0.40)
            # The UART reader runs from immediately after opening the port.
            # Discard boot/previous-fault status lines before ARM so an old
            # bad_pc_command cannot be mistaken for this ARM attempt failing.
            publish_received_statuses(received, ros_publisher)
            send(port, "ARM")
            if not wait_for_armed(received, timeout_s=5.0):
                raise RuntimeError("H723 did not report ARMED within 5 s")

            send(port, f"GAINS {args.kp:.4f} {args.kd:.4f} {args.torque_cap:.4f}")
            time.sleep(0.10)
            print("ARMED: entering 3 s crouch, trajectory, returning to ZERO, then holding ZERO.")

            for _, target in sample_targets(config):
                tick_start = time.monotonic()
                publish_received_statuses(received, ros_publisher)
                stream_target(port, target)
                remaining = DT_S - (time.monotonic() - tick_start)
                if remaining > 0.0:
                    time.sleep(remaining)

            hold_zero(port, received, ros_publisher, args.zero_hold_s)
        except KeyboardInterrupt:
            print("Interrupted.")
        finally:
            # Mechanical support remains required: this removes holding torque.
            send(port, "DISARM")
            time.sleep(0.05)
            send(port, "POWER OFF")
            print("DISARM and POWER OFF sent.")
            if ros_publisher is not None:
                ros_publisher.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe first Cartesian trajectory for the H723 3-DOF leg")
    parser.add_argument("--port", default="COM4", help="H723 USART1 port; only used with --execute")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cycles", type=int, default=2, choices=range(1, 6))
    parser.add_argument("--motion", choices=("ellipse", "vertical"), default="ellipse")
    parser.add_argument("--amplitude-x-mm", type=float, default=DEFAULT_AMPLITUDE_X_M * 1000.0,
                        help="ellipse only; ignored in vertical mode")
    parser.add_argument("--amplitude-z-mm", type=float, default=DEFAULT_AMPLITUDE_Z_M * 1000.0)
    parser.add_argument("--vertical-up-mm", type=float, default=None,
                        help="vertical mode: upward travel from z=-250 mm; defaults to --amplitude-z-mm")
    parser.add_argument("--vertical-down-mm", type=float, default=None,
                        help="vertical mode: downward travel from z=-250 mm; defaults to --amplitude-z-mm")
    parser.add_argument("--period-s", type=float, default=DEFAULT_ELLIPSE_PERIOD_S)
    parser.add_argument(
        "--zero-hold-s",
        type=float,
        default=DEFAULT_ZERO_HOLD_S,
        help="actively hold SET 0 0 0 after the trajectory; Ctrl-C ends early (default: 60)",
    )
    parser.add_argument("--kp", type=float, default=0.50)
    parser.add_argument("--kd", type=float, default=0.010)
    parser.add_argument("--torque-cap", type=float, default=0.080, help="motor N m; 0.080 is about 2 A")
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="publish measured H723 angles to /joint_states for robot_state_publisher/RViz; Linux ROS2 only",
    )
    parser.add_argument("--execute", action="store_true", help="open UART and run after an explicit RUN confirmation")
    args = parser.parse_args()

    if args.amplitude_z_mm <= 0.0 or args.period_s <= 0.0:
        raise SystemExit("Amplitude and period must be positive.")
    if args.zero_hold_s < 0.0:
        raise SystemExit("--zero-hold-s must be non-negative.")
    if args.motion == "ellipse" and args.amplitude_x_mm <= 0.0:
        raise SystemExit("Ellipse x amplitude must be positive.")
    vertical_up_mm = args.amplitude_z_mm if args.vertical_up_mm is None else args.vertical_up_mm
    vertical_down_mm = args.amplitude_z_mm if args.vertical_down_mm is None else args.vertical_down_mm
    if args.motion == "vertical":
        if vertical_up_mm <= 0.0 or vertical_down_mm < 0.0:
            raise SystemExit("Vertical up travel must be positive and down travel must be non-negative.")
        # z=-265 mm is the all-vertical ZERO pose and is the longest possible
        # leg configuration in this model.  Do not request a target below it.
        if vertical_down_mm > 15.0:
            raise SystemExit(
                "Vertical down travel exceeds the 15.0 mm available before the straight ZERO pose. "
                "Use --vertical-down-mm 15 (or less)."
            )
    config = TrajectoryConfig(
        cycles=args.cycles,
        motion=args.motion,
        amplitude_x_m=args.amplitude_x_mm / 1000.0,
        amplitude_z_m=args.amplitude_z_mm / 1000.0,
        vertical_up_m=vertical_up_mm / 1000.0,
        vertical_down_m=vertical_down_mm / 1000.0,
        period_s=args.period_s,
    )

    print_preview(config)
    if args.execute:
        execute(args, config)


if __name__ == "__main__":
    main()

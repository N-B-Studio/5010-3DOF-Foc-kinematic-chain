"""Plot static FK poses and a slow Cartesian sole trajectory offline.

Run:
    python3 plot_leg_ik.py

The window contains no serial or motor command path.  It is a safe first
check of branch selection, coordinate convention and controller sign mapping.
"""

from __future__ import annotations

from math import cos, pi, sin

import matplotlib.pyplot as plt

from leg_kinematics import (
    PHYSICAL_KNEE_BRANCH,
    controller_from_kinematic_deg,
    degrees,
    fk,
    ik,
    within_h723_soft_limit,
)


def draw_pose(ax, q_rad, color: str, label: str) -> None:
    pose = fk(q_rad)
    points = [pose.hip, pose.knee, pose.ankle, pose.sole]
    ax.plot([p.x * 1000.0 for p in points], [p.z * 1000.0 for p in points], "o-", color=color, label=label)


def main() -> None:
    fig, (ax_pose, ax_workspace) = plt.subplots(1, 2, figsize=(12, 6))

    # Neutral pose and a first fully-valid crouch target.  This is intentionally
    # not yet a hardware command; it visualises the selected physical branch.
    draw_pose(ax_pose, (0.0, 0.0, 0.0), "tab:gray", "neutral q=[0,0,0]")
    q = ik(0.000, -0.250, 0.0, knee_branch=PHYSICAL_KNEE_BRANCH)
    q_deg = tuple(degrees(v) for v in q)
    h723_deg = controller_from_kinematic_deg(q_deg)
    assert within_h723_soft_limit(h723_deg)
    draw_pose(
        ax_pose,
        q,
        "tab:orange",
        "physical branch -1: IK=" + str(tuple(round(v, 1) for v in q_deg)) + "°\n"
        + "H723 SET=" + str(tuple(round(v, 1) for v in h723_deg)) + "°",
    )

    # First all-valid sole ellipse: x +/-10 mm, z=-250 +/-5 mm.  It stays
    # inside the current H723 +/-60 degree soft limits for every sampled point.
    xs, zs = [], []
    valid = 0
    for index in range(121):
        phase = 2.0 * pi * index / 120.0
        x = 0.000 + 0.010 * cos(phase)
        z = -0.250 + 0.005 * sin(phase)
        try:
            q = ik(x, z, 0.0, knee_branch=PHYSICAL_KNEE_BRANCH)
        except ValueError:
            continue
        q_deg = tuple(degrees(v) for v in q)
        if not within_h723_soft_limit(controller_from_kinematic_deg(q_deg)):
            continue
        xs.append(x * 1000.0)
        zs.append(z * 1000.0)
        valid += 1

    ax_workspace.plot(xs, zs, color="tab:green", linewidth=2, label=f"first safe ellipse ({valid}/121 points)")
    ax_workspace.scatter([0.0], [-265.0], color="tab:gray", label="neutral sole")

    for ax in (ax_pose, ax_workspace):
        ax.axhline(0.0, color="black", linewidth=0.7)
        ax.axvline(0.0, color="black", linewidth=0.7)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True)
        ax.set_xlabel("x forward (mm)")
        ax.set_ylabel("z up (mm)")
        ax.legend(fontsize=8)

    ax_pose.set_title("FK pose: confirmed physical knee branch")
    ax_workspace.set_title("First all-valid Cartesian trajectory")
    fig.suptitle("3-DOF FOC leg offline FK/IK — no hardware command")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

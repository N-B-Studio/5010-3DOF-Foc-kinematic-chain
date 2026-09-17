# 3-DOF FOC Leg — Offline FK/IK

This is the safe, offline first step for the H723 three-joint leg.  It does not
open a serial port or command motors.

## Geometry

| quantity | value |
| --- | ---: |
| hip to knee | 100 mm |
| knee to ankle | 100 mm |
| ankle to sole contact | 65 mm |
| leg width | 70 mm |
| hip to test-fixture plate | 60 mm |

The planar model uses only the first three lengths.  The hip fixture offset and
leg width are retained for later URDF/collision modelling.

## Coordinates

The hip frame follows ROS body convention: **+X forward**, **+Z upward**.
`q=[0,0,0]` is the all-vertical pose captured by H723 `ZERO`; the sole contact
is therefore `(x=0, z=-265 mm)`.  `foot_pitch=0` keeps the final 65 mm sole
link vertical down.

`leg_kinematics.py` is an analytic planar 3R FK/IK model.  The physical
knee-forward folding side is `branch=-1` (orange in the initial plot).  The
confirmed mapping from mathematical angles to the H723 target order is
`[+1, -1, +1]`:

- H723 hip `+` moves the thigh forward;
- H723 knee `-` moves the shank forward;
- H723 ankle `+` lifts the forefoot.

This target mapping is independent of, and must not modify, H723 motor torque
sign calibration.

## Run

```bash
python3 leg_kinematics.py
python3 plot_leg_ik.py
python3 run_leg_trajectory.py
```

The second command requires `matplotlib` and displays only an offline plot.
The third command defaults to an offline target-range preview.  It opens the
H723 serial port only with `--execute`, then requires an exact `RUN` typed
confirmation.  It owns the 50 Hz `KEEP` traffic while armed, executes a
3-second entry, two small 0.25 Hz Cartesian sole ellipses, returns to ZERO,
and sends `DISARM` / `POWER OFF` even on Ctrl+C.

For the first hardware trial, keep the leg mechanically supported and use its
default motor torque cap of `0.080 N m` (about 2 A):

```bash
python3 run_leg_trajectory.py --port COM4 --execute
```

After the default `x=+/-10 mm`, `z=+/-5 mm` ellipse has run cleanly, the
next supported bench step is `x=+/-15 mm`, `z=+/-7.5 mm` with a 3 A cap.  Its
offline preflight remains below the +/-60 degree soft limit (knee maximum is
about 54.9 degrees, leaving about 5 degrees of margin):

```bash
python3 run_leg_trajectory.py --port COM4 \
  --amplitude-x-mm 15 --amplitude-z-mm 7.5 \
  --kp 0.80 --kd 0.010 --torque-cap 0.120 --execute
```

Do not use `20 mm x 10 mm` yet: it brings the knee to about 57.9 degrees and
leaves only about 2 degrees of target margin.

## Foot-level vertical motion

The IK uses `foot_pitch=0`, meaning the ankle-to-sole normal stays vertical and
the foot plate stays parallel to the ground.  Use pure vertical motion to hold
the sole at `x=0` while moving it only in Z.  The trajectory centre is
`z=-250 mm`, while the fully straight ZERO leg is `z=-265 mm`; therefore down
travel can be at most 15 mm.  Up and down travel are independent parameters.

Start with +7.5/-7.5 mm at the same 3 A bench cap:

```bash
python3 run_leg_trajectory.py --port COM4 --motion vertical \
  --amplitude-z-mm 7.5 --kp 0.80 --kd 0.010 --torque-cap 0.120 --execute
```

For the larger, foot-level vertical test use +17.5/-15 mm.  It needs an H723
soft-limit configuration of `[hip, knee, ankle] = +/-[60, 70, 60]` degrees and
commands approximately `[33.1, 66.2, 33.1]` degrees at its highest point:

```bash
python3 run_leg_trajectory.py --port COM4 --motion vertical \
  --vertical-up-mm 17.5 --vertical-down-mm 15 \
  --kp 0.80 --kd 0.010 --torque-cap 0.120 --execute
```

## Required validation before serial control

1. Verify `FK(IK(target))` and that mapped H723 targets remain inside the
   per-joint soft limits `[hip, knee, ankle] = +/-[60, 70, 60]` degrees.
2. Confirm the first trajectory offline before adding a 50–100 Hz serial
   trajectory publisher, with the H723 still
   owning torque caps, hard limits, foot/CAN/UART watchdogs and DISARM.

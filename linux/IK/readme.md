# WIndows:
python .\run_leg_trajectory.py --port COM4 `
  --amplitude-x-mm 15 --amplitude-z-mm 7.5 `
  --kp 0.50 --kd 0.010 --torque-cap 0.120 --execute

python .\run_leg_trajectory.py `
  --motion vertical `
  --vertical-up-mm 20 --vertical-down-mm 15 `
  --kp 0.50 --kd 0.010 --torque-cap 0.120  --execute

# LINUX:
python3 run_leg_trajectory.py --port /dev/ttyACM0 --amplitude-x-mm 15 --amplitude-z-mm 7.5 --kp 0.50 --kd 0.010 --torque-cap 0.120 --ros2 --execute



# AI read me
3-DOF FOC Leg — Offline FK/IK

This is the safe, offline first step for the H723 three-joint leg.  It does not
open a serial port or command motors.

Geometry

quantity

value

hip to knee

100 mm

knee to ankle

100 mm

ankle to sole contact

65 mm

leg width

70 mm

hip to test-fixture plate

60 mm

The planar model uses only the first three lengths.  The hip fixture offset and
leg width are retained for later URDF/collision modelling.

Coordinates

The hip frame follows ROS body convention: +X forward, +Z upward.
q=[0,0,0] is the all-vertical pose captured by H723 ZERO; the sole contact
is therefore (x=0, z=-265 mm).  foot_pitch=0 keeps the final 65 mm sole
link vertical down.

leg_kinematics.py is an analytic planar 3R FK/IK model.  The physical
knee-forward folding side is branch=-1 (orange in the initial plot).  The
confirmed mapping from mathematical angles to the H723 target order is
[+1, -1, +1]:

H723 hip + moves the thigh forward;

H723 knee - moves the shank forward;

H723 ankle + lifts the forefoot.

This target mapping is independent of, and must not modify, H723 motor torque
sign calibration.

Run

python3 leg_kinematics.py
python3 plot_leg_ik.py

The second command requires matplotlib and displays only an offline plot.

Required validation before serial control

Verify FK(IK(target)) and that mapped H723 targets remain inside soft
limits of +/-60 degrees.

Confirm the first trajectory offline before adding a 50–100 Hz serial
trajectory publisher, with the H723 still
owning torque caps, hard limits, foot/CAN/UART watchdogs and DISARM.
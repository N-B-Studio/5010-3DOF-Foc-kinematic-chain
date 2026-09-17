# H723 3-FOC leg: live ROS2/RViz monitor

This monitor deliberately uses **measured H723 status** rather than commanded
targets.  `run_leg_trajectory.py --ros2` is the only process that opens the
ST-Link UART.  It converts the returned controller angle order into the
kinematic joint convention and publishes `/joint_states` at the H723 status
rate.  `robot_state_publisher` then produces the TF tree used by RViz.

## Install on the Ubuntu ROS2 computer

Copy the `three_dof_leg_description` folder into a ROS2 workspace `src/`:

```bash
mkdir -p ~/ros2_ws/src
cp -r three_dof_leg_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select three_dof_leg_description
source install/setup.bash
```

## Run

Terminal A starts RViz and the URDF publisher:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch three_dof_leg_description live_leg_rviz.launch.py
```

Terminal B runs the existing trajectory script **on the same Ubuntu machine**.
Use the ST-Link device path, not the Windows COM port:

```bash
cd /path/to/leg_ik_poc
source /opt/ros/jazzy/setup.bash
python3 run_leg_trajectory.py --port /dev/ttyACM0 \
  --amplitude-x-mm 15 --amplitude-z-mm 7.5 \
  --kp 0.50 --kd 0.010 --torque-cap 0.120 --ros2 --execute
```

For the vertical test:

```bash
python3 run_leg_trajectory.py --port /dev/ttyACM0 \
  --motion vertical --vertical-up-mm 20 --vertical-down-mm 15 \
  --kp 0.50 --kd 0.010 --torque-cap 0.120 --ros2 --execute
```

RViz's fixed frame is `base_link`.  At H723 `ZERO`, the visual model must be a
straight vertical leg.  First run a supported, unloaded test.  Stop if the
model moves in the opposite direction to the hardware; that indicates a joint
sign/zero mapping issue, not an RViz problem.

## Why not MoveIt?

MoveIt is a planner for arm-like collision-free trajectories.  This is a
measured-state monitoring step, so it adds no useful function yet.  The same
URDF can be extended to MoveIt later if the leg is mounted as a manipulator.

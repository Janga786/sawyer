# Sawyer Robot Scripts

This repository contains example scripts for controlling the Rethink Robotics Sawyer robot using the Intera SDK.

## Complex Movement Example

The `scripts/complex_movement.py` file demonstrates a sequence of movements that include waving, picking up an object, and placing it elsewhere. It requires a running ROS environment with the Intera SDK installed on the robot or simulator.

### Usage

```bash
rosrun sawyer scripts/complex_movement.py
```

Make sure to enable the robot and clear any existing poses before running the script.

To activate ros on a machine, use either :
- source /opt/ros/humble (for Ubuntu)
- micromamba activate humble (for Arch)

In this repository, load Humble and the workspace together:

```bash
source scripts/enter.sh
```

That sets `TURTLEBOT3_MODEL=burger`, `ROS_DOMAIN_ID=30`, and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
The robot address is `config/robot.env` (copy from `config/robot.env.example`). SSH with `scripts/robot_ssh.sh`.
Nodes run on the laptop. SSH is only used to start the robot.
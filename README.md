# robust-nonlinear-controller

Python ROS 2 Humble workspace for a TurtleBot3 Burger. Nodes run on this laptop. The robot is on the local network. SSH is only used to start the robot.

ROS 2 Humble is the `humble` micromamba environment. Activate the workspace with:

```bash
source scripts/enter.sh
```

That command activates Humble, sets the Burger defaults, and sources the colcon install space when it exists.

## Defaults

These values live in `config/workspace.env`. The robot must use the same ones.

| Variable | Value | Role |
| --- | --- | --- |
| `TURTLEBOT3_MODEL` | `burger` | Selects the Burger description and simulation |
| `ROS_DOMAIN_ID` | `30` | Isolates this robot from other ROS 2 graphs |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Matches the default middleware on Ubuntu Humble |
| `ROS_LOCALHOST_ONLY` | `0` | Allows discovery on the local network |

NumPy and Matplotlib were already in the Humble environment. SciPy is installed there as well.

## Build

```bash
scripts/build_workspace.sh
source scripts/enter.sh
```

Run the build again after you edit a Python file. The install step copies the package into `install/`.

## Publisher and subscriber

The `rnc_demo` package publishes and subscribes to `std_msgs/String` on the `chatter` topic.

```bash
source scripts/enter.sh
ros2 launch rnc_demo talker_listener.launch.py
```

Two separate terminals:

```bash
ros2 run rnc_demo talker
ros2 run rnc_demo listener
```

## Simulation

The testing zone is `src/rnc_demo/worlds/table_1m.world`. It is a square table. The clear surface inside the rim is 1.00 m by 1.00 m, and the top face is 0.75 m above the floor. The Burger starts in the center.

Run the simulation on domain 0. `scripts/enter.sh` joins the real robot on domain 30, and a shared domain would mix `/cmd_vel` with the Burger.

```bash
source scripts/enter.sh
export ROS_DOMAIN_ID=0 # To not control the real turtlebot
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop
ros2 launch rnc_demo table_world.launch.py
```

In another terminal:

```bash
source scripts/enter.sh
export ROS_DOMAIN_ID=0
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop
ros2 run turtlebot3_teleop teleop_keyboard
```

`x_pose`, `y_pose`, and `z_pose` change the spawn pose. The default `z_pose` is `0.76`, one centimetre above the table top.

## Real robot

Bringup runs on the robot. It talks to the motors and the sensors, and it subscribes to `/cmd_vel`.

Teleop and later controllers run on this laptop. They publish `/cmd_vel`. The messages cross the local network. SSH is not required for that traffic.

Both machines must use the same `ROS_DOMAIN_ID` and the same `RMW_IMPLEMENTATION`.

Discovery uses a TCP server on the robot, port `42100`, because UDP replies from the robot do not reach this laptop. `/cmd_vel` itself still travels over UDP, which is what the motor node reads. `scripts/enter.sh` writes that client profile. Start the server on the robot once after each boot:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
fastdds discovery -i 0 -t 0.0.0.0 -q 42100
```

Bringup and teleop on the robot must use the same client profile, `~/rnc/fastdds_client.xml`, via `FASTRTPS_DEFAULT_PROFILES_FILE`. Restart bringup after setting that variable. Then, on this laptop:

```bash
source scripts/enter.sh
ros2 topic list
```

1. Copy `config/robot.env.example` to `config/robot.env`.
2. Set `ROBOT_HOST` to the robot IPv4 address.
3. Connect with `scripts/robot_ssh.sh` when you need a shell on the robot.

On the robot, source the Ubuntu Humble install and use the same domain, model, and middleware:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch turtlebot3_bringup robot.launch.py
```

Back on this laptop:

```bash
source scripts/enter.sh
ros2 topic list
ros2 run turtlebot3_teleop teleop_keyboard
```

`/scan`, `/odom`, and `/imu` should appear in `ros2 topic list` before you drive. Later controller nodes use those topics and `/cmd_vel`.

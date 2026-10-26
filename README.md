# Robust Nonlinear Controller (SMC) for TurtleBot3
This ROS 2 Humble package implements a **Sliding Mode Controller (SMC)** for a TurtleBot3 Burger. The robot tracks a dynamically generated parametric trajectory. It uses an Extended Kalman Filter (EKF) to fuse Odometry and IMU data, ensuring high-frequency state estimation and robustness against wheel slip.

## Global Architecture
The control loop is completely decoupled and relies on three custom ROS 2 nodes running simultaneously:
1. **`trajectory_node`**: The parametric trajectory generator. It uses symbolic math (`sympy`) to compute the desired position, velocity, and acceleration at any given time $t$. It waits for the robot's first pose message before starting its timer to prevent desynchronization. It publishes on `/desired_trajectory`.
2. **`odom_node`**: The state estimator bridge. It subscribes to the EKF output (`/odometry/filtered`), converts the 3D quaternion into a simple 2D Yaw angle, and publishes a clean `Pose2D` on `/robot_pose`.
3. **`smc_node`**: The core controller. It subscribes to both `/robot_pose` and `/desired_trajectory`. It calculates the tracking errors, applies the Sliding Mode Control mathematical law, and publishes velocity commands to `/cmd_vel`. It also saves the live trajectory data to a CSV file for post-process plotting.
---

## Prerequisites & Installation

### Dependencies
Install the ROS 2 EKF package and required Python libraries. *(Note: `numpy<2` is enforced to prevent compatibility issues with ROS 2's default matplotlib).*
```bash
sudo apt install ros-humble-robot-localization
pip install sympy pandas matplotlib "numpy<2"
```

### Build
You can build the workspace using standard `colcon`:
```bash
colcon build --symlink-install
```
---

## Environment Variables
We provide two specific scripts to quickly configure your DDS middleware, domain ID, and Gazebo paths depending on whether you are running a simulation or the physical robot.
**For Simulation (Localhost, CycloneDDS, Domain 0):**
```bash
source sim_vars.sh
```
**For the Real Robot (Network enabled, FastRTPS, Domain 30):**
```bash
source real_robot_vars.sh
```
---

## Launch Files
The project features three distinct launch files tailored for different use cases. All launch files automatically handle the required delays to ensure Gazebo and DDS are fully initialized before the control loop starts.
### 1. `drawn_trajectory_simulation.launch.py` (Recommended for Dev)
Runs a lightweight, headless Gazebo server in the background and opens **RViz2** for 3D visualization. It displays the robot, the TF frames, and draws the desired path in real-time.
```bash
ros2 launch rnc_demo drawn_trajectory_simulation.launch.py
```
### 2. `simulation.launch.py` (Full Physics GUI)
Runs the full **Gazebo Client (GUI)**. Useful if you want to visually inspect the physical interactions, add obstacles, or check for wheel slip in the 3D world.
```bash
ros2 launch rnc_demo simulation.launch.py
```
### 3. `real_robot.launch.py` (Physical Deployment)
To be used when the physical TurtleBot3 is powered on and connected to the same Wi-Fi network. This file only launches RViz2 and the three control nodes. (Gazebo and spawn nodes are excluded).
```bash
ros2 launch rnc_demo real_robot.launch.py
```
---

## Configuration
Control tuning and sensor fusion parameters are exposed in YAML files inside the `config/` directory:
* **`smc_params.yaml`**: Tune the SMC behavior.
* **`ekf.yaml`**: Configures the Extended Kalman Filter.
---

## Plotting Results
During execution, `smc_node` continuously logs the desired and actual trajectories. Once the trajectory is complete, it safely closes `trajectory_data.csv`.
To generate a high-quality Matplotlib graph for academic reporting and performance validation, run:
```bash
python3 plot_trajectory.py
```
This will display the plot and save it as `trajectory_result.png` in your workspace.

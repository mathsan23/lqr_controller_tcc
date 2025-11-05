# Lateral-Directional LQR-Servo (ROSFlight/ROSPlane)

Discrete LQR controller (with course-error integrator) for fixed-wing UAVs on ROSFlight/ROSPlane. Includes Q/R profiles for slow/normal/fast behavior and Gazebo simulation steps.

## Prerequisites
- Install ROS 2 Humble Desktop + dev tools:
  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

## Install ROSFlight (ROS 2)
- Follow:
  - https://docs.rosflight.org/git-main/user-guide/ros2-setup/#installing-rosflight
- Typical:
  ```bash
  mkdir -p ~/dev/rosflight_ws/src
  cd ~/dev/rosflight_ws/src
  # clone rosflight_ros_pkgs per docs
  cd ~/dev/rosflight_ws
  colcon build
  source install/setup.bash
  ```

## Shell configuration (~/.bashrc)
```bash
source /opt/ros/humble/setup.bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # WSL2 + GPU only
source /usr/share/gazebo/setup.sh
chmod 0700 /run/user/1000
```
- NVIDIA WSL2 guide: https://docs.nvidia.com/cuda/wsl-user-guide/index.html

## Install ROSPlane (same workspace)
- Follow:
  - https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/
- Rebuild:
  ```bash
  cd ~/dev/rosflight_ws
  colcon build
  source install/setup.bash
  ```

## Run simulation
1) Launch Gazebo:
   ```bash
   ros2 launch rosflight_sim fixedwing_gazebo.launch.py use_vimfly:=true
   ```
2) Load firmware params:
   ```bash
   cd ~/dev/rosflight_ws/src/rosflight_ros_pkgs/rosflight_sim/params
   ros2 service call /param_load_from_file rosflight_msgs/srv/ParamFile "{filename: $(pwd)/fixedwing_firmware.yaml}"
   ```
3) Calibrate sensors:
   ```bash
   ros2 service call /calibrate_imu std_srvs/srv/Trigger
   ros2 service call /calibrate_baro std_srvs/srv/Trigger
   ```
4) Persist params:
   ```bash
   ros2 service call /param_write std_srvs/srv/Trigger
   ```
5) Launch ROSPlane autonomy:
   ```bash
   ros2 launch rosplane_sim sim.launch.py
   ```
6) Launch GCS (RViz):
   ```bash
   ros2 launch rosplane_gcs rosplane_gcs.launch.py
   ```
7) Load mission:
   ```bash
   cd ~/dev/rosflight_ws/src/rosplane/rosplane/missions
   ros2 service call /load_mission_from_file rosflight_msgs/srv/ParamFile "{filename: $(pwd)/fixedwing_mission.yaml}"
   ```
8) Arm and start:
   ```bash
   ros2 service call /toggle_arm std_srvs/srv/Trigger
   ros2 service call /toggle_override std_srvs/srv/Trigger
   ```
9) Optional:
   ```bash
   ros2 service call /publish_next_waypoint std_srvs/srv/Trigger
   ros2 param set /path_planner num_waypoints_to_publish_at_start 100
   ros2 param set /autopilot lqr_true true
   ```

## Q/R profiles (state: [v, p, r, phi, chi, Ichi], input: delta_a)
- Slow: `Q = diag(0.3, 0.3, 0.5, 6.0, 8.0, 0.5)`, `R = 20.0`
- Normal: `Q = diag(1.0, 1.2, 1.8, 20.0, 24.0, 2.0)`, `R = 10.0`
- Fast: `Q = diag(4.0, 6.0, 8.0, 80.0, 60.0, 6.0)`, `R = 5.0`

## Credits
- ROSFlight docs: https://docs.rosflight.org/ — GitLab: https://gitlab.com/rosflight/rosflight
- ROSPlane docs: https://docs.rosflight.org/git-main/user-guide/installation/installation-sim/ — GitLab: https://gitlab.com/rosflight/rosplane


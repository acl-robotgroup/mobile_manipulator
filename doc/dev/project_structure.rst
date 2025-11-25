===========================
ROS Workspace Architecture
===========================

This tutorial outlines a typical ROS robot workspace and shows how the mobile
manipulator repository follows that pattern.

Typical ROS Robot Workspace
===========================

- Workspace root: `src/` contains all packages; Docker/compose stacks set up the
  ROS environment and should be used for ROS commands.
- Metapackage/tooling: a small package to manage external sources (`.repos`
  files), patch scripts, and shared tooling.
- Robot description: URDF/Xacro, meshes, and RViz configurations plus a
  `robot_state_publisher` launch file.
- Bringup: launch and parameter files to start hardware drivers, sensors, and
  namespaces for duplicated devices.
- Simulation: Gazebo worlds, spawn/bridge launchers, and topic bridges to sync
  ROS <-> Gazebo transports.
- Docs: Sphinx entrypoint for project notes, diagrams, and operator guides.

How This Repository Implements It
=================================

- `mobile_manipulator`: metapackage plus repo tooling. Holds
  `mobile_manipulator.repos` for external source fetches, `patches.yaml`, and
  `scripts/patch_repos.py` to apply upstream patches after import.
- `mobile_manipulator_description`: URDF/Xacro and RViz setup.
  `description.launch.py` uses `FindPackageShare`/`PathJoinSubstitution` to
  start `robot_state_publisher` with configurable namespaces and `use_sim_time`.
- `mobile_manipulator_bringup`: hardware launch and parameters. Includes
  `sensors.yaml`, lidar namespaces (`lidar_front`, `lidar_rear`), and Orbbec
  camera startup with `TimerAction` delays to stagger initialization.
- `mobile_manipulator_gazebo_sim`: simulation support. Provides Gazebo worlds,
  bridge configuration in `config/gz_bridge.yaml`, and launch files
  (`sim.launch.py`, `spawn.launch.py`) to start the simulator and spawn the
  robot.
- `docs/`: Sphinx documentation entrypoint (this guide) for structure, usage
  notes, and future guides.

Simulation and Launch Notes
===========================

- Override Xacro parameters through `xacro_args` (e.g., `use_sim_time:=true`) to
  keep launch files generic across sim and hardware.
- Extend Gazebo <-> ROS topic mirroring in
  `mobile_manipulator_gazebo_sim/config/gz_bridge.yaml` instead of ad-hoc
  bridges.
- Keep sensor namespaces explicit (`lidar_front`, `lidar_rear`, `camera_*`) to
  avoid frame collisions in RViz and downstream consumers.

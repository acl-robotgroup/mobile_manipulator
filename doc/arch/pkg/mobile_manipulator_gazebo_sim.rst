.. _arch_pkg_mobile_manipulator_gazebo_sim:

==============================
mobile_manipulator_gazebo_sim
==============================

Purpose
=======

- Runs the robot in Gazebo using the same description as hardware.
- Spawns the robot and bridges key topics between ROS 2 and Gazebo transport
  for sensor and command streams.
- Hosts simulation-specific URDF overlays (Gazebo plugins such as DiffDrive)
  and world assets.

Key Files
=========

- `launch/sim.launch.py`: boots Gazebo and prepares bridge configuration.
- `launch/spawn.launch.py`: spawns the robot entity into a chosen world.
- `config/gz_bridge.yaml`: declares Gazebo <-> ROS topic bridges (sensors,
  ``cmd_vel``, odometry, TF).
- `urdf/*.xacro`: Gazebo-specific macros (e.g., DiffDrive and sensor plugin
  hooks) layered on top of the base description.
- `worlds/*.sdf`: Gazebo world files for testing environments.

Technologies
============

- `Gazebo Sim <https://gazebosim.org/docs/harmonic>`_ with ROS 2 integration
  (`ros_gz <https://gazebosim.org/docs/harmonic/ros2_integration>`_ and bridge
  configuration `ros_gz topics
  <https://github.com/gazebosim/ros_gz/tree/ros2#bridge-configuration>`_).
- Gazebo ``DiffDrive`` system plugin for base kinematics instead of
  ``ros2_control``.
- `ROS 2 launch
  <https://docs.ros.org/en/jazzy/Concepts/Intermediate/Launch/Launch-system.html>`_
  for composing simulator startup and spawn flows.

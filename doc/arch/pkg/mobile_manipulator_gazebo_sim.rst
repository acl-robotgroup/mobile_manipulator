.. _arch_pkg_mobile_manipulator_gazebo_sim:

==============================
mobile_manipulator_gazebo_sim
==============================

Purpose
=======

- Runs the robot in Gazebo with the same description used for hardware.
- Spawns the robot, loads controllers, and bridges key topics between ROS 2 and
  Gazebo transport for sensor and command streams.
- Hosts simulation-specific URDF overlays (controllers, Gazebo plugins) and
  world assets.

Key Files
=========

- `launch/sim.launch.py`: boots Gazebo, loads controllers, and prepares bridge
  configuration.
- `launch/spawn.launch.py`: spawns the robot entity into a chosen world.
- `config/gz_bridge.yaml`: declares Gazebo <-> ROS topic bridges.
- `config/controllers.yaml`: controller setup for `ros2_control` integration.
- `urdf/*.xacro`: Gazebo-specific macros (e.g., `ros2_control` and plugin hooks)
  layered on top of the base description.
- `worlds/*.sdf`: Gazebo world files for testing environments.

Technologies
============

- `Gazebo Sim <https://gazebosim.org/docs/harmonic>`_ with ROS 2 integration
  (`ros_gz <https://gazebosim.org/docs/harmonic/ros2_integration>`_ and bridge
  configuration `ros_gz topics
  <https://github.com/gazebosim/ros_gz/tree/ros2#bridge-configuration>`_).
- `ros2_control <https://control.ros.org/humble/index.html>`_ and
  `ros2_controllers` for actuator interfaces inside simulation.
- `ROS 2 launch
  <https://docs.ros.org/en/humble/Concepts/Intermediate/Launch/Launch-system.html>`_
  for composing simulator startup and spawn flows.

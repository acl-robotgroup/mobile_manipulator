.. _arch_pkg_mobile_manipulator_bringup:

==========================
mobile_manipulator_bringup
==========================

Purpose
=======

- Starts the hardware stack: base drivers, lidars, depth cameras, and any
  auxiliary nodes required to operate the platform.
- Applies sensor parameters and namespaces so duplicated devices (e.g.,
  `lidar_front`, `lidar_rear`, `camera_*`) stay isolated in TF and topics.
- Provides RViz setups for monitoring live hardware.

Key Files
=========

- `launch/bringup.launch.py`: orchestrates hardware nodes and namespaces; uses
  `TimerAction` delays to stagger camera startup.
- `config/sensors.yaml`: parameter file for drivers and topic names.
- `launch/visualize.launch.py` and `rviz/display.rviz`: RViz visualization for
  hardware sessions.

Technologies
============

- `ROS 2 launch
  <https://docs.ros.org/en/humble/Concepts/Intermediate/Launch/Launch-system.html>`_
  for composing bringup flows and timed actions.
- Namespaces/remapping (`ros2` remapping guide
  <https://docs.ros.org/en/humble/Concepts/Intermediate/Remapping/Remapping.html>`_)
  to prevent topic/frame collisions across identical sensors.
- `tf2 <https://docs.ros.org/en/humble/Concepts/Intermediate/TF.html>`_ to keep
  hardware frames consistent with the description package.
- Parameter YAML files (`ROS 2 parameters
  <https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html>`_) to
  configure drivers without code changes.

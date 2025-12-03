.. _arch_pkg_mobile_manipulator_bringup:

==========================
mobile_manipulator_bringup
==========================

Purpose
=======

- Boots the hardware stack: two RPLIDARs plus four Orbbec depth cameras, with
  namespacing so identical devices remain isolated in TF and topics.
- Uses a single parameter file to hold serial numbers, device IDs, and shared
  driver settings.
- Optionally launches RViz alongside the drivers for live monitoring.

Key Files
=========

- `launch/bringup.launch.py`: includes the description publisher, spins up an
  `rclcpp_components` container with four ``orbbec_camera`` nodes, and runs
  two ``sllidar_node`` processes (front and rear). A ``visualize`` argument
  gates launching RViz.
- `config/bringup.yaml`: driver parameters shared across all sensors (serial
  numbers, frame IDs, resolutions, point cloud toggles, etc.).
- `rviz/visualize.rviz`: RViz layout for monitoring live sensor topics.

Technologies
============

- `ROS 2 launch
  <https://docs.ros.org/en/jazzy/Concepts/Intermediate/Launch/Launch-system.html>`_
  for composing the bringup flow.
- Namespaces/remapping (`ros2` remapping guide
  <https://docs.ros.org/en/jazzy/Concepts/Intermediate/Remapping/Remapping.html>_)
  keep duplicated sensors collision-free.
- `rclcpp_components` containers for hosting multiple `orbbec_camera` components
  in a single process.
- `tf2 <https://docs.ros.org/en/jazzy/Concepts/Intermediate/TF.html>`_ to keep
  hardware frames consistent with the description package.
- Parameter YAML files (`ROS 2 parameters
  <https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html>`_) to
  configure drivers without code changes.

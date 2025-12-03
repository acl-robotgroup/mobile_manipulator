.. _arch_pkg_mobile_manipulator_navigation:

==============================
mobile_manipulator_navigation
==============================

Purpose
=======

- Hosts the Nav2-based navigation stack configured for the robot footprint and
  odom/map frames.
- Uses NVBLOX costmap layers generated from the perception stack to plan around
  3D obstacles.
- Provides an optional RViz view for monitoring plans and costmaps.

Key Files
=========

- ``launch/navigation.launch.py``: starts Nav2 lifecycle manager plus behavior,
  planner, controller, and BT navigator nodes; toggles RViz with ``visualize``.
- ``config/navigation.yaml``: parameters for Nav2 behaviors, BT navigator,
  SMAC planner, graceful controller, and global/local costmaps with NVBLOX
  layers.
- ``rviz/visualize.rviz``: RViz configuration aligned with the Nav2 topics.

Technologies
============

- `Nav2 <https://docs.nav2.org/>`_ stack components (behavior server,
  bt_navigator, smac planner, graceful controller).
- `NVBLOX Nav2 costmap layer
  <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/nav2_costmap_plugin.html>`_
  to project 3D maps into 2D costmaps.
- `ROS 2 launch
  <https://docs.ros.org/en/jazzy/Concepts/Intermediate/Launch/Launch-system.html>`_
  for composing the navigation startup with optional simulation time and RViz.

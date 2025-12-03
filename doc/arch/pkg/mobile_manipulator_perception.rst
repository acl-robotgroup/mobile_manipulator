.. _arch_pkg_mobile_manipulator_perception:

=============================
mobile_manipulator_perception
=============================

Purpose
=======

- Provides perception pipelines for mapping and localization: front/rear lidar
  filtering and merging, visual SLAM (Isaac ROS), and dense mapping with
  NVBLOX.
- Records synchronized camera/odom bags plus helpers to convert recordings into
  mapping datasets and visual localization maps.
- Offers RViz layouts for inspecting perception outputs.

Key Files
=========

- ``launch/perception.launch.py``: main stack with laser filters/merger,
  ``visual_slam_node`` (mapping vs localization switch), ``nvblox_node``, and an
  optional RViz session.
- ``launch/record.launch.py``: runs H.264 encoders for stereo infra streams and
  records camera + odom topics into separate rosbag outputs.
- ``config/perception.yaml``: parameters for laser filters, dual_laser_merger,
  visual SLAM, NVBLOX (ESDF/TSDF), and related frames.
- ``config/rosbag_to_mapping_data.yaml``: camera topic mapping used by the map
  conversion helper.
- ``scripts/rosbag_to_mapping_data.sh``: post-processing script to turn recorded
  bags into visual localization map assets.
- ``rviz/visualize.rviz``: RViz layout tuned for the perception stack.

Technologies
============

- `Isaac ROS Visual SLAM
  <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/>`_
  for stereo VIO/SLAM with map save/load.
- `NVBLOX <https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/>`_
  for TSDF/ESDF reconstruction and costmap slicing.
- `laser_filters <http://wiki.ros.org/laser_filters>`_ and
  ``dual_laser_merger`` for scan cleanup and fusion.
- `rosbag2 <https://docs.ros.org/en/jazzy/How-To-Guides/Ros2bag.html>`_ with
  H.264 camera encoding (``isaac_ros_h264_encoder``) for efficient recording.

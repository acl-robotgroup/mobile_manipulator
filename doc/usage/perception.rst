.. _usage_perception:

================
Perception Stack
================

Launch files for mapping/localization, dense mapping, and bagging.

.. note::

   Checkout package architecture at
   :ref:`arch_pkg_mobile_manipulator_perception`.

Launch Files
============

``perception.launch.py``
------------------------

Runs laser filtering/merging, stereo visual SLAM, NVBLOX mapping, and optional
RViz.

.. code-block:: bash

   ros2 launch mobile_manipulator_perception perception.launch.py

Arguments
~~~~~~~~~

- ``use_sim_time`` (default ``false``): switch to simulation clock.
- ``is_mapping`` (default ``true``): if true, saves a map to ``map``; if false,
  loads an existing map from ``map``.
- ``map`` (default ``map``): directory for saving/loading maps.
- ``visualize`` (default ``false``): open RViz with ``rviz/visualize.rviz``.

What it does
~~~~~~~~~~~~

- Applies front/rear laser filters (angular window + range) and merges into a
  single ``scan`` topic via ``dual_laser_merger``.
- Starts ``visual_slam_node`` with stereo infra images from ``camera_front``;
  toggles map save/load parameter name based on ``is_mapping`` and publishes
  odometry.
- Launches ``nvblox_node`` consuming depth/color from front/left/right cameras
  to build TSDF/ESDF maps.
- Optionally opens RViz when ``visualize:=true``; sets ``use_sim_time`` for all
  nodes when requested.

``record.launch.py``
--------------------

Records stereo infra streams (compressed) and odometry while optionally using
simulation time.

.. code-block:: bash

   ros2 launch mobile_manipulator_perception record.launch.py bag_path:=/tmp/bags

Arguments
~~~~~~~~~

- ``use_sim_time`` (default ``false``): set recorded topics to simulation time.
- ``bag_path`` (default ``.``): directory to store the ``camera`` and ``pose``
  bag outputs.

What it does
~~~~~~~~~~~~

- Runs H.264 encoders for ``camera_front/infra1`` and ``infra2`` streams to
  produce compressed topics.
- Records TF, stereo camera info/streams, and odometry into separate rosbag
  outputs under ``bag_path``.
- Applies ``use_sim_time`` globally when enabled.

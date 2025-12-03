.. _usage_bringup:

==================
Hardware Bringup
==================

Launch files for the live robot: base, lidars, depth cameras, and visualization.

.. note::

   Checkout package architecture at :ref:`arch_pkg_mobile_manipulator_bringup`.

Launch Files
============

``bringup.launch.py``
---------------------

Starts all hardware drivers (two RPLIDARs + four Orbbec cameras) with optional
RViz visualization.

.. code-block:: bash

   ros2 launch mobile_manipulator_bringup bringup.launch.py

Arguments
~~~~~~~~~

- ``visualize`` (default ``false``): launch RViz with ``rviz/visualize.rviz``.
  The drivers run either way.

What it does
~~~~~~~~~~~~

- Includes ``mobile_manipulator_description/launch/description.launch.py`` so TF
  is available immediately.
- Starts two ``sllidar_node`` processes in ``lidar_front`` and ``lidar_rear``
  namespaces using parameters from ``config/bringup.yaml``.
- Spins up an ``rclcpp_components`` container running four
  ``orbbec_camera::OBCameraNodeDriver`` components in the ``camera_*``
  namespaces, reusing the same parameter file.
- Optionally opens RViz when ``visualize:=true``.

Configuration
=============

- Edit ``config/bringup.yaml`` to update device serial numbers, frame IDs, and
  camera resolutions before running the launch file.

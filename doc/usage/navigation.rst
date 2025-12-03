.. _usage_navigation:

=============
Navigation
=============

Launch files for Nav2 planning and control.

.. note::

   Checkout package architecture at
   :ref:`arch_pkg_mobile_manipulator_navigation`.

Launch Files
============

``navigation.launch.py``
------------------------

Starts Nav2 lifecycle manager and core servers with optional RViz view.

.. code-block:: bash

   ros2 launch mobile_manipulator_navigation navigation.launch.py

Arguments
~~~~~~~~~

- ``use_sim_time`` (default ``false``): switch Nav2 nodes to simulation clock.
- ``visualize`` (default ``false``): open RViz with ``rviz/visualize.rviz``.

What it does
~~~~~~~~~~~~

- Launches Nav2 lifecycle manager with behavior server, BT navigator, planner,
  and controller nodes using ``config/navigation.yaml``.
- Configures SMAC hybrid planner and Graceful Controller for the robot
  footprint, using NVBLOX costmap layers for obstacle data.
- Optionally opens RViz to view costmaps and planned paths when
  ``visualize:=true``.
- Applies ``use_sim_time`` to all nodes when enabled.

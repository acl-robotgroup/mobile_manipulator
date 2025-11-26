.. _usage_bringup:

==================
Hardware Bringup
==================

Launch files for the live robot: base, lidars, depth cameras, and visualization.
See package architecture at :ref:`mobile_manipulator_bringup`.

Full hardware stack
===================

.. code-block:: bash

   ros2 launch mobile_manipulator_bringup bringup.launch.py

Arguments
---------

- No arguments; device driver parameters are set in ``config/sensors.yaml``.

What it does
------------

- Includes ``mobile_manipulator_description/launch/description.launch.py`` so TF
  is available immediately.
- Namespaces duplicated sensors (``lidar_front``, ``lidar_rear``, ``camera_*``)
  to keep topics unique.
- Starts RPLIDAR nodes plus four Orbbec cameras, staggering their startup with
  ``TimerAction`` to avoid USB spikes.

RViz-only view
==============

.. code-block:: bash

   ros2 launch mobile_manipulator_bringup visualize.launch.py

Arguments
---------

- No launch-time arguments.

What it does
------------

- Starts RViz with ``rviz/display.rviz`` from this package.
- Leaves hardware drivers untouched—useful for remote visualization or pairing
  with simulation bridges.

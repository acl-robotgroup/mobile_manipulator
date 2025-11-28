.. _usage_gazebo_sim:

=================
Gazebo Simulation
=================

Run the robot in Gazebo with sensors and controllers simulated. See package
architecture at :ref:`mobile_manipulator_gazebo_sim`.

Full Simulation Stack
=====================

.. code-block:: bash

   ros2 launch mobile_manipulator_gazebo_sim sim.launch.py

Arguments
---------

- ``world`` (path, default package ``worlds/depo.sdf``): SDF world file to load;
  override with an absolute path or another file from
  ``mobile_manipulator_gazebo_sim/worlds``.

What it does
------------

- Starts ``ros_gz_sim`` with the requested world and clean shutdown behavior.
- Includes ``spawn.launch.py`` to push the URDF into Gazebo with Gazebo-specific
  plugins.
- Launches ``mobile_manipulator_bringup/launch/visualize.launch.py`` so RViz is
  open alongside simulation.
- Runs ``ros_gz_bridge`` with ``config/gz_bridge.yaml`` to bridge sensors,
  ``cmd_vel``, odometry, and TF.

Spawn only
==========

Use this when you already have Gazebo running and just need the robot entity.

.. code-block:: bash

   ros2 launch mobile_manipulator_gazebo_sim spawn.launch.py

Arguments
---------

- ``x`` (float, default ``0``): X coordinate for spawn.
- ``y`` (float, default ``0``): Y coordinate for spawn.
- ``z`` (float, default ``0``): Z coordinate for spawn.
- ``yaw`` (float, default ``0``): Yaw angle in radians.

What it does
------------

- Re-runs ``mobile_manipulator_description/launch/description.launch.py`` with a
  Gazebo-specific overlay and ``use_sim_time`` enabled.
- Uses ``ros_gz_sim create`` to spawn the model into the current simulation.
- Relies on Gazebo's native ``DiffDrive`` system plugin for base motion (no ROS
  controller manager).

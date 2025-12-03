.. _usage_gazebo_sim:

=================
Gazebo Simulation
=================

Run the robot in Gazebo with sensors and controllers simulated.

.. note::

   Checkout package architecture at
   :ref:`arch_pkg_mobile_manipulator_gazebo_sim`.

Launch Files
============

``sim.launch.py``
-----------------

Boots Gazebo, spawns the robot, and bridges topics to ROS 2.

.. code-block:: bash

   ros2 launch mobile_manipulator_gazebo_sim sim.launch.py

Arguments
~~~~~~~~~

- ``world`` (path, default package ``worlds/depo.sdf``): SDF world file to load;
  override with an absolute path or another file from
  ``mobile_manipulator_gazebo_sim/worlds``.

What it does
~~~~~~~~~~~~

- Starts ``ros_gz_sim`` with the requested world and clean shutdown behavior.
- Includes ``spawn.launch.py`` to expand the URDF with Gazebo overlays and push
  the entity into simulation.
- Runs ``ros_gz_bridge`` with ``config/gz_bridge.yaml`` to bridge sensors and
  commands between ROS 2 and Gazebo transport.

``spawn.launch.py``
-------------------

Use this when you already have Gazebo running and just need the robot entity.

.. code-block:: bash

   ros2 launch mobile_manipulator_gazebo_sim spawn.launch.py

Arguments
~~~~~~~~~

- ``visualize`` (default ``false``): launch RViz with the hardware layout while
  Gazebo runs.
- ``x`` (float, default ``0``): X coordinate for spawn.
- ``y`` (float, default ``0``): Y coordinate for spawn.
- ``z`` (float, default ``0``): Z coordinate for spawn.
- ``yaw`` (float, default ``0``): Yaw angle in radians.

What it does
~~~~~~~~~~~~

- Re-runs ``mobile_manipulator_description/launch/description.launch.py`` with a
  Gazebo-specific overlay (``urdf/gazebo.xacro``) and ``use_sim_time`` enabled.
- Uses ``ros_gz_sim create`` to spawn the model into the current simulation.
- Sequentially starts ``joint_state_broadcaster`` then
  ``diff_drive_base_controller`` once the entity exists.
- Optionally opens RViz with ``visualize:=true``.

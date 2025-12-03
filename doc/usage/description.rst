.. _usage_description:

=================
Robot Description
=================

Launch files that publish the URDF and provide a quick RViz view.

.. note::

   Checkout package architecture at
   :ref:`arch_pkg_mobile_manipulator_description`.

Launch Files
============

``description.launch.py``
-------------------------

Used to publish the robot description and TF tree via `robot_state_publisher`.

.. code-block:: bash

   ros2 launch mobile_manipulator_description description.launch.py

Arguments
~~~~~~~~~

- ``use_sim_time`` (default ``false``): Switch to simulation clock if set
  to ``true``.
- ``xacro_args`` (default empty): Extra ``name:=value`` pairs forwarded
  to the top-level Xacro. Bare tokens are treated as boolean flags.

What it does
~~~~~~~~~~~~

- Expands ``urdf/mobile_manipulator.xacro`` with the provided mappings.
- Rewrites ``package://`` URIs to absolute file paths for Gazebo (since Gazebo 
  does not understand ROS package URIs).
- Starts ``robot_state_publisher`` with the resulting URDF.

``display.launch.py``
---------------------

Used to visualize the robot model in RViz with sliders to move joints
interactively.

.. code-block:: bash

   ros2 launch mobile_manipulator_description display.launch.py

Arguments
~~~~~~~~~

- No arguments.

What it does
~~~~~~~~~~~~

- Includes ``description.launch.py`` to publish robot description and TF tree.
- Starts ``joint_state_publisher_gui`` so you can move joints interactively.
- Opens RViz with ``rviz/display.rviz`` preloaded.

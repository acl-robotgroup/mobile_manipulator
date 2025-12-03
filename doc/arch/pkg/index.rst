.. _arch_pkg:

===============
Package Details
===============

.. toctree::
   :maxdepth: 1
   :hidden:

   mobile_manipulator
   mobile_manipulator_description
   mobile_manipulator_bringup
   mobile_manipulator_gazebo_sim
   mobile_manipulator_perception
   mobile_manipulator_navigation

This section documents what packages are included in this project and what are
they responsible for.

This project is organized in the same way as a typical robot ROS 2 stack. Please
refer to `Package Organization For a ROS Stack
<https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices>`_
for detail explainations on the rationale behind this structure. Especially what
typically is included in each package as mentioned in the `Core ROS packages for
your ROS stack` section of that article.

In short, this project contains the following packages:

- :ref:`Metapackage <arch_pkg_mobile_manipulator>`: A small package to manage
  other packages of this project as well as external sources and patches for
  them to suit our needs.
- :ref:`Robot description <arch_pkg_mobile_manipulator_description>`:
  Description of the robot hardware layout in `Xacro
  format <https://wiki.ros.org/xacro>`_, which will be expanded into a full
  `URDF <https://wiki.ros.org/urdf>`_ model at runtime.
- :ref:`Bringup <arch_pkg_mobile_manipulator_bringup>`: Launch and parameter
  files to start hardware drivers, sensors.
- :ref:`Gazebo Simulation <arch_pkg_mobile_manipulator_gazebo_sim>`: Worlds,
  bridge and extra xacro files to simulate the robot in `Gazebo
  <https://gazebosim.org/home>`_.
- :ref:`Perception <arch_pkg_mobile_manipulator_perception>`: Laser filtering
  and merging, visual SLAM/localization, dense mapping (NVBLOX), RViz, and
  bagging utilities.
- :ref:`Navigation <arch_pkg_mobile_manipulator_navigation>`: Nav2 stack
  (behavior, planner, controller) configured for the robot footprint with
  NVBLOX-backed costmaps and optional RViz view.

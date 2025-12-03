.. _arch_pkg_mobile_manipulator_description:

==============================
mobile_manipulator_description
==============================

Purpose
=======

- Defines the robot model in Xacro/URDF, including kinematics, visuals, and
  sensor frames reused by both hardware bringup and simulation.
- Supplies RViz configurations and visualization launch files for quick model
  inspection.
- Publishes the TF tree via `robot_state_publisher` with configurable namespace
  and `use_sim_time` so hardware and simulation stay aligned.

Key Files
=========

- `urdf/*.xacro`: parameterized robot, base, arm, and sensor macros; resolves
  external meshes and description dependencies.
- `launch/description.launch.py`: builds the URDF via `xacro`, rewrites
  ``package://`` URIs to absolute paths, and starts `robot_state_publisher`
  with ``use_sim_time`` and arbitrary ``xacro_args`` mappings.
- `launch/display.launch.py` and `rviz/display.rviz`: RViz-based visualization
  entrypoint with a ``joint_state_publisher_gui``.

Technologies
============

- `URDF <https://wiki.ros.org/urdf>`_ for the robot model consumed by downstream
  tools.
- `Xacro <https://wiki.ros.org/xacro>`_ to keep the URDF modular and
  parameterized.
- `robot_state_publisher
  <https://docs.ros.org/en/jazzy/Concepts/About-State-Republication.html>`_
  and `tf2 <https://docs.ros.org/en/jazzy/Concepts/Intermediate/TF.html>`_ to
  broadcast the robot frame tree.
- `RViz 2 <https://docs.ros.org/en/jazzy/Tutorials/RViz/Introduction-To-RViz.html>`_
  for model visualization and debugging.

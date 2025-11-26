.. _arch_pkg_mobile_manipulator:

==================
mobile_manipulator
==================

Purpose
=======

- Provides a metapackage entry point so downstream workspaces can depend on a
  single package that brings in the full robot stack. Refer to `Using variants
  <https://docs.ros.org/en/jazzy/How-To-Guides/Using-Variants.html>`_ for more
  information on metapackages in ROS 2.
- Hosts repository management assets: `.repos` manifest for `vcstool
  <https://wiki.ros.org/vcstool>`_, which defines external dependencies of this
  project.
- Contains git patch files, and the `patch_repos.py` helper to apply curated
  fixes after fetching dependencies.

Key Files
=========

- `mobile_manipulator.repos`: `vcstool` source manifest to fetch external
  repositories.
- `patches.yaml` and `patches/*.diff`: Patch list and payloads for fetched
  repositories.
- `scripts/patch_repos.py`: Script for applying the patches defined in
  `patches.yaml`.

Technologies
============

- `rosdep <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html>`_:
  Unified tool for installing packages via system dependent tools like `apt`.
- `vcstool <https://wiki.ros.org/vcstool>`_: Tool for fetching dependent
  repositories unable to be installed via `rosdep`.
- `git apply <https://git-scm.com/docs/git-apply>`_: Used to apply patches to
  fetched repositories.

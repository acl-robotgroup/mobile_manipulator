.. _arch_pkg_mobile_manipulator:

==================
mobile_manipulator
==================

Purpose
=======

- Metapackage entry point so downstream workspaces depend on one package and
  receive the full stack. See `Using variants
  <https://docs.ros.org/en/jazzy/How-To-Guides/Using-Variants.html>`_ for more
  on ROS 2 metapackages.
- Hosts dependency management assets: a `.repos` manifest for `vcstool
  <https://wiki.ros.org/vcstool>`_ plus a patch manifest to bring third-party
  sources in line with this workspace.
- Provides `scripts/apply_patches.py`, a tiny helper that runs `git apply` for
  each entry in ``patches.yaml`` after sources are fetched.

Key Files
=========

- `mobile_manipulator.repos`: `vcstool` source manifest to fetch external
  repositories.
- `patches.yaml`: Maps cloned repositories to patch payloads in
  ``patches/*.diff``.
- `scripts/apply_patches.py`: Applies the curated patches recorded in
  ``patches.yaml`` using ``git apply``.

Technologies
============

- `rosdep <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html>`_:
  Unified tool for installing packages via system dependent tools like `apt`.
- `vcstool <https://wiki.ros.org/vcstool>`_: Tool for fetching dependent
  repositories unable to be installed via `rosdep`.
- `git apply <https://git-scm.com/docs/git-apply>`_: Used to apply patches to
  fetched repositories.

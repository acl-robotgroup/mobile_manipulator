.. _arch_pkg_mobile_manipulator:

==================
mobile_manipulator
==================

Purpose
=======

- Provides a metapackage entry point so downstream workspaces can depend on a
  single package that brings in the full robot stack.
- Hosts repository management assets: `.repos` manifest for `vcs import`, patch
  instructions, and the `patch_repos.py` helper to apply curated fixes after
  fetching dependencies.
- Carries ament metadata that wires the bringup and description packages
  together for builds and deployments.

Key Files
=========

- `mobile_manipulator.repos`: source manifest consumed by `vcs import` to fetch
  external repositories.
- `patches.yaml` and `patches/*.diff`: patch list and payloads for upstream
  packages.
- `scripts/patch_repos.py`: applies the patches in a workspace after import.

Technologies
============
- `ament_cmake <https://docs.ros.org/en/humble/Tutorials/Intermediate/Building-Your-First-ROS2-Package.html>`_
  metapackage structure for ROS 2 builds.
- `vcs import
  <https://docs.ros.org/en/humble/How-To-Guides/MultipleRepositories.html>`_
  to sync external sources in `src/`.
- `rosdep <https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html>`_
  for dependency installation after sources are fetched and patched.

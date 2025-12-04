.. _usage_install:

============
Installation
============

Use these steps from the source of the workspace.

1. Clone the repository.

   .. code-block:: bash

      git clone https://github.com/acl-robotgroup/mobile_manipulator

2. Fetch dependent repositories.

   .. code-block:: bash

      vcs import < mobile_manipulator/mobile_manipulator/mobile_manipulator.repos

3. Apply patches.

   .. code-block:: bash

      python3 mobile_manipulator/mobile_manipulator/scripts/apply_patches.py mobile_manipulator/mobile_manipulator/patches.yaml

4. Install dependencies.

   .. code-block:: bash

      rosdep install --from-paths . --ignore-src -r -y

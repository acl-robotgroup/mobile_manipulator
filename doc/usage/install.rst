.. _usage_install:

============
Installation
============

Use these steps from the source of the workspace.

1. Clone the repository.

   .. code-block:: bash

      git clone https://github.com/acl-robotgroup/mobile_manipulator

2. Fetch dependent repositories (from the workspace root that contains
   ``src/mobile_manipulator``).

   .. code-block:: bash

      vcs import src < src/mobile_manipulator/mobile_manipulator/mobile_manipulator.repos

3. Apply patches recorded in ``patches.yaml``.

   .. code-block:: bash

      python3 src/mobile_manipulator/mobile_manipulator/scripts/apply_patches.py src/mobile_manipulator/mobile_manipulator/patches.yaml

4. Install dependencies.

   .. code-block:: bash

      rosdep install -i --from-path src -y

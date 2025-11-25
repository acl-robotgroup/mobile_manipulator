# Mobile Manipulator

## Installation

```bash
vcs import src < src/mobile_manipulator/mobile_manipulator/mobile_manipulator.repos

cd src
python3 mobile_manipulator/mobile_manipulator/scripts/patch_repos.py mobile_manipulator/mobile_manipulator/patches.yaml
cd ..

rosdep install -i --from-path src -y
```

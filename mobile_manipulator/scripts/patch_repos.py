#!/usr/bin/env python3
import os
import subprocess
import sys
import yaml


def run(cmd: list[str], cwd: str | None = None):
    res = subprocess.run(cmd,
                         cwd=cwd,
                         text=True,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    if res.returncode != 0:
        raise RuntimeError(f"Command failed: {' '.join(cmd)}\n"
                           f"stdout:\n{res.stdout}\n"
                           f"stderr:\n{res.stderr}")


def main():
    if len(sys.argv) != 2:
        print("Usage: patch_repos.py <config.yaml>", file=sys.stderr)
        sys.exit(1)

    with open(sys.argv[1], "r") as f:
        config = yaml.safe_load(f)

    for repo, info in config["repositories"].items():
        patch = info["patch"]

        run(["git", "apply", os.path.abspath(patch)], cwd=repo)

    print("All patches applied successfully.")


if __name__ == "__main__":
    main()

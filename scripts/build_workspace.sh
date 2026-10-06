#!/usr/bin/env bash
# Build the colcon workspace with the humble environment active.

set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "micromamba is not on PATH." >&2
  exit 1
fi

# Gazebo's setup.sh reads unset path variables. Drop nounset for activation only.
set +u
eval "$(micromamba shell hook --shell bash)"
micromamba activate humble
set -u

# shellcheck disable=SC1091
source "${root}/config/workspace.env"

cd "${root}"
# setuptools 84 rejects `setup.py develop --editable`.
# `colcon build --symlink-install` always passes that flag, so use a normal install.
colcon build

echo "Build finished. Load the workspace with: source scripts/enter.sh"

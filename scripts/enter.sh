#!/usr/bin/env bash
# Load ROS 2 Humble and this workspace into the current shell.
# Usage, from the repository root or anywhere:
#   source scripts/enter.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Source this file so the environment stays in your shell:" >&2
  echo "  source scripts/enter.sh" >&2
  exit 1
fi

_rnc_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "micromamba is not on PATH." >&2
  return 1
fi

# Gazebo's setup.sh reads unset path variables. Restore nounset if the caller had it.
_rnc_nounset=0
case $- in
  *u*) _rnc_nounset=1 ;;
esac
set +u
eval "$(micromamba shell hook --shell bash)"
micromamba activate humble

# shellcheck disable=SC1091
source "${_rnc_root}/config/workspace.env"

if [[ -f "${_rnc_root}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_rnc_root}/install/setup.bash"
else
  echo "Workspace is not built yet. Run: scripts/build_workspace.sh" >&2
fi

if [[ "${_rnc_nounset}" == 1 ]]; then
  set -u
fi
unset _rnc_nounset
unset _rnc_root

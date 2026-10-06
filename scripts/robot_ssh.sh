#!/usr/bin/env bash
# Open an SSH session on the TurtleBot3. The robot address lives in config/robot.env.

set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
env_file="${root}/config/robot.env"

if [[ ! -f "${env_file}" ]]; then
  echo "Create ${env_file} from config/robot.env.example and set ROBOT_HOST." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${env_file}"

if [[ -z "${ROBOT_USER:-}" || -z "${ROBOT_HOST:-}" ]]; then
  echo "Set ROBOT_USER and ROBOT_HOST in ${env_file}." >&2
  exit 1
fi

ssh_args=()
if [[ -n "${ROBOT_SSH_KEY:-}" ]]; then
  ssh_args+=(-i "${ROBOT_SSH_KEY}")
fi

exec ssh "${ssh_args[@]}" "${ROBOT_USER}@${ROBOT_HOST}"

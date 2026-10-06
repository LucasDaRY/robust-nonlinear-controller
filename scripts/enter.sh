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

# UDP from the robot does not reach this laptop. Both sides join a TCP
# discovery server that listens on the robot.
_rnc_robot_host=""
if [[ -f "${_rnc_root}/config/robot.env" ]]; then
  # shellcheck disable=SC1091
  source "${_rnc_root}/config/robot.env"
  _rnc_robot_host="${ROBOT_HOST:-}"
fi
if [[ -n "${_rnc_robot_host}" ]]; then
  _rnc_fastdds="${_rnc_root}/config/fastdds.local.xml"
  cat > "${_rnc_fastdds}" << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>tcp_client</transport_id>
      <type>TCPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="tcp_client" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>tcp_client</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
          <discoveryServersList>
            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
              <metatrafficUnicastLocatorList>
                <locator>
                  <tcpv4>
                    <address>${_rnc_robot_host}</address>
                    <physical_port>42100</physical_port>
                  </tcpv4>
                </locator>
              </metatrafficUnicastLocatorList>
            </RemoteServer>
          </discoveryServersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
  export FASTRTPS_DEFAULT_PROFILES_FILE="${_rnc_fastdds}"
fi
unset _rnc_robot_host _rnc_fastdds

if [[ -f "${_rnc_root}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_rnc_root}/install/setup.bash"
else
  echo "Workspace is not built yet. Run: scripts/build_workspace.sh" >&2
fi

# The ROS 2 daemon keeps the middleware settings from the shell that started it.
ros2 daemon stop >/dev/null 2>&1 || true

if [[ "${_rnc_nounset}" == 1 ]]; then
  set -u
fi
unset _rnc_nounset
unset _rnc_root

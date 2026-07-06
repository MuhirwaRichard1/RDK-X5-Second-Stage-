#!/usr/bin/env bash
# Foreground runner for the navbot agent.
#
#   sudo ./run_agent.sh                # full agent (motors possible via modes)
#   ./run_agent.sh --motors-off        # unprivileged, motors forced off
#
# Root is only needed so motor_controller can write the PWM sysfs when a
# driving mode is started; everything else runs fine as sunrise.
# no -u: ROS setup.bash scripts reference unbound variables
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# shellcheck disable=SC1091
source /opt/tros/humble/setup.bash
# shellcheck disable=SC1091
source "$REPO/install/setup.bash"

export PYTHONPATH="$REPO/app/agent${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO"
exec python3 -m navbot_agent "$@"

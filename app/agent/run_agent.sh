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

# Fast-DDS leaks its /dev/shm data-sharing segments when a node is SIGKILLed
# (a launch that ignores SIGINT, an agent restart under it). They accumulate
# across mode switches until publishes start failing outright with
# "cannot publish data" — /cmd_vel commands then never leave the navigator and
# the robot sits still while it believes it is driving. Nothing is using DDS at
# this point (we are about to start the agent), so any leftover segment is
# stale: clear them so a long session cannot degrade into that state.
#
# Do NOT set FASTRTPS_DEFAULT_PROFILES_FILE here: TROS's hobot_shm hooks it for
# zero-copy and forces RMW_FASTRTPS_USE_QOS_FROM_XML=1, so a profile that does
# not redefine every writer/reader QoS makes all C++ nodes abort with
# NotEnoughMemoryException (see config/fastdds_no_shm.xml for the details).
if ! pgrep -f "ros2 launch|navbot_agent" >/dev/null 2>&1; then
    rm -f /dev/shm/fast_datasharing_* /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
fi
cd "$REPO"
exec python3 -m navbot_agent "$@"

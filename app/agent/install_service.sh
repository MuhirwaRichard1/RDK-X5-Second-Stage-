#!/usr/bin/env bash
# Install the navbot-agent systemd unit (DISABLED by default).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $EUID -ne 0 ]]; then
    echo "run with sudo: sudo $0" >&2
    exit 1
fi

cp "$HERE/navbot-agent.service" /etc/systemd/system/navbot-agent.service
systemctl daemon-reload

echo "installed (disabled). Commands:"
echo "  sudo systemctl start navbot-agent     # start now"
echo "  sudo systemctl stop navbot-agent      # stop (kills any running mode)"
echo "  sudo systemctl enable navbot-agent    # opt in to start at boot"
echo "  journalctl -u navbot-agent -f         # follow logs"

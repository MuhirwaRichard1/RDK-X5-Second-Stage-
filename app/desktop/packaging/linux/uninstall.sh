#!/usr/bin/env bash
# Remove everything install.sh created for the current user.
set -uo pipefail

PREFIX="$HOME/.local"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"

rm -rf "$PREFIX/opt/navbot-console"
rm -f "$PREFIX/bin/navbot-console" \
      "$DATA/applications/navbot-console.desktop" \
      "$DESKTOP_DIR/navbot-console.desktop"
for n in 32 48 64 128 256 512; do
    rm -f "$DATA/icons/hicolor/${n}x${n}/apps/navbot-console.png"
done
update-desktop-database "$DATA/applications" 2>/dev/null || true
echo "NavBot Console uninstalled."

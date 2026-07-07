#!/usr/bin/env bash
# Install NavBot Console for the current user (no root needed):
# app -> ~/.local/opt/navbot-console, launcher on PATH, app-menu entry,
# icons, and a desktop shortcut.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PREFIX="$HOME/.local"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}"
INSTALL_DIR="$PREFIX/opt/navbot-console"

rm -rf "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR" "$PREFIX/bin" "$DATA/applications"
cp -a "$HERE/app/." "$INSTALL_DIR/"
ln -sf "$INSTALL_DIR/navbot-console" "$PREFIX/bin/navbot-console"

for n in 32 48 64 128 256 512; do
    d="$DATA/icons/hicolor/${n}x${n}/apps"
    mkdir -p "$d"
    cp "$HERE/icons/$n.png" "$d/navbot-console.png"
done

sed "s|@EXEC@|$INSTALL_DIR/navbot-console|" "$HERE/navbot-console.desktop" \
    > "$DATA/applications/navbot-console.desktop"

# Desktop shortcut (GNOME needs the "trusted" bit to show it as clickable)
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
if [ -d "$DESKTOP_DIR" ]; then
    cp "$DATA/applications/navbot-console.desktop" "$DESKTOP_DIR/"
    chmod +x "$DESKTOP_DIR/navbot-console.desktop"
    gio set "$DESKTOP_DIR/navbot-console.desktop" \
        metadata::trusted true 2>/dev/null || true
fi

update-desktop-database "$DATA/applications" 2>/dev/null || true
gtk-update-icon-cache "$DATA/icons/hicolor" 2>/dev/null || true

echo "Installed: app menu 'NavBot Console', desktop shortcut, and"
echo "  $PREFIX/bin/navbot-console (make sure ~/.local/bin is on PATH)"

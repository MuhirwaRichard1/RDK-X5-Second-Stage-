#!/usr/bin/env bash
# Build the NavBot Console Linux installer tarball.
#
# PyInstaller does not cross-compile: run this on the SAME arch/OS family as
# the machines that will install it (x86_64 PC or CI runner).
#
#   bash app/desktop/packaging/linux/build.sh
#   -> app/desktop/dist/navbot-console-<version>-linux-<arch>.tar.gz
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> app/desktop

PY="${PYTHON:-python3}"
"$PY" -m pip install --quiet -r requirements.txt "pyinstaller>=6"
"$PY" -m PyInstaller --noconfirm packaging/navbot_console.spec

VER="$("$PY" -c 'import navbot_console; print(navbot_console.__version__)')"
ARCH="$(uname -m)"
NAME="navbot-console-$VER-linux-$ARCH"
STAGE="dist/$NAME"

rm -rf "$STAGE"
mkdir -p "$STAGE"
cp -a dist/navbot-console "$STAGE/app"
cp -a packaging/linux/icons "$STAGE/icons"
cp packaging/linux/navbot-console.desktop "$STAGE/"
cp packaging/linux/install.sh packaging/linux/uninstall.sh "$STAGE/"
chmod +x "$STAGE/install.sh" "$STAGE/uninstall.sh"

tar -C dist -czf "dist/$NAME.tar.gz" "$NAME"
echo "OK: app/desktop/dist/$NAME.tar.gz"

# PyInstaller spec for NavBot Console (PyInstaller >= 6).
# Build from app/desktop:  pyinstaller --noconfirm packaging/navbot_console.spec
# One-dir build: faster startup than one-file and friendlier to antivirus.
import sys
from pathlib import Path

here = Path(SPECPATH)                    # app/desktop/packaging  # noqa: F821
appdir = here.parent                     # app/desktop

a = Analysis(                            # noqa: F821
    [str(here / "launcher.py")],
    pathex=[str(appdir)],
    # mirror the source layout so Path(__file__)/assets works frozen too
    datas=[(str(appdir / "navbot_console" / "assets"), "navbot_console/assets")],
    excludes=["tkinter"],
)
pyz = PYZ(a.pure)                        # noqa: F821

exe = EXE(                               # noqa: F821
    pyz,
    a.scripts,
    exclude_binaries=True,
    name="navbot-console",
    console=False,
    icon=str(here / "icon.ico") if sys.platform == "win32" else None,
)
coll = COLLECT(                          # noqa: F821
    exe,
    a.binaries,
    a.datas,
    name="navbot-console",
)

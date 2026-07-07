#!/usr/bin/env python3
"""Generate the NavBot Console app icon (single source of truth).

Draws a stylized rover — light body, cyan camera visor, antenna with
signal arcs — on a dark rounded square, supersampled then downscaled.

Outputs (all committed):
    navbot_console/assets/icon.png   512px, runtime window icon + Linux
    packaging/linux/icons/<N>.png    hicolor sizes for install.sh
    packaging/icon.ico               multi-size, Windows exe/installer
"""

from pathlib import Path

from PIL import Image, ImageDraw

S = 1024                    # working canvas; every coordinate below is /1024
NAVY_TOP = (22, 40, 63)
NAVY_BOT = (10, 21, 35)
BODY = (233, 241, 247)
DARK = (14, 26, 43)
CYAN = (62, 199, 238)
WHEEL = (30, 58, 84)

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parent / "navbot_console" / "assets"
LINUX_ICONS = HERE / "linux" / "icons"


def rounded(draw, box, r, **kw):
    draw.rounded_rectangle(box, radius=r, **kw)


def make_base():
    # vertical gradient, then rounded-corner alpha mask
    grad = Image.linear_gradient("L").resize((S, S))
    img = Image.composite(Image.new("RGB", (S, S), NAVY_BOT),
                          Image.new("RGB", (S, S), NAVY_TOP), grad).convert("RGBA")
    mask = Image.new("L", (S, S), 0)
    rounded(ImageDraw.Draw(mask), (0, 0, S - 1, S - 1), int(S * 0.22), fill=255)
    img.putalpha(mask)
    d = ImageDraw.Draw(img)

    # antenna mast + tip, offset right of center
    ax = 640
    d.line((ax, 400, ax, 250), fill=BODY, width=26)
    d.ellipse((ax - 34, 216, ax + 34, 284), fill=CYAN)
    # signal arcs around the tip
    for r, w in ((110, 26), (185, 26)):
        d.arc((ax - r, 250 - r, ax + r, 250 + r), start=-105, end=-15,
              fill=CYAN, width=w)

    # wheels first, body overlaps their top half
    for cx in (340, 684):
        d.ellipse((cx - 78, 622, cx + 78, 778), fill=WHEEL)
        d.ellipse((cx - 30, 670, cx + 30, 730), fill=CYAN)

    # rover body + dark visor + camera lens
    rounded(d, (222, 400, 802, 690), 64, fill=BODY)
    rounded(d, (282, 452, 742, 590), 46, fill=DARK)
    d.ellipse((452, 461, 572, 581), fill=CYAN)
    d.ellipse((492, 487, 528, 523), fill=(240, 252, 255))
    return img


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    LINUX_ICONS.mkdir(parents=True, exist_ok=True)
    base = make_base()

    icon512 = base.resize((512, 512), Image.LANCZOS)
    icon512.save(ASSETS / "icon.png")
    for n in (512, 256, 128, 64, 48, 32):
        base.resize((n, n), Image.LANCZOS).save(LINUX_ICONS / f"{n}.png")
    icon512.save(HERE / "icon.ico",
                 sizes=[(16, 16), (24, 24), (32, 32), (48, 48),
                        (64, 64), (128, 128), (256, 256)])
    print(f"wrote {ASSETS / 'icon.png'}, {LINUX_ICONS}/<N>.png, {HERE / 'icon.ico'}")


if __name__ == "__main__":
    main()

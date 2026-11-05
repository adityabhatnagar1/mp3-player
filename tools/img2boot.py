#!/usr/bin/env python3
"""Convert any image into mp3/boot_art.h (128x64, 1-bit, MSB-first) for the boot screen.

  python img2boot.py my_art.png                      # cover-crop, Atkinson dither
  python img2boot.py my_art.jpg --fit contain        # letterbox instead of crop
  python img2boot.py my_art.png --dither bayer --gamma 0.8 --preview prev.png

Needs: pip install pillow
"""
import argparse, sys
from PIL import Image, ImageOps, ImageEnhance, ImageFilter, ImageChops

W, H = 128, 64
BAYER4 = [[0, 8, 2, 10], [12, 4, 14, 6], [3, 11, 1, 9], [15, 7, 13, 5]]


def detect_scale(g):
    """Largest integer block size that the (already 1-bit) image is made of, e.g. 8 for 8x-upscaled pixel art."""
    from math import gcd
    w, h = g.size
    px = g.load()
    k = 0
    for y in range(0, h, 3):
        run = 1
        for x in range(1, w):
            if px[x, y] == px[x - 1, y]:
                run += 1
            else:
                k = gcd(k, run); run = 1
        k = gcd(k, run)
    return max(k, 1)


def load_pixel_perfect(path, crop):
    """For pixel art: undo the upscale with nearest-neighbour and cut a native 128x64 window (no resampling)."""
    im = Image.open(path).convert("L")
    k = detect_scale(im.point(lambda v: 255 if v >= 128 else 0))
    nat = im.resize((im.width // k, im.height // k), Image.NEAREST)
    nw, nh = nat.size
    if nw < W or nh < H:
        sys.exit("native image %dx%d is smaller than %dx%d; use --fit contain instead" % (nw, nh, W, H))
    x = (nw - W) // 2 if crop[0] < 0 else min(crop[0], nw - W)
    y = (nh - H) // 2 if crop[1] < 0 else min(crop[1], nh - H)
    print("pixel-perfect: scale %dx -> native %dx%d, window at x=%d y=%d" % (k, nw, nh, x, y))
    return nat.crop((x, y, x + W, y + H))


def load(path, fit, focus):
    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "P"):                    # flatten transparency onto black
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (0, 0, 0, 255))
        im = Image.alpha_composite(bg, im)
    im = im.convert("L")
    if fit == "cover":
        # Supersample 4x first so the final downscale averages instead of aliasing.
        big = ImageOps.fit(im, (W * 4, H * 4), Image.LANCZOS, centering=(focus[0], focus[1]))
        return big.resize((W, H), Image.LANCZOS)
    if fit == "contain":
        im.thumbnail((W * 4, H * 4), Image.LANCZOS)
        canvas = Image.new("L", (W * 4, H * 4), 0)
        canvas.paste(im, ((W * 4 - im.width) // 2, (H * 4 - im.height) // 2))
        return canvas.resize((W, H), Image.LANCZOS)
    return im.resize((W, H), Image.LANCZOS)               # stretch


def atkinson(im):
    px = [[float(im.getpixel((x, y))) for x in range(W)] for y in range(H)]
    out = [[0] * W for _ in range(H)]
    for y in range(H):
        for x in range(W):
            old = px[y][x]
            new = 255.0 if old >= 128 else 0.0
            out[y][x] = 1 if new else 0
            err = (old - new) / 8.0
            for dx, dy in ((1, 0), (2, 0), (-1, 1), (0, 1), (1, 1), (0, 2)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H:
                    px[ny][nx] += err
    return out


def dither(im, mode):
    if mode == "atkinson":
        return atkinson(im)
    if mode == "fs":
        bw = im.convert("1", dither=Image.FLOYDSTEINBERG)
        return [[1 if bw.getpixel((x, y)) else 0 for x in range(W)] for y in range(H)]
    if mode == "bayer":
        return [[1 if im.getpixel((x, y)) / 255.0 * 16 > BAYER4[y % 4][x % 4] else 0
                 for x in range(W)] for y in range(H)]
    return [[1 if im.getpixel((x, y)) >= 128 else 0 for x in range(W)] for y in range(H)]


def pack(bits):
    data = []
    for y in range(H):
        for bx in range(W // 8):
            v = 0
            for i in range(8):
                if bits[y][bx * 8 + i]:
                    v |= 0x80 >> i                       # MSB = leftmost pixel (u8g2 drawBitmap)
            data.append(v)
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("-o", "--out", default="boot_art.h")
    ap.add_argument("--fit", choices=["cover", "contain", "stretch"], default="cover")
    ap.add_argument("--focus", default="0.5,0.5", help="crop focus x,y in 0..1 (cover mode)")
    ap.add_argument("--dither", choices=["atkinson", "fs", "bayer", "threshold"], default="atkinson")
    ap.add_argument("--gamma", type=float, default=1.0, help="<1 brightens, >1 darkens")
    ap.add_argument("--contrast", type=float, default=1.0)
    ap.add_argument("--local", type=float, default=0.0,
                    help="local-contrast boost (try 2-4): flattens glows/gradients so shapes survive dithering")
    ap.add_argument("--pixel-perfect", action="store_true",
                    help="pixel art mode: undo upscaling and crop a native 128x64 window, no blur/dither")
    ap.add_argument("--crop", default="-1,-1", help="window x,y in native pixels for --pixel-perfect (-1 = centre)")
    ap.add_argument("--invert", action="store_true")
    ap.add_argument("--no-autocontrast", action="store_true")
    ap.add_argument("--preview", help="write an upscaled PNG preview")
    a = ap.parse_args()

    focus = tuple(float(v) for v in a.focus.split(","))
    if a.pixel_perfect:
        im = load_pixel_perfect(a.image, tuple(int(v) for v in a.crop.split(",")))
        a.dither, a.no_autocontrast = "threshold", True
    else:
        im = load(a.image, a.fit, focus)
    if not a.no_autocontrast:
        im = ImageOps.autocontrast(im, cutoff=1)
    if a.contrast != 1.0:
        im = ImageEnhance.Contrast(im).enhance(a.contrast)
    if a.local > 0:
        blur = im.filter(ImageFilter.GaussianBlur(9))
        hp = ImageChops.subtract(ImageChops.add(im, Image.new("L", im.size, 128)), blur)   # im - blur + 128
        hp = hp.point(lambda v: max(0, min(255, int(128 + (v - 128) * a.local))))
        im = ImageChops.blend(hp, im, 0.25)
        im = ImageOps.autocontrast(im, cutoff=1)
    if a.gamma != 1.0:
        im = im.point(lambda v: int(255 * ((v / 255.0) ** a.gamma)))
    if a.invert:
        im = ImageOps.invert(im)

    bits = dither(im, a.dither)
    data = pack(bits)
    with open(a.out, "w") as f:
        f.write("#pragma once\n// Generated by tools/img2boot.py from %s - 128x64, 1 bpp, MSB-first.\n"
                "#include <stdint.h>\n\nstatic const uint8_t BOOT_ART[%d] = {\n" % (a.image.split('/')[-1], len(data)))
        for i in range(0, len(data), 16):
            f.write("    " + ", ".join("0x%02X" % b for b in data[i:i + 16]) + ",\n")
        f.write("};\n")
    if a.preview:
        S = 6
        pv = Image.new("L", (W * S, H * S), 0)
        for y in range(H):
            for x in range(W):
                if bits[y][x]:
                    pv.paste(255, (x * S, y * S, x * S + S - 1, y * S + S - 1))
        pv.save(a.preview)
    print("wrote %s (%d bytes)" % (a.out, len(data)))


if __name__ == "__main__":
    sys.exit(main())
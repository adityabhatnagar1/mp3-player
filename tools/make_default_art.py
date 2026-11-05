#!/usr/bin/env python3
"""Default boot art: original Fuji / rising sun / seigaiha waves / sakura scene (grayscale, 1024x512)."""
import math, random
from PIL import Image, ImageDraw, ImageFilter

W, H = 1024, 512
im = Image.new("L", (W, H), 0)
d = ImageDraw.Draw(im)

# sky gradient: dark at top, bright toward horizon
for y in range(int(H * 0.72)):
    v = int(235 * (y / (H * 0.72)) ** 1.6)
    d.line([(0, y), (W, y)], fill=v)

# stars
random.seed(7)
for _ in range(26):
    x, y = random.randint(10, W - 10), random.randint(6, 150)
    d.line([(x - 5, y), (x + 5, y)], fill=255, width=2)
    d.line([(x, y - 5), (x, y + 5)], fill=255, width=2)

# rising sun
sx, sy, sr = 740, 190, 108
d.ellipse([sx - sr - 16, sy - sr - 16, sx + sr + 16, sy + sr + 16], fill=120)
d.ellipse([sx - sr, sy - sr, sx + sr, sy + sr], fill=255)

# Mt Fuji silhouette with snow cap
fuji = [(-40, 392), (200, 300), (330, 214), (400, 160), (440, 156), (470, 160), (540, 214), (680, 300), (900, 392), (900, 440), (-40, 440)]
d.polygon(fuji, fill=8)
snow = [(400, 160), (440, 156), (470, 160), (500, 188), (478, 180), (458, 204), (440, 184), (420, 206), (402, 182), (380, 190)]
d.polygon(snow, fill=250)

# distant haze band
for y in range(392, 420):
    d.line([(0, y), (W, y)], fill=int(70 * (1 - (y - 392) / 28)))

# seigaiha waves
d.rectangle([0, 400, W, H], fill=0)
R = 64
for row in range(0, 9):
    cy = 400 + row * 28
    off = (R if row % 2 else 0)
    for cx in range(-R * 2 + off, W + R * 2, R * 2):
        d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=0)
        for k, r in enumerate((R, R * 3 // 4, R // 2, R // 4)):
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=235, width=7)

# sakura petals
def petal(cx, cy, s, ang):
    pts = []
    for i in range(10):
        a = ang + i * math.pi / 5
        rr = s if i % 2 == 0 else s * 0.45
        pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
    d.polygon(pts, fill=255)
    d.ellipse([cx - s * .18, cy - s * .18, cx + s * .18, cy + s * .18], fill=60)

random.seed(11)
spots = [(90, 90, 30), (210, 210, 22), (330, 80, 18), (130, 330, 26), (560, 70, 20), (930, 330, 24), (610, 330, 16), (960, 70, 18), (60, 250, 14)]
for cx, cy, s in spots:
    petal(cx, cy, s, random.random() * 6.28)

im = im.filter(ImageFilter.GaussianBlur(0.8))
im.save("boot_art_default.png")
print("ok")
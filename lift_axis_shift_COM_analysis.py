#!/usr/bin/env python3
"""Lift axis X-shift vs robot CoM: parametric study (READ-ONLY calc).

Exact inputs (no rounding): support quad BR,FR,FL,BL; O; COM_robot(B);
lift axis XY. Model: rigid X-shift of whole carriage by DX.
Outputs: PNG plot (mm rel O) + TXT tables. Does NOT touch the .blend.
"""
from PIL import Image, ImageDraw, ImageFont

V = [(3.348778, -2.118084), (3.866977, -2.117639),
     (3.866170, -1.603425), (3.349833, -1.602137)]
O = (3.607794, -1.860476)
CR = (3.677600, -1.867928)
SUP = [((x - O[0]) * 1000.0, (y - O[1]) * 1000.0) for x, y in V]
DXS = [0, -20, -40, -60, -70, -80, -100, -120]
COMS = [((CR[0] + dx / 1000.0 - O[0]) * 1000.0, (CR[1] - O[1]) * 1000.0) for dx in DXS]
OPT = ((CR[0] - 0.069806 - O[0]) * 1000.0, (CR[1] - O[1]) * 1000.0)

W = H = 1400
LM, RM, TM, BM = 130, 50, 90, 170
PW, PH = W - LM - RM, H - TM - BM
LO, HI = -320.0, 320.0
SC = PW / (HI - LO)


def mp(x, y):
    return (LM + (x - LO) * SC, TM + (HI - y) * SC)


F = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
import PIL.ImageFont as IF
f_title, f_leg, f_lab, f_tick = IF.truetype(F, 28), IF.truetype(F, 21), IF.truetype(F, 17), IF.truetype(F, 16)

base = Image.new("RGB", (W, H), "white")
ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
g = 50
x = LO
while x <= HI + 1e-9:
    c = (30, 30, 30, 255) if abs(x) < 1e-9 else ((200, 200, 200, 255) if round(x) % 100 == 0 else (233, 233, 233, 255))
    dr.line([mp(x, LO), mp(x, HI)], fill=c, width=2 if abs(x) < 1e-9 else 1)
    if round(x) % 100 == 0:
        dr.text((mp(x, LO)[0] - 24, mp(x, LO)[1] + 8), str(int(round(x))), font=f_tick, fill=(60, 60, 60, 255))
    x += g
y = LO
while y <= HI + 1e-9:
    c = (30, 30, 30, 255) if abs(y) < 1e-9 else ((200, 200, 200, 255) if round(y) % 100 == 0 else (233, 233, 233, 255))
    dr.line([mp(LO, y), mp(HI, y)], fill=c, width=2 if abs(y) < 1e-9 else 1)
    if round(y) % 100 == 0 and abs(y) > 1e-9:
        dr.text((mp(LO, y)[0] - 64, mp(LO, y)[1] - 12), str(int(round(y))), font=f_tick, fill=(60, 60, 60, 255))
    y += g
dr.text((mp(HI, LO)[0] - 90, mp(HI, LO)[1] + 34), "X, mm", font=f_leg, fill=(0, 0, 0, 255))
dr.text((mp(LO, HI)[0] + 6, mp(LO, HI)[1] - 42), "Y, mm", font=f_leg, fill=(0, 0, 0, 255))

dr.polygon([mp(a, b) for a, b in SUP], fill=(120, 120, 120, 30))
dr.line([mp(a, b) for a, b in SUP] + [mp(*SUP[0])], fill=(110, 110, 110, 255), width=4, joint="curve")
for (a, b) in SUP:
    px, py = mp(a, b)
    dr.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(110, 110, 110, 255))

ox, oy = mp(0, 0)
dr.line([ox - 12, oy, ox + 12, oy], fill=(0, 0, 0, 255), width=3)
dr.line([ox, oy - 12, ox, oy + 12], fill=(0, 0, 0, 255), width=3)
dr.text((ox + 16, oy - 36), "O (support centroid)", font=f_leg, fill=(0, 0, 0, 255))

OFFS = [(14, -30), (14, 8), (-64, 8), (-64, -30), (14, 8), (-70, 8), (-70, -30), (14, -30)]
for k, ((cx, cy), dx) in enumerate(zip(COMS, DXS)):
    px, py = mp(cx, cy)
    dr.ellipse([px - 7, py - 7, px + 7, py + 7], fill=(30, 120, 220, 255))
    oxf, oyf = OFFS[k]
    dr.text((px + oxf, py + oyf), "DX=%d" % dx, font=f_lab, fill=(30, 120, 220, 255))
sx, sy = mp(*OPT)
r = 13
dr.polygon([(sx, sy - r), (sx + r * 0.35, sy - r * 0.35), (sx + r, sy),
            (sx + r * 0.35, sy + r * 0.35), (sx, sy + r),
            (sx - r * 0.35, sy + r * 0.35), (sx - r, sy),
            (sx - r * 0.35, sy - r * 0.35)], fill=(230, 140, 0, 255))
dr.text((sx + 18, sy - 40), "OPT DX=-69.806", font=f_lab, fill=(230, 140, 0, 255))

dr.text((LM + 10, 14), "Lift axis X-shift: robot CoM vs support polygon (mm rel O)", font=f_title, fill=(0, 0, 0, 255))
dr.text((LM + 10, H - 60), "Blue: COM at DX=0..-120. Gold star: optimum (-69.806, -7.452). Gray: support quad.", font=f_lab, fill=(80, 80, 80, 255))
dr.text((LM + 10, H - 32), "Model: rigid carriage shift dX; COM_x(dX)=COM_x(0)+dX. Axis already over O (0.6 mm).", font=f_lab, fill=(80, 80, 80, 255))

img = Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB")
img.save("/home/v/projects/робот/lift_axis_shift_COM_analysis.png")

L = []
L.append("inputs: O=(3.607794,-1.860476) COMrob=(3.6776,-1.867928, B 68.54kg) axis=(3.608398,-1.86476)")
L.append("DX COMx_rel COMy_rel nearest_edge margin_mm inside")
ED = [("FR-FL(+X)", 1), ("BR-FR(-Y)", 0), ("BL-BR(-X)", 3), ("FL-BL(+Y)", 2)]
import math
Q = SUP
for dx in DXS:
    cx = (CR[0] + dx / 1000.0 - O[0]) * 1000.0
    cy = (CR[1] - O[1]) * 1000.0
    ds = []
    for i in range(4):
        ax, ay = Q[i]
        bx, by = Q[(i + 1) % 4]
        ex, ey = bx - ax, by - ay
        Ll = math.hypot(ex, ey)
        ux, uy = ey / Ll, -ex / Ll
        if ux * (0 - ax) + uy * (0 - ay) > 0:
            ux, uy = -ux, -uy
        ds.append((ux * (cx - ax) + uy * (cy - ay), i))
    ds.sort()
    L.append("%d %.2f %.2f edge%d %.1f %s" % (dx, cx, cy, ds[-1][1], -ds[-1][0], all(d < 0 for d, _ in ds)))
L.append("OPT DX=-69.806 COM=(0.000,-7.452)")
L.append("TASK5 payload-on-axis totals rel O and margins:")
CR5 = CR
for dx in DXS:
    cx = (68.54 * (CR5[0] + dx / 1000.0) + 200 * 3.608398) / 268.54
    cy = (68.54 * CR5[1] + 200 * (-1.86476)) / 268.54
    rx, ry = (cx - O[0]) * 1000.0, (cy - O[1]) * 1000.0
    ds = []
    for i in range(4):
        ax, ay = Q[i]
        bx, by = Q[(i + 1) % 4]
        ex, ey = bx - ax, by - ay
        Ll = math.hypot(ex, ey)
        ux, uy = ey / Ll, -ex / Ll
        if ux * (0 - ax) + uy * (0 - ay) > 0:
            ux, uy = -ux, -uy
        ds.append((ux * (rx - ax) + uy * (ry - ay), i))
    ds.sort()
    L.append("%d %.2f %.2f edge%d %.1f %s" % (dx, rx, ry, ds[-1][1], -ds[-1][0], all(d < 0 for d, _ in ds)))
open("/home/v/projects/робот/lift_axis_shift_COM_analysis.txt", "w").write("\n".join(L) + "\n")
print("files written")

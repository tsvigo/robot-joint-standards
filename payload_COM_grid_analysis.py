#!/usr/bin/env python3
"""Payload-grid static stability vs lift shift (READ-ONLY calc).

Exact inputs (no rounding): support quad BR,FR,FL,BL (CCW); O;
robot B: M=68.54, R0=(69.806,-7.452)mm rel O; payload 200 kg;
grid Xpay -400..400 (9), Ypay -300..300 (7); DX range [-200,100] step 1.
Sign: outward normals; inside <=> all signed dists < 0;
margin = -(max signed dist) (positive inside).
Outputs: 3 PNG + TXT. Does NOT touch the .blend.
"""
import math
from PIL import Image, ImageDraw, ImageFont

V = [(3.348778, -2.118084), (3.866977, -2.117639),
     (3.866170, -1.603425), (3.349833, -1.602137)]
O = (3.607794, -1.860476)
MR, MP = 68.54, 200.0
MT = MR + MP
R0 = (69.806, -7.452)
XS = [-400, -300, -200, -100, 0, 100, 200, 300, 400]
YS = [-300, -200, -100, 0, 100, 200, 300]
DXS = [0, -20, -40, -60, -70, -80, -100, -120]
DXS7 = [0, -40, -60, -69.806, -70, -80, -100]

ED = []
for i in range(4):
    ax, ay = V[i]
    bx, by = V[(i + 1) % 4]
    ex, ey = bx - ax, by - ay
    L = math.hypot(ex, ey)
    ux, uy = ey / L, -ex / L
    if ux * (O[0] - ax) + uy * (O[1] - ay) > 0:
        ux, uy = -ux, -uy
    ED.append((ax, ay, ux, uy))


def info(cx, cy):
    ds = []
    for (ax, ay, ux, uy) in ED:
        ds.append(ux * (cx - ax) + uy * (cy - ay))
    inn = all(d < 0 for d in ds)
    m = -max(ds)
    bi = max(range(4), key=lambda i: ds[i])
    return inn, m, bi


def total(px, py, dx):
    rx = (R0[0] + dx) * MR / MT + px * MP / MT
    ry = R0[1] * MR / MT + py * MP / MT
    return rx, ry


def ox(x):
    return (x - O[0]) * 1000.0


def oy(y):
    return (y - O[1]) * 1000.0


Rabs = (O[0] + R0[0] / 1000.0, O[1] + R0[1] / 1000.0)
T1 = {}
for dx in DXS:
    for px in XS:
        for py in YS:
            cx = (MR * (Rabs[0] + dx / 1000.0) + MP * (O[0] + px / 1000.0)) / MT
            cy = (MR * Rabs[1] + MP * (O[1] + py / 1000.0)) / MT
            inn, m, bi = info(cx, cy)
            T1[(dx, px, py)] = ((cx - O[0]) * 1000.0, (cy - O[1]) * 1000.0, inn, m, bi)

T2 = {}
for px in XS:
    for py in YS:
        best = None
        for dx in range(-200, 101):
            cx = (MR * (Rabs[0] + dx / 1000.0) + MP * (O[0] + px / 1000.0)) / MT
            cy = (MR * Rabs[1] + MP * (O[1] + py / 1000.0)) / MT
            inn, m, bi = info(cx, cy)
            if best is None or m > best[0]:
                best = (m, dx, (cx - O[0]) * 1000.0, (cy - O[1]) * 1000.0, bi, inn)
        T2[(px, py)] = best

SHORT = [0, -40, -60, -69.806, -70, -80, -100]
T3 = {}
for px in XS:
    for py in YS:
        T3[(px, py)] = max(SHORT, key=lambda dx: T1[(dx, px, py)][3])

mmA, mmC, mxA, mxC = {}, {}, {}, {}
for dx in range(-200, 101):
    ms, xs = [], []
    for px in XS:
        for py in YS:
            cx = (MR * (Rabs[0] + dx / 1000.0) + MP * (O[0] + px / 1000.0)) / MT
            cy = (MR * Rabs[1] + MP * (O[1] + py / 1000.0)) / MT
            inn, m, bi = info(cx, cy)
            ms.append(m)
            xs.append(math.hypot((cx - O[0]) * 1000.0, (cy - O[1]) * 1000.0))
    mmA[dx], mmC[dx], mxA[dx], mxC[dx] = min(ms), dx, max(xs), dx
bestA = max(mmA, key=lambda d: mmA[d])
bestC = min(mxC, key=lambda d: mxC[d])

k = MT / MP
REG = {}
for dx in DXS7:
    rx = R0[0] + dx
    P = [(rx + k * (ox(vx) - rx), R0[1] + k * (oy(vy) - R0[1])) for vx, vy in V]
    ar = abs(sum(P[i][0] * P[(i + 1) % 4][1] - P[(i + 1) % 4][0] * P[i][1] for i in range(4))) / 2.0 / 1e6
    REG[dx] = P + [ar]

with open("/home/v/projects/робот/payload_COM_analysis.txt", "w") as f:
    f.write("TASK1 dx,px,py totalx totaly inside margin edge\n")
    for dx in DXS:
        for px in XS:
            for py in YS:
                t = T1[(dx, px, py)]
                f.write("%s %d %d %.2f %.2f %s %.1f %d\n" % (dx, px, py, t[0], t[1], t[2], t[3], t[4]))
    f.write("TASK2 px py dxopt totalx totaly margin edge inside\n")
    for px in XS:
        for py in YS:
            m, dx, tx, ty, bi, inn = T2[(px, py)]
            f.write("%d %d %d %.2f %.2f %.1f %d %s\n" % (px, py, dx, tx, ty, m, bi, inn))
    f.write("TASK4A best_dx=%d margin=%.1f\n" % (bestA, mmA[bestA]))
    f.write("TASK4B area invariant (homothety, same k): check per DX7 below\n")
    f.write("TASK4C best_dx=%d maxoff=%.1f\n" % (bestC, mxC[bestC]))
    f.write("TASK5 dx area verts\n")
    for dx in DXS7:
        r = REG[dx]
        f.write("%s %.6f %s\n" % (dx, r[4], [(round(x, 2), round(y, 2)) for x, y in r[:4]]))

F = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
import PIL.ImageFont as IF
fT, fL, fS, fTic = IF.truetype(F, 28), IF.truetype(F, 21), IF.truetype(F, 17), IF.truetype(F, 16)


def frame(W, H, LM, RM, TM, BM, LO, HI):
    base = Image.new("RGB", (W, H), "white")
    ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
    dr = ImageDraw.Draw(ov)
    PW, PH = W - LM - RM, H - TM - BM
    SC = PW / (HI - LO)

    def mp(x, y):
        return (LM + (x - LO) * SC, TM + (HI - y) * SC)
    g = 50 if HI - LO <= 1000 else 100
    v = LO
    while v <= HI + 1e-9:
        c = (30, 30, 30, 255) if abs(v) < 1e-9 else ((200, 200, 200, 255) if round(v) % 100 == 0 else (233, 233, 233, 255))
        dr.line([mp(v, LO), mp(v, HI)], fill=c, width=2 if abs(v) < 1e-9 else 1)
        if round(v) % 100 == 0:
            dr.text((mp(v, LO)[0] - 24, mp(v, LO)[1] + 8), str(int(round(v))), font=fTic, fill=(60, 60, 60, 255))
        v += g
    v = LO
    while v <= HI + 1e-9:
        c = (30, 30, 30, 255) if abs(v) < 1e-9 else ((200, 200, 200, 255) if round(v) % 100 == 0 else (233, 233, 233, 255))
        dr.line([mp(LO, v), mp(HI, v)], fill=c, width=2 if abs(v) < 1e-9 else 1)
        if round(v) % 100 == 0 and abs(v) > 1e-9:
            dr.text((mp(LO, v)[0] - 66, mp(LO, v)[1] - 12), str(int(round(v))), font=fTic, fill=(60, 60, 60, 255))
        v += g
    return base, ov, dr, mp


COLS = [(30, 120, 220, 255), (220, 60, 50, 255), (230, 140, 0, 255), (60, 160, 60, 255),
        (140, 60, 180, 255), (0, 160, 160, 255), (180, 60, 120, 255)]
base, ov, dr, mp = frame(1500, 1500, 130, 50, 100, 190, -480, 480)
SUP = [((x - O[0]) * 1000.0, (y - O[1]) * 1000.0) for x, y in V]
dr.polygon([mp(a, b) for a, b in SUP], fill=(120, 120, 120, 30))
dr.line([mp(a, b) for a, b in SUP] + [mp(*SUP[0])], fill=(110, 110, 110, 255), width=4, joint="curve")
for k, dx in enumerate(DXS7):
    pts = REG[dx][:4]
    dr.polygon([mp(a, b) for a, b in pts], fill=COLS[k] + (26,))
    dr.line([mp(a, b) for a, b in pts] + [mp(*pts[0])], fill=COLS[k], width=3, joint="curve")
    mx = max(p[0] for p in pts)
    dr.text((mp(mx, 0)[0] + 10, mp(mx, 0)[1] - 60 - k * 2), "DX=%s" % dx, font=fS, fill=COLS[k])
ox, oy = mp(0, 0)
dr.line([ox - 12, oy, ox + 12, oy], fill=(0, 0, 0, 255), width=3)
dr.line([ox, oy - 12, ox, oy + 12], fill=(0, 0, 0, 255), width=3)
dr.text((ox + 16, oy - 36), "O", font=fL, fill=(0, 0, 0, 255))
dr.text((130 + 10, 14), "Allowed 200-kg payload CoM per lift DX (mm rel O)", font=fT, fill=(0, 0, 0, 255))
dr.text((130 + 10, 1500 - 62), "Gray: support quad (total-CoM space, ref). Colors: payload regions.", font=fS, fill=(80, 80, 80, 255))
dr.text((130 + 10, 1500 - 34), "Boundary = tipping onset. NOT a safe working area.", font=fS, fill=(80, 80, 80, 255))
dr.text((mp(480, -480)[0] - 90, mp(480, -480)[1] + 30), "X, mm", font=fL, fill=(0, 0, 0, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/payload_COM_regions_vs_lift_shift.png")

W2, H2 = 1250, 1050
LM2, RM2, TM2, BM2 = 130, 260, 90, 120
base = Image.new("RGB", (W2, H2), "white")
ov = Image.new("RGBA", (W2, H2), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
CW, CH = (W2 - LM2 - RM2) / 9.0, (H2 - TM2 - BM2) / 7.0
vmax = max(abs(T1[(-70, px, py)][3]) for px in XS for py in YS)
for ix, px in enumerate(XS):
    for iy, py in enumerate(YS):
        t = T1[(-70, px, py)]
        m = t[3]
        x0 = LM2 + ix * CW
        y0 = TM2 + (6 - iy) * CH
        if m >= 0:
            gg = min(1.0, m / vmax)
            col = (int(200 - 160 * gg), int(235 - 35 * gg), int(200 - 160 * gg), 255)
        else:
            rr = min(1.0, -m / vmax)
            col = (255, int(235 - 160 * rr), int(235 - 160 * rr), 255)
        dr.rectangle([x0, y0, x0 + CW, y0 + CH], fill=col, outline=(90, 90, 90, 255))
        dr.text((x0 + CW / 2 - 28, y0 + CH / 2 - 12), "%d" % round(m), font=fS, fill=(0, 0, 0, 255))
    dr.text((LM2 + ix * CW + CW / 2 - 24, TM2 + 7 * CH + 8), "%d" % px, font=fTic, fill=(0, 0, 0, 255))
for iy, py in enumerate(YS):
    dr.text((LM2 - 64, TM2 + (6 - iy) * CH + CH / 2 - 12), "%d" % py, font=fTic, fill=(0, 0, 0, 255))
dr.text((LM2, TM2 + 7 * CH + 40), "X_payload, mm", font=fL, fill=(0, 0, 0, 255))
dr.text((8, TM2 + 200), "Y_payload, mm", font=fL, fill=(0, 0, 0, 255))
dr.text((LM2, 12), "Margin map at DX=-70 mm (min edge distance; green>=0 inside, red<0 outside)", font=fT, fill=(0, 0, 0, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/payload_COM_margin_map.png")

base = Image.new("RGB", (1250, 800), "white")
ov = Image.new("RGBA", (1250, 800), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
LX0, LX1, LY0, LY1 = -420, 420, -210, 110
LM3, RM3, TM3, BM3 = 110, 300, 80, 110
PW3, PH3 = 1250 - LM3 - RM3, 800 - TM3 - BM3


def mp3(x, y):
    return (LM3 + (x - LX0) / (LX1 - LX0) * PW3, TM3 + (LY1 - y) / (LY1 - LY0) * PH3)


for gx in range(-400, 401, 100):
    dr.line([mp3(gx, LY0), mp3(gx, LY1)], fill=(225, 225, 225, 255))
    dr.text((mp3(gx, LY0)[0] - 18, mp3(gx, LY0)[1] + 6), str(gx), font=fTic, fill=(60, 60, 60, 255))
for gy in range(-200, 101, 50):
    dr.line([mp3(LX0, gy), mp3(LX1, gy)], fill=(225, 225, 225, 255))
    dr.text((mp3(LX0, gy)[0] - 46, mp3(LX0, gy)[1] - 12), str(gy), font=fTic, fill=(60, 60, 60, 255))
dr.line([mp3(LX0, 0), mp3(LX1, 0)], fill=(150, 150, 150, 255), width=2)
LC = [(40, 140, 200, 255), (200, 60, 60, 255), (230, 140, 0, 255), (60, 150, 60, 255),
      (140, 60, 180, 255), (0, 150, 150, 255), (180, 60, 120, 255)]
for iy, py in enumerate(YS):
    pts = []
    for px in XS:
        pts.append((px, T2[(px, py)][1]))
    dr.line([mp3(a, b) for a, b in pts], fill=LC[iy], width=3, joint="curve")
    for a, b in pts:
        qx, qy = mp3(a, b)
        dr.ellipse([qx - 4, qy - 4, qx + 4, qy + 4], fill=LC[iy])
    ex, ey = mp3(XS[-1], T2[(XS[-1], py)][1])
    dr.text((ex + 10, ey - 12), "Y=%d" % py, font=fS, fill=LC[iy])
dr.text((LM3, 10), "Optimal DX vs payload X (one line per payload Y)", font=fT, fill=(0, 0, 0, 255))
dr.text((mp3(LX1, LY0)[0] - 130, mp3(LX1, LY0)[1] + 34), "X_payload, mm", font=fL, fill=(0, 0, 0, 255))
dr.text((14, mp3(LX0, 60)[1] - 12), "DX_opt, mm", font=fL, fill=(0, 0, 0, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/lift_shift_optimum.png")
print("done areas:", [round(REG[dx][4], 6) for dx in DXS7])

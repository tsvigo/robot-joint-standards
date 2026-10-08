#!/usr/bin/env python3
"""Static stability: allowed 200-kg payload CoM regions (READ-ONLY calc).

Inputs (exact values from Blender audits, NO rounding):
  support polygon vertices (wheel contact XY, meters, CCW BR,FR,FL,BL)
  O = support polygon area centroid (plot origin)
  robot scenarios A/B/C: (mass, COM_robot XY)
  payload mass 200 kg
Transform (exact homothety): P = R + k*(V - R), k = (M + 200)/200.
Output: PNG plot (mm rel O) + TXT numbers. Does NOT touch the .blend.
"""
import math
from PIL import Image, ImageDraw, ImageFont

V_SUP = [(3.348778, -2.118084), (3.866977, -2.117639),
         (3.866170, -1.603425), (3.349833, -1.602137)]  # BR,FR,FL,BL
O = (3.607794, -1.860476)
SCEN = {"A": (30.82, (3.659789, -1.865590)),
        "B": (68.54, (3.677600, -1.867928)),
        "C": (104.76, (3.690478, -1.870974))}
M_PAY = 200.0
A_SUP = 0.266435

REG = {}
for s, (m, r) in SCEN.items():
    k = (m + M_PAY) / M_PAY
    P = [(r[0] + k * (x - r[0]), r[1] + k * (y - r[1])) for x, y in V_SUP]
    Pr = [((x - O[0]) * 1000.0, (y - O[1]) * 1000.0) for x, y in P]
    ar = abs(sum(Pr[i][0] * Pr[(i + 1) % 4][1] - Pr[(i + 1) % 4][0] * Pr[i][1]
                 for i in range(4))) / 2.0 / 1e6
    res = max(abs((P[i][0] - r[0]) - k * (V_SUP[i][0] - r[0])) +
                  abs((P[i][1] - r[1]) - k * (V_SUP[i][1] - r[1])) for i in range(4))
    REG[s] = {"k": k, "P": Pr, "area": ar, "resid": res,
              "Rmm": ((r[0] - O[0]) * 1000.0, (r[1] - O[1]) * 1000.0)}

SUP = [((x - O[0]) * 1000.0, (y - O[1]) * 1000.0) for x, y in V_SUP]

# ---- plot ----
W = H = 1700
LM, RM, TM, BM = 150, 60, 110, 150
PW, PH = W - LM - RM, H - TM - BM
LO, HI = -480.0, 480.0
SC = PW / (HI - LO)


def mp(x, y):
    return (LM + (x - LO) * SC, TM + (HI - y) * SC)


F_TITLE = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
FM = lambda sz: __import__("PIL.ImageFont", fromlist=["truetype"]).truetype(F_TITLE, sz)
f_title, f_leg, f_lab, f_tick = FM(30), FM(22), FM(17), FM(17)

base = Image.new("RGB", (W, H), "white")
ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
g = 50
x = LO
while x <= HI + 1e-9:
    w = 2 if abs(x) < 1e-9 else 1
    col = (30, 30, 30, 255) if abs(x) < 1e-9 else ((200, 200, 200, 255) if round(x) % 100 == 0 else (232, 232, 232, 255))
    dr.line([mp(x, LO), mp(x, HI)], fill=col, width=w)
    if round(x) % 100 == 0:
        dr.text((mp(x, LO)[0] - 22, mp(x, LO)[1] + 8), str(int(round(x))), font=f_tick, fill=(60, 60, 60, 255))
    x += g
y = LO
while y <= HI + 1e-9:
    w = 2 if abs(y) < 1e-9 else 1
    col = (30, 30, 30, 255) if abs(y) < 1e-9 else ((200, 200, 200, 255) if round(y) % 100 == 0 else (232, 232, 232, 255))
    dr.line([mp(LO, y), mp(HI, y)], fill=col, width=w)
    if round(y) % 100 == 0 and abs(y) > 1e-9:
        dr.text((mp(LO, y)[0] - 62, mp(LO, y)[1] - 12), str(int(round(y))), font=f_tick, fill=(60, 60, 60, 255))
    y += g
dr.text((mp(HI, LO)[0] - 90, mp(HI, LO)[1] + 34), "X, mm", font=f_leg, fill=(0, 0, 0, 255))
dr.text((mp(LO, HI)[0] + 6, mp(LO, HI)[1] - 44), "Y, mm", font=f_leg, fill=(0, 0, 0, 255))

COLORS = {"SUP": (110, 110, 110, 255), "SUPF": (110, 110, 110, 28),
          "A": (30, 120, 220, 255), "AF": (30, 120, 220, 30),
          "B": (220, 60, 50, 255), "BF": (220, 60, 50, 30),
          "C": (230, 140, 0, 255), "CF": (230, 140, 0, 30)}


def poly(pts, outline, fill, width=3):
    dr.polygon([mp(x, y) for x, y in pts], fill=fill)
    dr.line([mp(x, y) for x, y in pts] + [mp(*pts[0])], fill=outline, width=width, joint="curve")


for s in "ABC":
    poly(REG[s]["P"], COLORS[s], COLORS[s + "F"])
poly(SUP, COLORS["SUP"], COLORS["SUPF"], 5)

for s in "ABC":
    pts = REG[s]["P"]
    ccx = sum(p[0] for p in pts) / 4.0
    ccy = sum(p[1] for p in pts) / 4.0
    for (x, y) in pts:
        px, py = mp(x, y)
        dr.ellipse([px - 5, py - 5, px + 5, py + 5], fill=COLORS[s])
        dx, dy = x - ccx, y - ccy
        L = math.hypot(dx, dy) or 1.0
        lx, ly = px + dx / L * 26 - 52, py + dy / L * 26 - 8
        dr.text((lx, ly), "(%.1f,%.1f)" % (x, y), font=f_lab, fill=COLORS[s])
ox, oy = mp(0, 0)
dr.line([ox - 12, oy, ox + 12, oy], fill=(0, 0, 0, 255), width=3)
dr.line([ox, oy - 12, ox, oy + 12], fill=(0, 0, 0, 255), width=3)
dr.text((ox + 16, oy - 34), "O (0,0)", font=f_leg, fill=(0, 0, 0, 255))
OFF = {"A": (12, -12), "B": (12, 16), "C": (12, 44)}
for s in "ABC":
    rx, ry = REG[s]["Rmm"]
    px, py = mp(rx, ry)
    dr.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(0, 0, 0, 255))
    dr.text((px + OFF[s][0], py + OFF[s][1]), "COM %s (%.1f,%.1f)" % (s, rx, ry), font=f_lab, fill=(0, 0, 0, 255))

dr.text((LM + 10, 18), "Static stability: allowed 200-kg payload CoM regions (mm rel O)", font=f_title, fill=(0, 0, 0, 255))
leg = [("support polygon (C_total space, ref)", COLORS["SUP"]),
       ("A: 30.82 kg robot", COLORS["A"]),
       ("B: 68.54 kg robot", COLORS["B"]),
       ("C: 104.76 kg robot", COLORS["C"])]
ly = H - BM + 44
for txt, col in leg:
    lx = LM + 10 + (leg.index((txt, col)) % 2) * 700
    lyy = ly + (leg.index((txt, col)) // 2) * 34
    dr.rectangle([lx, lyy, lx + 44, lyy + 22], fill=col)
    dr.text((lx + 54, lyy - 4), txt, font=f_leg, fill=(0, 0, 0, 255))
note = ("Regions: payload CoM keeping total CoM inside support polygon. "
        "Boundary = tipping onset. NOT a safe working area. "
        "Heavier robot -> larger region (homothety k=(M+200)/200).")
dr.text((LM + 10, H - 46), note[:118], font=f_lab, fill=(80, 80, 80, 255))
dr.text((LM + 10, H - 20), "NOT a safe working area: dynamics, CoM truth, stiffness, margin still required.", font=f_lab, fill=(80, 80, 80, 255))

img = Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB")
img.save("/home/v/projects/робот/static_stability_payload_COM.png")

with open("/home/v/projects/робот/static_stability_payload_COM.txt", "w") as f:
    f.write("support area=0.266435 m2  O=(3.607794,-1.860476)\n")
    for s in "ABC":
        m = SCEN[s][0]
        f.write("%s M=%.2f k=%.4f area=%.6f verts(mm rel O)=%s COMrobot=%s\n" %
                (s, m, REG[s]["k"], REG[s]["area"],
                 [(round(x, 2), round(y, 2)) for x, y in REG[s]["P"]],
                 tuple(round(v, 2) for v in REG[s]["Rmm"])))
    f.write("homothety_max_resid=%.2e\n" % max(REG[s]["resid"] for s in "ABC"))
    f.write("B max+X check: %.2f mm (est ~323.5: diff = rounding k to 1.34)\n" %
            max(p[0] for p in REG["B"]["P"]))
print("files written; areas:",
      [(s, round(REG[s]["area"], 6), REG[s]["k"]) for s in "ABC"])

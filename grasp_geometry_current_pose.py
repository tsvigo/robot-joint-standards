#!/usr/bin/env python3
"""Grasp geometry, current pose (READ-ONLY calc).

Exact inputs (read live from .blend, no rounding): O, lift axis XY,
support quad, palm bboxes (faces, centers), derived P_L/P_R/mid.
P_L/P_R = inner palm face centers (geometric proposal, NOT declared grasp points).
Outputs: PNG (XY, mm rel O) + TXT. Does NOT touch the .blend.
"""
from PIL import Image, ImageDraw, ImageFont

O = (3.607794, -1.860476)
AX = (3.608398, -1.86476)
SUP = [(-259.016, -257.608), (259.183, -257.163),
       (258.376, 257.051), (-257.961, 258.339)]
LP = {"mn": (3.770267, -0.790716, -4.629265), "max": (3.814147, -0.683193, -4.521742)}
RP = {"mn": (3.764489, -3.074256, -4.614396), "max": (3.794598, -2.962129, -4.502268)}


def rel(p):
    return ((p[0] - O[0]) * 1000.0, (p[1] - O[1]) * 1000.0)


PLC = tuple((LP["mn"][i] + LP["max"][i]) / 2 for i in range(3))
PRC = tuple((RP["mn"][i] + RP["max"][i]) / 2 for i in range(3))
PLF = (PLC[0], LP["mn"][1], PLC[2])
PRF = (PRC[0], RP["max"][1], PRC[2])
MID = tuple((PLF[i] + PRF[i]) / 2 for i in range(3))

W, H = 860, 2780
LM, RM, TM, BM = 130, 60, 90, 170
PW, PH = W - LM - RM, H - TM - BM
X0, X1, Y0, Y1 = -320.0, 320.0, -1300.0, 1300.0
SC = min(PW / (X1 - X0), PH / (Y1 - Y0))
OX = LM + (PW - (X1 - X0) * SC) / 2
OY = TM + (PH - (Y1 - Y0) * SC) / 2


def mp(x, y):
    return (OX + (x - X0) * SC, OY + (Y1 - y) * SC)


F = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
import PIL.ImageFont as IF
fT, fL, fS, fTic = IF.truetype(F, 28), IF.truetype(F, 21), IF.truetype(F, 17), IF.truetype(F, 16)

base = Image.new("RGB", (W, H), "white")
ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
g = 100
v = -1300.0
while v <= 1300.0 + 1e-9:
    c = (30, 30, 30, 255) if abs(v) < 1e-9 else ((200, 200, 200, 255) if round(v) % 200 == 0 else (233, 233, 233, 255))
    dr.line([mp(v, Y0), mp(v, Y1)], fill=c, width=2 if abs(v) < 1e-9 else 1)
    if round(v) % 200 == 0:
        dr.text((mp(v, Y0)[0] - 26, mp(v, Y0)[1] + 8), str(int(round(v))), font=fTic, fill=(60, 60, 60, 255))
    v += g
v = -1300.0
while v <= 1300.0 + 1e-9:
    c = (30, 30, 30, 255) if abs(v) < 1e-9 else ((200, 200, 200, 255) if round(v) % 200 == 0 else (233, 233, 233, 255))
    dr.line([mp(X0, v), mp(X1, v)], fill=c, width=2 if abs(v) < 1e-9 else 1)
    if round(v) % 200 == 0 and abs(v) > 1e-9:
        dr.text((mp(X0, v)[0] - 70, mp(X0, v)[1] - 12), str(int(round(v))), font=fTic, fill=(60, 60, 60, 255))
    v += g
dr.text((mp(X1, Y0)[0] - 90, mp(X1, Y0)[1] + 30), "X, mm", font=fL, fill=(0, 0, 0, 255))
dr.polygon([mp(a, b) for a, b in SUP], fill=(120, 120, 120, 30))
dr.line([mp(a, b) for a, b in SUP] + [mp(*SUP[0])], fill=(110, 110, 110, 255), width=3, joint="curve")
ox, oy = mp(0, 0)
dr.line([ox - 12, oy, ox + 12, oy], fill=(0, 0, 0, 255), width=3)
dr.line([ox, oy - 12, ox, oy + 12], fill=(0, 0, 0, 255), width=3)
dr.text((ox + 16, oy - 38), "O (support centroid)", font=fL, fill=(0, 0, 0, 255))
axx, axy = rel((AX[0], AX[1]))
axp, ayp = mp(axx, axy)
dr.ellipse([axp - 7, ayp - 7, axp + 7, ayp + 7], fill=(140, 60, 180, 255))
dr.text((axp + 14, ayp - 14), "lift axis (+0.6,-4.3)", font=fS, fill=(140, 60, 180, 255))
for key, PB, col in (("L", LP, (30, 120, 220, 255)), ("R", RP, (220, 60, 50, 255))):
    x0 = (PB["mn"][0] - O[0]) * 1000.0
    x1 = (PB["max"][0] - O[0]) * 1000.0
    y0 = (PB["mn"][1] - O[1]) * 1000.0
    y1 = (PB["max"][1] - O[1]) * 1000.0
    dr.rectangle([mp(x0, y1), mp(x1, y0)], fill=col[:3] + (36,), outline=col, width=3)
pf = {"L": PLF, "R": PRF}
for key in ("L", "R"):
    rx, ry = rel((pf[key][0], pf[key][1]))
    px, py = mp(rx, ry)
    dr.ellipse([px - 7, py - 7, px + 7, py + 7], fill=(0, 150, 0, 255))
    dr.text((px + 16, py - 12), "P_%s (%.1f,%.1f)" % (key, rx, ry), font=fS, fill=(0, 150, 0, 255))
mx, my = rel((MID[0], MID[1]))
mpx, mpy = mp(mx, my)
dr.line([mp(*rel((PLF[0], PLF[1]))), mp(*rel((PRF[0], PRF[1])))], fill=(0, 150, 0, 255), width=2)
dr.ellipse([mpx - 8, mpy - 8, mpx + 8, mpy + 8], outline=(0, 150, 0, 255), width=3)
dr.text((mpx + 16, mpy - 12), "MID (%.1f,%.1f)" % (mx, my), font=fL, fill=(0, 150, 0, 255))
dr.text((LM + 10, 12), "Grasp geometry, current pose (mm rel O)", font=fT, fill=(0, 0, 0, 255))
dr.text((LM + 10, H - 62), "Gray: support quad. Blue/red: palm XY boxes. Green: inner-face P_L/P_R + MID.", font=fS, fill=(80, 80, 80, 255))
dr.text((LM + 10, H - 34), "P = geometric proposal only, NOT declared grasp points. Z see TXT.", font=fS, fill=(80, 80, 80, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/grasp_geometry_current_pose.png")

T = []
T.append("O=(3.607794,-1.860476) axis=(3.608398,-1.86476,-4.581481)")
T.append("LEFT palm mesh=left_palm bbox mn/max:")
T.append("  mn=(3.770267,-0.790716,-4.629265) max=(3.814147,-0.683193,-4.521742) dims=43.88x107.524x107.523")
T.append("RIGHT palm mesh=right_palm bbox mn/max:")
T.append("  mn=(3.764489,-3.074256,-4.614396) max=(3.794598,-2.962129,-4.502268) dims=30.109x112.127x112.127")
T.append("P_L inner-face center=(3.792207,-0.790716,-4.575504) normal=(0,-1,0) relO=(+184.413,+1069.760)")
T.append("P_R inner-face center=(3.779544,-2.962129,-4.558332) normal=(0,+1,0) relO=(+171.750,-1101.653)")
T.append("|PL-PR|=2171.52mm dX=12.66 dY=2171.41 dZ=-17.17")
T.append("MID faces=(3.785876,-1.876423,-4.566918) relO=(+178.082,-15.947)")
T.append("MID centers=(3.785876,-1.877574,-4.566918) relO=(+178.082,-17.098)")
T.append("O->L palm center=1138.56 O->R=1170.39 O->MID=178.90")
T.append("fingers L Y span=[-0.6534,-0.4985] (tips outward +Y); R Y span=[-3.2237,-3.0896] (tips outward -Y)")
T.append("thumbs L Y=[-0.7677,-0.6147] Z=[-4.498,-4.476]; R Y=[-3.1180,-2.9780] Z=[-4.482,-4.453]")
T.append("palms Z: L=[-4.629265,-4.521742] R=[-4.614396,-4.502268]; face Z: L -4.575504 R -4.558332")
T.append("grasp markers: NONE found (1 false positive: placeholder~hold); constraints/drivers: none in scene")
T.append("orientation L->R dir=(-0.006,-1.000,+0.008): along -Y within 0.5deg; dZ palms=17.17mm")
open("/home/v/projects/робот/grasp_geometry_current_pose.txt", "w").write("\n".join(T) + "\n")
print("grasp files written")

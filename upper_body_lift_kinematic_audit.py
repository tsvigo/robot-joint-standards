#!/usr/bin/env python3
"""Upper-body + lift kinematic audit: tables + tree diagram (READ-ONLY calc).

Data embedded from live .blend probes (2026-10-08) + robot_joints.md.
Never touches the .blend. Outputs TXT + PNG into the project.
"""
from PIL import Image, ImageDraw, ImageFont

MD = {
    "shoulder_joint_empty_3": ("Y", -180, 60), "shoulder_joint_empty_2": ("Z", -90, 90),
    "shoulder_joint_empty_1": ("X", -90, 130), "elbow_joint_empty": ("Z", -10, 150),
    "proximal_radioulnar_joint_empty": ("Y", -90, 90),
    "wrist_joint_1_empty": ("Z", -90, 90), "wrist_joint_2_empty": ("X", -25, 45),
    "atlanto_occipital_joint_empty": ("Y", -50, 85), "atlanto_axial_joint_empty": ("Z", -90, 90),
    "intervertebral_joints_C2_C7_empty": ("X", -45, 45),
}
WPOS = {
    "vertical_lift_joint": (3.608398, -1.86476, -4.581481),
    "torso_link": (3.608398, -1.86476, -4.561481),
    "atlanto_occipital_joint_empty": (3.708353, -1.86476, -4.510311),
    "atlanto_axial_joint_empty": (3.745458, -1.863258, -4.529799),
    "intervertebral_joints_C2_C7_empty": (3.745458, -1.86367, -4.550722),
    "head_link": (3.708353, -1.86476, -4.410679),
    "left_shoulder_joint_empty_3": (3.772351, -1.710971, -4.561809),
    "left_shoulder_joint_empty_2": (3.772351, -1.662502, -4.561297),
    "left_shoulder_joint_empty_1": (3.772351, -1.610971, -4.560788),
    "left_elbow_joint_empty": (3.792201, -1.209357, -4.559339),
    "left_proximal_radioulnar_joint_empty": (3.791164, -0.957482, -4.560872),
    "left_wrist_joint_1_empty": (3.792202, -0.84757, -4.559119),
    "left_wrist_joint_2_empty": (3.792207, -0.811089, -4.568344),
    "left_palm_link": (3.792248, -0.736917, -4.575455),
    "right_shoulder_joint_empty_3": (3.779664, -2.014746, -4.558747),
    "right_shoulder_joint_empty_2": (3.779665, -2.063215, -4.558236),
    "right_shoulder_joint_empty_1": (3.779664, -2.114746, -4.557726),
    "right_elbow_joint_empty": (3.779537, -2.544408, -4.557245),
    "right_proximal_radioulnar_joint_empty": (3.778485, -2.820695, -4.552513),
    "right_wrist_joint_1_empty": (3.779534, -2.913791, -4.549999),
    "right_wrist_joint_2_empty": (3.779539, -2.946454, -4.550828),
    "right_palm_link": (3.779544, -3.018193, -4.558332),
}

L = []
L.append("=== SEC3 parent>joint>child (from .blend; eulers ~0, constraints 0) ===")
L.append("base_link > vertical_lift_joint[prismatic +Z EXPLICIT] > torso_link")
L.append("torso_link > atlanto_occipital > atlanto_axial > intervertebral_C2_C7 > head_link")
for s in ("left", "right"):
    L.append("%s: torso > j3 > link3 > j2 > link2 > j1 > shoulder_link > elbow > forearm > radioulnar > rlink > wrist1 > pitch > wrist2 > palm" % s)
L.append("=== SEC4 axes: ONLY vertical_lift_joint has explicit axis [0,0,1]; all revolute AXIS_NOT_EXPLICITLY_ENCODED ===")
L.append("=== SEC5 current: all eulers ~0 (identity); MD zero = straight limb; scene pose matches MD zero geometrically ===")
L.append("=== SEC6 match table: joint|parent|child|type|axis|current|MDmin|MDmax|zero|status ===")
L.append("vertical_lift_joint|base_link|torso_link|prismatic|[0,0,1] EXPLICIT|static frame|-|-|ZERO_UNKNOWN(travel datum)|NO_MD_ENTRY")
for s in ("left", "right"):
    for j, rng in (("3", (-180, 60, "Y")), ("2", (-90, 90, "Z")), ("1", (-90, 130, "X"))):
        L.append("%s_shoulder_joint_empty_%s|chain-verified|chain-verified|revolute|%s MD-only|0|%s|%s|MD straight 0|MATCH" % (s, j, rng[2], rng[0], rng[1]))
    for jn, rng in (("elbow_joint_empty", (-10, 150, "Z")), ("proximal_radioulnar_joint_empty", (-90, 90, "Y")),
                     ("wrist_joint_1_empty", (-90, 90, "Z")), ("wrist_joint_2_empty", (-25, 45, "X"))):
        L.append("%s_%s|chain-verified|chain-verified|revolute|%s MD-only|0|%s|%s|MD straight 0|MATCH" % (s, jn, rng[2], rng[0], rng[1]))
for jn, rng in (("atlanto_occipital_joint_empty", (-50, 85, "Y")), ("atlanto_axial_joint_empty", (-90, 90, "Z")),
                ("intervertebral_joints_C2_C7_empty", (-45, 45, "X"))):
    L.append("%s|chain-verified|chain-verified|revolute|%s MD-only (no yaml)|0|%s|%s|MD straight 0|MD_RANGE_ONLY" % (jn, rng[2], rng[0], rng[1]))
L.append("=== SEC7 shoulder order from torso: joint_3(Y) > joint_2(Z) > joint_1(X) > arm; numbers do NOT encode order ===")
L.append("=== SEC8 elbow: parent shoulder_link child forearm_link; MD Z -10/+150 applies unambiguously ===")
L.append("=== SEC9 radioulnar: parent forearm_link child radioulnar_link; MD Y +-90 applies unambiguously ===")
L.append("=== SEC10 wrist: forearm > [radioulnar stage] > wrist_1(Z +-90) > pitch > wrist_2(X -25/+45) > palm ===")
L.append("=== SEC11 neck: torso > occipital(C0-C1 Y -50/+85) > axial(C1-C2 Z +-90) > intervertebral(C2-C7 X +-45) > head ===")
L.append("=== SEC12 LIFT A:geometric lower=torso-on-base(-774.567 rel current) upper=head-to-stop_top(+427.315) PRESENT ===")
L.append("=== SEC12 B:documented: ABSENT (no MD entry; full MD read verified) ===")
L.append("=== SEC12 C:software: ABSENT (66/66 nc=0, no limit/constraint/driver props) ===")
L.append("=== SEC12 D:mechanical: upper stop_top PRESENT; lower stop REMOVED; catcher PRESENT(-48); base PRESENT ===")
L.append("=== SEC12 E:catcher: PRESENT placeholder plate (emergency) ===")
L.append("=== SEC14 RELEVANCE: shoulders/elbows/wrists move palms/load XY (tipping); head/neck small levers; lift Z sets height; torso +80mm X offset loads screws asymmetrically ===")
open("/home/v/projects/робот/upper_body_lift_kinematic_audit.txt", "w").write("\n".join(L) + "\n")

F = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
import PIL.ImageFont as IF
fT, fN, fS = IF.truetype(F, 26), IF.truetype(F, 17), IF.truetype(F, 15)
W, H = 1700, 2920
base = Image.new("RGB", (W, H), "white")
ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
COL = {"j": (180, 215, 245, 255), "jt": (0, 40, 120, 255), "l": (225, 225, 225, 255),
       "lt": (60, 60, 60, 255), "lift": (255, 200, 120, 255), "root": (190, 190, 190, 255)}
XS = {"c": 850, "L": 300, "R": 1400}
ys = {}


def node(x, label, sub, kind, row):
    y = 130 + row * 122
    ys[label] = y
    w = 400 if kind != "l" else 330
    dr.rectangle([x - w / 2, y - 34, x + w / 2, y + 34], fill=COL[kind], outline=(40, 40, 40, 255), width=2)
    dr.text((x - w / 2 + 12, y - 24), label[:30], font=fN, fill=COL[kind + "t"] if kind + "t" in COL else (0, 0, 0, 255))
    dr.text((x - w / 2 + 12, y + 2), sub[:34], font=fS, fill=(40, 40, 40, 255))
    return y


def arrow(x1, label1, x2, label2):
    dr.line([(x1, ys[label1] + 34), (x2, ys[label2] - 34)], fill=(40, 40, 40, 255), width=2)


r = 0
node(XS["c"], "base_link", "static root", "root", r)
r += 1
node(XS["c"], "vertical_lift_joint", "prismatic +Z EXPLICIT, NO_MD", "lift", r)
r += 1
node(XS["c"], "torso_link", "carriage beam", "j", r)
r += 1
node(XS["c"], "atlanto_occipital", "C0-C1 Y -50/+85", "j", r)
r += 1
node(XS["c"], "atlanto_axial", "C1-C2 Z +-90", "j", r)
r += 1
node(XS["c"], "intervertebral_C2_C7", "C2-C7 X +-45", "j", r)
r += 1
node(XS["c"], "head_link", "end", "j", r)
for k, s in enumerate(("L", "R")):
    side = "left" if s == "L" else "right"
    chain = [("shoulder_j3", "Y -180/+60"), ("link_3", "dummy"), ("shoulder_j2", "Z +-90"),
             ("link_2", "dummy"), ("shoulder_j1", "X -90/+130"), ("shoulder_link", "mesh"),
             ("elbow", "Z -10/+150"), ("forearm", "mesh"), ("radioulnar", "Y +-90"),
             ("rlink", "dummy"), ("wrist_1", "Z +-90"), ("pitch", "dummy"),
             ("wrist_2", "X -25/+45"), ("palm", "mesh")]
    for i, (nm, sub) in enumerate(chain):
        node(XS[s], "%s_%s" % (side, nm) if "link" not in nm and nm not in ("palm",) else nm, sub,
             "l" if "link" in nm or nm in ("palm", "shoulder_link", "forearm") else "j", 8 + i)
side_rows = {}
for a, b in [("base_link", "vertical_lift_joint"), ("vertical_lift_joint", "torso_link"),
             ("torso_link", "atlanto_occipital"), ("atlanto_occipital", "atlanto_axial"),
             ("atlanto_axial", "intervertebral_C2_C7"), ("intervertebral_C2_C7", "head_link")]:
    arrow(XS["c"], a, XS["c"], b)
for _sx in (XS["L"], XS["R"]):
    for _r in range(8, 21):
        dr.line([(_sx, 130 + _r * 122 + 34), (_sx, 130 + (_r + 1) * 122 - 34)], fill=(40, 40, 40, 255), width=2)
dr.text((XS["c"] - 330, 16), "Upper-body kinematics (parent>child verified)", font=fT, fill=(0, 0, 0, 255))
dr.text((XS["c"] - 330, 52), "Blue=joints(MD ranges) gray=links orange=lift. Fingers excluded.", font=fS, fill=(80, 80, 80, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/upper_body_lift_kinematic_audit.png")
print("kinematic audit files written")

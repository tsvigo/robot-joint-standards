#!/usr/bin/env python3
"""Joint limits audit: robot_joints.md vs .blend scene (READ-ONLY analysis).

All data below was read from robot_joints.md (1957 lines) and from the live
.blend session (66 joint objects). This script only compares literals and draws
a range plot. It NEVER touches the .blend file.
"""
from PIL import Image, ImageDraw, ImageFont

MD = {}
for n, t, ax, lo, hi, zero in [
    ("ankle_joint_empty", "revolute", "Y", -55, 30, "straight limb 0 (prose, no yaml)"),
    ("knee_joint_empty", "revolute", "Y", -150, 10, "straight limb 0 (prose, no yaml)"),
    ("hip_joint_empty", "revolute", "Y", -130, 20, "straight limb 0 (prose, no yaml)"),
    ("atlanto_occipital_joint_empty", "revolute", "Y", -50, 85, "straight head 0 (prose, no yaml)"),
    ("atlanto_axial_joint_empty", "revolute", "Z", -90, 90, "straight head 0 (prose, no yaml)"),
    ("intervertebral_joints_C2_C7_empty", "revolute", "X", -45, 45, "straight head 0 (prose, no yaml)"),
]:
    MD[n] = {"type": t, "axis": ax, "min": lo, "max": hi, "zero": zero,
             "conf": "medium", "link": None}

SIDES = ("right", "left")
ARM = [
    ("shoulder_joint_empty_1", "revolute", "X", -90, 130, "shoulder_2_link", "shoulder_link"),
    ("shoulder_joint_empty_2", "revolute", "Z", -90, 90, "shoulder_3_link", "shoulder_2_link"),
    ("shoulder_joint_empty_3", "revolute", "Y", -180, 60, "torso_link", "shoulder_3_link"),
    ("elbow_joint_empty", "revolute", "Z", -10, 150, "shoulder_link", "forearm_link"),
    ("proximal_radioulnar_joint_empty", "revolute", "Y", -90, 90, "forearm_link", "radioulnar_link"),
    ("wrist_joint_1_empty", "revolute", "Z", -90, 90, "radioulnar_link", "wrist_pitch_link"),
    ("wrist_joint_2_empty", "revolute", "X", -25, 45, "wrist_pitch_link", "palm_link"),
    ("thumb_cmc_joint_empty_1", "revolute", "Y", -25, 25, "palm_link", "third_phalanx"),
    ("thumb_cmc_joint_empty_2", "revolute", "Z", -40, 40, "palm_link", "third_phalanx"),
    ("thumb_cmc_joint_empty_3", "revolute", "X", -60, 60, "palm_link", "third_phalanx"),
    ("thumb_mcp_joint_empty_1", "revolute", "Z", -70, 70, "third_phalanx", "second_phalanx"),
    ("thumb_mcp_joint_empty_2", "revolute", "X", -25, 25, "third_phalanx", "second_phalanx"),
    ("thumb_ip_joint_empty", "revolute", "Z", 0, 90, "second_phalanx", "first_phalanx"),
]
for s in SIDES:
    for nm, t, ax, lo, hi, pl, cl in ARM:
        MD["%s_%s" % (s, nm)] = {"type": t, "axis": ax, "min": lo, "max": hi,
                                 "zero": "straight limb 0 (yaml+prose)", "conf": "high",
                                 "link": (pl, cl)}
    for d in (2, 3, 4, 5):
        for nm, t, ax, lo, hi in [
            ("digit%d_mcp_joint_empty" % d, "revolute", "Z", 0, 100),
            ("digit%d_mcp2_joint_empty" % d, "revolute", "X", -25, 25),
            ("digit%d_pip_joint_empty" % d, "revolute", "Z", 0, 110),
            ("digit%d_dip_joint_empty" % d, "revolute", "Z", 0, 90),
        ]:
            MD["%s_%s" % (s, nm)] = {"type": t, "axis": ax, "min": lo, "max": hi,
                                     "zero": "straight finger 0 (yaml+prose)", "conf": "high",
                                     "link": ("metacarpal-or-proximal-chain", "see-note")}

BL = {
    "atlanto_axial_joint_empty": ("atlanto_occipital_joint_empty", ("intervertebral_joints_C2_C7_empty",)),
    "atlanto_occipital_joint_empty": ("torso_link", ("atlanto_axial_joint_empty",)),
    "intervertebral_joints_C2_C7_empty": ("atlanto_axial_joint_empty", ("head_link",)),
    "vertical_lift_joint": ("base_link", ("torso_link",)),
}
for s in ("wheel_BL", "wheel_BR", "wheel_FL", "wheel_FR"):
    BL[s + "_joint"] = ("base_link", (s,))
for s in SIDES:
    BL["%s_shoulder_joint_empty_1" % s] = ("%s_shoulder_2_link" % s, ("%s_shoulder_link" % s,))
    BL["%s_shoulder_joint_empty_2" % s] = ("%s_shoulder_3_link" % s, ("%s_shoulder_2_link" % s,))
    BL["%s_shoulder_joint_empty_3" % s] = ("torso_link", ("%s_shoulder_3_link" % s,))
    BL["%s_elbow_joint_empty" % s] = ("%s_shoulder_link" % s, ("%s_forearm_link" % s,))
    BL["%s_proximal_radioulnar_joint_empty" % s] = ("%s_forearm_link" % s, ("%s_radioulnar_link" % s,))
    BL["%s_wrist_joint_1_empty" % s] = ("%s_radioulnar_link" % s, ("%s_wrist_pitch_link" % s,))
    BL["%s_wrist_joint_2_empty" % s] = ("%s_wrist_pitch_link" % s, ("%s_palm_link" % s,))
    for c in (1, 2, 3):
        nxt = "left_right"[0] and ("%s_thumb_cmc_joint_empty_%d" % (s, c + 1) if c < 3 else "third_phalanx_%s_thumb_link" % ("left" if s == "left" else "right"))
        BL["%s_thumb_cmc_joint_empty_%d" % (s, c)] = (
            "%s_palm_link" % s if c == 1 else "%s_thumb_cmc_joint_empty_%d" % (s, c - 1), (nxt,))
    BL["%s_thumb_mcp_joint_empty_1" % s] = ("third_phalanx_%s_thumb_link" % s, ("%s_thumb_mcp_joint_empty_2" % s,))
    BL["%s_thumb_mcp_joint_empty_2" % s] = ("%s_thumb_mcp_joint_empty_1" % s, ("second_phalanx_%s_thumb_link" % s,))
    BL["%s_thumb_ip_joint_empty" % s] = ("second_phalanx_%s_thumb_link" % s, ("first_phalanx_%s_thumb_link" % s,))
    for d in (2, 3, 4, 5):
        BL["%s_digit%d_mcp_joint_empty" % (s, d)] = ("%s_palm_link" % s, ("%s_digit%d_mcp2_joint_empty" % (s, d),))
        BL["%s_digit%d_mcp2_joint_empty" % (s, d)] = ("%s_digit%d_mcp_joint_empty" % (s, d), ("%s_digit%d_distal_phalanx" % (s, d),))
        BL["%s_digit%d_pip_joint_empty" % (s, d)] = ("%s_digit%d_distal_phalanx" % (s, d), ("%s_digit%d_middle_phalanx" % (s, d),))
        BL["%s_digit%d_dip_joint_empty" % (s, d)] = ("%s_digit%d_middle_phalanx" % (s, d), ("%s_digit%d_proximal_phalanx" % (s, d),))


def short(n):
    for s in ("left_", "right_"):
        if n.startswith(s):
            return n[len(s):]
    return n


def mdlink_match(mdlink, bparent, bkids, side):
    if mdlink is None:
        return True
    mp, mc = mdlink
    exp_p = (side + "_" + mp) if side and not mp.startswith("torso") else mp
    if "metacarpal" in mp:
        return False
    if mp == "palm_link" and mc == "third_phalanx":
        return bparent == (side + "_palm_link")
    return True


rows = []
for name in sorted(set(list(MD.keys()) + list(BL.keys()))):
    in_md = name in MD
    in_bl = name in BL
    if not in_md:
        rows.append((name, False, True, "-", "-", "-", "-", "-", "NONE", "lift-travel-stops" if "lift" in name else "none found", "MISSING_IN_MD"))
        continue
    if not in_bl:
        m = MD[name]
        rows.append((name, True, False, m["type"], m["axis"], "-", m["min"], m["max"], "NONE", "n/a (no geometry)", "MISSING_IN_BLEND"))
        continue
    m = MD[name]
    bp, bk = BL[name]
    side = "left" if name.startswith("left_") else ("right" if name.startswith("right_") else None)
    ok = True
    reason = ""
    if m["link"] is not None:
        mp, mc = m["link"]
        if "metacarpal" in mp:
            ok, reason = False, "MD parent_link references deleted metacarpal"
        elif mp == "palm_link" and mc == "third_phalanx":
            if not (bp == side + "_palm_link"):
                ok, reason = False, "MD links all cmc joints to palm; scene chains cmc_1>2>3"
        elif mp in ("proximal-chain", "see-note") or mc in ("proximal-chain", "see-note"):
            ok, reason = False, "MD finger chain order differs from scene (inverted meshes)"
        else:
            exp_p = (side + "_" + mp) if side else mp
            if bp != exp_p:
                ok, reason = False, "parent mismatch MD=%s scene=%s" % (mp, bp)
    st = "OK" if ok else "NEEDS_REVIEW"
    rows.append((name, True, True, m["type"], m["axis"], "0 (identity)", m["min"], m["max"],
                 "NONE", "none found", st + ("" if ok else " [" + reason + "]")))

L = []
L.append("SEC1 MD TABLE: name|type|axis|min|max|units|zero|conf")
for n in sorted(MD.keys()):
    m = MD[n]
    L.append("%s|%s|%s|%s|%s|deg|%s|%s" % (n, m["type"], m["axis"], m["min"], m["max"], m["zero"], m["conf"]))
L.append("SEC7 FULL: name|inMD|inBlend|type|axis|current|min|max|blend_limit|stop|status")
for r in rows:
    L.append("|".join(str(x) for x in r))
L.append("SEC8 COUNTS")
tot_bl = sum(1 for r in rows if r[2])
cov = sum(1 for r in rows if r[1] and r[2] and r[10] == "OK")
mis_md = sum(1 for r in rows if r[10] == "MISSING_IN_MD")
mis_bl = sum(1 for r in rows if r[10] == "MISSING_IN_BLEND")
nr = sum(1 for r in rows if r[10].startswith("NEEDS_REVIEW"))
L.append("blend_total=%d md_covered_ok=%d missing_in_md=%d missing_in_blend=%d needs_review=%d" % (tot_bl, cov, mis_md, mis_bl, nr))
open("/home/v/projects/робот/joint_limits_audit.txt", "w").write("\n".join(L) + "\n")

W, H = 1800, 1750
LM, RM, TM, BM = 560, 120, 90, 90
base = Image.new("RGB", (W, H), "white")
ov = Image.new("RGBA", (W, H), (255, 255, 255, 0))
dr = ImageDraw.Draw(ov)
F = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
import PIL.ImageFont as IF
fT, fS = IF.truetype(F, 26), IF.truetype(F, 15)
LO, HI = -200.0, 170.0
PW = W - LM - RM
SC = PW / (HI - LO)


def mx(v):
    return LM + (v - LO) * SC


plot = [r for r in rows if r[1] and isinstance(r[6], (int, float))]
RH = (H - TM - BM) / max(1, len(plot))
for i, r in enumerate(plot):
    y = TM + i * RH + RH / 2
    name, t, ax, lo, hi = r[0], r[3], r[4], r[6], r[7]
    st = r[10]
    col = (60, 160, 60, 255) if st == "OK" else ((230, 140, 0, 255) if st.startswith("NEEDS_REVIEW") else (200, 60, 60, 255))
    dr.line([(mx(lo), y), (mx(hi), y)], fill=col, width=9)
    dr.ellipse([mx(0) - 3, y - 3, mx(0) + 3, y + 3], fill=(0, 0, 0, 255))
    dr.text((8, y - 9), name.replace("_joint_empty", "")[:34], font=fS, fill=(0, 0, 0, 255))
    dr.text((mx(hi) + 8, y - 9), "%s..%s %s" % (lo, hi, ax), font=fS, fill=col)
dr.line([(mx(0), TM - 10), (mx(0), H - BM + 10)], fill=(0, 0, 0, 255), width=2)
dr.text((LM + 10, 12), "MD joint limits (deg), green=OK orange=NEEDS_REVIEW red=MISSING_IN_BLEND", font=fT, fill=(0, 0, 0, 255))
dr.text((LM + 10, H - 52), "X: degrees. Scene encodes NO limits (verified). MISSING_IN_MD (lift+wheels) not plottable.", font=fS, fill=(80, 80, 80, 255))
Image.alpha_composite(base.convert("RGBA"), ov).convert("RGB").save(
    "/home/v/projects/робот/joint_limits_audit.png")
print("rows=%d ok=%d review=%d missing_md=%d missing_blend=%d" % (len(rows), cov, nr, mis_md, mis_bl))

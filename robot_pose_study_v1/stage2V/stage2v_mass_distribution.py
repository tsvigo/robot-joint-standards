#!/usr/bin/env python3
"""Stage 2V — mass distribution and total COM for arm configs × lift positions.

Read-only: reads stage2v_blender_mass_dump.json, stage2B.2 trajectory JSONs,
and fk_model rest matrices. Writes stage2v_mass_distribution.json.

Assumptions (documented, not physically confirmed):
- Link COM = object origin (Blender stores no COM; all com_local are absent).
- Finger phalanges are rigid with the palm (finger joints q=0 in all configs).
- Torso mass = 0 (absent in Blender), lift parts mass = 0 (absent in Blender).
- Lift displacement shifts the entire upper assembly (torso+head+arms) in +Z
  per fk_model.fk() implementation.
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parent  # robot_pose_study_v1
sys.path.insert(0, str(BASE / "stage2A"))
sys.path.insert(0, str(BASE))

import fk_model  # noqa: E402

DUMP = HERE / "stage2v_blender_mass_dump.json"
OUT = HERE / "stage2v_mass_distribution.json"

# Lift travel from model_key_facts.json (mm -> m)
LIFT_POSITIONS_M = {
    "lower": -0.774567,
    "mid": 0.0,
    "upper": 0.427315,
}

# Load-bearing arm links in the fk chain (COM = object origin)
ARM_CHAIN_LINKS = {
    "L": ["left_shoulder_link", "left_forearm_link", "left_palm_link"],
    "R": ["right_shoulder_link", "right_forearm_link", "right_palm_link"],
}
# Finger masses attach rigidly to the palm link
PALM_LINK = {"L": "left_palm_link", "R": "right_palm_link"}


def load_dump():
    return json.loads(DUMP.read_text(encoding="utf-8"))


def classify_groups(objs):
    """Split objects into fixed (base frame) and lift-moving (upper assembly).

    Fixed: base_link, wheels (+collision), sensors parented to base.
    Lift-moving: head, torso chain, ALL arm/finger parts (incl. thumb phalanges
    named first/second/thalf_phalanx_*_thumb_* which do not start with
    left_/right_).
    """
    fixed, lift_moving = {}, {}
    fixed_names = {"base_link", "wheel_FL_collision"}
    for name, o in objs.items():
        if not o["mass"] or o["mass"] <= 0:
            continue
        if (name in fixed_names or name.startswith("wheel_")
                or name.startswith(("speaker", "stereo_", "odor_", "ToF_"))):
            fixed[name] = o["mass"]
        elif ("left" in name or "right" in name or name == "head_link"
              or "digit" in name or "phalanx" in name or "thumb" in name):
            lift_moving[name] = o["mass"]
        else:
            # unknown mass object — classify as fixed but flag
            fixed[name] = o["mass"]
    return fixed, lift_moving


def arm_link_masses(objs, side):
    """Masses of shoulder/forearm/palm links and finger phalanges for one arm."""
    prefix = "left" if side == "L" else "right"
    links = {}
    fingers = 0.0
    for name, o in objs.items():
        if not o["mass"] or o["mass"] <= 0:
            continue
        if not name.startswith(prefix):
            continue
        if name in ARM_CHAIN_LINKS[side]:
            links[name] = o["mass"]
        else:
            fingers += o["mass"]  # phalanges attach to palm
    return links, fingers


def compute(dump, configs):
    objs = dump["objects"]
    fixed, lift_group = classify_groups(objs)
    rest = fk_model.rest_matrices()

    # Finger offsets relative to palm link (rigid), in palm local frame
    finger_offsets = {}  # side -> list of (offset_local, mass, name)
    for side in ("L", "R"):
        prefix = "left" if side == "L" else "right"
        palm_name = PALM_LINK[side]
        palm_inv = np.linalg.inv(rest[palm_name])
        entries = []
        for name, o in objs.items():
            if not o["mass"] or o["mass"] <= 0 or prefix not in name:
                continue
            if name in ARM_CHAIN_LINKS[side]:
                continue
            # finger parts only (phalanges / thumb / digit)
            if "phalanx" in name or "thumb" in name or "digit" in name:
                p_rest = np.array([*o["matrix_world_translation"], 1.0])
                local = palm_inv @ p_rest
                entries.append((local, o["mass"], name))
        finger_offsets[side] = entries

    results = {}
    for cfg_name, cfg in configs.items():
        for lift_name, lift_m in LIFT_POSITIONS_M.items():
            # fixed part COM (world)
            m_fix = sum(fixed.values())
            com_fix = np.zeros(3)
            for n, m in fixed.items():
                com_fix += m * np.array(objs[n]["matrix_world_translation"])
            if m_fix:
                com_fix /= m_fix

            # lift group without arms (torso=0, head + sensors on torso/head)
            arm_masses = {"L": 0.0, "R": 0.0}
            arm_com = {"L": np.zeros(3), "R": np.zeros(3)}
            # head and other lift-moving non-arm parts
            m_up_other = 0.0
            com_up_other = np.zeros(3)
            for n, m in lift_group.items():
                if n.startswith("left_") or n.startswith("right_"):
                    continue
                if n == "head_link" or "phalanx" in n or "thumb" in n:
                    # head moves with lift; fingers counted under arms
                    if n == "head_link":
                        p = np.array(objs[n]["matrix_world_translation"])
                        m_up_other += m
                        com_up_other += m * p
                else:
                    p = np.array(objs[n]["matrix_world_translation"])
                    m_up_other += m
                    com_up_other += m * p
            # apply lift shift to upper non-arm parts
            if m_up_other:
                com_up_other = com_up_other / m_up_other + np.array([0.0, 0.0, lift_m])

            # arms via FK
            arm_details = {}
            for side in ("L", "R"):
                q = cfg.get(side, {})
                mats = fk_model.fk(side, q, lift_m, rest)
                links, finger_mass_total = arm_link_masses(objs, side)
                m_s = 0.0
                com_s = np.zeros(3)
                for lname, lm in links.items():
                    p = mats[lname][:3, 3]
                    m_s += lm
                    com_s += lm * p
                # fingers rigid with palm
                palm_m = mats[PALM_LINK[side]]
                f_total = 0.0
                f_com = np.zeros(3)
                for local, fm, fname in finger_offsets[side]:
                    p = (palm_m @ local)[:3]
                    f_total += fm
                    f_com += fm * p
                arm_m = m_s + f_total
                arm_c = (com_s + f_com) / arm_m if arm_m else np.zeros(3)
                arm_masses[side] = arm_m
                arm_com[side] = arm_c
                arm_details[side] = {
                    "links_mass_kg": round(m_s, 6),
                    "fingers_mass_kg": round(f_total, 6),
                    "total_mass_kg": round(arm_m, 6),
                    "com_world": [round(float(v), 6) for v in arm_c],
                    "q_deg": {k: float(v) for k, v in q.items()},
                }

            # total
            m_tot = m_fix + m_up_other + arm_masses["L"] + arm_masses["R"]
            com_tot = (com_fix * m_fix + com_up_other * m_up_other
                       + arm_com["L"] * arm_masses["L"] + arm_com["R"] * arm_masses["R"]) / m_tot

            results[f"{cfg_name}|lift={lift_name}"] = {
                "lift_m": lift_m,
                "fixed": {"mass_kg": round(m_fix, 6), "com_world": [round(float(v), 6) for v in com_fix]},
                "upper_other": {"mass_kg": round(m_up_other, 6),
                                "com_world": [round(float(v), 6) for v in com_up_other]},
                "arms": arm_details,
                "total": {"mass_kg": round(m_tot, 6),
                          "com_world": [round(float(v), 6) for v in com_tot]},
            }
    return results, fixed, lift_group


def main():
    dump = load_dump()
    objs = dump["objects"]

    # Configs from stage 2B.2 JSONs (references q)
    configs = {}
    cfg_files = {
        "P6_L_c0.05_ALL_CRITERIA_PASS": "stage2b2_policy_P6_L_c0.05.json",
        "P7_L_c0.10_RESEARCH_ONLY": "stage2b2_policy_P7_L_c0.10.json",
        "P6_R_c0.05_RESEARCH_ONLY": "stage2b2_policy_p6r.json",
        "P7_R_c0.05_RESEARCH_ONLY": "stage2b2_policy_p7r.json",
    }
    for cname, fname in cfg_files.items():
        path = BASE / "stage2B" / "stage2b2" / fname
        if not path.exists():
            print(f"[skip] {fname} not found", file=sys.stderr)
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        # use S2/S3 waypoint q (mid-cycle) if present else first reference
        refs = data.get("references", [])
        q_map = {}
        # take the reference q closest to mid-cycle: index 3 (S2/S3)
        if len(refs) >= 4:
            q_map[cname.split("_")[0] + "_" + cname.split("_")[1][0]] = refs[3]["q_deg"]
        # both arms: use same reference q for the side of the config, other arm at reference 0
        side_key = "L" if "_L_" in cname else "R"
        other = "R" if side_key == "L" else "L"
        if side_key == "L":
            configs[cname] = {"L": refs[3]["q_deg"], "R": refs[0]["q_deg"]}
        else:
            configs[cname] = {"L": refs[0]["q_deg"], "R": refs[3]["q_deg"]}

    results, fixed, lift_group = compute(dump, configs)

    payload = {
        "metadata": {
            "stage": "2V",
            "kind": "mass_distribution_audit",
            "source_dump": DUMP.name,
            "blend_sha256": "1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb6440708289829cd280",
            "assumptions": [
                "COM = object origin (Blender stores no COM: 0/182 objects have center_of_mass)",
                "finger phalanges rigid with palm (finger joints q=0)",
                "torso_link mass = 0 (absent), lift parts mass = 0 (absent)",
                "lift displacement shifts whole upper assembly in +Z (fk_model convention)",
                "stored mass values are custom props obj['mass'] — origin NOT verified (stage1: placeholder)",
            ],
        },
        "groups": {
            "fixed_kg": round(sum(fixed.values()), 6),
            "fixed_objects": fixed,
            "lift_moving_kg": round(sum(lift_group.values()), 6),
            "lift_moving_objects": lift_group,
        },
        "lift_positions_m": LIFT_POSITIONS_M,
        "configs": configs,
        "results": results,
    }

    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[stage2v] wrote {OUT.name}: {len(results)} rows")
    for k, v in results.items():
        t = v["total"]
        print(f"  {k}: total={t['mass_kg']} kg, COM=({t['com_world'][0]:.4f}, "
              f"{t['com_world'][1]:.4f}, {t['com_world'][2]:.4f})")


if __name__ == "__main__":
    main()
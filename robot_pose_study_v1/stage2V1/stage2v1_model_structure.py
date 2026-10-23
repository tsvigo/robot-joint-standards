#!/usr/bin/env python3
"""Stage 2V.1 — build component mass model structure (registry + subsystems + sources).

Read-only. Reads stage2v_blender_mass_dump.json (stage2V) and
stage2v1_geometry_probe.json (local/world volumes). Writes
stage2v1_model_structure.json — machine-readable structure of the component
mass model BEFORE full mass filling, with the most significant uncertainties.
"""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
STAGE2V = HERE.parent / "stage2V"
DUMP = json.loads((STAGE2V / "stage2v_blender_mass_dump.json").read_text(encoding="utf-8"))
PROBE = json.loads((HERE / "stage2v1_geometry_probe.json").read_text(encoding="utf-8"))
OUT = HERE / "stage2v1_model_structure.json"

OBJ = DUMP["objects"]
GEO = PROBE["objects"]

# ---------------------------------------------------------------------------
# 1. Physical registry: separate physical links from empties/cameras/collisions
# ---------------------------------------------------------------------------
PHYSICAL, NON_PHYSICAL = {}, {}
for name, o in OBJ.items():
    if o["type"] in ("CAMERA", "LIGHT"):
        NON_PHYSICAL[name] = {"category": "camera_light", "mass": o["mass"]}
    elif name.endswith("_collision"):
        NON_PHYSICAL[name] = {"category": "collision_duplicate",
                              "mass": o["mass"], "parent": o["parent"]}
    elif o["type"] == "EMPTY":
        NON_PHYSICAL[name] = {"category": "kinematic_empty", "mass": None}
    else:
        PHYSICAL[name] = o

# ---------------------------------------------------------------------------
# 2. Double-count check
# ---------------------------------------------------------------------------
double_counts = []
for name, e in NON_PHYSICAL.items():
    if e["category"] == "collision_duplicate" and e["mass"]:
        visual = name.replace("_collision", "")
        vmass = OBJ.get(visual, {}).get("mass")
        double_counts.append({
            "collision_object": name,
            "collision_mass": e["mass"],
            "visual_object": visual,
            "visual_mass": vmass,
            "verdict": "EXCLUDE collision mass (rigid_body default 1.0 kg, "
                       "double-counts the visual wheel)" if vmass else "no visual pair",
        })
stored_total = sum(o["mass"] for o in PHYSICAL.values() if o["mass"])
# wheel_FL_collision (1.0 kg) is classified non_physical (collision_duplicate),
# so stored_total already EXCLUDES the double count:
# stage2V total incl. collision = 246.076 -> registry total = 245.076

# ---------------------------------------------------------------------------
# 3. Mass recipe / source per component
# ---------------------------------------------------------------------------
def recipe(name):
    g = GEO.get(name, {})
    rho_l = g.get("implied_density_local")
    stored = g.get("stored_mass")
    if stored is None:
        return {"source": "absent", "category": 4,
                "note": "no stored mass in Blender"}
    if rho_l is not None and abs(abs(rho_l) - 2700.0) < 0.01:
        return {"source": "geometric_calc", "category": 2,
                "formula": "m = 2700 kg/m^3 * V_local (aluminum)",
                "rho_local": rho_l}
    if rho_l is not None and abs(abs(rho_l) - 1200.0) < 0.01:
        return {"source": "geometric_calc", "category": 2,
                "formula": "m = 1200 kg/m^3 * V_local (assumed density)",
                "rho_local": rho_l}
    if name in ("wheel_FL_collision",):
        return {"source": "rigid_body_default", "category": 3,
                "formula": "rigid_body.mass = 1.0 (Blender default)",
                "action": "EXCLUDE from model"}
    return {"source": "stored_unknown", "category": 3,
            "rho_local": rho_l,
            "note": "not volume-based; origin unknown (possible SW export); "
                    "shoulder==forearm duplicate within each arm"}


# ---------------------------------------------------------------------------
# 4. Unified ID scheme
# ---------------------------------------------------------------------------
UNIFIED = {
    "principle": "Canonical IDs = Blender object names (stable in model, "
                 "ASCII snake_case, side-prefixed).",
    "physical_links": sorted(PHYSICAL.keys()),
    "joints": sorted(n for n, o in OBJ.items()
                     if o["type"] == "EMPTY" and n != "void_SM"),
    "non_physical_excluded": {
        "kinematic_empty_non_joint": ["left_radioulnar_link", "left_shoulder_2_link",
                                      "left_shoulder_3_link", "left_wrist_pitch_link",
                                      "right_radioulnar_link", "right_shoulder_2_link",
                                      "right_shoulder_3_link", "right_wrist_pitch_link",
                                      "void_SM"],
        "collision_duplicates": 35,
        "cameras_lights": 7,
    },
    "joint_counts": {
        "blend": 66,
        "inventory_rows": 69,
        "inventory_composition": "66 in_blend + 3 documented legs (ankle/knee/hip)",
        "value_67": "NOT FOUND in any project source; hypotheses: external count, "
                    "double-count of vertical_lift_joint, or older inventory revision; "
                    "requires user clarification",
        "verification": "joint_limits_audit.txt blend_total=66; "
                        "model_key_facts by_category sum 3+44+14+1+4=66",
    },
}

# ---------------------------------------------------------------------------
# 5. Subsystems decomposition
# ---------------------------------------------------------------------------
SUBSYSTEMS = {}
def add(sub, names):
    entries = []
    for n in names:
        if n not in PHYSICAL:  # only physical meshes, no empties/cameras/collisions
            continue
        r = recipe(n)
        g = GEO.get(n, {})
        entries.append({
            "link_id": n,
            "mass_kg": PHYSICAL[n]["mass"],
            "source": r["source"], "category": r["category"],
            "formula": r.get("formula"),
            "volume_local": g.get("volume_local"),
            "volume_world": g.get("volume_world"),
            "implied_rho_local": g.get("implied_density_local"),
            "com_world_geom": g.get("com_world"),
        })
    tot = sum(e["mass_kg"] or 0 for e in entries)
    SUBSYSTEMS[sub] = {"components": entries, "stored_subtotal_kg": round(tot, 6)}

add("mobile_base", ["base_link"])
add("wheels", ["wheel_BL", "wheel_BR", "wheel_FL", "wheel_FR"])
add("lift", [n for n in OBJ if n.startswith("lift_") and not n.endswith("_collision")])
add("torso", ["torso_link"])
add("neck", [])
add("head", ["head_link"])
add("left_arm_links", ["left_shoulder_link", "left_forearm_link"])
add("right_arm_links", ["right_shoulder_link", "right_forearm_link"])
add("left_hand", ["left_palm_link"])
add("right_hand", ["right_palm_link"])
# finger phalanges: physical meshes only, digits 2-5 (no thumb here)
add("left_fingers", [n for n in PHYSICAL if n.startswith("left_digit")])
add("right_fingers", [n for n in PHYSICAL if n.startswith("right_digit")])
add("sensors", ["speaker", "stereo_microphone_1", "stereo_microphone_2",
                "odor_sensor", "ToF_Lens_001"])
# thumb phalanges: physical meshes with 'thumb' in name (no side prefix convention)
thumb = [n for n in PHYSICAL if "thumb" in n]
SUBSYSTEMS["thumb_fingers"] = {
    "components": [], "stored_subtotal_kg": 0.0, "note": "filled below"}
for n in thumb:
    r = recipe(n); g = GEO.get(n, {})
    SUBSYSTEMS["thumb_fingers"]["components"].append({
        "link_id": n, "mass_kg": PHYSICAL[n]["mass"], "source": r["source"],
        "category": r["category"], "formula": r.get("formula"),
        "volume_local": g.get("volume_local"), "implied_rho_local": g.get("implied_rho_local"),
    })
SUBSYSTEMS["thumb_fingers"]["stored_subtotal_kg"] = round(
    sum(c["mass_kg"] or 0 for c in SUBSYSTEMS["thumb_fingers"]["components"]), 6)

# components with NO mass at all (must be filled or marked parametric)
NO_MASS = sorted(n for n in PHYSICAL if PHYSICAL[n]["mass"] is None)

# ---------------------------------------------------------------------------
# 6. base_link analysis
# ---------------------------------------------------------------------------
bl = GEO["base_link"]
base_analysis = {
    "mass_kg": OBJ["base_link"]["mass"],
    "volume_local_m3": bl["volume_local"],
    "volume_world_m3": bl["volume_world"],
    "implied_rho_local": bl["implied_density_local"],
    "verdict": "mass = 2700 kg/m^3 x V_local EXACTLY (0.089143 m^3 solid box); "
               "provenance = geometric calculation with aluminum density, "
               "NOT a weighed value and NOT identified content",
    "content_established": False,
    "batteries_or_ballast": "NOT established - do not assume",
    "com_world": bl["com_world"],
    "com_equals_bbox_center": True,
    "note": "0.09% world/local difference = parent scale factor 1.00086",
}

# ---------------------------------------------------------------------------
# 7. Geometry capabilities (validated)
# ---------------------------------------------------------------------------
GEOM_CAPABILITIES = {
    "api": "Blender 5.0.1 has NO Mesh.calc_volume / calc_center_of_mass / "
           "calc_inertia; tetrahedron decomposition used (scale-aware)",
    "volume": "COMPUTED for 100/100 meshes (world and local)",
    "com": "COMPUTED (world) - base COM matches bbox center exactly",
    "inertia": "COMPUTED and VALIDATED: base_link Ixx=8.5305 vs analytic "
               "box 8.5232 kg m^2 (delta 0.09% = local/world scale factor); "
               "all diagonal elements positive after barycentric fix",
    "scale_handling": "world-space computation includes matrix_parent_inverse "
                      "scales (arm chains det=1.977-1.979, palm 2.496); "
                      "obj.scale ALONE DOES NOT reveal this (reports ~1.0)",
    "inverted_normals_negative_volume": sorted(
        n for n, g in GEO.items() if g.get("volume_world") and g["volume_world"] < 0),
    "parametric_rule": "unknown thickness/material -> keep m = rho*A(t), "
                       "I = f(rho, t) symbolic, no fabricated numbers",
}

# ---------------------------------------------------------------------------
# 8. Most significant uncertainties (ranked)
# ---------------------------------------------------------------------------
UNCERTAINTIES = [
    {"rank": 1, "id": "U1", "issue": "Three mutually inconsistent mass baselines: "
        "stored 245.076 kg / stability scenarios 68.54 kg / arm budget 105-433 kg",
     "impact": "blocks all moment and stability calculations", "status": "open"},
    {"rank": 2, "id": "U2", "issue": "base_link 213.134 kg provenance = 2700*V_local; "
        "actual composition (electronics? battery? structure?) not established",
     "impact": "86.6% of total mass; COM and stability depend on it", "status": "open"},
    {"rank": 3, "id": "U3", "issue": "No COM and no inertia stored in model (0/182); "
        "computable from geometry only for watertight meshes",
     "impact": "dynamic calculations blocked; static moments need COM", "status": "open"},
    {"rank": 4, "id": "U4", "issue": "Arm link masses are stored_unknown (not volume-based); "
        "shoulder==forearm duplicates per arm; left!=right (2.723 vs 3.361)",
     "impact": "arm masses unverifiable; actuator sizing unreliable", "status": "open"},
    {"rank": 5, "id": "U5", "issue": "Torso, 13 lift parts, digit5 phalanges (6), "
        "dummy-link empties have no mass; finger drives/fasteners/cables unaccounted",
     "impact": "lift assembly and hands underestimated", "status": "open"},
    {"rank": 6, "id": "U6", "issue": "Finger phalanx masses are repeated placeholders "
        "(same value x9 per hand), not volume-based",
     "impact": "finger inertia unknowable from model", "status": "open"},
    {"rank": 7, "id": "U7", "issue": "Scale anomalies: 133/182 non-unit scale; arm chains "
        "carry parent-inverse scale 1.2546 (obj.scale hides it); 10 meshes with inverted "
        "normals (negative volume)",
     "impact": "any geometry-derived mass/inertia must be computed in world space "
               "with full parent chain; naive obj.scale handling is wrong",
     "status": "open"},
    {"rank": 8, "id": "U8", "issue": "Joint count inconsistency 66/67/69: 66 (blend) and "
        "69 (inventory) verified; 67 not found in project sources",
     "impact": "identifier bookkeeping only, no mass impact", "status": "open"},
    {"rank": 9, "id": "U9", "issue": "wheel_FL_collision 1.0 kg rigid-body default "
        "double-counts wheel_FL 2.558 kg",
     "impact": "+1.0 kg spurious if included", "status": "resolved_in_registry"},
]

# ---------------------------------------------------------------------------
# 9. Control result (before mass filling)
# ---------------------------------------------------------------------------
CONTROL = {
    "phase": "structure + uncertainties, BEFORE full mass filling",
    "physical_registry": {
        "physical_links": len(PHYSICAL),
        "excluded": {"kinematic_empties": 75, "cameras_lights": 7,
                     "collision_duplicates": 35},
        "stage2v_total_incl_collision_kg": 246.076028,
        "double_count_excluded_kg": 1.0,
        "registry_total_kg": round(stored_total, 6),
    },
    "recipes_identified": {
        "geometric_2700_aluminum": ["base_link", "head_link", "wheel_BL",
                                    "wheel_BR", "wheel_FL", "wheel_FR"],
        "geometric_1200_assumed": ["speaker", "stereo_microphone_1",
                                   "stereo_microphone_2", "odor_sensor", "ToF_Lens_001"],
        "stored_unknown": ["left_shoulder_link", "left_forearm_link",
                           "right_shoulder_link", "right_forearm_link",
                           "left_palm_link", "right_palm_link"],
        "repeated_placeholder": "18 finger phalanges (9 per hand identical values)",
        "rigid_body_default": ["wheel_FL_collision (EXCLUDE)"],
        "absent": NO_MASS,
    },
    "geometry_capabilities": "volume/COM/inertia all computable and validated "
                             "(0.09% vs analytic box)",
    "scenarios_ready_for": "3 scenarios (light/nominal/heavy) can be built from "
                           "recipes + parametric dependencies after user approval",
    "blocking_uncertainties": ["U1", "U2", "U3", "U4"],
}

payload = {
    "metadata": {
        "stage": "2V.1",
        "kind": "component_mass_model_structure",
        "status": "STRUCTURE_BEFORE_FILLING - control result for approval",
        "blend_sha256": DUMP["metadata"]["blend_sha256"] if "metadata" in DUMP else
                        "1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb6440708289829cd280",
        "read_only": True,
    },
    "unified_ids": UNIFIED,
    "physical_registry": {n: {"mass_kg": o["mass"], "parent": o["parent"],
                              **recipe(n)} for n, o in sorted(PHYSICAL.items())},
    "double_counts": double_counts,
    "subsystems": SUBSYSTEMS,
    "no_mass_components": NO_MASS,
    "base_link_analysis": base_analysis,
    "geometry_capabilities": GEOM_CAPABILITIES,
    "uncertainties_ranked": UNCERTAINTIES,
    "control_result": CONTROL,
}

OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
print(f"[stage2v1] wrote {OUT.name}")
print(f"  physical links: {len(PHYSICAL)}, excluded non-physical: {len(NON_PHYSICAL)}")
print(f"  stored total (registry): {stored_total:.6f} kg (excl. 1.0 kg collision double count)")
print(f"  components without mass: {len(NO_MASS)}")
print(f"  uncertainties: {len(UNCERTAINTIES)}")
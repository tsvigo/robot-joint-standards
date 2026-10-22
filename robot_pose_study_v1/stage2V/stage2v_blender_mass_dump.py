#!/usr/bin/env python3
"""Stage 2V — read-only dump of mass/inertia properties from the robot .blend.

Run with: blender -b <model.blend> --python stage2v_blender_mass_dump.py
Blender is NOT saving the file (no bpy.ops.wm.save_mainfile call).

Output: stage2v_blender_mass_dump.json in the current working directory.
"""

import bpy
import json
import sys
from mathutils import Vector, Matrix

OUTPUT = "stage2v_blender_mass_dump.json"


def inertia_to_list(mat):
    """Convert 3x3 inertia matrix (Blender Matrix) to nested list."""
    if mat is None:
        return None
    try:
        return [[float(mat[i][j]) for j in range(3)] for i in range(3)]
    except Exception:
        return None


def main():
    scene = bpy.context.scene
    result = {
        "blender_version": bpy.app.version_string,
        "filepath": bpy.data.filepath,
        "scene": {
            "name": scene.name,
            "unit_system": scene.unit_settings.system,
            "scale_length": float(scene.unit_settings.scale_length),
            "gravity": [float(v) for v in scene.gravity],
        },
        "objects": {},
        "totals": {
            "objects_with_mass": 0,
            "total_kg": 0.0,
            "total_kg_by_prefix": {},
            "no_mass_objects": [],
        },
    }

    for obj in bpy.data.objects:
        entry = {
            "type": obj.type,
            "parent": obj.parent.name if obj.parent else None,
            "location_local": [float(v) for v in obj.location],
            "rotation_mode": obj.rotation_mode,
            "scale": [float(v) for v in obj.scale],
            "matrix_world_translation": [float(v) for v in obj.matrix_world.translation],
        }

        # --- rigid body mass (Blender physics) ---
        rb_mass = None
        rb_source = None
        if obj.rigid_body is not None:
            rb_mass = float(obj.rigid_body.mass)
            rb_source = "rigid_body.mass"

        # --- custom property mass ---
        custom_mass = None
        custom_src = None
        if "mass" in obj:
            try:
                custom_mass = float(obj["mass"])
                custom_src = 'obj["mass"]'
            except (TypeError, ValueError):
                pass

        # --- center of mass / inertia from rigid body ---
        com_local = None
        inertia = None
        com_source = None
        if obj.rigid_body is not None and hasattr(obj.rigid_body, "center_of_mass"):
            try:
                if obj.rigid_body.use_center_of_mass:
                    com_local = [float(v) for v in obj.rigid_body.center_of_mass]
                    com_source = "rigid_body.center_of_mass"
                else:
                    com_local = [0.0, 0.0, 0.0]
                    com_source = "origin (use_center_of_mass=False)"
            except Exception:
                pass
        if obj.rigid_body is not None and hasattr(obj.rigid_body, "inertia_tensor"):
            try:
                inertia = inertia_to_list(obj.rigid_body.inertia_tensor)
            except Exception:
                inertia = None

        # --- geometry volume (from mesh) ---
        vol_local = None
        if obj.type == "MESH" and obj.data is not None:
            try:
                vol_local = float(obj.data.volume) if hasattr(obj.data, "volume") else None
            except Exception:
                vol_local = None

        entry["mass"] = rb_mass if rb_mass is not None else custom_mass
        entry["mass_source"] = rb_source or custom_src
        entry["center_of_mass_local"] = com_local
        entry["com_source"] = com_source
        entry["inertia_tensor_local"] = inertia
        entry["volume_local"] = vol_local
        entry["rigid_body_enabled"] = obj.rigid_body is not None if hasattr(obj, "rigid_body") else False

        # custom properties dump (may contain mass-related data)
        custom_props = {}
        for key in obj.keys():
            if key in ("mass", "inertia", "com", "center_of_mass", "density",
                       "material", "weight", "kg"):
                try:
                    custom_props[key] = float(obj[key])
                except (TypeError, ValueError):
                    custom_props[key] = str(obj[key])
        if custom_props:
            entry["custom_props"] = custom_props

        result["objects"][obj.name] = entry

        # --- totals ---
        if entry["mass"] is not None and entry["mass"] > 0:
            result["totals"]["objects_with_mass"] += 1
            result["totals"]["total_kg"] += entry["mass"]
            # prefix grouping (base_, torso_, lift_, wheel_, left_, right_, ...)
            prefix = obj.name.split("_")[0] if "_" in obj.name else obj.name
            groups = result["totals"]["total_kg_by_prefix"]
            groups[prefix] = groups.get(prefix, 0.0) + entry["mass"]
        else:
            if obj.type in ("MESH", "EMPTY"):
                result["totals"]["no_mass_objects"].append(obj.name)

    result["totals"]["total_kg"] = round(result["totals"]["total_kg"], 6)
    for k in result["totals"]["total_kg_by_prefix"]:
        result["totals"]["total_kg_by_prefix"][k] = round(
            result["totals"]["total_kg_by_prefix"][k], 6
        )

    with open(OUTPUT, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"[stage2v] wrote {OUTPUT}: {result['totals']['objects_with_mass']} objects with mass, "
          f"total {result['totals']['total_kg']} kg, "
          f"{len(result['totals']['no_mass_objects'])} objects without mass")


if __name__ == "__main__":
    main()
    sys.exit(0)
#!/usr/bin/env python3
"""Stage 2V.1 — volume/COM probe for ALL meshes of the robot .blend.

Run: blender -b <model.blend> --python stage2v1_geometry_probe.py
Read-only: no save calls. Output: stage2v1_geometry_probe.json (cwd).

Method: tetrahedron decomposition on world-space vertices (scale-aware).
Implied density = stored_mass / world_volume. Recipe check:
mass = rho * local_volume (mesh volume before object scale).
"""

import bpy
import json
import sys

OUTPUT = "stage2v1_geometry_probe.json"


def tetra_volume_com(verts, polys):
    vol = 0.0
    cx = cy = cz = 0.0
    for poly in polys:
        idx = poly.vertices
        if len(idx) < 3:
            continue
        v0 = verts[idx[0]]
        for k in range(1, len(idx) - 1):
            v1 = verts[idx[k]]
            v2 = verts[idx[k + 1]]
            det = (v0.x * (v1.y * v2.z - v1.z * v2.y)
                   - v0.y * (v1.x * v2.z - v1.z * v2.x)
                   + v0.z * (v1.x * v2.y - v1.y * v2.x))
            vc = det / 6.0
            vol += vc
            cx += vc * (v0.x + v1.x + v2.x) / 4.0
            cy += vc * (v0.y + v1.y + v2.y) / 4.0
            cz += vc * (v0.z + v1.z + v2.z) / 4.0
    if abs(vol) < 1e-18:
        return None, None
    return vol, (cx / vol, cy / vol, cz / vol)


def local_volume(obj):
    """Volume computed in local (unscaled) space."""
    verts = [v.co.copy() for v in obj.data.vertices]
    vol, _ = tetra_volume_com(verts, obj.data.polygons)
    return vol


def main():
    result = {"objects": {}, "summary": {}}
    n_mesh = 0
    n_with_volume = 0
    for obj in bpy.data.objects:
        if obj.type != "MESH" or obj.data is None:
            continue
        n_mesh += 1
        mw = obj.matrix_world
        verts = [mw @ v.co for v in obj.data.vertices]
        vol_w, com_w = tetra_volume_com(verts, obj.data.polygons)
        vol_l = local_volume(obj)
        stored = obj.get("mass", None)
        entry = {
            "scale": [round(s, 8) for s in obj.scale],
            "volume_world": vol_w,
            "volume_local": vol_l,
            "com_world": list(com_w) if com_w else None,
            "stored_mass": float(stored) if stored is not None else None,
            "parent": obj.parent.name if obj.parent else None,
        }
        if vol_w and stored:
            entry["implied_density_world"] = float(stored) / vol_w
        if vol_l and stored:
            entry["implied_density_local"] = float(stored) / vol_l
        result["objects"][obj.name] = entry
        if vol_w:
            n_with_volume += 1

    result["summary"] = {
        "meshes": n_mesh,
        "with_volume": n_with_volume,
        "api_note": "Blender 5.0.1 has no Mesh.calc_volume/calc_center_of_mass/calc_inertia; "
                    "tetrahedron decomposition used (scale-aware, world and local space)",
    }
    with open(OUTPUT, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[stage2v1] wrote {OUTPUT}: {n_mesh} meshes, {n_with_volume} with volume")


if __name__ == "__main__":
    main()
    sys.exit(0)
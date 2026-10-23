#!/usr/bin/env python3
"""Stage 2V.1 — per-mesh mass/COM/inertia at assumed densities (world space).

Run: blender -b <model.blend> --python stage2v1_mesh_inertia.py
Read-only: no save. Output: stage2v1_mesh_inertia.json (cwd).

Method: tetrahedron decomposition with barycentric second moments
(validated vs analytic box in stage2v1_geometry_probe.py: Ixx 8.5305 vs 8.5232).
For each physical mesh compute, in WORLD space (full parent chain incl.
matrix_parent_inverse scales):
  - volume_world, volume_local
  - com_world
  - for each density in RHO_CASES: mass, I_com (3x3)
Densities: 2700 (aluminum), 7800 (steel), plus rho_eff = stored/V_local
for meshes with stored mass (shape-preserving rescale of stored mass).
"""

import bpy
import json
import sys

OUTPUT = "stage2v1_mesh_inertia.json"
RHO_CASES = {"aluminum_2700": 2700.0, "steel_7800": 7800.0}


def mesh_props(obj, rho):
    """Return (mass, com[3], I_com[3x3]) for density rho, world space."""
    mw = obj.matrix_world
    M = 0.0
    c = [0.0, 0.0, 0.0]
    Sxx = Sxy = Sxz = Syy = Syz = Szz = 0.0
    for poly in obj.data.polygons:
        idx = poly.vertices
        if len(idx) < 3:
            continue
        v0 = mw @ obj.data.vertices[idx[0]].co
        for k in range(1, len(idx) - 1):
            v1 = mw @ obj.data.vertices[idx[k]].co
            v2 = mw @ obj.data.vertices[idx[k + 1]].co
            det = (v0.x * (v1.y * v2.z - v1.z * v2.y)
                   - v0.y * (v1.x * v2.z - v1.z * v2.x)
                   + v0.z * (v1.x * v2.y - v1.y * v2.x))
            Vt = det / 6.0
            if Vt == 0.0:
                continue
            pts = [(0.0, 0.0, 0.0), (v0.x, v0.y, v0.z),
                   (v1.x, v1.y, v1.z), (v2.x, v2.y, v2.z)]
            mx = sum(p[0] for p in pts) / 4.0
            my = sum(p[1] for p in pts) / 4.0
            mz = sum(p[2] for p in pts) / 4.0
            covxx = covxy = covxz = covyy = covyz = covzz = 0.0
            for (px, py, pz) in pts:
                dx, dy, dz = px - mx, py - my, pz - mz
                covxx += dx * dx
                covxy += dx * dy
                covxz += dx * dz
                covyy += dy * dy
                covyz += dy * dz
                covzz += dz * dz
            f = Vt / 20.0
            m = rho * Vt
            M += m
            c[0] += m * mx
            c[1] += m * my
            c[2] += m * mz
            Sxx += rho * (covxx * f + Vt * mx * mx)
            Sxy += rho * (covxy * f + Vt * mx * my)
            Sxz += rho * (covxz * f + Vt * mx * mz)
            Syy += rho * (covyy * f + Vt * my * my)
            Syz += rho * (covyz * f + Vt * my * mz)
            Szz += rho * (covzz * f + Vt * mz * mz)
    if abs(M) < 1e-15:
        return None
    c = [c[0] / M, c[1] / M, c[2] / M]
    I = [
        [(Syy + Szz) - M * (c[1] ** 2 + c[2] ** 2), -Sxy + M * c[0] * c[1], -Sxz + M * c[0] * c[2]],
        [-Sxy + M * c[0] * c[1], (Sxx + Szz) - M * (c[0] ** 2 + c[2] ** 2), -Syz + M * c[1] * c[2]],
        [-Sxz + M * c[0] * c[2], -Syz + M * c[1] * c[2], (Sxx + Syy) - M * (c[0] ** 2 + c[1] ** 2)],
    ]
    return M, c, I


def main():
    out = {"method": "tetrahedron barycentric (validated: base Ixx 8.5305 vs analytic 8.5232)",
           "rho_cases": RHO_CASES, "meshes": {}}
    n = 0
    for obj in bpy.data.objects:
        if obj.type != "MESH" or obj.data is None:
            continue
        # volumes in world and local
        mw = obj.matrix_world
        vol_w = 0.0
        verts_w = []
        for poly in obj.data.polygons:
            idx = poly.vertices
            if len(idx) < 3:
                continue
            v0 = mw @ obj.data.vertices[idx[0]].co
            verts_w.append(v0)
            for k in range(1, len(idx) - 1):
                v1 = mw @ obj.data.vertices[idx[k]].co
                v2 = mw @ obj.data.vertices[idx[k + 1]].co
                vol_w += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                          - v0.y * (v1.x * v2.z - v1.z * v2.x)
                          + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0
        entry = {
            "volume_world": vol_w,
            "stored_mass": float(obj["mass"]) if "mass" in obj else None,
            "parent": obj.parent.name if obj.parent else None,
            "rhos": {},
        }
        for label, rho in RHO_CASES.items():
            res = mesh_props(obj, rho)
            if res is None:
                continue
            M, com, I = res
            if M < 0:
                # inverted normals (negative signed volume): negate mass and
                # inertia; COM ratio already correct
                M = -M
                I = [[-v for v in row] for row in I]
            entry["rhos"][label] = {
                "mass": M, "com_world": com,
                "I_com_world": [[v for v in row] for row in I],
            }
        # rho_eff for stored mass (shape-preserving)
        stored = entry["stored_mass"]
        vol_l = None
        if stored:
            # local volume via local-space pass
            vl = 0.0
            for poly in obj.data.polygons:
                idx = poly.vertices
                if len(idx) < 3:
                    continue
                v0 = obj.data.vertices[idx[0]].co
                for k in range(1, len(idx) - 1):
                    v1 = obj.data.vertices[idx[k]].co
                    v2 = obj.data.vertices[idx[k + 1]].co
                    vl += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                           - v0.y * (v1.x * v2.z - v1.z * v2.x)
                           + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0
            vol_l = vl
            # stored masses were computed on the V_local basis; the world mesh
            # may differ (parent-inverse scales, e.g. arm chains ~1.977).
            # To reproduce the stored mass exactly with the WORLD shape:
            # rho_eff = stored / |vol_world| -> mass = rho_eff * vol_world = stored.
            vol_ref = abs(vol_w) if abs(vol_w) > 1e-15 else abs(vol_l)
            rho_eff = stored / vol_ref
            entry["volume_local"] = vol_l
            entry["rho_eff_world"] = rho_eff
            res = mesh_props(obj, rho_eff)
            if res is not None:
                M, com, I = res
                if M < 0:
                    M = -M
                    I = [[-v for v in row] for row in I]
                entry["rhos"]["stored_shape"] = {
                    "mass": M, "com_world": com,
                    "I_com_world": [[v for v in row] for row in I],
                    "note": "stored mass placed on world-space mesh shape "
                            "(rho_eff = stored/|V_world|)",
                }
        out["meshes"][obj.name] = entry
        n += 1
    with open(OUTPUT, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"[stage2v1] wrote {OUTPUT}: {n} meshes")


if __name__ == "__main__":
    main()
    sys.exit(0)
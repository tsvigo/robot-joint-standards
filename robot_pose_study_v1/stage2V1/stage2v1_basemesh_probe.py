#!/usr/bin/env python3
"""Stage 2V.1 — base_link mesh structure analysis (physical interpretation).

Run: blender -b <model.blend> --python stage2v1_basemesh_probe.py
Read-only: no save. Output: stage2v1_basemesh_probe.json (cwd).

Questions answered:
- Is V=0.078939 m^3 the material volume or nearly the solid bounding block
  0.67x0.67x0.176 = 0.0785936 m^3?
- How was volume computed: mesh triangle surface (closed shell) vs solids
- Internal cavities, wall thickness, duplicate/coincident surfaces
"""

import bpy
import json
import sys

OUTPUT = "stage2v1_basemesh_probe.json"


def analyze(name):
    obj = bpy.data.objects[name]
    me = obj.data
    n_verts = len(me.vertices)
    n_polys = len(me.polygons)
    n_tris = sum(1 for p in me.polygons if len(p.vertices) == 3)
    n_quads = sum(1 for p in me.polygons if len(p.vertices) == 4)
    n_ngon = n_polys - n_tris - n_quads

    # world and local AABB from mesh
    local_coords = [v.co.copy() for v in me.vertices]
    xs = [c.x for c in local_coords]
    ys = [c.y for c in local_coords]
    zs = [c.z for c in local_coords]
    aabb_local = {
        "min": [min(xs), min(ys), min(zs)],
        "max": [max(xs), max(ys), max(zs)],
    }
    dims_local = [aabb_local["max"][i] - aabb_local["min"][i] for i in range(3)]
    box_volume = dims_local[0] * dims_local[1] * dims_local[2]

    # volume by tetra decomposition (same method as stage2v1 probes)
    mw = obj.matrix_world
    vol_w = 0.0
    for poly in me.polygons:
        idx = poly.vertices
        if len(idx) < 3:
            continue
        v0 = mw @ me.vertices[idx[0]].co
        for k in range(1, len(idx) - 1):
            v1 = mw @ me.vertices[idx[k]].co
            v2 = mw @ me.vertices[idx[k + 1]].co
            vol_w += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                      - v0.y * (v1.x * v2.z - v1.z * v2.x)
                      + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0
    vol_l = 0.0
    for poly in me.polygons:
        idx = poly.vertices
        if len(idx) < 3:
            continue
        v0 = me.vertices[idx[0]].co
        for k in range(1, len(idx) - 1):
            v1 = me.vertices[idx[k]].co
            v2 = me.vertices[idx[k + 1]].co
            vol_l += (v0.x * (v1.y * v2.z - v1.z * v2.y)
                      - v0.y * (v1.x * v2.z - v1.z * v2.x)
                      + v0.z * (v1.x * v2.y - v1.y * v2.x)) / 6.0

    # open edges / non-manifold check (closed shell required for volume validity)
    edge_use = {}
    for poly in me.polygons:
        idx = list(poly.vertices)
        for i in range(len(idx)):
            e = tuple(sorted((idx[i], idx[(i + 1) % len(idx)])))
            edge_use[e] = edge_use.get(e, 0) + 1
    boundary_edges = sum(1 for c in edge_use.values() if c == 1)
    nonmanifold_edges = sum(1 for c in edge_use.values() if c > 2)

    # duplicate vertices (coincident positions within 1e-6)
    seen = {}
    dup_pairs = 0
    for c in local_coords:
        key = (round(c.x, 6), round(c.y, 6), round(c.z, 6))
        if key in seen:
            dup_pairs += 1
        else:
            seen[key] = 1

    # coincident/duplicate faces (same sorted vertex set)
    face_keys = {}
    dup_faces = 0
    for poly in me.polygons:
        key = tuple(sorted(poly.vertices))
        if key in face_keys:
            dup_faces += 1
        else:
            face_keys[key] = 1

    # materials
    mats = [m.name if m else None for m in me.materials]

    # thickness estimation: no true cavity detection possible from shell alone;
    # report vertex distribution: count vertices on AABB shells (outer box shell)
    tol = 1e-4
    on_min = sum(1 for c in local_coords if abs(c.x - aabb_local["min"][0]) < tol
                 or abs(c.y - aabb_local["min"][1]) < tol or abs(c.z - aabb_local["min"][2]) < tol)
    on_max = sum(1 for c in local_coords if abs(c.x - aabb_local["max"][0]) < tol
                 or abs(c.y - aabb_local["max"][1]) < tol or abs(c.z - aabb_local["max"][2]) < tol)

    return {
        "object": name,
        "vertices": n_verts,
        "polygons": n_polys,
        "triangles": n_tris,
        "quads": n_quads,
        "ngons": n_ngon,
        "aabb_local": aabb_local,
        "dims_local": dims_local,
        "box_volume_aabb": box_volume,
        "volume_local_tetra": vol_l,
        "volume_world_tetra": vol_w,
        "vol_vs_box_ratio": vol_l / box_volume if box_volume else None,
        "boundary_edges_open": boundary_edges,
        "nonmanifold_edges": nonmanifold_edges,
        "duplicate_vertices": dup_pairs,
        "duplicate_faces": dup_faces,
        "materials": mats,
        "verts_on_aabb_min": on_min,
        "verts_on_aabb_max": on_max,
        "interpretation": None,  # filled below
    }


def main():
    res = analyze("base_link")
    # comparison values from model_key_facts: base dims [0.67, 0.67, 0.176]
    print(f"base_link mesh: verts={res['vertices']} polys={res['polygons']} "
          f"(tri={res['triangles']} quad={res['quads']})")
    print(f"AABB dims (local): {res['dims_local']}")
    print(f"box_volume_aabb = {res['box_volume_aabb']:.9f} m^3")
    print(f"volume_local(tetra) = {res['volume_local_tetra']:.9f} m^3 "
          f"(ratio to box: {res['vol_vs_box_ratio']:.6f})")
    print(f"volume_world(tetra) = {res['volume_world_tetra']:.9f} m^3")
    print(f"open boundary edges: {res['boundary_edges_open']}, "
          f"non-manifold: {res['nonmanifold_edges']}")
    print(f"dup vertices: {res['duplicate_vertices']}, dup faces: {res['duplicate_faces']}")
    print(f"materials: {res['materials']}")
    print(f"verts on AABB min faces: {res['verts_on_aabb_min']}, "
          f"max: {res['verts_on_aabb_max']}")

    with open(OUTPUT, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    print(f"[stage2v1] wrote {OUTPUT}")


if __name__ == "__main__":
    main()
    sys.exit(0)
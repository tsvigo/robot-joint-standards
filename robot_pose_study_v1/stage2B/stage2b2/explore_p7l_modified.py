#!/usr/bin/env python3
"""Explore P7 L c=0.10 with modified S3_manip (+2cm Z)"""

import sys, json, math, zlib
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "stage2A"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "stage2A_ik"))

import bed_study as bed
import fk_model
import ik_study as ik
import motion_study as ms
import stage2b2_control_p7r as ctl

ms.MODE = "6d"
JOINTS = ms.JOINT_KEYS

OPERATION = "P7"
SIDE = "L"
CLEARANCE = 0.10
HEIGHT = 0.7
WIDTH = 1.0
FINGER_YAW = 80.0

TASK_TOL = 1e-6
TASK_ITER_MAX = 60
STEP_CAP_DEG = 6.0
BIAS_CAP_DEG = 0.35
BIAS_STEPS = 1
WAYPOINT_BIAS_STEPS = 3
POSE_POS_TOL_MM = 0.01
POSE_ORI_TOL_DEG = 1e-3

def geometry():
    return bed.bed_geometry("lateral", HEIGHT, WIDTH, CLEARANCE)

def finger_vector():
    yaw = math.radians(FINGER_YAW)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)

def references(geom):
    sequence = ms.operation_sequence(OPERATION, SIDE, geom)
    rotation = bed.orientation_from_normal_finger(SIDE, bed.DOWN, finger_vector())
    for item in sequence:
        item["target"]["rotation"] = rotation.tolist()
        item["target"]["finger"] = [float(v) for v in finger_vector()]
    out = []
    for item in sequence:
        if item["label"] == "S3_manip":
            new_pos = np.array(item["position_floor"]) + np.array([0, 0, 0.02])
            target = item["target"].copy()
            target["position_floor"] = new_pos.tolist()
        else:
            target = item["target"]
        seed = zlib.crc32(f"{OPERATION}|{SIDE}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
        result = ms.static_solve(SIDE, target, int(seed))
        chosen = ms.pick_reference(result, SIDE, geom)
        out.append({
            "label": item["label"],
            "target": target,
            "position_floor": item["position_floor"],
            "q_deg": chosen["q_deg"],
            "solutions": [s["q_deg"] for s in result.get("solutions", []) if s.get("converged")],
        })
    # Force S5_ready = S0_ready for loop closure
    out[5]["q_deg"] = {k: v for k, v in out[0]["q_deg"].items()}
    out[5]["solutions"] = out[0]["solutions"]
    out[5]["target"] = out[0]["target"]
    return out

def q_array(q_deg): return np.array([q_deg[key] for key in JOINTS], dtype=float)
def q_dict(values): return {key: float(value) for key, value in zip(ms.JOINT_KEYS, values)}
def euclidean(a, b): return float(math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in JOINTS)))

def track_sample(side, target, q_seed, tol=1e-6):
    q = ik.clip_q(side, q_array(q_seed))
    damping = 1e-3
    qd = q_dict(q)
    best = ik.task_metrics(side, qd, ms.LIFT_M, target, ms.MODE)
    iterations = 0
    for iteration in range(60):
        iterations = iteration + 1
        if float(best["task_error_norm"]) <= 1e-6:
            break
        jacobian, error, _ = ctl.task_jacobian(side, qd, target)
        _, pinv = ctl.nullspace(jacobian, damping)
        step = pinv @ error
        norm = float(np.linalg.norm(step))
        if norm > math.radians(6.0):
            step *= math.radians(6.0) / norm
        candidate = ik.clip_q(side, q + np.degrees(step))
        candidate_metrics = ik.task_metrics(side, q_dict(candidate), ms.LIFT_M, target, ms.MODE)
        if float(candidate_metrics["task_error_norm"]) < float(best["task_error_norm"]):
            q = candidate; qd = q_dict(q); best = candidate_metrics; damping = max(damping * 0.7, 1e-6); continue
        damping = min(damping * 2.0, 1e2)
        half = ik.clip_q(side, q + np.degrees(0.5 * step))
        half_metrics = ik.task_metrics(side, q_dict(half), ms.LIFT_M, target, ms.MODE)
        if float(half_metrics["task_error_norm"]) < float(best["task_error_norm"]):
            q = half; qd = q_dict(q); best = half_metrics; damping = max(damping * 0.7, 1e-6); continue
        if damping >= 1e2: break
    return qd, best, iterations

def null_bias(side, target, q_deg, anchor_deg, cap_deg=0.35, steps=1):
    current = q_array(q_deg); anchor = q_array(anchor_deg); injected = 0.0
    for _ in range(steps):
        qd = q_dict(current)
        jacobian, _, task = ctl.task_jacobian(side, qd, target)
        pos_mm = float(np.linalg.norm(task["position_error"]) * 1000.0)
        if pos_mm > 0.01 or float(task["orientation_error_deg"]) > 1e-3: break
        projector, _ = ctl.nullspace(jacobian)
        direction = projector @ (anchor - current)
        largest = float(np.max(np.abs(direction)))
        if largest < 1e-9: break
        if largest > cap_deg: direction *= cap_deg / largest
        candidate = ik.clip_q(side, current + direction)
        corrected, _ = ctl.project_to_pose(side, candidate, target, iterations=4)
        check = ik.task_metrics(side, q_dict(corrected), ms.LIFT_M, target, ms.MODE)
        if float(np.linalg.norm(check["position_error"]) * 1000.0) > 0.01: break
        if float(check["orientation_error_deg"]) > 1e-3: break
        moved = float(np.max(np.abs(corrected - current)))
        if moved < 1e-9: break
        current = corrected; injected = max(injected, moved)
    return q_dict(current), injected

def anchor_for(refs, segment, fraction):
    return ctl.lerp_q(refs[segment]["q_deg"], refs[segment + 1]["q_deg"], fraction)

def plan_cycle(side, refs, geom):
    samples = []
    q_current = {k: float(v) for k, v in refs[0]["q_deg"].items()}
    start = ctl.metrics(side, q_current, refs[0]["target"], geom, ms.P0_Q)
    start.update({"t": 0.0, "segment": 0, "segment_t": 0.0, "injected_deg": 0.0, "iterations": 0,
                  "waypoint": refs[0]["label"], "fallback": False})
    samples.append(start)
    for segment, (first, second) in enumerate(zip(refs, refs[1:])):
        p0 = np.asarray(first["position_floor"], dtype=float)
        p1 = np.asarray(second["position_floor"], dtype=float)
        distance = float(np.linalg.norm(p1 - p0))
        count = max(4, min(ms.MAX_SAMPLES, int(math.ceil(distance / ms.CART_STEP_M))))
        for index in range(1, count + 1):
            fraction = index / count
            target = ms.interp_target(first["target"], second["target"], fraction)
            anchor = ctl.lerp_q(refs[segment]["q_deg"], refs[segment + 1]["q_deg"], fraction)
            q_track, task, iterations = track_sample(side, target, anchor)
            fallback = False
            pos_mm = float(np.linalg.norm(task["position_error"]) * 1000.0)
            ori_deg = float(task["orientation_error_deg"])
            if pos_mm > 0.01 or ori_deg > 1e-2:
                alt, _ = ctl.project_to_pose(side, q_array(anchor), target, iterations=8)
                alt_metrics = ik.task_metrics(side, q_dict(alt), ms.LIFT_M, target, ms.MODE)
                alt_pos = float(np.linalg.norm(alt_metrics["position_error"]) * 1000.0)
                if alt_pos < pos_mm or float(alt_metrics["orientation_error_deg"]) < ori_deg:
                    q_track = q_dict(alt); fallback = True
            step_from_previous = max(abs(float(q_track[key]) - float(q_current[key])) for key in JOINTS)
            is_waypoint = (index == count)
            if not is_waypoint and step_from_previous > 5.0:
                q_alt, _, iterations_alt = track_sample(side, target, q_current)
                alt_metrics = ik.task_metrics(side, q_alt, ms.LIFT_M, target, ms.MODE)
                alt_pos = float(np.linalg.norm(alt_metrics["position_error"]) * 1000.0)
                alt_ori = float(alt_metrics["orientation_error_deg"])
                alt_step = max(abs(float(q_alt[key]) - float(q_current[key])) for key in JOINTS)
                if alt_pos <= 0.01 and alt_ori <= 1e-2 and alt_step < step_from_previous:
                    q_track = q_alt; iterations = iterations_alt; fallback = True
            injected = 0.0
            steps = 3 if index == count else 1
            q_new, injected = null_bias(side, target, q_track, anchor, cap_deg=0.35, steps=steps)
            row = ctl.metrics(side, q_new, target, geom, ms.P0_Q)
            row.update({"t": 0.0, "segment": segment, "segment_t": float(fraction),
                       "injected_deg": float(injected), "iterations": int(iterations),
                       "waypoint": second["label"] if is_waypoint else None, "fallback": fallback})
            samples.append(row); q_current = q_new
    for index, row in enumerate(samples):
        row["t"] = index / max(1, len(samples) - 1)
    return samples

def euclidean(a, b): return float(math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in JOINTS)))

def path_metrics(side, samples):
    length = sum(euclidean(a["q_deg"], b["q_deg"]) for a, b in zip(samples, samples[1:]))
    per_joint = {k: sum(abs(float(b["q_deg"][k]) - float(a["q_deg"][k])) for a, b in zip(samples, samples[1:])) for k in JOINTS}
    worst_margin = min(samples, key=lambda r: r["min_margin_deg"])
    worst_cond = max(samples, key=lambda r: r["condition"])
    worst_pos = max(samples, key=lambda r: r["position_error_mm"])
    worst_ori = max(samples, key=lambda r: r["orientation_error_deg"])
    worst_clear = min(samples, key=lambda r: r["clearance_min_m"])
    return {"joint_path_length_deg": sum(euclidean(a["q_deg"], b["q_deg"]) for a, b in zip(samples, samples[1:])),
            "joint_path_length_by_joint_deg": per_joint,
            "min_margin_deg": float(worst_margin["min_margin_deg"]), "min_margin_joint": worst_margin["min_margin_joint"],
            "min_margin_at": float(worst_margin["t"]), "max_condition": float(worst_cond["condition"]),
            "max_position_deviation_mm": float(worst_pos["position_error_mm"]),
            "max_orientation_deviation_deg": float(worst_ori["orientation_error_deg"]),
            "min_clearance_m": float(worst_clear["clearance_min_m"]), "min_clearance_at": float(worst_clear["t"]),
            "end_margin_deg": float(samples[-1]["min_margin_deg"]), "end_margin_joint": samples[-1]["min_margin_joint"],
            "sample_count": len(samples)}

def continuity_summary(side, samples):
    max_step = 0.0; max_euclid = 0.0; joints = {k: 0.0 for k in JOINTS}
    max_link = 0.0; max_palm = 0.0; max_pos_err = 0.0; max_ori_err = 0.0; inj_total = 0.0
    for a, b in zip(samples, samples[1:]):
        diff = {k: float(b["q_deg"][k]) - float(a["q_deg"][k]) for k in JOINTS}
        for k, v in diff.items(): joints[k] = max(joints[k], abs(v))
        step = max(abs(v) for v in diff.values())
        max_step = max(max_step, step)
        max_euclid = max(max_euclid, euclidean(a["q_deg"], b["q_deg"]))
        links_a = ctl.link_points(side, a["q_deg"]); links_b = ctl.link_points(side, b["q_deg"])
        max_link = max(max_link, float(np.max(np.linalg.norm(links_b - links_a, axis=1))))
        palm_a = fk_model.palm_geometry(side, a["q_deg"], ms.LIFT_M)["face_floor"]
        palm_b = fk_model.palm_geometry(side, b["q_deg"], ms.LIFT_M)["face_floor"]
        max_palm = max(max_palm, float(np.linalg.norm(np.array(palm_b) - np.array(palm_a))))
        max_pos_err = max(max_pos_err, float(b["position_error_mm"]))
        max_ori_err = max(max_ori_err, float(b["orientation_error_deg"]))
        inj_total += float(b.get("injected_deg") or 0.0)
    return {"interval_count": len(samples)-1, "max_joint_step_deg": max_step, "max_euclidean_dq_deg": max_euclid,
            "max_joint_by_joint_deg": joints, "max_link_displacement_m": max_link, "max_palm_displacement_m": max_palm,
            "max_position_error_mm": max_pos_err, "max_orientation_error_deg": max_ori_err,
            "collision_free": all(bool(s["collision_free"]) for s in samples),
            "within_limits": all(bool(s["within_limits"]) for s in samples), "injected_total_deg": inj_total}

def certify_path(side, samples, geom):
    intervals = []
    for index, (first, second) in enumerate(zip(samples, samples[1:])):
        step = max(abs(float(second["q_deg"][key]) - float(first["q_deg"][key])) for key in JOINTS)
        node = ctl.certify_interval(side, first["q_deg"], second["q_deg"], geom, 0)
        leaves = []; ctl.flatten_certification(node, leaves)
        clearance = min(leaf["clearance_m"] for leaf in leaves)
        bound = max(leaf["bound_m"] for leaf in leaves)
        intervals.append({"interval": index, "max_joint_step_deg": step, "leaf_count": len(leaves),
                          "clearance_m": float(clearance), "bound_m": float(bound),
                          "margin_m": float(clearance - bound), "status": "certified" if all(l["status"]=="certified" for l in leaves) else "undefined"})
    return {"interval_count": len(intervals), "certified": sum(1 for i in intervals if i["status"]=="certified"),
            "undefined": sum(1 for i in intervals if i["status"]!="certified"),
            "min_clearance_m": min(i["clearance_m"] for i in intervals)}

def geometry():
    return bed.bed_geometry("lateral", 0.7, 1.0, 0.10)

def finger_vector():
    yaw = math.radians(80)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)

def main():
    geom = geometry()
    refs = []
    sequence = ms.operation_sequence("P7", "L", geometry())
    rotation = bed.orientation_from_normal_finger("L", bed.DOWN, np.array([math.sin(math.radians(80)), math.cos(math.radians(80)), 0.0]))
    for item in sequence:
        item["target"]["rotation"] = rotation.tolist()
        item["target"]["finger"] = [float(v) for v in np.array([math.sin(math.radians(80)), math.cos(math.radians(80)), 0.0])]
    for item in sequence:
        if item["label"] == "S3_manip":
            new_pos = np.array(item["position_floor"]) + np.array([0, 0, 0.02])
            target = item["target"].copy()
            target["position_floor"] = new_pos.tolist()
        else:
            target = item["target"]
        seed = zlib.crc32(f"P7|L|{item['label']}|0.10".encode("utf-8"))
        result = ms.static_solve("L", target, int(zlib.crc32(f"P7|L|{item['label']}|0.10".encode())))
        chosen = ms.pick_reference(result, "L", geometry())
        refs.append({"label": item["label"], "target": target, "position_floor": item["position_floor"],
                     "q_deg": chosen["q_deg"], "solutions": [s["q_deg"] for s in result.get("solutions", []) if s.get("converged")]})
    refs[5]["q_deg"] = {k: v for k, v in refs[0]["q_deg"].items()}
    refs[5]["solutions"] = refs[0]["solutions"]
    refs[5]["target"] = refs[0]["target"]

    samples = plan_cycle("L", refs, geometry())
    metrics = path_metrics("L", samples)
    cont = continuity_summary("L", samples)
    cert = certify_path("L", samples, geometry())
    closure = {"max_abs_deg": max(abs(float(samples[-1]["q_deg"][k]) - float(refs[0]["q_deg"][k])) for k in JOINTS),
               "euclidean_deg": math.sqrt(sum((float(samples[-1]["q_deg"][k]) - float(refs[0]["q_deg"][k]))**2 for k in JOINTS))}

    print("=== P7 L c=0.10 MODIFIED (S3_manip +2cm Z) ===")
    print(f"Path length: {metrics['joint_path_length_deg']:.1f}°")
    print(f"Min margin: {metrics['min_margin_deg']:.3f}° ({metrics['min_margin_joint']}) at t={metrics['min_margin_at']:.3f}")
    print(f"Max condition: {metrics['max_condition']:.3f}")
    print(f"Max pos dev: {metrics['max_position_deviation_mm']:.4f} mm")
    print(f"Max ori dev: {metrics['max_orientation_deviation_deg']:.6f}°")
    print(f"Min clearance: {metrics['min_clearance_m']:.6f} m")
    print(f"Closure: {max(abs(float(samples[-1]['q_deg'][k]) - float(refs[0]['q_deg'][k])) for k in JOINTS):.6f}°")
    print(f"Max step: {continuity_summary('L', samples)['max_joint_step_deg']:.2f}°")
    print(f"Certified: {certify_path('L', plan_cycle('L', refs, geometry()), geometry())['certified']}/{certify_path('L', plan_cycle('L', refs, geometry()), geometry())['interval_count']}")
    print("Waypoints:")
    for s in samples:
        if s.get("waypoint"):
            print(f"  {s['label']:10s}: margin={s['min_margin_deg']:.3f} ({s['min_margin_joint']})")

if __name__ == "__main__":
    main()
EOF
python3 /home/v/projects/робот/robot_pose_study_v1/stage2B/stage2b2/explore_p7l_modified.py
"""Generate Blender ground truth for the stage 2A control poses.

Run from Blender, read-only with respect to the .blend:

    blender -b robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend \
        --python robot_pose_study_v1/stage2A/blender_control_pose_gt.py

The script resets the 14 arm joints and the lift, applies each pose, and
writes palm world matrices to ``control_pose_gt.json`` next to this script.
It never calls ``save_mainfile``.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
OUT_PATH = HERE / "control_pose_gt.json"

JOINTS = {
    "L_sh3": "left_shoulder_joint_empty_3",
    "L_sh2": "left_shoulder_joint_empty_2",
    "L_sh1": "left_shoulder_joint_empty_1",
    "L_el": "left_elbow_joint_empty",
    "L_rad": "left_proximal_radioulnar_joint_empty",
    "L_w1": "left_wrist_joint_1_empty",
    "L_w2": "left_wrist_joint_2_empty",
    "R_sh3": "right_shoulder_joint_empty_3",
    "R_sh2": "right_shoulder_joint_empty_2",
    "R_sh1": "right_shoulder_joint_empty_1",
    "R_el": "right_elbow_joint_empty",
    "R_rad": "right_proximal_radioulnar_joint_empty",
    "R_w1": "right_wrist_joint_1_empty",
    "R_w2": "right_wrist_joint_2_empty",
}
AXIS_INDEX = {"sh3": 1, "sh2": 2, "sh1": 0, "el": 2, "rad": 1, "w1": 2, "w2": 0}

# Joint angles in degrees.  Lift is separate, in millimetres.
POSES = {
    "P0": {"L": {}, "R": {}},
    "palmdown0": {"L": {"sh3": -45, "rad": -45}, "R": {"sh3": 45, "rad": 45}},
    "forward": {"L": {"sh2": -90}, "R": {"sh2": 90}},
    "backward": {"L": {"sh2": 90}, "R": {"sh2": -90}},
    "forward_bent": {"L": {"sh2": -75, "el": 45}, "R": {"sh2": 75, "el": 45}},
    "el90": {"L": {"el": 90}, "R": {"el": 90}},
    "lowered_sym": {"L": {"sh1": -21, "w2": -25}, "R": {"sh1": 21, "w2": 25}},
    "p4pos_old": {"L": {"sh1": -21, "w2": -25}, "R": {"sh1": -21, "w2": -25}},
    "p6pos_old": {
        "L": {"sh1": -21, "el": 20, "w1": 90, "w2": -25},
        "R": {"sh1": -21, "el": 20, "w1": 90, "w2": -25},
    },
    "folded": {"L": {"sh2": -75, "el": 150}, "R": {"sh2": 75, "el": 150}},
    "lift100": {"L": {}, "R": {}},
    "liftm50": {"L": {}, "R": {}},
}
LIFT_MM = {"lift100": 100.0, "liftm50": -50.0}


def reset_pose() -> None:
    for name in JOINTS.values():
        obj = bpy.data.objects[name]
        obj.rotation_mode = "XYZ"
        obj.rotation_euler = (0.0, 0.0, 0.0)
    bpy.data.objects["vertical_lift_joint"].location = (0.0, 0.0, 0.0)
    bpy.context.view_layer.update()


def main() -> None:
    result = {}
    for pose_name, sides in POSES.items():
        reset_pose()
        lift_mm = LIFT_MM.get(pose_name, 0.0)
        if lift_mm:
            bpy.data.objects["vertical_lift_joint"].location.z = lift_mm / 1000.0
        for side, values in sides.items():
            for key, value in values.items():
                empty = JOINTS[f"{side}_{key}"]
                euler = [0.0, 0.0, 0.0]
                euler[AXIS_INDEX[key]] = math.radians(float(value))
                bpy.data.objects[empty].rotation_euler = euler
        bpy.context.view_layer.update()
        result[pose_name] = {
            side: [
                list(row)
                for row in bpy.data.objects[
                    ("left" if side == "L" else "right") + "_palm_link"
                ].matrix_world
            ]
            for side in ("L", "R")
        }
    reset_pose()

    source = Path(bpy.data.filepath)
    digest = hashlib.sha256(source.read_bytes()).hexdigest() if source.exists() else None
    payload = {
        "source_file": str(source),
        "source_sha256": digest,
        "blender_version": bpy.app.version_string,
        "pose_definitions": POSES,
        "lift_mm": LIFT_MM,
        "poses": result,
    }
    OUT_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"WROTE {OUT_PATH}")
    print(f"POSES {len(result)}")


if __name__ == "__main__":
    main()

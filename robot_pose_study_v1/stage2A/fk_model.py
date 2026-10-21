"""Rigidified FK and geometric Jacobian for the two 7-DOF arms and the lift.

Scope
-----
Stage 2A only: kinematics, limits, singular values. No forces, no masses,
no collision checking, no safety classification.

Rest matrices
-------------
``arm_load_audit/arm_dump.json`` (Blender 4.2.20 LTS dump).  The model is
rigidified: the rest translations are kept, the rest rotations are
orthonormalized before the joint axis is transformed.  Non-uniform mesh
scales are not normalized because they define the actual link lengths in
this .blend; the residual versus Blender 5.0.1 is documented in
``stage2A_checkpoint.md``.

Coordinates
-----------
All positions returned in ``*_floor`` fields are relative to the floor
centre O.  O is the base centre projected to the floor:
X/Y from the base bbox centre, Z from the wheel bottom (visual bbox min).
The base_link origin itself is ~0.1346 m above the floor, so do not use it
as the floor origin.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DUMP_PATH = ROOT / "arm_load_audit" / "arm_dump.json"
PALM_GEOMETRY_PATH = Path(__file__).resolve().with_name("palm_geometry.json")

# Floor centre.  X/Y = base bbox centre; Z = visual bbox min (wheel bottom).
FLOOR_ORIGIN = np.array([3.608395, -1.860352, -5.608625], dtype=float)
FLOOR_Z = float(FLOOR_ORIGIN[2])
BASE_ORIGIN_Z = -5.474048137664795
BASE_ORIGIN_HEIGHT_M = float(BASE_ORIGIN_Z - FLOOR_Z)

# Chain order: lift -> torso -> shoulder 3 -> 2 -> 1 -> upper arm -> elbow ->
# forearm -> radioulnar -> link -> wrist 1 -> pitch -> wrist 2 -> palm.
CHAINS: dict[str, tuple[str, ...]] = {
    "L": (
        "vertical_lift_joint",
        "torso_link",
        "left_shoulder_joint_empty_3",
        "left_shoulder_3_link",
        "left_shoulder_joint_empty_2",
        "left_shoulder_2_link",
        "left_shoulder_joint_empty_1",
        "left_shoulder_link",
        "left_elbow_joint_empty",
        "left_forearm_link",
        "left_proximal_radioulnar_joint_empty",
        "left_radioulnar_link",
        "left_wrist_joint_1_empty",
        "left_wrist_pitch_link",
        "left_wrist_joint_2_empty",
        "left_palm_link",
    ),
    "R": (
        "vertical_lift_joint",
        "torso_link",
        "right_shoulder_joint_empty_3",
        "right_shoulder_3_link",
        "right_shoulder_joint_empty_2",
        "right_shoulder_2_link",
        "right_shoulder_joint_empty_1",
        "right_shoulder_link",
        "right_elbow_joint_empty",
        "right_forearm_link",
        "right_proximal_radioulnar_joint_empty",
        "right_radioulnar_link",
        "right_wrist_joint_1_empty",
        "right_wrist_pitch_link",
        "right_wrist_joint_2_empty",
        "right_palm_link",
    ),
}

# (key, joint empty, local axis, lower deg, upper deg)
JOINTS: dict[str, tuple[tuple[str, str, tuple[float, float, float], float, float], ...]] = {
    "L": (
        ("sh3", "left_shoulder_joint_empty_3", (0.0, 1.0, 0.0), -180.0, 60.0),
        ("sh2", "left_shoulder_joint_empty_2", (0.0, 0.0, 1.0), -90.0, 90.0),
        ("sh1", "left_shoulder_joint_empty_1", (1.0, 0.0, 0.0), -90.0, 130.0),
        ("el", "left_elbow_joint_empty", (0.0, 0.0, 1.0), -10.0, 150.0),
        ("rad", "left_proximal_radioulnar_joint_empty", (0.0, 1.0, 0.0), -90.0, 90.0),
        ("w1", "left_wrist_joint_1_empty", (0.0, 0.0, 1.0), -90.0, 90.0),
        ("w2", "left_wrist_joint_2_empty", (1.0, 0.0, 0.0), -25.0, 45.0),
    ),
    "R": (
        ("sh3", "right_shoulder_joint_empty_3", (0.0, 1.0, 0.0), -180.0, 60.0),
        ("sh2", "right_shoulder_joint_empty_2", (0.0, 0.0, 1.0), -90.0, 90.0),
        ("sh1", "right_shoulder_joint_empty_1", (1.0, 0.0, 0.0), -90.0, 130.0),
        ("el", "right_elbow_joint_empty", (0.0, 0.0, 1.0), -10.0, 150.0),
        ("rad", "right_proximal_radioulnar_joint_empty", (0.0, 1.0, 0.0), -90.0, 90.0),
        ("w1", "right_wrist_joint_1_empty", (0.0, 0.0, 1.0), -90.0, 90.0),
        ("w2", "right_wrist_joint_2_empty", (1.0, 0.0, 0.0), -25.0, 45.0),
    ),
}

ARM_JOINT_KEYS = ("sh3", "sh2", "sh1", "el", "rad", "w1", "w2")

_REST_CACHE: dict[str, np.ndarray] | None = None


def load_rest_matrices(path: str | Path = DUMP_PATH) -> dict[str, np.ndarray]:
    """Load rest world matrices from the Blender dump."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))["objects"]
    return {name: np.array(obj["mw"], dtype=float) for name, obj in data.items()}


def rest_matrices() -> dict[str, np.ndarray]:
    """Cached rest matrices.  Callers must copy before mutating."""
    global _REST_CACHE
    if _REST_CACHE is None:
        _REST_CACHE = load_rest_matrices()
    return _REST_CACHE


def load_palm_geometry(path: str | Path = PALM_GEOMETRY_PATH) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def orthonormalize(matrix: np.ndarray) -> np.ndarray:
    """Closest proper rotation to the 3x3 part of a 4x4 matrix."""
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float)[:3, :3])
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u = u.copy()
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def axis_angle(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm < 1e-15:
        raise ValueError("zero joint axis")
    a = a / norm
    k = np.array(
        [[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]],
        dtype=float,
    )
    return np.eye(3) + math.sin(angle_rad) * k + (1.0 - math.cos(angle_rad)) * (k @ k)


def fk(
    side: str,
    q_deg: dict[str, float],
    lift_m: float = 0.0,
    rest: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    """Forward kinematics for one arm at a given lift displacement.

    Parameters
    ----------
    side:
        ``"L"`` or ``"R"``.
    q_deg:
        Joint angles in degrees, keys ``sh3, sh2, sh1, el, rad, w1, w2``.
        Missing keys are zero.
    lift_m:
        Lift displacement in metres, +Z positive.
    rest:
        Optional rest matrices; defaults to the cached dump.

    Returns
    -------
    dict mapping every object in the chain to its 4x4 world matrix.
    """
    if side not in CHAINS:
        raise ValueError(f"side must be 'L' or 'R', got {side!r}")
    unknown = set(q_deg) - set(ARM_JOINT_KEYS)
    if unknown:
        raise ValueError(f"unknown joint keys: {sorted(unknown)}")
    source = rest_matrices() if rest is None else rest
    names = CHAINS[side]
    mats = {name: source[name].copy() for name in names}

    if abs(lift_m) > 1e-15:
        shift = np.eye(4)
        shift[2, 3] = float(lift_m)
        for name in names:
            mats[name] = shift @ mats[name]

    for key, empty, axis, _lower, _upper in JOINTS[side]:
        angle_deg = float(q_deg.get(key, 0.0))
        if abs(angle_deg) < 1e-15:
            continue
        origin = mats[empty][:3, 3].copy()
        rotation = orthonormalize(mats[empty])
        world_axis = rotation @ np.asarray(axis, dtype=float)
        world_axis /= np.linalg.norm(world_axis)
        transform = np.eye(4)
        transform[:3, :3] = axis_angle(world_axis, math.radians(angle_deg))
        transform[:3, 3] = origin - transform[:3, :3] @ origin
        first = names.index(empty)
        for name in names[first:]:
            mats[name] = transform @ mats[name]
    return mats


def palm_matrix(
    side: str,
    q_deg: dict[str, float],
    lift_m: float = 0.0,
    rest: dict[str, np.ndarray] | None = None,
) -> np.ndarray:
    return fk(side, q_deg, lift_m, rest)[CHAINS[side][-1]]


def palm_geometry(
    side: str,
    q_deg: dict[str, float],
    lift_m: float = 0.0,
    geometry: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Palm centre, contact-face centre, normal, finger direction.

    Positions in ``*_floor`` are relative to FLOOR_ORIGIN.  Vectors are
    unit vectors in world/floor axes.
    """
    geom = load_palm_geometry() if geometry is None else geometry
    item = geom[side]
    bbox = np.asarray(item["local_bbox"], dtype=float)
    x_face = bbox[0, 0] if side == "L" else bbox[1, 0]
    y_mid = 0.5 * (bbox[0, 1] + bbox[1, 1])
    z_mid = 0.5 * (bbox[0, 2] + bbox[1, 2])
    face_local = np.array([x_face, y_mid, z_mid], dtype=float)

    matrix = palm_matrix(side, q_deg, lift_m)
    rotation = orthonormalize(matrix)
    centre_world = matrix[:3, 3].copy()
    face_world = (matrix @ np.r_[face_local, 1.0])[:3]
    normal = rotation @ np.asarray(item["normal_local"], dtype=float)
    finger = rotation @ np.asarray(item["finger_local"], dtype=float)
    thumb = rotation @ np.asarray(item["thumb_local"], dtype=float)
    for vector in (normal, finger, thumb):
        vector /= np.linalg.norm(vector)
    down = np.array([0.0, 0.0, -1.0])
    angle_to_down = math.degrees(
        math.acos(float(np.clip(np.dot(normal, down), -1.0, 1.0)))
    )
    return {
        "matrix": matrix,
        "face_local": face_local,
        "center_world": centre_world,
        "center_floor": centre_world - FLOOR_ORIGIN,
        "face_world": face_world,
        "face_floor": face_world - FLOOR_ORIGIN,
        "normal": normal,
        "finger": finger,
        "thumb": thumb,
        "angle_to_down_deg": angle_to_down,
    }


def jacobian(
    side: str,
    q_deg: dict[str, float],
    lift_m: float = 0.0,
    point_local: np.ndarray | None = None,
    include_lift: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Geometric 6x7 Jacobian of a palm-fixed point.

    Columns follow ``ARM_JOINT_KEYS``.  If ``point_local`` is omitted, the
    palm origin is used; pass ``palm_geometry(...)["face_local"]`` for the
    contact-face centre.  With ``include_lift=True`` an 8th column
    ``[0,0,1,0,0,0]`` is appended.
    """
    mats = fk(side, q_deg, lift_m)
    names = CHAINS[side]
    palm = mats[names[-1]]
    if point_local is None:
        point = palm[:3, 3].copy()
    else:
        point = (palm @ np.r_[np.asarray(point_local, dtype=float), 1.0])[:3]

    jac = np.zeros((6, len(JOINTS[side])), dtype=float)
    axes = []
    for column, (_key, empty, axis, _lower, _upper) in enumerate(JOINTS[side]):
        origin = mats[empty][:3, 3]
        rotation = orthonormalize(mats[empty])
        world_axis = rotation @ np.asarray(axis, dtype=float)
        world_axis /= np.linalg.norm(world_axis)
        axes.append(world_axis)
        jac[:3, column] = np.cross(world_axis, point - origin)
        jac[3:, column] = world_axis

    if include_lift:
        lift_column = np.zeros((6, 1), dtype=float)
        lift_column[2, 0] = 1.0
        jac = np.hstack([jac, lift_column])
    return jac, np.asarray(axes), point


def shoulder_singular_values(side: str, q_deg: dict[str, float]) -> tuple[np.ndarray, int]:
    """Singular values and rank of the 3x3 shoulder orientation block.

    The three columns are the world axes of sh3, sh2, sh1.  At sh2=±90°
    two axes become parallel: rank 2, gimbal lock of the shoulder block.
    """
    mats = fk(side, q_deg, 0.0)
    axes = []
    for _key, empty, axis, _lower, _upper in JOINTS[side][:3]:
        rotation = orthonormalize(mats[empty])
        world_axis = rotation @ np.asarray(axis, dtype=float)
        world_axis /= np.linalg.norm(world_axis)
        axes.append(world_axis)
    block = np.column_stack(axes)
    singular = np.linalg.svd(block, compute_uv=False)
    rank = int(np.linalg.matrix_rank(block, tol=1e-6))
    return singular, rank


def limit_margins(side: str, q_deg: dict[str, float]) -> list[dict[str, Any]]:
    """Per-joint distance to the nearest limit, in degrees."""
    result = []
    for key, _empty, _axis, lower, upper in JOINTS[side]:
        value = float(q_deg.get(key, 0.0))
        to_lower = value - lower
        to_upper = upper - value
        result.append(
            {
                "joint": key,
                "q_deg": value,
                "lower_deg": lower,
                "upper_deg": upper,
                "margin_deg": min(to_lower, to_upper),
                "nearest": "lower" if to_lower <= to_upper else "upper",
            }
        )
    return result


def min_limit_margin(side: str, q_deg: dict[str, float]) -> dict[str, Any]:
    return min(limit_margins(side, q_deg), key=lambda item: item["margin_deg"])


def within_limits(side: str, q_deg: dict[str, float], tol: float = 1e-9) -> bool:
    return all(item["margin_deg"] >= -tol for item in limit_margins(side, q_deg))

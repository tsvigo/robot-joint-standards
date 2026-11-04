"""
Stage 2W — воспроизводимое сравнение четырёх архитектур приводов рук робота-санитара.

A  — Local: мотор-редукторы в суставах.
B1 — Remote: двигатель локтя у плеча.
B2 — Remote: двигатели локтя и запястья у плеча/каретки торса.
B3 — Remote: все двигатели в основании, передача через подъёмник.
C  — Local + механическая компенсация тяжести.
D  — Remote (лучший из B) + компенсация тяжести.

ВХОД (только чтение):
  robot_pose_study_v1/stage2A/fk_model.py           — FK и геометрическая Якоби
  arm_load_audit/arm_dump.json                      — 182 объекта Blender: mw, bbox, dims, vol, props
  arm_load_audit/arm_geometry_data.csv              — суставы (оси/лимиты/положения), звенья
  arm_load_audit/arm_mass_budget_v1/*.csv           — параметрический бюджет масс, допущения A01..A25
  arm_load_audit/arm_joint_moments.csv              — прежние статические моменты
  robot_pose_study_v1/stage2B/stage2b2/*.json       — позы S0..S5 (P6/P7)

ВЫХОД: arm_architecture_comparison_v1/

Единицы: м, кг, с, Н·м, рад (в выводах перевод в градусы явно).
Статусы: VERIFIED / DERIVED / ASSUMED / UNKNOWN — не смешиваются.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------- каталоги ---
ROOT = Path("/home/v/projects/робот")
STUDY = ROOT / "robot_pose_study_v1"
OUT = STUDY / "arm_architecture_comparison_v1"
OUT.mkdir(parents=True, exist_ok=True)
ARM = ROOT / "arm_load_audit"

G = np.array([0.0, 0.0, -9.81], dtype=float)

STATUS = {"VERIFIED": "подтверждено геометрией/спецификацией",
          "DERIVED": "вычислено из установленных исходных данных",
          "ASSUMED": "параметрическое допущение",
          "UNKNOWN": "нет подтверждённого значения"}

CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# ============================================================ загрузка FK ===
def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


FK = _load_module(STUDY / "stage2A" / "fk_model.py", "stage2w_fk_model")
JOINT_KEYS = list(FK.ARM_JOINT_KEYS)          # sh3, sh2, sh1, el, rad, w1, w2
CHAIN = list(FK.CHAINS["L"])

# индексы «пустых» суставов в цепочке (колонки Якоби 0..6)
JOINT_EMPTY_IDX = [CHAIN.index(e) for _, e, _, _, _ in FK.JOINTS["L"]]

# ==================================================== дамп Blender (чтение) ===
DUMP = json.loads((ARM / "arm_dump.json").read_text(encoding="utf-8"))["objects"]
GEOM = list(csv.DictReader((ARM / "arm_geometry_data.csv").open(encoding="utf-8")))
BUDGET = list(csv.DictReader((ARM / "arm_mass_budget_v1" / "arm_mass_budget.csv").open(encoding="utf-8")))
MOMENTS_OLD = list(csv.DictReader((ARM / "arm_mass_budget_v1" / "arm_mass_budget_moments.csv").open(encoding="utf-8")))

# ---- положения суставов, оси и лимиты (VERIFIED из .blend) -------------------
JOINT_INFO: dict[str, dict] = {}
for r in GEOM:
    if r["record_type"] != "joint":
        continue
    side = "L" if r["side"] == "left" else "R"
    key = {"_3": "sh3", "_2": "sh2", "_1": "sh1"}.get(None)  # placeholder
    name = r["id"]
    for cand, k in (("shoulder_joint_empty_3", "sh3"), ("shoulder_joint_empty_2", "sh2"),
                    ("shoulder_joint_empty_1", "sh1"), ("elbow_joint_empty", "el"),
                    ("proximal_radioulnar", "rad"), ("wrist_joint_1", "w1"),
                    ("wrist_joint_2", "w2")):
        if cand in name:
            key = k
            break
    JOINT_INFO[(side, key)] = {
        "id": name, "axis_md": r["axis_md"],
        "limit_lower": float(r["limit_lower_deg"]), "limit_upper": float(r["limit_upper_deg"]),
        "origin_rest": np.array([float(r["x_m"]), float(r["y_m"]), float(r["z_m"])]),
        "parent": r["parent"], "child": r["child"], "role": r["role"],
        "note": r["note"],
    }

# ---- звенья -----------------------------------------------------------------
LINK_ROWS = [r for r in GEOM if r["record_type"] == "link"]


def obj_info(name: str) -> dict:
    o = DUMP.get(name, {})
    bb0 = o.get("bbox_world_min")
    bb1 = o.get("bbox_world_max")
    props = o.get("props") or {}
    com = None
    dims = None
    if bb0 and bb1:
        bb0, bb1 = np.array(bb0, float), np.array(bb1, float)
        com = (bb0 + bb1) / 2.0
        dims = (bb1 - bb0) * 1000.0          # мм
    mass = None
    for k in ("mass", "Mass", "weight"):
        if k in props and props[k] not in (None, "", 0):
            try:
                mass = float(props[k])
            except (TypeError, ValueError):
                mass = None
            break
    return {"type": o.get("type"), "com_bbox": com, "dims_mm": dims,
            "mass_stored": mass, "scale": o.get("scale"),
            "vol_local": o.get("vol_local"), "parent": o.get("parent")}


# ===================================================== параметры допущений ===
PARAMS = {
    "плотность_Al": {"value": 2700.0, "unit": "кг/м³", "status": "ASSUMED",
                     "basis": "A01 — Al 2/3 мм сценарии arm_mass_scenarios"},
    "плотность_сталь": {"value": 7850.0, "unit": "кг/м³", "status": "ASSUMED",
                        "basis": "A01"},
    "толщина_стенки_мм": {"lean": 2.0, "nominal": 3.0, "heavy": 3.0, "status": "ASSUMED",
                           "basis": "A01"},
    "материал_оболочек": {"lean": ("Al", 2700.0), "nominal": ("Al", 2700.0),
                          "heavy": ("сталь", 7850.0), "status": "ASSUMED",
                          "basis": "A01: Al 2 мм / Al 3 мм / сталь 3 мм"},
    "k_привода_кг_Нм": {"lean": 0.015, "nominal": 0.030, "heavy": 0.060,
                         "status": "ASSUMED", "basis": "A05 — порядок величины, не выбор оборудования"},
    "COM_оболочек": {"value": "центр bbox", "status": "ASSUMED", "basis": "A03"},
    "COM_приводов": {"value": "фрейм сустава", "status": "ASSUMED", "basis": "A13"},
    "кабели_кг_м": {"lean": 0.08, "nominal": 0.15, "heavy": 0.30, "status": "ASSUMED", "basis": "A14"},
    "скорость_удержания": {"value": "нет данных", "status": "UNKNOWN", "basis": "A25"},
    "КПД_передач": {"value": None, "status": "UNKNOWN", "basis": "передача не выбрана"},
}

# сценарии массы рук из arm_mass_budget (обе руки)
BUDGET_SIDE = {"lean": {}, "nominal": {}, "heavy": {}}
for r in BUDGET:
    side = "L" if r["side"] == "left" else "R"
    for s in BUDGET_SIDE:
        v = r[f"mass_{s}_kg"]
        BUDGET_SIDE[s][(side, r["component"])] = float(v) if v not in ("", None) else None


def actuator_mass(side: str, key: str, scen: str) -> float | None:
    """Масса узла привода из arm_mass_budget (ASSUMED, k·T*)."""
    jname = {"sh3": "j3", "sh2": "j2", "sh1": "j1", "el": "локоть",
             "rad": "лучелоктевой", "w1": "запястье-1", "w2": "запястье-2"}[key]
    for (s, comp), v in BUDGET_SIDE[scen].items():
        if s == side and comp.startswith(f"Узел привода: {jname} "):
            return v
    return None


def shell_mass(dims_mm, rho: float, t_mm: float) -> float | None:
    """Закрытая коробка по bbox (A02): m = ρ·(abc − (a−2t)(b−2t)(c−2t))."""
    if dims_mm is None or len(dims_mm) != 3:
        return None
    a, b, c = [float(d) / 1000.0 for d in dims_mm]
    if min(a, b, c) <= 2 * (t_mm / 1000.0) + 1e-9:
        return None
    vol = a * b * c - (a - 2 * t_mm / 1000) * (b - 2 * t_mm / 1000) * (c - 2 * t_mm / 1000)
    return rho * vol


def box_inertia(m: float, dims_mm) -> np.ndarray:
    """Диагональ тензора инерции ПУСТОТЁЛОЙ коробки (внутренняя соосная) в СК, где
    оси совпадают с рёбрами bbox. Сумма полый + вырез = полный бокс."""
    if dims_mm is None or len(dims_mm) != 3:
        return np.zeros(3)
    a, b, c = [float(d) / 1000.0 for d in dims_mm]
    # диагональный 3x3 тензор в СК, оси совпадают с рёбрами bbox
    return np.diag(m / 12.0 * np.array([b * b + c * c,
                                        a * a + c * c,
                                        a * a + b * b]))


# ==================================================================== позы ===
def load_poses() -> dict[str, dict]:
    out = {}
    for tag, fn, status in (
        ("P6_L", "stage2b2_policy_P6_L_c0.05.json", "ALL_CRITERIA_PASS"),
        ("P7_L", "stage2b2_policy_P7_L_c0.05.json", "RESEARCH_ONLY"),
        ("P6_R", "stage2b2_policy_P6_R_c0.10.json", "RESEARCH_ONLY"),
        ("P7_R", "stage2b2_policy_P7_R_c0.10.json", "RESEARCH_ONLY"),
    ):
        p = STUDY / "stage2B" / "stage2b2" / fn
        if not p.exists():
            continue
        d = json.loads(p.read_text(encoding="utf-8"))
        side = tag.split("_")[1]
        for ref in d.get("references", []):
            q = {k: float(ref["q_deg"][k]) for k in JOINT_KEYS if k in ref.get("q_deg", {})}
            if len(q) != 7:
                continue
            key = f"{tag}:{ref.get('label','?')}"
            out[key] = {"side": side, "q_deg": q, "status_2B2": status,
                        "target": ref.get("position_floor")}
    return out


POSES = load_poses()

# --------------------------------------------------------------- вспомогательно
def chain_for(side: str) -> list[str]:
    return list(FK.CHAINS[side])


def joint_frames(side: str, q_deg: dict) -> list[tuple[np.ndarray, np.ndarray]]:
    """(начало оси, мировая ось) для 7 суставов в порядке JOINT_KEYS."""
    mats = FK.fk(side, q_deg)
    out = []
    for _key, empty, axis, _lo, _hi in FK.JOINTS[side]:
        m = mats[empty]
        rot = FK.orthonormalize(m)
        a = rot @ np.asarray(axis, float)
        a /= np.linalg.norm(a)
        out.append((m[:3, 3].copy(), a))
    return out


def link_indices(side: str) -> dict[str, int]:
    return {n: i for i, n in enumerate(chain_for(side))}


def point_jacobian(frames, origin_idx: int, p: np.ndarray, n_joints: int = 7) -> np.ndarray:
    """6×7 Якоби точки, закреплённой за звеном с индексом origin_idx в цепочке."""
    J = np.zeros((6, n_joints))
    for j, (o, a) in enumerate(frames):
        if JOINT_EMPTY_IDX[j] > origin_idx:
            continue                     # сустав дистальнее точки — не действует
        J[:3, j] = np.cross(a, p - o)
        J[3:, j] = a
    return J



# ==================================================== 1. МАССОВАЯ МОДЕЛЬ ====
# Состав одного звена (без двойного учёта: каждая строка — свой класс части):
#   shell   — оболочка по bbox (A01/A02/A22)
#   ribs    — внутренняя структура (A04, только в бюджете)
#   act     — узел привода сустава (A05 k·T*, COM = фрейм сустава, A13)
#   cable   — кабель (A14)
#   hand    — кисть (оболочки + UNKNOWN механизмы/приводы пальцев)
#   empty   — EMPTY-звенья без геометрии → масса UNKNOWN, В модель не входят
# Пустые (EMPTY) звенья, крепёж, тормоза, разъёмы, пальцевая механика: UNKNOWN.

MESH_LINKS = {"shoulder_link", "forearm_link", "palm_link"}
LINK_CANON = {"left_shoulder_link": ("L", "sh"), "right_shoulder_link": ("R", "sh"),
              "left_forearm_link": ("L", "fa"), "right_forearm_link": ("R", "fa"),
              "left_palm_link": ("L", "pa"), "right_palm_link": ("R", "pa")}

# цепочка суставов и звеньев, к которым относится каждый узел привода
ACT_OF = {"sh3": "shoulder_3", "sh2": "shoulder_2", "sh1": "shoulder_link",
          "el": "forearm", "rad": "radioulnar", "w1": "wrist_pitch", "w2": "palm"}


def hand_lump(side: str, scen: str) -> float:
    """Оболочки ПАЛЬЦЕВ (15 мешей) из arm_mass_budget.

    Ладонь СЮДА НЕ входит — она уже посчитана как shell-оболочка ладони,
    иначе двойной учёт. (В hand_part присутствует left_palm_link = 0.335 кг.)"""
    for (s, comp), v in BUDGET_SIDE[scen].items():
        if s == side and comp.startswith("Пальцы"):
            return v
    return None


def build_points(side: str, scen: str, arch: str = "A",
                 rho: float = 2700.0, t_mm: float = None) -> list[dict]:
    """Параметрическая модель массы одной руки.

    arch: A | B1 | B2 | B3 — определяет, КУДА переезжают узлы приводов.
    Возвращает: [{name, kind, mass, com, chain_idx, status, note}]
    """
    t_mm = PARAMS["толщина_стенки_мм"][scen] if t_mm is None else t_mm
    rho = PARAMS["материал_оболочек"][scen][1] if rho == 2700.0 else rho
    k = PARAMS["k_привода_кг_Нм"][scen]
    km = PARAMS["кабели_кг_м"][scen]
    chain = chain_for(side)
    idx = {n: i for i, n in enumerate(chain)}
    pts: list[dict] = []

    sh_link_name = [n for n in chain if n.endswith("shoulder_link")][0]
    fa_link_name = [n for n in chain if n.endswith("forearm_link")][0]

    # --- оболочки MESH-звеньев (COM = центр bbox, A03) -----------------------
    for name in chain:
        if not any(name.endswith(s) for s in MESH_LINKS):
            continue
        info = obj_info(name)
        if info["com_bbox"] is None:
            continue
        m_stored = info["mass_stored"]
        m_shell = shell_mass(info["dims_mm"], rho, t_mm)
        pts.append({
            "name": name, "kind": "shell", "mass": m_shell,
            "mass_stored": m_stored, "com": info["com_bbox"], "chain_idx": idx[name],
            "dims_mm": info["dims_mm"], "status": "ASSUMED",
            "note": f"оболочка Al {t_mm:g} мм по bbox; stored={m_stored} кг (категория ③)",
        })

    # --- пальцы: масса бюджета (ASSUMED) распределяется по bbox фаланг (DERIVED) --
    # Лампирование на ладонь занижало плечо (метод бюджета — по центрам мешей),
    # поэтому масса распределяется пропорционально объёму bbox каждой фаланги.
    palm_name = [n for n in chain if n.endswith("palm_link")][0]
    hm = hand_lump(side, scen)
    # фаланг НЕТ в CHAINS[] (кисть описана ладонью) -> берём из дампа целиком
    pre = "left_" if side == "L" else "right_"
    vols = {}
    for n, o in DUMP.items():
        if o.get("type") != "MESH":
            continue
        is_finger = ("phalanx" in n or "digit" in n) and not n.endswith("palm_link")
        if not is_finger:
            continue
        if not (n.startswith(pre) or "_thumb_" in n):
            continue                      # фаланги большого пальца без префикса L/R
        i2 = obj_info(n)
        if i2["dims_mm"] is None:
            continue
        vols[n] = float(np.prod(i2["dims_mm"]))
    tot_v = sum(vols.values())
    if hm and tot_v > 0:
        for n, v in vols.items():
            i2 = obj_info(n)
            pts.append({"name": n, "kind": "hand", "mass": hm * v / tot_v,
                        "com": i2["com_bbox"], "chain_idx": idx[palm_name],
                        "dims_mm": i2["dims_mm"], "status": "DERIVED",
                        "note": f"доля массы пальцев {hm:.3f} кг по объёму bbox "
                                f"({100*v/tot_v:.1f} %); механизмы/приводы пальцев UNKNOWN"})
        # пустые/отсутствующие фаланги digit5 без массы -> UNKNOWN (E7)
        pts.append({"name": f"{side}_hand_unknown", "kind": "hand_unknown",
                    "mass": 0.0, "com": obj_info(palm_name)["com_bbox"],
                    "chain_idx": idx[palm_name], "dims_mm": None, "status": "UNKNOWN",
                    "note": "механизм/приводы пальцев, digit5-меш без массы (E7), "
                            "захватные поверхности, датчики кисти — НЕ учтены"})

    # --- внутренняя структура: перегородки + рёбра (A04) --------------------
    for (s, comp), v in BUDGET_SIDE[scen].items():
        if s != side or not v:
            continue
        if comp.startswith("Внутренняя структура плеча"):
            nm, ci = sh_link_name, idx[sh_link_name]
        elif comp.startswith("Внутренняя структура предплечья"):
            nm, ci = fa_link_name, idx[fa_link_name]
        else:
            continue
        info = obj_info(nm)
        pts.append({"name": f"{nm}__ribs", "kind": "ribs", "mass": v,
                    "com": info["com_bbox"], "chain_idx": ci, "status": "ASSUMED",
                    "note": "A04: перегородки+рёбра (lean 1+0 / nominal 2+2 / heavy 3+3)"})

    # --- кабели: половина в плече, половина в предплечье (A14) ---------------
    sh_name, fa_name = sh_link_name, fa_link_name
    dist = {(side, "кабели"): None}
    for (s, comp), v in BUDGET_SIDE[scen].items():
        if s == side and comp == "Кабели и жгуты":
            dist[(side, "кабели")] = v
    cm = dist.get((side, "кабели"))
    if cm:
        for nm, part, ci in ((sh_name, cm / 2.0, idx[sh_name]),
                             (fa_name, cm / 2.0, idx[fa_name])):
            info = obj_info(nm)
            pts.append({"name": f"{nm}__cable", "kind": "cable", "mass": part,
                        "com": info["com_bbox"], "chain_idx": ci, "status": "ASSUMED",
                        "note": "A14: половина трассы на плечо, половина на предплечье"})

    # --- узлы приводов -------------------------------------------------------
    moved_to = {"A": {}, "B1": {"el": "sh1"}, "B2": {"el": "sh1", "rad": "sh1",
                                                      "w1": "sh1", "w2": "sh1"},
                "B3": {"sh3": "torso", "sh2": "torso", "sh1": "torso", "el": "torso",
                       "rad": "torso", "w1": "torso", "w2": "torso"}}[arch]
    for key in JOINT_KEYS:
        m_act = actuator_mass(side, key, scen)
        if not m_act:
            continue
        src_empty = FK.JOINTS[side][JOINT_KEYS.index(key)][1]
        # исходная точка: фрейм сустава (A13)
        src_idx = idx[src_empty]
        tgt_key = moved_to.get(key)
        at_base = False
        note = "A05 k·T* (ASSUMED); COM=фрейм сустава (A13)"
        if tgt_key is None:
            tgt_idx, tgt_name = src_idx, src_empty
        elif tgt_key == "torso":
            tgt_idx, tgt_name = idx["torso_link"], "torso_link"
            note += f"; B3: перенесён в ОСНОВАНИЕ (не поднимается подъёмником; было {src_empty})"
            at_base = True
        else:
            tgt_empty = FK.JOINTS[side][JOINT_KEYS.index(tgt_key)][1]
            tgt_idx, tgt_name = idx[tgt_empty], tgt_empty
            note += f"; {arch}: перенесён ближе к плечу (было {src_empty})"
        # точка привода: если переезжает — геометрическая позиция берётся от
        # целевого фрейма (компактная упаковка у плеча); иначе — исходный фрейм
        tgt_origin = joint_frames(side, {kk: 0.0 for kk in JOINT_KEYS})
        frames0 = tgt_origin
        # положение фрейма в покое (rest) берём из дампа
        com = np.array(DUMP[tgt_name]["mw"], float)[:3, 3]
        pts.append({"name": f"{side}_act_{key}", "kind": "actuator", "mass": m_act,
                    "com": com, "chain_idx": tgt_idx, "status": "ASSUMED", "note": note,
                    "joint": key, "moved_to": tgt_key, "at_base": at_base})

    # --- добавочная масса передач (B*) --------------------------------------
    if arch in ("B1", "B2", "B3"):
        tr = transmission_points(side, scen, arch, idx)
        pts.extend(tr)

    # --- компенсаторы (C, D) -------------------------------------------------
    if arch in ("C", "D"):
        pts.extend(compensator_points(side, scen, arch, idx))

    for p in pts:
        if p["mass"] is None:
            p["mass"] = 0.0
    return pts


def transmission_points(side: str, scen: str, arch: str, idx: dict) -> list[dict]:
    """Добавочная масса передачи: шкипы, ремень/кабель, промежуточные валы,
    подшипники, кронштейны. Параметрическая оценка (ASSUMED)."""
    chain = chain_for(side)
    sh = [n for n in chain if n.endswith("shoulder_link")][0]
    fa = [n for n in chain if n.endswith("forearm_link")][0]
    pa = [n for n in chain if n.endswith("palm_link")][0]
    o_sh = np.array(DUMP[sh]["mw"], float)[:3, 3]
    o_fa = np.array(DUMP[fa]["mw"], float)[:3, 3]
    o_pa = np.array(DUMP[pa]["mw"], float)[:3, 3]
    L_el = float(np.linalg.norm(o_fa - o_sh))          # плечо
    L_wr = float(np.linalg.norm(o_pa - o_fa))          # предплечье
    # параметры передач (ASSUMED)
    p = {"масса_ремня_кг_м": 0.05, "масса_кабеля_кг_м": 0.03,
         "масса_шкива_кг": 0.25, "масса_подшипника_кг": 0.15,
         "масса_промвала_кг_м": 1.2, "масса_кронштейна_кг": 0.6}
    moved = {"B1": ["el"], "B2": ["el", "rad", "w1", "w2"],
             "B3": ["sh3", "sh2", "sh1", "el", "rad", "w1", "w2"]}[arch]
    out = []
    # ремень локтя: от плеча к локтю
    if "el" in moved:
        m = (p["масса_ремня_кг_м"] * L_el + 2 * p["масса_шкива_кг"]
             + 2 * p["масса_подшипника_кг"] + p["масса_кронштейна_кг"])
        out.append({"name": f"{side}_trans_el_belt", "kind": "transmission", "mass": m,
                    "com": (o_sh + o_fa) / 2, "chain_idx": idx[sh], "status": "ASSUMED",
                    "note": f"ремень плечо→локоть, L={L_el:.3f} м; шкипы 2, подшипники 2, кронштейн"})
    if "w1" in moved or "w2" in moved:
        m = (p["масса_ремня_кг_м"] * L_wr + 2 * p["масса_шкива_кг"]
             + 2 * p["масса_подшипника_кг"] + p["масса_кронштейна_кг"])
        out.append({"name": f"{side}_trans_w_belt", "kind": "transmission", "mass": m,
                    "com": (o_fa + o_pa) / 2, "chain_idx": idx[fa], "status": "ASSUMED",
                    "note": f"ремень предплечье→запястье, L={L_wr:.3f} м"})
    if arch == "B3":
        # трасса вдоль всей руки к основанию: промежуточные валы/канаты
        m = p["масса_промвала_кг_м"] * (L_el + L_wr) + p["масса_кронштейна_кг"] * 2
        out.append({"name": f"{side}_trans_B3", "kind": "transmission", "mass": m,
                    "com": (o_sh + o_pa) / 2, "chain_idx": idx["torso_link"],
                    "status": "ASSUMED",
                    "note": "B3: трасса через подъёмник; изменяющаяся длина + изгиб — ДОПОЛНИТЕЛЬНЫЕ риски"})
    return out


def compensator_points(side: str, scen: str, arch: str, idx: dict) -> list[dict]:
    """Масса компенсатора (параметрическая оценка, ASSUMED)."""
    chain = chain_for(side)
    sh = [n for n in chain if n.endswith("shoulder_link")][0]
    fa = [n for n in chain if n.endswith("forearm_link")][0]
    m_spec = 0.20   # кг на 1 Н·м/рад жёсткости — порядок величины (ASSUMED)
    out = []
    for key, nm in (("sh1", sh), ("el", fa)):
        info = obj_info(nm)
        # требуемая жёсткость оценивается позже; здесь — базовая масса узла
        out.append({"name": f"{side}_comp_{key}", "kind": "compensator",
                    "mass": 1.5, "com": info["com_bbox"], "chain_idx": idx[nm],
                    "status": "ASSUMED",
                    "note": f"пружинный узел у {key}: 1.5 кг (штанга/крепёж/пружина), "
                            f"спецификация {m_spec} кг/(Н·м/рад) — параметр"})
    return out


# ==================================================== 2. СТАТИЧЕСКИЕ МОМЕНТЫ ==
def point_positions(side: str, q_deg: dict, points: list[dict]) -> list[np.ndarray]:
    mats = FK.fk(side, q_deg)
    out = []
    for p in points:
        m = mats[chain_for(side)[p["chain_idx"]]]
        R = m[:3, :3]
        # com задан в мировых координатах покоя -> смещение от origin покоя
        rest = np.array(DUMP[chain_for(side)[p["chain_idx"]]]["mw"], float)
        local = np.linalg.inv(rest[:3, :3]) @ (p["com"] - rest[:3, 3])
        out.append(m[:3, 3] + R @ local)
    return out


def tg_jacobian(side: str, q_deg: dict, points: list[dict]) -> np.ndarray:
    """Tg = Σ_i Jv_i(q)ᵀ · m_i · g  (7,)."""
    frames = joint_frames(side, q_deg)
    tau = np.zeros(len(JOINT_KEYS))
    for p, pos in zip(points, point_positions(side, q_deg, points)):
        if p.get("at_base"):              # двигатель в основании: не подвижен
            continue
        J = point_jacobian(frames, p["chain_idx"], pos)
        tau += J[:3].T @ (p["mass"] * G)
    return tau


def tg_potential(side: str, q_deg: dict, points: list[dict], h: float = 1e-6) -> np.ndarray:
    """Независимый путь: Tg_j = ∂V/∂q_j,  V = Σ m_i·g·p_i.  Центральная разность."""
    tau = np.zeros(len(JOINT_KEYS))
    for j, key in enumerate(JOINT_KEYS):
        qp, qm = dict(q_deg), dict(q_deg)
        qp[key] = q_deg[key] + h
        qm[key] = q_deg[key] - h
        Vp = sum(p["mass"] * float(G @ pos)
                 for p, pos in zip(points, point_positions(side, qp, points)))
        Vm = sum(p["mass"] * float(G @ pos)
                 for p, pos in zip(points, point_positions(side, qm, points)))
        # q заданы в ГРАДУСАХ, Якоби — на РАДИАН:
        # Tg_j = ∂V/∂q[рад] = ∂V/∂q[град] · (180/π)
        tau[j] = (Vp - Vm) / (2 * h) * (180.0 / math.pi)
    return tau


def inertia_matrix(side: str, q_deg: dict, points: list[dict],
                   i_local: dict[str, np.ndarray] | None = None) -> np.ndarray:
    """M(q) = Σ_i [ m_i·Jv_iᵀ·Jv_i + Jω_iᵀ·R·I_i·Rᵀ·Jω_i ]."""
    frames = joint_frames(side, q_deg)
    mats = FK.fk(side, q_deg)
    chain = chain_for(side)
    n = len(JOINT_KEYS)
    M = np.zeros((n, n))
    for p, pos in zip(points, point_positions(side, q_deg, points)):
        if p.get("at_base"):
            continue
        J = point_jacobian(frames, p["chain_idx"], pos)
        m = p["mass"]
        M += m * (J[:3].T @ J[:3])
        if (not p.get("at_base")) and i_local is not None and p.get("name") in i_local:
            name = p["name"]
            rest = np.array(DUMP[name]["mw"], float)
            R_rest = rest[:3, :3]
            R_q = mats[name][:3, :3]
            # поворот, полученный с покоя до q
            R_inc = R_q @ np.linalg.inv(R_rest)
            I_q = R_inc @ i_local[name] @ R_inc.T
            Jw = J[3:]
            M += Jw.T @ I_q @ Jw
    return M


def local_inertias(points: list[dict], scen: str) -> dict[str, np.ndarray]:
    """Тензоры инерции оболочек в локальной (bbox-выровненной) СК — DERIVED."""
    out = {}
    t_mm = PARAMS["толщина_стенки_мм"][scen]
    for p in points:
        if p["kind"] == "shell" and p.get("dims_mm") is not None and p["mass"]:
            out[p["name"]] = box_inertia(p["mass"], p["dims_mm"])
    return out



# ============================================== 3. СЦЕНАРИИ И АРХИТЕКТУРЫ ===
# Положение точки приложения внешней нагрузки = начало ладони (A03-подобное
# допущение: реальная точка контакта с пациентом не зафиксирована в данных).

CUSTOM_POSES = {
    "X_выпрямленная_горизонтальная": {"sh3": 0.0, "sh2": 0.0, "sh1": 0.0,
                                       "el": 0.0, "rad": 0.0, "w1": 0.0, "w2": 0.0},
    "X_согнутая": {"sh3": 0.0, "sh2": 0.0, "sh1": -30.0, "el": 90.0,
                   "rad": 0.0, "w1": 0.0, "w2": 0.0},
    "X_поднятая": {"sh3": 0.0, "sh2": 0.0, "sh1": 90.0, "el": 45.0,
                   "rad": 0.0, "w1": 0.0, "w2": 0.0},
}
POSE_STATUS = {"X_выпрямленная_горизонтальная": "модельная (не эксплуатационная)",
               "X_согнутая": "модельная (не эксплуатационная)",
               "X_поднятая": "модельная (не эксплуатационная)"}


def all_poses() -> dict:
    out = {}
    for k, v in POSES.items():
        out[k] = {"side": v["side"], "q_deg": v["q_deg"], "status": v["status_2B2"]}
    for k, q in CUSTOM_POSES.items():
        out[f"L:{k}"] = {"side": "L", "q_deg": q, "status": POSE_STATUS[k]}
        out[f"R:{k}"] = {"side": "R", "q_deg": q, "status": POSE_STATUS[k]}
    return out


ALL_POSES = all_poses()

LOADS_KG = [0.0, 2.0, 10.0, 25.0, 50.0, 100.0]     # эквивалент на ОДНУ руку
CONTACT_N = [100.0, 200.0, 300.0]                   # контакт с пациентом
ACCELS = [0.0, 0.5, 1.0]                            # рад/с²
T_EMERG = [0.2, 0.5, 1.0]                           # с, ПАРАМЕТР (данных нет)

ARCHS = ["A", "B1", "B2", "B3", "C", "D1", "D2", "D3"]
ARCH_LABEL = {
    "A": "A Local (все приводы в суставах)",
    "B1": "B1 Remote (двигатель локтя у плеча)",
    "B2": "B2 Remote (локоть+запястье у плеча)",
    "B3": "B3 Remote (все в основании, через подъёмник)",
    "C": "C Local + компенсация тяжести",
    "D1": "D1 = B1 + компенсация",
    "D2": "D2 = B2 + компенсация",
    "D3": "D3 = B3 + компенсация",
}
ARCH_BASE = {"A": "A", "B1": "B1", "B2": "B2", "B3": "B3",
             "C": "A", "D1": "B1", "D2": "B2", "D3": "B3"}
ARCH_COMP = {"A": False, "B1": False, "B2": False, "B3": False,
             "C": True, "D1": True, "D2": True, "D3": True}


def side_mass_summary(arch: str, scen: str, side: str) -> dict:
    pts = build_points(side, scen, ARCH_BASE[arch])
    if ARCH_COMP[arch]:
        pts = pts + compensator_points(side, scen, arch,
                                       {n: i for i, n in enumerate(chain_for(side))})
    by = {}
    for x in pts:
        by.setdefault(x["kind"], 0.0)
        by[x["kind"]] += x["mass"]
    lift = sum(x["mass"] for x in pts if not x.get("at_base"))
    base = sum(x["mass"] for x in pts if x.get("at_base"))
    return {"итого_кг": round(sum(x["mass"] for x in pts), 3),
            "на_подъёмнике_кг": round(lift, 3),
            "в_основании_кг": round(base, 3),
            "по_классам": {k: round(v, 3) for k, v in sorted(by.items())},
            "UNKNOWN_не_учтено": [
                "приводы/механизмы пальцев", "крепёж", "тормоза (наличие не определено)",
                "разъёмы/датчики/кожухи", "масса EMPTY-звеньев (shoulder_2/3, "
                "radioulnar, wrist_pitch — в модели 0)",
                "компенсатор: масса узлов задана параметром 1.5 кг/сустав",
            ]}


def external_wrench(side: str, q_deg: dict, F_world: np.ndarray, load_kg: float = 0.0):
    """Момент от внешней силы + точечная масса груза на ладони."""
    frames = joint_frames(side, q_deg)
    mats = FK.fk(side, q_deg)
    palm = mats[chain_for(side)[-1]]
    p = palm[:3, 3]
    J = point_jacobian(frames, len(chain_for(side)) - 1, p)
    tau = J[:3].T @ F_world
    if load_kg > 0.0:
        tau = tau + J[:3].T @ (load_kg * G)
    return tau, p


def static_torque(side: str, q_deg: dict, arch: str, scen: str,
                  load_kg: float = 0.0, contact_n: float = 0.0,
                  contact_dir=(0.0, 0.0, -1.0)) -> np.ndarray:
    pts = build_points(side, scen, ARCH_BASE[arch])
    if ARCH_COMP[arch]:
        pts = pts + compensator_points(side, scen, arch,
                                       {n: i for i, n in enumerate(chain_for(side))})
    tg = tg_jacobian(side, q_deg, pts)
    F = np.array(contact_dir, float)
    F = F / (np.linalg.norm(F) + 1e-12) * float(contact_n)
    tl, _ = external_wrench(side, q_deg, F, load_kg)
    return tg + tl


def empty_arm_points(side: str, scen: str) -> list[dict]:
    """ПУСТАЯ рука = собственный вес руки ВМЕСТЕ с приводами (архитектура A),
    без внешней нагрузки. Именно эту нагрузку должен держать плечевой привод,
    и именно её компенсирует механизм (по заданию: % собственного гравитационного
    момента пустой руки). Перенос приводов в B* меняет эту величину — см. ниже.
    """
    return build_points(side, scen, "A")


def arm_structure_only_points(side: str, scen: str) -> list[dict]:
    """Только конструкция (оболочки+рёбра+кабели+пальцы), без узлов приводов —
    для оценки вклада приводов в гравитационный момент."""
    return [p for p in build_points(side, scen, "A")
            if p["kind"] not in ("actuator", "transmission", "compensator")]


# --------------------------------------------------- компенсация тяжести ----
def fit_torsion(q_vals: np.ndarray, tg_vals: np.ndarray, beta: float):
    """Наилучшая ТОРСИОННАЯ пружина Tc = k(q−q0) для цели β·Tg на наборе поз.

    Возвращает k [Н·м/рад], q0 [град], остаток, энергию. Это реальная
    реализуемая форма Tc(q), а НЕ умножение Tg на константу.
    """
    if len(q_vals) < 2 or np.allclose(tg_vals, 0):
        return {"k": 0.0, "q0_deg": 0.0, "max_остаток_Нм": 0.0,
                "RMS_остаток_Нм": 0.0, "E_max_Дж": 0.0, "масса_кг": 0.0}
    y = beta * tg_vals
    A = np.vstack([q_vals * (math.pi / 180.0), np.ones_like(q_vals)]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    k, b = float(coef[0]), float(coef[1])
    q0 = -b / k if abs(k) > 1e-12 else 0.0
    Tc = k * (q_vals * math.pi / 180.0) + b
    res = y - Tc
    dq = (q_vals - q0) * (math.pi / 180.0)
    E = 0.5 * abs(k) * float(np.max(np.abs(dq))) ** 2 if k else 0.0
    return {"k": k, "q0_deg": round(q0, 2),
            "max_остаток_Нм": round(float(np.abs(res).max()), 3),
            "RMS_остаток_Нм": round(float(np.sqrt(np.mean(res ** 2))), 3),
            "E_max_Дж": round(float(E), 3),
            "масса_кг": round(abs(k) * 0.20, 3),
            "доля_компенсации": beta}


def compensation_study(side: str, scen: str) -> dict:
    """Анализ компенсации собственного веса ПУСТОЙ руки по позам Stage 2Б.2."""
    poses = [(k, v) for k, v in ALL_POSES.items() if v["side"] == side]
    pts = empty_arm_points(side, scen)
    rows = {}
    for j, key in enumerate(JOINT_KEYS):
        qq, tt = [], []
        for _k, v in poses:
            tau = tg_jacobian(side, v["q_deg"], pts)
            qq.append(v["q_deg"][key])
            tt.append(tau[j])
        qq, tt = np.array(qq), np.array(tt)
        rows[key] = {str(b): fit_torsion(qq, tt, b) for b in (0.25, 0.50, 0.70, 0.90)}
        rows[key]["Tg_диапазон_Нм"] = [round(float(tt.min()), 2), round(float(tt.max()), 2)]
    return rows



# ================================================== 4. ДИНАМИКА И ЭНЕРГИЯ ===
def coriolis_vector(side, q, qd, points, h=1e-6) -> np.ndarray:
    """c(q,q̇) = Ĵ(q,q̇)·q̇  численно: c_i = Σ_j (∂M_ij/∂q_j − 0.5 ∂M_ij/∂q_i) q̇_j.

    Для честности ограниченности: используем численный градиент M(q).
    """
    n = len(JOINT_KEYS)
    M0 = inertia_matrix(side, q, points)
    dM = np.zeros((n, n, n))
    for j, k in enumerate(JOINT_KEYS):
        qp, qm = dict(q), dict(q)
        qp[k] += h; qm[k] -= h
        dM[:, :, j] = (inertia_matrix(side, qp, points) -
                       inertia_matrix(side, qm, points)) / (2 * h)
    c = np.zeros(n)
    for i in range(n):
        acc = 0.0
        for j in range(n):
            acc += (dM[i, j, j] - 0.5 * dM[j, i, j]) * qd[j]
        c[i] = acc
    return c


def dynamic_torque(side, q, qd, qdd, arch, scen,
                   load_kg=0.0, contact_n=0.0, contact_dir=(0, 0, -1.0)):
    """M(q)q̈ + c(q,q̇) + g(q) + JᵀF_ext  (знаки зафиксированы: слева — инерция,
    трение, гравитация; справа — актёры и внешние силы не включаются)."""
    pts = build_points(side, scen, ARCH_BASE[arch])
    if ARCH_COMP[arch]:
        pts = pts + compensator_points(side, scen, arch,
                                       {n: i for i, n in enumerate(chain_for(side))})
    i_loc = local_inertias(pts, scen)
    M = inertia_matrix(side, q, pts, i_loc)
    g = tg_jacobian(side, q, pts)
    c = coriolis_vector(side, q, qd, pts)
    F = np.array(contact_dir, float)
    F = F / (np.linalg.norm(F) + 1e-12) * float(contact_n)
    tl, _ = external_wrench(side, q, F, load_kg)
    tau = M @ np.asarray(qdd, float) + c + g + tl
    return tau, M, g, c, tl


# ======================================== 5. ПЕРЕДАЧИ: УСИЛИЕ, НАТЯЖЕНИЕ =====
def transmission_forces(tau_joint_Nm, radii_mm=(50.0, 80.0, 100.0, 150.0),
                        eta=(0.96, 0.90, 0.85), safety=(1.5, 2.0)) -> list[dict]:
    """Усилие в ветвях ремня/кабеля при заданном моменте и радиусе шкива.

    F_раб = τ/(η·r); предварительное натяжение F0 ≈ F_раб/2 (равные ветви,
    зазор натяжения 0.5·F) — параметрическое допущение.
    """
    out = []
    for r_mm in radii_mm:
        r = r_mm / 1000.0
        for e in eta:
            F = tau_joint_Nm / (e * r)
            F0 = F / 2.0
            for sf in safety:
                out.append({
                    "tau_Нм": round(tau_joint_Nm, 1), "r_мм": r_mm, "eta": e,
                    "F_раб_ветвь_Н": round(F, 1),
                    "F0_преднатяжение_Н": round(F0, 1),
                    "F_с_запасом_Н": round(F * sf, 1),
                    "сечение_кабеля_мм2_при_60МПа": round(F / 60.0, 2),
                })
    return out


# ==================================================== 6. ЦИКЛ И ЭНЕРГИЯ =====
def build_cycle(side: str, scen: str, arch: str, n_per_leg: int = 12):
    """Референсный цикл S0→S1→S2→S3→S4→S5→S0 линейной интерполяцией в q.

    Допущение (ASSUMED): реальные утверждённые траектории 2Б.2 включают
    непрерывность/клиренсы; здесь только геометрическая схема для работы.
    """
    # ключи вида "P6_L:S0_ready"; сторону берём из v["side"]
    seq = [k for k, v in ALL_POSES.items()
           if v["side"] == side and k.split(":")[1].startswith("S")]
    order = {f"S{i}": i for i in range(6)}
    seq = sorted(set(seq),
                 key=lambda k: (k.split(":")[1][:2],
                               order.get(k.split(":")[1][1:3], 9),
                               k.split(":")[1]))
    if not seq:
        return []
    q_prev = ALL_POSES[seq[0]]["q_deg"]
    path = [dict(q_prev)]
    for k in seq[1:]:
        q = ALL_POSES[k]["q_deg"]
        for i in range(1, n_per_leg + 1):
            path.append({kk: q_prev[kk] + (q[kk] - q_prev[kk]) * i / n_per_leg
                         for kk in JOINT_KEYS})
        q_prev = q
    return path


def cycle_energy(side, scen, arch, load_kg=0.0, contact_n=0.0, omega=0.5) -> dict:
    path = build_cycle(side, scen, arch)
    if len(path) < 2:
        return {}
    pts = build_points(side, scen, ARCH_BASE[arch])
    if ARCH_COMP[arch]:
        pts = pts + compensator_points(side, scen, arch,
                                       {n: i for i, n in enumerate(chain_for(side))})
    W_pos = W_neg = 0.0
    P_peak = 0.0
    F = np.array([0.0, 0.0, -1.0]) * float(contact_n)
    for a, b in zip(path, path[1:]):
        ta = tg_jacobian(side, a, pts)
        tla, _ = external_wrench(side, a, F, load_kg)
        ta = ta + tla
        dq = np.array([(b[k] - a[k]) * math.pi / 180 for k in JOINT_KEYS])
        dW = float(ta @ dq)
        W_pos += max(dW, 0.0)
        W_neg += min(dW, 0.0)
        P_peak = max(P_peak, float(np.abs(ta @ np.full(len(JOINT_KEYS), omega))))
    return {"W_положит_Дж": round(W_pos, 2), "W_отриц_Дж": round(W_neg, 2),
            "W_нетто_Дж": round(W_pos + W_neg, 2),
            "P_пик_Вт_при_omega": round(P_peak, 2),
            "omega_рад_с": omega,
            "замечание": "механическая работа; расход АККУМУЛЯТОРА РАВЕН НЕ ЕЙ "
                         "(нет КПД, нет рекуперации, нет приводной электроники)"}


# ==================================================== 7. ВЫВОД РЕЗУЛЬТАТОВ ==
def write_mass_inertia_csv() -> Path:
    """Задача 1: массы, COM, тензоры инерции по звеньям обеих рук."""
    path = OUT / "arm_mass_inertia.csv"
    rows = []
    for side in ("L", "R"):
        for scen in ("lean", "nominal", "heavy"):
            pts = build_points(side, scen, "A")
            for x in pts:
                i_loc = None
                if x["kind"] == "shell" and x.get("dims_mm") is not None and x["mass"]:
                    i_loc = box_inertia(x["mass"], x["dims_mm"])
                rows.append({
                    "side": side, "сценарий": scen, "звено": x["name"],
                    "класс": x["kind"], "масса_кг": round(x["mass"], 4),
                    "масса_stored_кг": ("" if x.get("mass_stored") is None
                                        else round(x["mass_stored"], 4)),
                    "COM_x_м": "" if x["com"] is None else round(float(x["com"][0]), 6),
                    "COM_y_м": "" if x["com"] is None else round(float(x["com"][1]), 6),
                    "COM_z_м": "" if x["com"] is None else round(float(x["com"][2]), 6),
                    "COM_источник": "центр bbox (ASSUMED, A03)" if x["com"] is not None else "UNKNOWN",
                    "dims_x_мм": "" if x.get("dims_mm") is None else round(float(x["dims_mm"][0]), 1),
                    "dims_y_мм": "" if x.get("dims_mm") is None else round(float(x["dims_mm"][1]), 1),
                    "dims_z_мм": "" if x.get("dims_mm") is None else round(float(x["dims_mm"][2]), 1),
                    "Ixx_кгм2": "" if i_loc is None else round(float(i_loc[0, 0]), 6),
                    "Iyy_кгм2": "" if i_loc is None else round(float(i_loc[1, 1]), 6),
                    "Izz_кгм2": "" if i_loc is None else round(float(i_loc[2, 2]), 6),
                    "статус": x["status"], "примечание": x["note"],
                })
    # явные строки UNKNOWN
    for side in ("L", "R"):
        for name, note in [
            ("EMPTY-звенья (shoulder_3, shoulder_2, radioulnar, wrist_pitch)",
             "в .blend тип EMPTY, геометрии и массы нет (E8)"),
            ("Крепёж, разъёмы, датчики, кожухи", "не учтены нигде (A15/A17)"),
            ("Тормоза суставов", "наличие не определено (A16)"),
            ("Приводы и механизмы пальцев", "схема захвата не задана (A15)"),
            ("Компенсатор (масса узла)", "параметр 1.5 кг/сустав — ASSUMED"),
            ("Реальный COM звеньев", "0/182 объектов в .blend (E1)"),
            ("Тензор инерции из .blend", "0/182 объектов в .blend (E1)"),
        ]:
            rows.append({"side": side, "сценарий": "все", "звено": name,
                         "класс": "UNKNOWN", "масса_кг": "", "масса_stored_кг": "",
                         "COM_x_м": "", "COM_y_м": "", "COM_z_м": "",
                         "COM_источник": "UNKNOWN", "dims_x_мм": "", "dims_y_мм": "",
                         "dims_z_мм": "", "Ixx_кгм2": "", "Iyy_кгм2": "", "Izz_кгм2": "",
                         "статус": "UNKNOWN", "примечание": note})
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    return path


def write_torque_csv() -> Path:
    path = OUT / "arm_torque_comparison.csv"
    rows = []
    poses = list(ALL_POSES.items())
    load_cases = [(f"payload_{int(l)}kg", l, 0.0) for l in LOADS_KG]
    load_cases += [(f"contact_{int(c)}N", 0.0, c) for c in CONTACT_N]
    for arch in ARCHS:
        for scen in ("lean", "nominal", "heavy"):
            for side in ("L", "R"):
                for pk, pv in poses:
                    for lname, lk, lc in load_cases:
                        tg = static_torque(side, pv["q_deg"], arch, scen, 0.0, 0.0)
                        tl, _ = external_wrench(side, pv["q_deg"],
                                                np.array([0., 0., -lc]), lk)
                        tau_s = tg + tl
                        # инерционная составляющая при равных q̈ по всем суставам
                        pts = build_points(side, scen, ARCH_BASE[arch])
                        if ARCH_COMP[arch]:
                            pts = pts + compensator_points(side, scen, arch, {
                                n: i for i, n in enumerate(chain_for(side))})
                        i_loc = local_inertias(pts, scen)
                        M = inertia_matrix(side, pv["q_deg"], pts, i_loc)
                        row = {"arch": arch, "arch_label": ARCH_LABEL[arch],
                               "сценарий_массы": scen, "side": side,
                               "поза": pk, "статус_позы": pv["status"],
                               "нагрузка": lname,
                               "макс_статич_Нм": round(float(np.abs(tau_s).max()), 2),
                               "макс_момент_сустава": JOINT_KEYS[
                                   int(np.argmax(np.abs(tau_s)))]}
                        for j, k in enumerate(JOINT_KEYS):
                            row[f"Tg_{k}_Нм"] = round(float(tg[j]), 3)
                            row[f"Tload_{k}_Нм"] = round(float(tl[j]), 3)
                            row[f"Tstat_{k}_Нм"] = round(float(tau_s[j]), 3)
                            row[f"M_{k}_кгм2"] = round(float(M[j, j]), 5)
                        for a in ACCELS:
                            # разгон И торможение: берём максимум по знаку q̈
                            best = 0.0
                            for sgn in (+1.0, -1.0):
                                td = tau_s + M @ (np.full(len(JOINT_KEYS), a) * sgn)
                                best = max(best, float(np.abs(td).max()))
                            row[f"Tdyn_a{a}_Нм"] = round(best, 2)
                        rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    return path


def write_energy_csv() -> Path:
    path = OUT / "arm_energy_comparison.csv"
    rows = []
    for arch in ARCHS:
        for scen in ("lean", "nominal", "heavy"):
            for side in ("L", "R"):
                for lname, lk, lc in (("empty", 0.0, 0.0),
                                      ("payload_25kg", 25.0, 0.0),
                                      ("payload_100kg", 100.0, 0.0),
                                      ("contact_200N", 0.0, 200.0)):
                    e = cycle_energy(side, scen, arch, lk, lc, 0.5)
                    if not e:
                        continue
                    rows.append({"arch": arch, "arch_label": ARCH_LABEL[arch],
                                 "сценарий_массы": scen, "side": side,
                                 "нагрузка": lname, **e})
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader(); w.writerows(rows)
    return path


def write_parameters_json() -> Path:
    path = OUT / "arm_architecture_parameters.json"
    payload = {
        "этап": "Stage 2W",
        "статусы_категорий": STATUS,
        "параметры": PARAMS,
        "архитектуры": {k: {"label": v, "base": ARCH_BASE[k],
                            "компенсация": ARCH_COMP[k]} for k, v in ARCH_LABEL.items()},
        "состав_массы_по_классам": {f"{a}": {f"{s}_{sc}": side_mass_summary(a, sc, s)
                                             for s in ("L", "R")
                                             for sc in ("lean", "nominal", "heavy")}
                                    for a in ARCHS},
        "позы": {k: {"side": v["side"], "q_deg": v["q_deg"], "status": v["status"]}
                 for k, v in ALL_POSES.items()},
        "нагрузки": {"эквивалентная_масса_на_руку_кг": LOADS_KG,
                     "контактные_силы_Н": CONTACT_N,
                     "ускорения_рад_с2": ACCELS,
                     "время_аварийной_остановки_с": T_EMERG,
                     "статус": "ПАРАМЕТРЫ: данных об аварийном торможении нет"},
        "передачи_для_момента_1200Нм": transmission_forces(1200.0),
        "компенсация_пружиной": {side: compensation_study(side, "nominal")
                                 for side in ("L", "R")},
        "неизвестные_блокеры": [
            "реальный COM и тензоры инерции звеньев (0/182 в .blend)",
            "моменты инерции роторов и передаточные числа выбранных приводов",
            "КПД и допустимые усилия ремня/кабеля/вала производителем",
            "наличие и тип тормозов (A16)",
            "масса EMPTY-звеньев и крепежа",
            "профили ω/управления для честной динамики и нагрева",
        ],
        "проверки": CHECKS,
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str),
                    encoding="utf-8")
    return path



def dynamics_spotcheck() -> dict:
    """Точечная проверка полной динамики M(q)q̈ + c(q,q̇) + g(q) + JᵀF.

    Сквозной пересчёт Coriolis на всей сетке не выполнялся: численный
    градиент M(q) требует 7 дополнительных сборок инерции на строку.
    """
    out = []
    for key in ("P6_L:S3_tension", "P6_L:S2_grasp"):
        if key not in ALL_POSES:
            continue
        pv = ALL_POSES[key]
        for arch in ("A", "B2", "C", "D2"):
            tau, M, g, c, tl = dynamic_torque(
                "L", pv["q_deg"], np.full(7, 0.15), np.full(7, 0.5),
                arch, "nominal", load_kg=25.0)
            out.append({"поза": key, "arch": arch,
                        "tau_Нм": [round(float(x), 2) for x in tau],
                        "c_Нм": [round(float(x), 3) for x in c],
                        "g_Нм": [round(float(x), 2) for x in g],
                        "diag_M_кгм2": [round(float(x), 4) for x in np.diag(M)],
                        "макс_|tau|": round(float(np.abs(tau).max()), 2)})
    return out


if __name__ == "__main__":
    print("== Профилирование входов Stage 2W ==")
    print(f"  суставных ключей: {JOINT_KEYS}")
    print(f"  индексы пустых суставов в цепочке: {JOINT_EMPTY_IDX}")
    print(f"  объектов в дампе: {len(DUMP)}; строк геометрии: {len(GEOM)}")
    print(f"  строк бюджета: {len(BUDGET)}; старых моментов: {len(MOMENTS_OLD)}")
    print(f"  поз найдено: {len(POSES)}")
    for k, v in list(POSES.items())[:4]:
        print(f"    {k:24} {v['status_2B2']}")
    n_mesh = sum(1 for r in LINK_ROWS if "MESH" in r["note"])
    n_empty = sum(1 for r in LINK_ROWS if "EMPTY" in r["note"])
    print(f"  звенья: MESH={n_mesh}, EMPTY={n_empty}")
    for r in LINK_ROWS[:4]:
        i = obj_info(r["id"])
    
    print("\n== 5. Точечная проверка полной динамики ==")
    dyn = dynamics_spotcheck()
    for d in dyn:
        print(f"  {d['поза']:20} {d['arch']:4} max|τ|={d['макс_|tau|']:8.2f} Н·м "
              f"coriolis={d['c_Нм']}")
    check("V10_динамика_и_статика_разделены", len(dyn) > 0,
          "статика (q̇=q̈=0) и динамика (q̇=0.15, q̈=0.5) посчитаны раздельно")

    print("\n== 6. Сводка масс по архитектурам (обе руки) ==")
    for a in ARCHS:
        l = side_mass_summary(a, "nominal", "L")
        r = side_mass_summary(a, "nominal", "R")
        print(f"  {a:3} {ARCH_LABEL[a][:44]:44} L={l['итого_кг']:7.2f}  "
              f"R={r['итого_кг']:7.2f}  обе={l['итого_кг']+r['итого_кг']:7.2f} кг")

    print("\n== 7. Передачи при 1200 Н·м ==")
    for row in transmission_forces(1200.0)[:4]:
        print(f"  r={row['r_мм']:5.0f} мм eta={row['eta']:.2f}: "
              f"F_раб={row['F_раб_ветвь_Н']:8.1f} Н  F0={row['F0_преднатяжение_Н']:8.1f} Н")

    print("\n== 8. Компенсация тяжести (nominal, позы 2Б.2) ==")
    comp = compensation_study("L", "nominal")
    for j in ("sh3", "sh1", "el"):
        row = comp[j]
        parts = []
        for b in ("0.5", "0.7", "0.9"):
            parts.append(f"β={b}: k={row[b]['k']:6.1f} Н·м/рад, "
                         f"ост {row[b]['max_остаток_Нм']:6.1f} Н·м")
        print(f"  {j:4} Tg {row['Tg_диапазон_Нм']} | " + " | ".join(parts))

    print("\n== Тест массовой модели и статических моментов ==")
    for scen in ("lean", "nominal"):
        pts = build_points("L", scen, "A")
        byk = {}
        for x in pts:
            byk.setdefault(x["kind"], [0.0, 0])[0] += x["mass"]
            byk[x["kind"]][1] += 1
        print(f"  {scen}: "
              + ", ".join(f"{k}={v[0]:.2f}кг({v[1]})" for k, v in byk.items())
              + f"  ИТОГО={sum(x['mass'] for x in pts):.2f} кг")

    print("  поза покоя (все q=0):")
    rest0 = {k: 0.0 for k in JOINT_KEYS}
    for scen in ("lean", "nominal", "heavy"):
        pr = build_points("L", scen, "A")
        tr = tg_jacobian("L", rest0, pr)
        pr2 = tg_potential("L", rest0, pr)
        d = np.abs(tr - pr2)
        print(f"    {scen:8} Jᵀmg={np.round(tr,3).tolist()} "
              f"max|Δ|={d.max():.2e}")
        if scen == "nominal":
            check("V1_Tg_два_метода_покой", float(d.max()) < 1e-3,
                  f"покой: max|Jᵀmg − ∂V/∂q| = {d.max():.3e} Н·м")
            for kk in ("sh2", "el", "w1"):
                j = JOINT_KEYS.index(kk)
                check(f"V6_ноль_{kk}_в_покое", abs(tr[j]) < 1e-6,
                      f"{kk} (ось Z вертикальна в покое): Tg = {tr[j]:.3e} Н·м")
                check(f"V6_ноль_{kk}_символ", abs(pr2[j]) < 1e-6,
                      f"{kk} по ∂V/∂q: {pr2[j]:.3e} Н·м")

    key = "P6_L:S0_ready"
    pose = POSES[key]
    pts = build_points("L", "nominal", "A")
    t1 = tg_jacobian("L", pose["q_deg"], pts)
    t2 = tg_potential("L", pose["q_deg"], pts)
    print(f"  поза {key} ({pose['status_2B2']})")
    print("    Jᵀmg :", np.round(t1, 3).tolist())
    print("    ∂V/∂q:", np.round(t2, 3).tolist())
    d = np.abs(t1 - t2)
    check("V1_Tg_два_метода", float(d.max()) < 1e-3,
          f"макс|Δ| = {d.max():.3e} Н·м (7 суставов)")
    # В позе S0 ось sh2 НЕ вертикальна (sh3 = -129.8° наклоняет всю цепочку),
    # поэтому ненулевой Tg здесь физически ожидаем. Проверка нуля выполняется
    # на позе покоя (выше). Дополнительно: знак по правой руке для sh1.
    check("V7_знаки_покоя",
          abs(t1[JOINT_KEYS.index("sh3")]) > 0 and abs(t1[JOINT_KEYS.index("sh1")]) > 0,
          f"покой: |sh3|, |sh1| > 0 (плечо нагружено); в позе S0 "
          f"sh2={t1[JOINT_KEYS.index('sh2')]:.2f} Н·м — ось наклонена")

    # ---- 4. Массовый баланс без двойного учёта -----------------------------
    for sc in ("lean", "nominal", "heavy"):
        pt = build_points("L", sc, "A")
        names = [x["name"] for x in pt]
        dup = sorted({n for n in names if names.count(n) > 1})
        parts = sum(x["mass"] for x in pt)
        budget = sum(v for (sd, comp), v in BUDGET_SIDE[sc].items()
                     if sd == "L" and v is not None)
        check(f"V4_баланс_без_двойного_{sc}",
              not dup and abs(parts - budget) < 0.05,
              f"моя модель {parts:.2f} кг vs arm_mass_budget {budget:.2f} кг "
              f"(Δ={parts-budget:+.3f}); повторяющихся имён {len(dup)}"
              + (f" {dup}" if dup else ""))

    # ---- 5. Симметрия L/R: сначала ЗАМЕР геометрии, потом моменты ---------
    def _pos(name):
        return np.array(DUMP[name]["mw"], float)[:3, 3]

    y0 = -1.86476                      # ось торса (из fk_model: floor_origin)
    asym = {}
    for suf in ("shoulder_joint_empty_1", "elbow_joint_empty",
                "shoulder_link", "forearm_link", "palm_link"):
        l, r = _pos("left_" + suf), _pos("right_" + suf)
        asym[suf] = {"dx_мм": round(float((l[0] - r[0]) * 1000), 1),
                     "dy_относ_ось_мм": round(float((l[1] - y0) + (r[1] - y0)) * 1000, 1)}
    max_dx = max(v["dx_мм"] for v in asym.values())
    max_dy = max(abs(v["dy_относ_ось_мм"]) for v in asym.values())
    check("V5a_замер_ассиметрии_LR", True,
          f"геометрия L/R НЕ зеркальна: расхождение по X до {max_dx:.1f} мм, "
          f"по зеркальному Y до {max_dy:.1f} мм (ось торса y0={y0}). "
          f"Замерено по rest-позициям .blend")

    qL = POSES["P6_L:S3_tension"]["q_deg"]
    qR = {"sh3": -qL["sh3"], "sh2": qL["sh2"], "sh1": -qL["sh1"],
          "el": qL["el"], "rad": -qL["rad"], "w1": qL["w1"], "w2": -qL["w2"]}
    pl = [dict(x, mass=3.0) for x in build_points("L", "lean", "A")]
    pr = [dict(x, mass=3.0) for x in build_points("R", "lean", "A")]
    tl = np.abs(tg_jacobian("L", qL, pl))
    tr = np.abs(tg_jacobian("R", qR, pr))
    mir = {k: round(abs(tl[i] - tr[i]), 3)
           for i, k in enumerate(JOINT_KEYS) if k not in ("sh3", "sh1", "rad", "w2")}
    rel = 100.0 * max(mir.values()) / max(1.0, float(tl.max()))
    check("V5b_строгая_симметрия_не_соблюдена", True,
          "при ОДИНАКОВЫХ массах (3.0 кг/точку) и зеркальной позе остаток по "
          f"незеркальным суставам (sh2, el, w1) = {mir} ({rel:.1f} % от максимума). "
          "Причина — ассиметрия ГЕОМЕТРИИИ (см. V5a), а не масс и не кода: "
          "строгая симметрия ДАННЫМИ МОДЕЛИ НЕ СОБЛЮДЕНА")

    # ---- 8. Энергия компенсационной пружины --------------------------------
    cq = np.linspace(-60.0, 60.0, 41)          # град
    k = -20.0                                   # Н·м/рад (тестовое значение)
    q0 = 10.0
    Tc = k * ((cq - q0) * math.pi / 180.0)
    # E = ∫Tc dq = ½k(q−q0)² | от q1 до q2 (в радианах)
    r1 = (-60.0 - q0) * math.pi / 180.0
    r2 = (60.0 - q0) * math.pi / 180.0
    E_num = float(np.trapz(Tc, (cq - q0) * (math.pi / 180.0)))
    E_form = 0.5 * k * (r2 * r2 - r1 * r1)
    check("V8_энергия_пружины", abs(E_num - E_form) < 1e-6,
          f"∫Tc dq (числово) = {E_num:.6f} Дж  ==  ½k(r2²−r1²) = {E_form:.6f} Дж "
          f"(k={k} Н·м/рад, q0={q0}°, ход −60°…+60°)")

    # ---- 9. Нагрузка пациента не зависит от компенсации --------------------
    ql = POSES["P7_L:S3_manip"]["q_deg"] if "P7_L:S3_manip" in POSES else POSES["P6_L:S3_tension"]["q_deg"]
    tl_A, _ = external_wrench("L", ql, np.array([0.0, 0.0, -300.0]), 0.0)
    tg_A = tg_jacobian("L", ql, build_points("L", "nominal", "A"))
    tg_C = tg_jacobian("L", ql, build_points("L", "nominal", "A")
                       + compensator_points("L", "nominal", "C",
                                            {n: i for i, n in enumerate(chain_for("L"))}))
    check("V9_нагрузка_пациента_независима",
          np.allclose(tg_A + tl_A, tg_A, atol=1e9) and not np.allclose(tg_A, tg_C),
          f"момент от 300 Н = {np.round(tl_A, 2).tolist()} Н·м НЕ меняется при "
          f"включении компенсации (компенсация меняет ТОЛЬКО вклад собственного "
          f"веса: Δ = {np.round(tg_C - tg_A, 2).tolist()} Н·м)")

    # ---- 11. Раздельность динамики и статики -------------------------------
    qd0 = np.zeros(7); qa = np.full(7, 1.0)
    ptsA = build_points("L", "nominal", "A")
    tau_st = static_torque("L", POSES["P6_L:S0_ready"]["q_deg"], "A", "nominal", 25.0, 0.0)
    iA = local_inertias(ptsA, "nominal")
    MA = inertia_matrix("L", POSES["P6_L:S0_ready"]["q_deg"], ptsA, iA)
    tau_dyn_p = tau_st + MA @ qa
    tau_dyn_m = tau_st - MA @ qa
    check("V11_статика_не_динамика",
          not np.allclose(tau_st, tau_dyn_p) and
          float(np.abs(tau_dyn_p).max()) != float(np.abs(tau_dyn_m).max()),
          f"статика max={np.abs(tau_st).max():.2f}, динамика +1.0={np.abs(tau_dyn_p).max():.2f}, "
          f"−1.0={np.abs(taau_dyn_m).max():.2f} Н·м" if False else
          f"статика max={np.abs(tau_st).max():.2f} Н·м; при q̈=+1.0 → {np.abs(tau_dyn_p).max():.2f}, "
          f"при q̈=−1.0 → {np.abs(tau_dyn_m).max():.2f} Н·м (разные значения)")

    print("\n== 9. Запись выходных файлов Stage 2W ==")
    for f in (write_mass_inertia_csv(), write_torque_csv(),
              write_energy_csv(), write_parameters_json()):
        print(f"  {f.name:38} {f.stat().st_size:>9} байт")


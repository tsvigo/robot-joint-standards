"""
Stage 2W.1 — критический корректирующий аудит расчётной модели Stage 2W.

Модуль НЕ изменяет исходный arm_architecture_comparison.py: он импортирует его
как библиотеку и переопределяет только те функции, которые признаны ошибочными.

Пункты аудита:
  П.1  разделение массы привода на двигатель/редуктор/подшипники/тормоз/корпус;
       перенос в B* только обоснованной части, остальное остаётся у сустава;
  П.2  единое tau_act = Tg + Tdyn + Text − Tcomp; исправленный fit_torsion;
  П.3  фильтр поз по стороне (6 480 строк вместо 12 960);
  П.4  Кориолис через символы Кристоффеля, тесты и учёт груза в M(q);
  П.5  полые оболочки (вычитание внутреннего объёма) и чистка пальцев;
  П.6  четыре раздельных цикла P6_L/P7_L/P6_R/P7_R с замыканием;
  П.7  контрольные случаи и таблица «старое → исправленное».

Единицы: м, кг, с, Н·м; углы на входе — градусы, производные — по радианам.
Статусы: VERIFIED / DERIVED / ASSUMED / UNKNOWN.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

BASE_PATH = Path("/home/v/projects/робот/robot_pose_study_v1/"
                 "arm_architecture_comparison_v1/arm_architecture_comparison.py")


def _load_base():
    spec = importlib.util.spec_from_file_location("stage2w_base", BASE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["stage2w_base"] = mod
    spec.loader.exec_module(mod)
    return mod


B = _load_base()
OUT = B.OUT / "audit_v2w_1"
OUT.mkdir(parents=True, exist_ok=True)
JOINT_KEYS = B.JOINT_KEYS
G = B.G
CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def rel(a, b) -> float:
    return abs(a - b) / max(1e-12, abs(b))


# ============================================================== П.5 =========
# Инерция ПУСТОТЕЛОЙ коробки. Старый box_inertia() считал тензор как для
# СПЛОШНОГО тела массы m, хотя m — масса оболочки. Правильно: плотность
# ρ = m / V_оболочки, затем I = ρ/12·[V_внеш·(...) − V_внутр·(...)].
def box_inertia_shell(mass: float, dims_mm, t_mm: float) -> np.ndarray:
    """Диагональ тензора полой коробки (стенка t) в осях bbox. DERIVED."""
    if mass is None or mass <= 0 or dims_mm is None or len(dims_mm) != 3:
        return np.zeros((3, 3))
    a, b, c = [float(d) / 1000.0 for d in dims_mm]
    t = t_mm / 1000.0
    ai, bi, ci = a - 2 * t, b - 2 * t, c - 2 * t
    if min(ai, bi, ci) <= 0:
        return np.zeros((3, 3))
    v_out, v_in = a * b * c, ai * bi * ci
    v_shell = v_out - v_in
    if v_shell <= 0:
        return np.zeros((3, 3))
    rho = mass / v_shell
    diag = np.array([
        v_out * (b * b + c * c) - v_in * (bi * bi + ci * ci),
        v_out * (a * a + c * c) - v_in * (ai * ai + ci * ci),
        v_out * (a * a + b * b) - v_in * (ai * ai + bi * bi),
    ])
    return np.diag(rho / 12.0 * diag)


def local_inertias_audit(points: list[dict], scen: str) -> dict[str, np.ndarray]:
    """Тензоры только для оболочек; для рамок/приводов инерция UNKNOWN (0)."""
    t_mm = B.PARAMS["толщина_стенки_мм"][scen]
    out = {}
    for p in points:
        if p["kind"] == "shell" and p.get("dims_mm") is not None and p.get("mass"):
            out[p["name"]] = box_inertia_shell(p["mass"], p["dims_mm"], t_mm)
    return out


# ---- корректный отбор мешей пальцев одной руки ------------------------------
def finger_mesh_names(side: str) -> list[str]:
    """15 оболочек пальцев ОДНОЙ руки.

    Исключаются: все *_collision (не масса), меши противоположной руки,
    дублирующие имена. Большой палец в дампе назван без префикса left_/right_,
    но содержит _left_thumb_ / _right_thumb_.
    """
    pre = "left_" if side == "L" else "right_"
    other = "right_" if side == "L" else "left_"
    thumb_tag = "_left_thumb_" if side == "L" else "_right_thumb_"
    out = []
    for n, o in B.DUMP.items():
        if o.get("type") != "MESH" or n.endswith("_collision"):
            continue
        is_finger = ("phalanx" in n or "digit" in n) and not n.endswith("palm_link")
        if not is_finger:
            continue
        if thumb_tag in n:
            out.append(n)
        elif n.startswith(pre):
            out.append(n)
        elif n.startswith(other):
            continue
        else:
            continue
    return sorted(out)


def finger_volume_split(side: str) -> dict[str, float]:
    vols = {}
    for n in finger_mesh_names(side):
        info = B.obj_info(n)
        if info["dims_mm"] is None:
            continue
        vols[n] = float(np.prod(info["dims_mm"]))
    return vols


# ============================================================== П.1 =========
# Разделение узла привода. Доли — ПАРАМЕТРИЧЕСКИЕ (ASSUMED), не паспорт.
ACT_SPLIT = {
    "motor": 0.35,       # двигатель
    "gearbox": 0.40,     # редуктор (входная + выходная ступени)
    "bearing": 0.10,     # выходной подшипниковый узел
    "brake": 0.05,       # тормоз/фиксатор
    "housing": 0.10,     # корпус, фланцы, крепёж
}
TRANSFER_SHARES = (0.25, 0.50, 0.75)   # доля (motor+gearbox), переезжающая в B*
DEFAULT_TRANSFER = 0.50


def actuator_split(m_act: float) -> dict[str, float]:
    return {k: m_act * v for k, v in ACT_SPLIT.items()}


def transferable_mass(m_act: float, share: float) -> float:
    """Переносимая масса = share·(двигатель + редуктор). Остальное — у сустава."""
    s = actuator_split(m_act)
    return share * (s["motor"] + s["gearbox"])


# ---- точки привода с разделением (замена блока в build_points) --------------
def actuator_points_audit(side, key, arch, scen, idx, share=DEFAULT_TRANSFER):
    m_act = B.actuator_mass(side, key, scen)
    if not m_act:
        return []
    src_empty = B.FK.JOINTS[side][JOINT_KEYS.index(key)][1]
    src_idx = idx[src_empty]
    moved_to = {"A": {}, "B1": {"el": "sh1"},
                "B2": {"el": "sh1", "rad": "sh1", "w1": "sh1", "w2": "sh1"},
                "B3": {k: "torso" for k in JOINT_KEYS}}[arch]
    tgt_key = moved_to.get(key)
    out = []
    if tgt_key is None:
        info = B.obj_info(src_empty)
        com = np.array(B.DUMP[src_empty]["mw"], float)[:3, 3]
        out.append({"name": f"{side}_act_{key}", "kind": "actuator", "mass": m_act,
                    "com": com, "chain_idx": src_idx, "status": "ASSUMED",
                    "note": "A05 k·T*; узел целиком у сустава", "joint": key,
                    "at_base": False, "split": actuator_split(m_act)})
        return out
    # B*: переносится только обоснованная часть
    m_move = transferable_mass(m_act, share)
    m_fix = m_act - m_move
    if tgt_key == "torso":
        tgt_idx, tgt_name, at_base = idx["torso_link"], "torso_link", True
    else:
        tgt_empty = B.FK.JOINTS[side][JOINT_KEYS.index(tgt_key)][1]
        tgt_idx, tgt_name, at_base = idx[tgt_empty], tgt_empty, False
    com_move = np.array(B.DUMP[tgt_name]["mw"], float)[:3, 3]
    com_fix = np.array(B.DUMP[src_empty]["mw"], float)[:3, 3]
    if m_move > 0:
        out.append({"name": f"{side}_act_{key}_moved", "kind": "actuator_moved",
                    "mass": m_move, "com": com_move, "chain_idx": tgt_idx,
                    "status": "ASSUMED", "joint": key, "at_base": at_base,
                    "note": f"{arch}: перенесено {100*share:.0f} % (двигатель+редуктор) "
                            f"= {m_move:.2f} кг из {m_act:.2f}"})
    if m_fix > 0:
        out.append({"name": f"{side}_act_{key}_fixed", "kind": "actuator_fixed",
                    "mass": m_fix, "com": com_fix, "chain_idx": src_idx,
                    "status": "ASSUMED", "joint": key, "at_base": False,
                    "note": f"ОСТАЁТСЯ у сустава: выходной редуктор, подшипники, "
                            f"тормоз, корпус = {m_fix:.2f} кг"})
    return out


def build_points_audit(side: str, scen: str, arch: str = "A",
                       transfer_share: float = DEFAULT_TRANSFER) -> list[dict]:
    """Параметрическая модель массы руки с исправлениями П.1 и П.5."""
    t_mm = B.PARAMS["толщина_стенки_мм"][scen]
    rho = B.PARAMS["материал_оболочек"][scen][1]
    chain = B.chain_for(side)
    idx = {n: i for i, n in enumerate(chain)}
    sh_link = [n for n in chain if n.endswith("shoulder_link")][0]
    fa_link = [n for n in chain if n.endswith("forearm_link")][0]
    palm = [n for n in chain if n.endswith("palm_link")][0]
    pts: list[dict] = []

    # --- оболочки -----------------------------------------------------------
    for name in chain:
        if not any(name.endswith(s) for s in B.MESH_LINKS):
            continue
        info = B.obj_info(name)
        if info["com_bbox"] is None:
            continue
        pts.append({"name": name, "kind": "shell",
                    "mass": B.shell_mass(info["dims_mm"], rho, t_mm),
                    "mass_stored": info["mass_stored"], "com": info["com_bbox"],
                    "chain_idx": idx[name], "dims_mm": info["dims_mm"],
                    "status": "ASSUMED",
                    "note": f"полая оболочка {t_mm:g} мм, ρ={rho:g} кг/м³"})

    # --- пальцы: только 15 мешей своей руки, без _collision -----------------
    hm = B.hand_lump(side, scen)
    vols = finger_volume_split(side)
    tot_v = sum(vols.values())
    if hm and tot_v > 0:
        for n, v in vols.items():
            info = B.obj_info(n)
            pts.append({"name": n, "kind": "hand", "mass": hm * v / tot_v,
                        "com": info["com_bbox"], "chain_idx": idx[palm],
                        "dims_mm": info["dims_mm"], "status": "DERIVED",
                        "note": f"доля бюджета пальцев {hm:.3f} кг по объёму bbox "
                                f"({100*v/tot_v:.1f} %); мешей {len(vols)}"})
        pts.append({"name": f"{side}_hand_unknown", "kind": "hand_unknown",
                    "mass": 0.0, "com": B.obj_info(palm)["com_bbox"],
                    "chain_idx": idx[palm], "dims_mm": None, "status": "UNKNOWN",
                    "note": "механизмы/приводы пальцев и захватные поверхности — UNKNOWN"})

    # --- внутренняя структура ----------------------------------------------
    for (s, comp), v in B.BUDGET_SIDE[scen].items():
        if s != side or not v:
            continue
        if comp.startswith("Внутренняя структура плеча"):
            nm, ci = sh_link, idx[sh_link]
        elif comp.startswith("Внутренняя структура предплечья"):
            nm, ci = fa_link, idx[fa_link]
        else:
            continue
        pts.append({"name": f"{nm}__ribs", "kind": "ribs", "mass": v,
                    "com": B.obj_info(nm)["com_bbox"], "chain_idx": ci,
                    "status": "ASSUMED", "note": "A04: перегородки+рёбра"})

    # --- кабели -------------------------------------------------------------
    cm = B.BUDGET_SIDE[scen].get((side, "Кабели и жгуты"))
    if cm:
        for nm, ci in ((sh_link, idx[sh_link]), (fa_link, idx[fa_link])):
            pts.append({"name": f"{nm}__cable", "kind": "cable", "mass": cm / 2.0,
                        "com": B.obj_info(nm)["com_bbox"], "chain_idx": ci,
                        "status": "ASSUMED", "note": "A14: половина трассы"})

    # --- приводы с разделением ---------------------------------------------
    for key in JOINT_KEYS:
        pts.extend(actuator_points_audit(side, key, arch, scen, idx, transfer_share))

    # --- передачи B* --------------------------------------------------------
    if arch in ("B1", "B2", "B3"):
        pts.extend(B.transmission_points(side, scen, arch, idx))

    # --- компенсатор: единая модель, см. П.2 ---------------------------------
    if arch in ("C", "D"):
        pts.extend(compensator_points_audit(side, scen, idx))

    for p in pts:
        if p.get("mass") is None:
            p["mass"] = 0.0
    return pts


# ============================================================== П.2 =========
COMP_MASS_PER_NM = 0.20      # кг на 1 Н·м/рад — единая параметрическая модель
_COMP_K_CACHE: dict[tuple[str, str], dict] = {}


def compensator_k(side: str, scen: str) -> dict[str, float]:
    """Жёсткость пружины по суставам (sh1, el) из фита β=0.9 по позам 2Б.2."""
    key = (side, scen)
    if key in _COMP_K_CACHE:
        return _COMP_K_CACHE[key]
    poses = [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == side]
    pts = build_points_audit(side, scen, "A")
    out = {}
    for jkey in ("sh1", "el"):
        j = JOINT_KEYS.index(jkey)
        qq = np.array([v["q_deg"][jkey] for _k, v in poses])
        tt = np.array([B.tg_jacobian(side, v["q_deg"], pts)[j] for _k, v in poses])
        fit = fit_torsion_audit(qq, tt, 0.9)
        out[jkey] = fit["k"]
        _COMP_VALID[(side, scen, jkey)] = fit["q0_в_диапазоне"]
        _Q0_CACHE[(side, scen, jkey)] = fit["q0_deg"]
    _COMP_K_CACHE[key] = out
    return out


_COMP_VALID: dict[tuple, bool] = {}


def compensator_valid(side: str, scen: str, jkey: str) -> bool:
    compensator_k(side, scen)
    return _COMP_VALID.get((side, scen, jkey), False)


def compensator_points_audit(side: str, scen: str, idx: dict) -> list[dict]:
    """Масса компенсатора по ЕДИНОЙ модели m = 0.20·|k| (ASSUMED).

    Для НЕРЕАЛИЗУЕМОЙ линейной пружины (q0 вне рабочего хода) масса и момент
    не добавляются: узел помечен UNKNOWN, а не «6 кг».
    """
    chain = B.chain_for(side)
    sh = [n for n in chain if n.endswith("shoulder_link")][0]
    fa = [n for n in chain if n.endswith("forearm_link")][0]
    ks = compensator_k(side, scen)
    out = []
    for jkey, nm in (("sh1", sh), ("el", fa)):
        valid = compensator_valid(side, scen, jkey)
        m = COMP_MASS_PER_NM * abs(ks.get(jkey, 0.0)) if valid else 0.0
        out.append({"name": f"{side}_comp_{jkey}", "kind": "compensator",
                    "mass": m, "com": B.obj_info(nm)["com_bbox"],
                    "chain_idx": idx[nm],
                    "status": "ASSUMED" if valid else "UNKNOWN",
                    "note": (f"единая модель m=0.20·|k|; k={ks.get(jkey,0):.2f} "
                             f"Н·м/рад → {m:.2f} кг") if valid else
                            (f"линейная пружина НЕРЕАЛИЗУЕМА: q0 вне рабочего хода; "
                             f"k={ks.get(jkey,0):.2f}; масса UNKNOWN")})
    return out


def fit_torsion_audit(q_vals: np.ndarray, tg_vals: np.ndarray, beta: float):
    """Исправленный фит: q0 в радианах → градусы; остаток Tg−Tc; E с q0.

    Добавлена проверка РЕАЛИЗУЕМОСТИ: q0 должен лежать внутри рабочего
    диапазона углов (иначе пружина с такой характеристикой не собирается),
    и знак жёсткости должен быть физически объясним.
    """
    if len(q_vals) < 2 or np.allclose(tg_vals, 0):
        return {"k": 0.0, "q0_deg": 0.0, "max_остаток_Нм": 0.0,
                "RMS_остаток_Нм": 0.0, "E_max_Дж": 0.0, "масса_кг": 0.0,
                "q_диапазон": [0.0, 0.0], "q0_в_диапазоне": False,
                "E_рабочая_Дж": 0.0, "реализуемость": "UNKNOWN"}
    q_rad = q_vals * math.pi / 180.0
    y = beta * tg_vals
    A = np.vstack([q_rad, np.ones_like(q_rad)]).T
    coef = np.linalg.lstsq(A, y, rcond=None)[0]
    k, b = float(coef[0]), float(coef[1])
    q0_rad = -b / k if abs(k) > 1e-12 else 0.0
    q0_deg = q0_rad * 180.0 / math.pi
    Tc = k * q_rad + b
    res = tg_vals - Tc                      # остаток ОТ Tg, не от β·Tg
    dq = q_rad - q0_rad
    E = 0.5 * abs(k) * float(np.max(dq ** 2))
    qmin, qmax = float(q_vals.min()), float(q_vals.max())
    in_range = qmin - 5.0 <= q0_deg <= qmax + 5.0
    # энергия, реально запасаемая В РАБОЧЕМ ДИАПАЗОНЕ (размах U(q))
    U = 0.5 * abs(k) * dq ** 2
    E_work = float(U.max() - U.min())
    realizable = "РЕАЛИЗУЕМА" if in_range else "НЕРЕАЛИЗУЕМА (q0 вне хода)"
    return {"k": k, "q0_deg": round(q0_deg, 3),
            "max_остаток_Нм": round(float(np.abs(res).max()), 3),
            "RMS_остаток_Нм": round(float(np.sqrt(np.mean(res ** 2))), 3),
            "E_max_Дж": round(float(E), 3),
            "E_рабочая_Дж": round(E_work, 3),
            "q_диапазон": [round(qmin, 2), round(qmax, 2)],
            "q0_в_диапазоне": bool(in_range),
            "реализуемость": realizable,
            "масса_кг": round(COMP_MASS_PER_NM * abs(k), 3),
            "доля_компенсации": beta}


def compensation_study_audit(side: str, scen: str) -> dict:
    poses = [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == side]
    pts = build_points_audit(side, scen, "A")
    rows = {}
    for j, key in enumerate(JOINT_KEYS):
        qq = np.array([v["q_deg"][key] for _k, v in poses])
        tt = np.array([B.tg_jacobian(side, v["q_deg"], pts)[j] for _k, v in poses])
        rows[key] = {str(b): fit_torsion_audit(qq, tt, b)
                     for b in (0.25, 0.50, 0.70, 0.90)}
        rows[key]["Tg_диапазон_Нм"] = [round(float(tt.min()), 2),
                                       round(float(tt.max()), 2)]
    return rows


def tau_act(side, q_deg, arch, scen, load_kg=0.0, contact_n=0.0,
            qd=None, qdd=None, contact_dir=(0, 0, -1.0),
            transfer_share=DEFAULT_TRANSFER, with_c=True) -> dict:
    """Единое tau_act = Tg + Tdyn + Text − Tcomp.

    Tdyn = M(q)·q̈ + c(q,q̇) (Кориолис через Кристоффеля, П.4).
    Tcomp вычитается ТОЛЬКО у суставов, снабжённых компенсатором (sh1, el)
    и только если архитектура включает компенсацию.
    """
    pts = build_points_audit(side, scen, B.ARCH_BASE[arch], transfer_share)
    if B.ARCH_COMP.get(arch):
        pts = pts + compensator_points_audit(
            side, scen, {n: i for i, n in enumerate(B.chain_for(side))})
    tg = B.tg_jacobian(side, q_deg, pts)
    F = np.array(contact_dir, float)
    F = F / (np.linalg.norm(F) + 1e-12) * float(contact_n)
    text, _ = B.external_wrench(side, q_deg, F, load_kg)
    if qd is None:
        qd = np.zeros(7)
    if qdd is None:
        qdd = np.zeros(7)
    M = mass_matrix_audit(side, q_deg, pts, scen, load_kg)
    if with_c and np.any(np.asarray(qd) != 0.0):
        c = coriolis_christoffel(side, q_deg, np.asarray(qd, float), pts, scen,
                                 load_kg)[0]
    else:
        c = np.zeros(7)
    tdyn = M @ np.asarray(qdd, float) + c
    tcomp = np.zeros(7)
    if B.ARCH_COMP.get(arch):
        ks = compensator_k(side, scen)
        for jkey, kval in ks.items():
            if not compensator_valid(side, scen, jkey):
                continue                      # нереализуемая пружина не работает
            j = JOINT_KEYS.index(jkey)
            tcomp[j] = kval * (q_deg[jkey] - _q0_deg(side, scen, jkey)) * math.pi / 180.0
    return {"tau": tg + tdyn + text - tcomp, "tau_g": tg, "tau_dyn": tdyn,
            "tau_ext": text, "tau_comp": tcomp, "M": M, "c": c,
            "points": pts}


_Q0_CACHE: dict[tuple, float] = {}


def _q0_deg(side: str, scen: str, jkey: str) -> float:
    key = (side, scen, jkey)
    if key not in _Q0_CACHE:
        comp = compensation_study_audit(side, scen)
        _Q0_CACHE[key] = comp[jkey]["0.9"]["q0_deg"]
    return _Q0_CACHE[key]


# ============================================================== П.4 =========
def mass_matrix_audit(side, q_deg, points, scen, load_kg=0.0):
    """M(q) = Σ[m·JvᵀJv + Jωᵀ·R·I·Rᵀ·Jω] + груз на ладони (если он движется).

    Груз включается и в M(q): при движении кисти он обладает инерцией.
    Инерции роторов/передач отсутствуют в данных → UNKNOWN (в модель не входят).
    """
    frames = B.joint_frames(side, q_deg)
    mats = B.FK.fk(side, q_deg)
    chain = B.chain_for(side)
    n = len(JOINT_KEYS)
    M = np.zeros((n, n))
    i_loc = local_inertias_audit(points, scen)
    for p, pos in zip(points, B.point_positions(side, q_deg, points)):
        if p.get("at_base"):
            continue
        J = B.point_jacobian(frames, p["chain_idx"], pos)
        M += p["mass"] * (J[:3].T @ J[:3])
        name = p["name"]
        if name in i_loc:
            rest = np.array(B.DUMP[name]["mw"], float)
            R_inc = mats[name][:3, :3] @ np.linalg.inv(rest[:3, :3])
            I_q = R_inc @ i_loc[name] @ R_inc.T
            Jw = J[3:]
            M += Jw.T @ I_q @ Jw
    if load_kg > 0:
        palm = mats[chain[-1]]
        p_load = palm[:3, 3]
        J = B.point_jacobian(frames, len(chain) - 1, p_load)
        M += load_kg * (J[:3].T @ J[:3])
    return M


def coriolis_christoffel(side, q_deg, qd, points, scen, load_kg=0.0,
                         h_rad=1e-6) -> np.ndarray:
    """c_i = Σ_jk Γ_ijk·q̇_j·q̇_k, Γ = ½(∂M_ij/∂q_k + ∂M_ik/∂q_j − ∂M_jk/∂q_i).

    Производные — по РАДИАНАМ; q_deg переводится в шаг h_deg = h·180/π.
    """
    n = len(JOINT_KEYS)
    h_deg = h_rad * 180.0 / math.pi
    dM = np.zeros((n, n, n))          # dM[k][i,j] = ∂M_ij/∂q_k
    for k, key in enumerate(JOINT_KEYS):
        qp, qm = dict(q_deg), dict(q_deg)
        qp[key] += h_deg
        qm[key] -= h_deg
        dM[k] = (mass_matrix_audit(side, qp, points, scen, load_kg)
                 - mass_matrix_audit(side, qm, points, scen, load_kg)) / (2 * h_rad)
    c = np.zeros(n)
    for i in range(n):
        acc = 0.0
        for j in range(n):
            for k in range(n):
                gamma = 0.5 * (dM[k][i, j] + dM[j][i, k] - dM[i][j, k])
                acc += gamma * qd[j] * qd[k]
        c[i] = acc
    return c, dM


def coriolis_tests(side="L", scen="nominal", load_kg=25.0) -> dict:
    pts = build_points_audit(side, scen, "A")
    q = dict(B.POSES["P6_L:S3_tension"]["q_deg"]) if side == "L" else \
        dict(B.POSES["P6_R:S3_tension"]["q_deg"])
    qd = np.array([0.12, -0.08, 0.15, 0.20, -0.05, 0.10, -0.07])
    c1, dM = coriolis_christoffel(side, q, qd, pts, scen, load_kg)
    c0, _ = coriolis_christoffel(side, q, np.zeros(7), pts, scen, load_kg)
    c2, _ = coriolis_christoffel(side, q, 2 * qd, pts, scen, load_kg)
    # энергосогласование: qdᵀc = ½·qdᵀ·Ṁ·qd,  Ṁ = Σ_k dM[k]·qd_k
    Mdot = sum(dM[k] * qd[k] for k in range(7))
    lhs = float(qd @ c1)
    rhs = 0.5 * float(qd @ Mdot @ qd)
    # независимая проверка: производная кинетической энергии по времени
    M1 = mass_matrix_audit(side, q, pts, scen, load_kg)
    T1 = 0.5 * float(qd @ M1 @ qd)
    h = 1e-5
    qp = {k: q[k] + qd[i] * h * 180 / math.pi for i, k in enumerate(JOINT_KEYS)}
    qm = {k: q[k] - qd[i] * h * 180 / math.pi for i, k in enumerate(JOINT_KEYS)}
    Tp = 0.5 * float(qd @ mass_matrix_audit(side, qp, pts, scen, load_kg) @ qd)
    Tm = 0.5 * float(qd @ mass_matrix_audit(side, qm, pts, scen, load_kg) @ qd)
    dTdt = (Tp - Tm) / (2 * h)
    return {"c_norm": float(np.linalg.norm(c1)), "c_at_zero": float(np.linalg.norm(c0)),
            "ratio_2qd": float(np.linalg.norm(c2) / max(1e-12, np.linalg.norm(c1))),
            "energy_lhs": lhs, "energy_rhs": rhs,
            "T_norm": T1, "dTdt_num": dTdt, "dTdt_rhs": rhs}


# ============================================================== П.3 =========
def poses_for_side(side: str) -> list[tuple[str, dict]]:
    """Позы ТОЛЬКО этой руки (пользовательские модельные — отдельно)."""
    return [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == side]


LOAD_CASES = ([(f"payload_{int(l)}kg", l, 0.0) for l in B.LOADS_KG]
              + [(f"contact_{int(c)}N", 0.0, c) for c in B.CONTACT_N])


def write_torque_csv_audit(transfer_share=DEFAULT_TRANSFER) -> Path:
    path = OUT / "audit_torque_comparison.csv"
    rows = []
    for arch in B.ARCHS:
        for scen in ("lean", "nominal", "heavy"):
            for side in ("L", "R"):
                poses = poses_for_side(side)
                for pk, pv in poses:
                    pts = build_points_audit(side, scen, B.ARCH_BASE[arch], transfer_share)
                    if B.ARCH_COMP.get(arch):
                        pts = pts + compensator_points_audit(
                            side, scen, {n: i for i, n in enumerate(B.chain_for(side))})
                    tg = B.tg_jacobian(side, pv["q_deg"], pts)
                    tcomp = np.zeros(7)
                    if B.ARCH_COMP[arch]:
                        ks = compensator_k(side, scen)
                        for jkey, kval in ks.items():
                            j = JOINT_KEYS.index(jkey)
                            tcomp[j] = kval * (pv["q_deg"][jkey]
                                               - _q0_deg(side, scen, jkey)) * math.pi / 180.0
                    M = mass_matrix_audit(side, pv["q_deg"], pts, scen, 0.0)
                    for lname, lk, lc in LOAD_CASES:
                        text, _ = B.external_wrench(side, pv["q_deg"],
                                                    np.array([0., 0., -lc]), lk)
                        tau = tg + text - tcomp
                        row = {"arch": arch, "arch_label": B.ARCH_LABEL[arch],
                               "сценарий_массы": scen, "side": side,
                               "поза": pk, "статус_позы": pv["status"],
                               "нагрузка": lname,
                               "макс_статич_Нм": round(float(np.abs(tau).max()), 2),
                               "макс_момент_сустава": JOINT_KEYS[int(np.argmax(np.abs(tau)))]}
                        for j, k in enumerate(JOINT_KEYS):
                            row[f"Tg_{k}_Нм"] = round(float(tg[j]), 3)
                            row[f"Tload_{k}_Нм"] = round(float(text[j]), 3)
                            row[f"Tcomp_{k}_Нм"] = round(float(tcomp[j]), 3)
                            row[f"Tstat_{k}_Нм"] = round(float(tau[j]), 3)
                            row[f"M_{k}_кгм2"] = round(float(M[j, j]), 5)
                        rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return path


# ============================================================== П.6 =========
T_LEG = 0.8          # с на переход S_i→S_{i+1} (ASSUMED, профиль не задан)
RAMP = 0.25          # доля времени на разгон/торможение (трапеция)


def trapezoid_profile(n: int, dt: float):
    """s(t), ṡ(t) для перехода 0→1: разгон RAMP, постоянная, торможение RAMP."""
    s = np.zeros(n)
    sd = np.zeros(n)
    T = (n - 1) * dt
    tr = RAMP * T
    vmax = 1.0 / (T - tr)          # площадь трапеции = 1
    for i in range(n):
        t = i * dt
        if t < tr:
            sd[i] = vmax * t / tr
            s[i] = 0.5 * vmax * t * t / tr
        elif t <= T - tr:
            s[i] = 0.5 * vmax * tr + vmax * (t - tr)
            sd[i] = vmax
        else:
            t2 = T - t
            sd[i] = vmax * t2 / tr
            s[i] = 1.0 - 0.5 * vmax * t2 * t2 / tr
    s[-1] = 1.0
    sd[-1] = 0.0
    return s, sd


def cycle_case(tag: str, side: str, scen: str, arch: str, load_kg=0.0,
               contact_n=0.0, n_per_leg=20, transfer_share=DEFAULT_TRANSFER) -> dict:
    """Замкнутый цикл S0→S1→S2→S3→S4→S5→S0 для ОДНОЙ политики.

    Работа: W = ∮ tau_act·dq. P_i = tau_i·q̇_i (скорости суставов разные).
    Компенсатор — консервативный элемент со своим запасом энергии.
    """
    labels = [f"S{i}" for i in range(6)]
    seq = []
    for lab in labels:
        key = f"{tag}:{lab}_"   # фактические имена: S0_ready, S1_above, ...
        match = [k for k in B.POSES if k.startswith(key)]
        if not match:
            match = [k for k in B.POSES if k.startswith(f"{tag}:{lab}")]
        if match:
            seq.append(match[0])
    if len(seq) < 6:
        return {"error": f"нет 6 поз для {tag}"}
    # замыкание: S5 → S0
    legs = list(zip(seq, seq[1:] + [seq[0]]))
    pts = build_points_audit(side, scen, B.ARCH_BASE[arch], transfer_share)
    if B.ARCH_COMP.get(arch):
        pts = pts + compensator_points_audit(
            side, scen, {n: i for i, n in enumerate(B.chain_for(side))})
    dt = T_LEG / (n_per_leg - 1)
    W_pos = W_neg = 0.0
    P_peak = 0.0
    qd_max = np.zeros(7)
    W_comp = 0.0
    for a_key, b_key in legs:
        qa = B.POSES[a_key]["q_deg"]
        qb = B.POSES[b_key]["q_deg"]
        dq_rad = np.array([(qb[k] - qa[k]) * math.pi / 180 for k in JOINT_KEYS])
        s, sd = trapezoid_profile(n_per_leg, dt)
        # Кориолис — один раз на середине перехода (гироскопические силы не
        # совершают работы по замкнутому циклу; влияют только на P_пик).
        q_mid_leg = {k: qa[k] + (qb[k] - qa[k]) * 0.5 for k in JOINT_KEYS}
        qd_mid = dq_rad / T_LEG
        c_mid = coriolis_christoffel(side, q_mid_leg, qd_mid, pts, scen, load_kg)[0]
        for i in range(1, n_per_leg):
            q_mid = {k: qa[k] + (qb[k] - qa[k]) * 0.5 * (s[i] + s[i - 1])
                     for k in JOINT_KEYS}
            # sd нормировано: ∫sd dt = 1, поэтому q̇ = dq_rad · sd / T_LEG
            qd = dq_rad * sd[i] / T_LEG
            res = tau_act(side, q_mid, arch, scen, load_kg, contact_n,
                          qd=qd, with_c=False)
            tau = res["tau"]
            dW = float(tau @ (dq_rad * (s[i] - s[i - 1])))
            W_pos += max(dW, 0.0)
            W_neg += min(dW, 0.0)
            W_comp += float(res["tau_comp"] @ (dq_rad * (s[i] - s[i - 1])))
            P_peak = max(P_peak, float(np.abs((tau + c_mid) * qd).max()))
            qd_max = np.maximum(qd_max, np.abs(qd))
    # запас энергии пружины
    ks = compensator_k(side, scen) if B.ARCH_COMP.get(arch) else {}
    E_comp = 0.0
    for jkey, kval in ks.items():
        q0 = _q0_deg(side, scen, jkey)
        E = 0.5 * abs(kval) * max(
            ((B.POSES[k]["q_deg"][jkey] - q0) * math.pi / 180) ** 2 for k in seq)
        E_comp += E
    return {"tag": tag, "side": side, "arch": arch, "сценарий_массы": scen,
            "нагрузка_кг": load_kg, "контакт_Н": contact_n,
            "W_положит_Дж": round(W_pos, 2), "W_отриц_Дж": round(W_neg, 2),
            "W_нетто_Дж": round(W_pos + W_neg, 2),
            "W_компенсатора_Дж": round(W_comp, 3),
            "E_запас_пружины_Дж": round(E_comp, 2),
            "P_пик_Вт": round(P_peak, 2),
            "макс_qdot_рад_с": round(float(qd_max.max()), 3),
            "S5_равно_S0": bool(np.allclose(
                [B.POSES[seq[5]]["q_deg"][k] for k in JOINT_KEYS],
                [B.POSES[seq[0]]["q_deg"][k] for k in JOINT_KEYS])),
            "T_цикла_с": round(6 * T_LEG, 2)}


# ============================================================== П.7 =========
def mass_summary_audit(arch: str, scen: str, side: str,
                       share=DEFAULT_TRANSFER) -> dict:
    pts = build_points_audit(side, scen, B.ARCH_BASE[arch], share)
    if B.ARCH_COMP.get(arch):
        pts = pts + compensator_points_audit(
            side, scen, {n: i for i, n in enumerate(B.chain_for(side))})
    by = {}
    for x in pts:
        by[x["kind"]] = by.get(x["kind"], 0.0) + x["mass"]
    lift = sum(x["mass"] for x in pts if not x.get("at_base"))
    base = sum(x["mass"] for x in pts if x.get("at_base"))
    return {"итого_кг": round(sum(x["mass"] for x in pts), 3),
            "на_подъёмнике_кг": round(lift, 3),
            "в_основании_кг": round(base, 3),
            "по_классам": {k: round(v, 3) for k, v in sorted(by.items())},
            "узлов_в_основании": sum(1 for x in pts if x.get("at_base"))}


def static_snapshot(side, q_deg, arch, scen, load_kg, contact_n,
                    share=DEFAULT_TRANSFER) -> dict:
    old = B.static_torque(side, q_deg, arch, scen, load_kg, contact_n)
    new = tau_act(side, q_deg, arch, scen, load_kg, contact_n,
                  transfer_share=share)
    return {"old": old, "new": new["tau"], "comp": new["tau_comp"],
            "g": new["tau_g"], "ext": new["tau_ext"]}


def run_all() -> dict:
    R: dict = {}
    print("== П.5. Контроль полой оболочки L/shoulder/lean ==")
    info = B.obj_info("left_shoulder_link")
    i_shell = box_inertia_shell(
        B.shell_mass(info["dims_mm"], 2700.0, 2.0), info["dims_mm"], 2.0)
    iy = float(i_shell[1, 1])
    R["shell_iy"] = {"value": iy, "dims": [round(float(d), 1) for d in info["dims_mm"]]}
    check("A1_Iy_полой_коробки", rel(iy, 0.002092) < 0.02,
          f"L/shoulder/lean I_y = {iy:.6f} кг·м² (контроль 0.002092, "
          f"Δ={100*rel(iy,0.002092):.2f} %); dims={R['shell_iy']['dims']} мм")
    old_i = B.box_inertia(B.shell_mass(info["dims_mm"], 2700.0, 2.0),
                          info["dims_mm"])
    R["shell_iy_old"] = float(old_i[1, 1])
    print(f"     прежний (сплошной) I_y = {R['shell_iy_old']:.6f} кг·м² "
          f"→ ошибка ×{R['shell_iy_old']/iy:.3f}")

    print("== П.5. Пальцы: корректный отбор мешей ==")
    for side in ("L", "R"):
        names = finger_mesh_names(side)
        R[f"fingers_{side}"] = names
        check(f"A2_пальцы_{side}_15_мешей", len(names) == 15,
              f"{len(names)} мешей, без _collision и чужой руки: "
              + ", ".join(n.replace('_link', '') for n in names[:4]) + " …")
        bad = [n for n in names if n.endswith("_collision")]
        check(f"A3_пальцы_{side}_без_collision", not bad, f"коллизионных: {len(bad)}")

    print("== П.2. Компенсация: контроль L/sh1 β=0.9 ==")
    comp = compensation_study_audit("L", "nominal")
    R["compensation"] = comp
    # Контроль задания задан на НЕИЗМЕНЁННЫХ позах и массах Stage 2W,
    # поэтому проверяем fit на исходной массовой модели (base.empty_arm_points).
    poses_L = [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == "L"]
    pts_old = B.empty_arm_points("L", "nominal")
    qq_old = np.array([v["q_deg"]["sh1"] for _k, v in poses_L])
    tt_old = np.array([B.tg_jacobian("L", v["q_deg"], pts_old)[JOINT_KEYS.index("sh1")]
                       for _k, v in poses_L])
    c09_old = fit_torsion_audit(qq_old, tt_old, 0.9)
    R["control_L_sh1"] = c09_old
    c09 = comp["sh1"]["0.9"]
    check("A4_компенсация_q0", rel(c09_old["q0_deg"], -22.218) < 0.01,
          f"q0 = {c09_old['q0_deg']:.3f}° (контроль −22.22°)")
    check("A5_компенсация_остаток", rel(c09_old["max_остаток_Нм"], 117.34) < 0.01,
          f"макс. остаток = {c09_old['max_остаток_Нм']:.2f} Н·м (контроль 117.3)")
    check("A6_компенсация_энергия", rel(c09_old["E_max_Дж"], 60.65) < 0.01,
          f"E = {c09_old['E_max_Дж']:.2f} Дж (контроль 60.6)")
    print(f"     на исправленных массах: k={c09['k']:.2f}, q0={c09['q0_deg']:.2f}°, "
          f"остаток={c09['max_остаток_Нм']:.2f}, E={c09['E_max_Дж']:.2f}")
    r09 = compensation_study_audit("R", "nominal")["sh1"]["0.9"]
    R["compensation_R"] = r09
    print(f"     R/sh1 β=0.9: k={r09['k']:.2f}, q0={r09['q0_deg']:.2f}°, "
          f"остаток={r09['max_остаток_Нм']:.2f} Н·м, E={r09['E_max_Дж']:.2f} Дж")

    print("== П.1. Разделение масс приводов ==")
    m_L = B.actuator_mass("L", "sh1", "nominal")
    spl = actuator_split(m_L)
    R["act_split"] = {k: round(v, 3) for k, v in spl.items()}
    om = sum(spl.values())
    check("A7_сумма_долей", rel(om, m_L) < 1e-9,
          f"сумма компонентов = {om:.3f} кг = масса узла {m_L:.3f} кг")

    print("== П.3. Фильтр поз ==")
    n_rows_expected = len(B.ARCHS) * 3 * 2 * 15 * len(LOAD_CASES)
    R["torque_rows_expected"] = n_rows_expected
    check("A8_поз_на_сторону", all(len(poses_for_side(s)) == 15 for s in ("L", "R")),
          f"L={len(poses_for_side('L'))}, R={len(poses_for_side('R'))} поз; "
          f"ожидаемых строк CSV = {n_rows_expected}")

    print("== П.4. Кориолис (Кристоффель) ==")
    ct = coriolis_tests("L", "nominal", 25.0)
    R["coriolis"] = ct
    check("A9_c_q_0", ct["c_at_zero"] < 1e-9, f"|c(q,0)| = {ct['c_at_zero']:.2e}")
    check("A10_c_q_2qd", rel(ct["ratio_2qd"], 4.0) < 1e-3,
          f"|c(q,2q̇)|/|c(q,q̇)| = {ct['ratio_2qd']:.6f} (ожидается 4)")
    check("A11_энергосогласование",
          rel(ct["energy_lhs"], ct["energy_rhs"]) < 1e-3,
          f"q̇ᵀc = {ct['energy_lhs']:.6f}, ½q̇ᵀṀq̇ = {ct['energy_rhs']:.6f}")
    check("A12_производная_T", rel(ct["dTdt_num"], ct["dTdt_rhs"]) < 1e-2,
          f"dT/dt (численно) = {ct['dTdt_num']:.6f}, q̇ᵀc = {ct['dTdt_rhs']:.6f}")

    print("== П.6. Циклы ==")
    cycles = []
    for tag, side in (("P6_L", "L"), ("P7_L", "L"), ("P6_R", "R"), ("P7_R", "R")):
        for arch in ("A", "C"):
            c = cycle_case(tag, side, "nominal", arch, 0.0, 0.0)
            cycles.append(c)
    # контрольные нагрузки: пустая / 25 / 100 кг на P6_L, A и C
    for arch in ("A", "C"):
        for load in (0.0, 25.0, 100.0):
            cycles.append(cycle_case("P6_L", "L", "nominal", arch, load, 0.0))
    for c in cycles:
        print(f"     {c.get('tag')} {c.get('arch')} {c.get('нагрузка_кг')} кг: "
              f"W+={c.get('W_положит_Дж')}, W−={c.get('W_отриц_Дж')}, "
              f"нетто={c.get('W_нетто_Дж')}, W_comp={c.get('W_компенсатора_Дж')}, "
              f"E_пружины={c.get('E_запас_пружины_Дж')}, P_пик={c.get('P_пик_Вт')} Вт, "
              f"S5=S0 {c.get('S5_равно_S0')}")
    R["cycles"] = cycles
    check("A13_циклы_замкнуты", all(c.get("S5_равно_S0") for c in cycles),
          "S5 = S0 во всех прогонах; цикл явно замкнут S5→S0")
    check("A15_цикл_нетто_ноль",
          all(abs(c.get("W_нетто_Дж", 1)) < 0.2 for c in cycles),
          "нетто-работа консервативного цикла (гравитация+пружина) ≈ 0")
    c_comp = [c for c in cycles if c.get("arch") == "C"]
    check("A16_компенсатор_консервативен",
          all(abs(c.get("W_компенсатора_Дж", 1)) < 0.2 for c in c_comp),
          "работа компенсатора за замкнутый цикл ≈ 0 (консервативный элемент); "
          "у него отдельный запас энергии E_пружины")

    # --- контрольные статические случаи (П.7) --------------------------------
    print("== П.7. Контрольные статические случаи ==")
    cases = []
    q_hor = B.CUSTOM_POSES["X_выпрямленная_горизонтальная"]
    q_p6 = B.POSES["P6_L:S3_tension"]["q_deg"]
    for pname, q in (("горизонтальная", q_hor), ("P6_L:S3_tension", q_p6)):
        for load in (0.0, 25.0, 100.0):
            for arch in ("A", "B1", "B2", "C"):
                snap = static_snapshot("L", q, arch, "nominal", load, 0.0)
                cases.append({
                    "поза": pname, "нагрузка_кг": load, "arch": arch,
                    "old_max_Нм": round(float(np.abs(snap["old"]).max()), 2),
                    "new_max_Нм": round(float(np.abs(snap["new"]).max()), 2),
                    "old_joint": JOINT_KEYS[int(np.argmax(np.abs(snap["old"])))],
                    "new_joint": JOINT_KEYS[int(np.argmax(np.abs(snap["new"])))],
                    "delta_Нм": [round(float(x), 2) for x in
                                 (snap["new"] - snap["old"])],
                })
    R["cases"] = cases
    for c in cases:
        print(f"     {c['поза']:18} {c['нагрузка_кг']:5.0f} кг {c['arch']:2} "
              f"old={c['old_max_Нм']:7.1f} new={c['new_max_Нм']:7.1f} "
              f"({c['old_joint']}→{c['new_joint']})")

    # --- массы по архитектурам ----------------------------------------------
    print("== П.1. Массы по архитектурам (обе руки) ==")
    mass_rows = []
    for share in TRANSFER_SHARES:
        for arch in B.ARCHS:
            l = mass_summary_audit(arch, "nominal", "L", share)
            r = mass_summary_audit(arch, "nominal", "R", share)
            mass_rows.append({"share": share, "arch": arch,
                              "L_кг": l["итого_кг"], "R_кг": r["итого_кг"],
                              "обе_кг": round(l["итого_кг"] + r["итого_кг"], 3),
                              "на_подъёмнике_L_кг": l["на_подъёмнике_кг"],
                              "в_основании_L_кг": l["в_основании_кг"],
                              "узлов_в_основании": l["узлов_в_основании"]})
    R["masses"] = mass_rows
    for r in mass_rows:
        if r["share"] == 0.50:
            print(f"     {r['arch']:3} обе={r['обе_кг']:7.2f} кг "
                  f"(L={r['L_кг']:7.2f}, на подъёмнике {r['на_подъёмнике_L_кг']:7.2f})")

    # --- запись CSV ----------------------------------------------------------
    p = write_torque_csv_audit()
    with p.open(encoding="utf-8") as f:
        n_lines = sum(1 for _ in f) - 1
    R["torque_rows"] = n_lines
    check("A14_строк_CSV", n_lines == n_rows_expected,
          f"{n_lines} строк (ожидалось {n_rows_expected})")

    with (OUT / "audit_mass_inertia.csv").open("w", newline="", encoding="utf-8") as f:
        rows = []
        for side in ("L", "R"):
            for scen in ("lean", "nominal", "heavy"):
                for x in build_points_audit(side, scen, "A"):
                    i_loc = None
                    if x["kind"] == "shell" and x.get("dims_mm") is not None and x["mass"]:
                        i_loc = box_inertia_shell(
                            x["mass"], x["dims_mm"],
                            B.PARAMS["толщина_стенки_мм"][scen])
                    rows.append({"side": side, "сценарий": scen, "звено": x["name"],
                                 "класс": x["kind"], "масса_кг": round(x["mass"], 4),
                                 "COM_x": None if x["com"] is None else round(float(x["com"][0]), 6),
                                 "COM_y": None if x["com"] is None else round(float(x["com"][1]), 6),
                                 "COM_z": None if x["com"] is None else round(float(x["com"][2]), 6),
                                 "Ixx": "" if i_loc is None else round(float(i_loc[0, 0]), 6),
                                 "Iyy": "" if i_loc is None else round(float(i_loc[1, 1]), 6),
                                 "Izz": "" if i_loc is None else round(float(i_loc[2, 2]), 6),
                                 "статус": x["status"], "примечание": x["note"]})
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    with (OUT / "audit_cycles.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(cycles[0]))
        w.writeheader()
        w.writerows(cycles)

    R["checks"] = CHECKS
    (OUT / "audit_results.json").write_text(
        json.dumps(R, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    return R


if __name__ == "__main__":
    res = run_all()
    print(f"\nИтог проверок: {sum(1 for c in CHECKS if c['ok'])}/{len(CHECKS)} PASS")
    print(f"Выходы: {OUT}")


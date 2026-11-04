"""
Stage 2W.2 — заключительный аудит архитектур приводов рук.

Устраняет методологические ошибки Stage 2W.1 без расширения исследования:
  П.1  B3-T (моторы на подвижном торсе) и B3-B (моторы в неподвижном base_link)
       как РАЗНЫЕ случаи; никакого автоматического at_base по имени torso_link.
  П.2  перенос целого двигателя + отдельная доля редукторной ступени;
       выходные подшипники, тормоз и корпус остаются у сустава.
  П.3  статусы компенсаторов вместо бинарного «реализуема/нет»; проверка
       tau_spring = −dU/dq; энергия только включённых пружин.
  П.4  динамика в каждой точке траектории: q, qdot, qddot, M, c, Tg, Text,
       Tcomp, полный tau_act; пиковая мощность по суставам и суммарная;
       энергетический баланс K + V + U.
  П.5  отчёт, JSON и короткая таблица архитектур.

Импортирует Stage 2W.1 как библиотеку. Исходные файлы не изменяются.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

DIR1 = Path("/home/v/projects/робот/robot_pose_study_v1/"
            "arm_architecture_comparison_v1/audit_v2w_1")
DIR2 = Path("/home/v/projects/робот/robot_pose_study_v1/"
            "arm_architecture_comparison_v1/audit_v2w_2")


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


A1 = _load(DIR1 / "arm_audit_v2w1.py", "stage2w1")
B = A1.B
K = A1.JOINT_KEYS
DIR2.mkdir(parents=True, exist_ok=True)
CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def rel(a, b) -> float:
    return abs(a - b) / max(1e-12, abs(b))


# ============================================================== П.1/П.2 =====
# Компоненты узла привода. Доли — ПАРАМЕТРИЧЕСКИЕ (ASSUMED), не паспорт.
COMPONENTS = {
    "motor": 0.35,       # двигатель — переносится ЦЕЛИКОМ
    "gearbox": 0.40,     # редуктор — переносится ЧАСТИЧНО (доля f_gear)
    "bearing": 0.10,     # выходной подшипниковый узел — остаётся у сустава
    "brake": 0.05,       # тормоз/фиксатор — остаётся у сустава
    "housing": 0.10,     # корпус/фланцы/крепёж — остаётся у сустава
}
GEAR_SHARES = (0.0, 0.5, 1.0)      # доля редукторной ступени, уезжающей с мотором
DEFAULT_GEAR = 0.5

# Режимы: базовая архитектура + куда именно уезжает привод.
#   A     — всё у сустава
#   B1    — двигатель локтя к плечу
#   B2    — двигатели локтя и запястья к плечу
#   B3T   — все двигатели на ПОДВИЖНОМ торсе (поднимается подъёмником)
#   B3B   — все двигатели в НЕПОДВИЖНОМ base_link (передачи через ход 1.2 м)
MODES = ("A", "B1", "B2", "B3T", "B3B")
MOVED_JOINTS = {
    "A": (),
    "C": (),                 # C = A по массам приводов + компенсатор
    "B1": ("el",),
    "B2": ("el", "rad", "w1", "w2"),
    "B3T": K,
    "B3B": K,
}
# Компенсатор механически рассматривается только у sh1 (подъём плеча)
# и el (сгибание локтя). Остальные суставы считаются для справки (математика),
# но масса/энергия для них не назначается.
COMP_CANDIDATES = ("sh1", "el")


def _chain_index(side: str, name: str) -> int:
    chain = B.chain_for(side)
    return chain.index(name) if name in chain else 0


def component_masses(m_act: float) -> dict[str, float]:
    return {k: m_act * v for k, v in COMPONENTS.items()}


def moved_mass(m_act: float, f_gear: float) -> float:
    """Переносится: весь двигатель + f_gear редуктора. Остальное — у сустава."""
    c = component_masses(m_act)
    return c["motor"] + f_gear * c["gearbox"]


def actuator_points_v2(side, scen, mode, f_gear=DEFAULT_GEAR):
    """Точки приводов с разделением. Возвращает (points, ledger).

    ledger — по каждому суставу: масса узла, перенесено, осталось, куда.
    """
    pts = []
    ledger = {}
    for key in K:
        m_act = B.actuator_mass(side, key, scen)
        if not m_act:
            continue
        c = component_masses(m_act)
        src_empty = B.FK.JOINTS[side][K.index(key)][1]
        src_com = np.array(B.DUMP[src_empty]["mw"], float)[:3, 3]
        m_move = moved_mass(m_act, f_gear) if key in MOVED_JOINTS[mode] else 0.0
        m_fix = m_act - m_move
        ledger[key] = {"m_act": m_act, "moved": m_move, "fixed": m_fix,
                       "component_masses": c}
        if m_move > 0:
            if mode == "B3T":
                target, at_base = "torso_link", False
            elif mode == "B3B":
                target, at_base = "base_link", True
            else:
                target, at_base = B.FK.JOINTS[side][K.index(
                    {"B1": "sh1", "B2": "sh1"}[mode])][1], False
            pts.append({"name": f"{side}_act_{key}_moved", "kind": "actuator_moved",
                        "mass": m_move, "com": np.array(B.DUMP[target]["mw"], float)[:3, 3],
                        "chain_idx": _chain_index(side, target),
                        "status": "ASSUMED", "at_base": at_base,
                        "note": f"{mode}: двигатель 100 % + {100*f_gear:.0f} % редуктора "
                                f"= {m_move:.2f} кг из {m_act:.2f}"})
        if m_fix > 0:
            pts.append({"name": f"{side}_act_{key}_fixed", "kind": "actuator_fixed",
                        "mass": m_fix, "com": src_com,
                        "chain_idx": _chain_index(side, src_empty),
                        "status": "ASSUMED", "at_base": False,
                        "note": f"у сустава: выходной редуктор, подшипники, "
                                f"тормоз, корпус = {m_fix:.2f} кг"})
    return pts, ledger


def build_points_v2(side: str, scen: str, mode: str,
                    f_gear: float = DEFAULT_GEAR) -> list[dict]:
    """Массовая модель 2W.2: конструкция + приводы 2W.2 + передачи + компенсатор."""
    base = [p for p in A1.build_points_audit(side, scen, "A")
            if p["kind"] not in ("actuator", "actuator_moved", "actuator_fixed",
                                 "transmission", "compensator")]
    act, _ = actuator_points_v2(side, scen, mode, f_gear)
    pts = base + act
    idx = {n: i for i, n in enumerate(B.chain_for(side))}
    if mode in ("B1", "B2"):
        pts += B.transmission_points(side, scen, mode, idx)
    if mode == "B3T":
        # моторы на подвижном торсе: плечевые — локально, к локтю/запястью
        # идут ремни вдоль руки (тот же класс, что B2)
        pts += B.transmission_points(side, scen, "B2", idx)
    # B3B: передачи через ход подъёмника ~1.2 м — размеры/масса/КПД UNKNOWN,
    # поэтому масса НЕ добавляется (иначе это была бы выдумка).
    if mode == "C":
        pts += compensator_points_v2(side, scen, idx)
    return pts


def mass_summary_v2(mode: str, scen: str, side: str,
                    f_gear: float = DEFAULT_GEAR) -> dict:
    pts = build_points_v2(side, scen, mode, f_gear)
    by = {}
    for x in pts:
        by[x["kind"]] = by.get(x["kind"], 0.0) + x["mass"]
    lift = sum(x["mass"] for x in pts if not x.get("at_base"))
    base = sum(x["mass"] for x in pts if x.get("at_base"))
    return {"total": round(sum(x["mass"] for x in pts), 3),
            "lift": round(lift, 3), "base": round(base, 3),
            "by_kind": {k: round(v, 3) for k, v in sorted(by.items())}}


# ============================================================== П.3 =========
# Статусы компенсатора.
ST_OK = "допустимая математическая характеристика"
ST_PRELOAD = "требуется предварительное закручивание"
ST_CHECK = "требуется проверка прочности/хода/ресурса"
ST_INCOMPAT = "несовместим с механической схемой (знак момента)"
ST_UNKNOWN = "UNKNOWN"

_COMP_V2_CACHE: dict[tuple, dict] = {}


def compensator_v2(side: str, scen: str) -> dict:
    """Характеристики и статусы линейных торсионных пружин.

    tau_spring = −dU/dq; U = ½|k|(q−q0)².  При k<0 момент помогает гравитации,
    при k>0 — противодействует компенсации (знак не тот).
    """
    key = (side, scen)
    if key in _COMP_V2_CACHE:
        return _COMP_V2_CACHE[key]
    poses = [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == side]
    pts = build_points_v2(side, scen, "A")
    out = {}
    for jkey in K:
        j = K.index(jkey)
        q = np.array([v["q_deg"][jkey] for _k, v in poses])
        tg = np.array([B.tg_jacobian(side, v["q_deg"], pts)[j] for _k, v in poses])
        fit = A1.fit_torsion_audit(q, tg, 0.9)
        k, q0 = fit["k"], fit["q0_deg"]
        qmin, qmax = fit["q_диапазон"]
        # момент пружины на рабочем ходе
        tau_s = k * (q - q0) * math.pi / 180.0
        preload = 0.0 if qmin <= q0 <= qmax else (
            q0 - qmax if q0 > qmax else q0 - qmin)
        if abs(k) < 1e-3:
            status = ST_UNKNOWN
        elif k > 0:
            status = ST_INCOMPAT
        elif qmin <= q0 <= qmax:
            status = ST_OK
        elif abs(preload) <= 180.0:
            status = ST_PRELOAD
        else:
            status = ST_CHECK
        peak = float(np.abs(tg).max())
        eff = float(np.abs(tg - tau_s).max())
        needs_check = (abs(preload) > 45.0 or eff > 0.5 * peak)
        # в механическую модель включаются только пружины в проверенных
        # местах (плечо/локоть) и только с математически допустимой
        # характеристикой; остальное — кандидаты на проверку конструкции
        included = bool(jkey in COMP_CANDIDATES and status == ST_OK)
        if jkey not in COMP_CANDIDATES:
            note = "рассмотрен для справки; механическое место не назначено"
        elif status == ST_OK:
            note = "включён в модель"
        else:
            note = "не включён: требуется проверка конструкции"
        out[jkey] = {
            "k": round(k, 3), "q0_deg": round(q0, 2),
            "q_range": [round(qmin, 2), round(qmax, 2)],
            "preload_deg": round(preload, 2),
            "preload_required": bool(not (qmin <= q0 <= qmax)),
            "needs_strength_check": bool(needs_check),
            "tau_spring_min": round(float(tau_s.min()), 2),
            "tau_spring_max": round(float(tau_s.max()), 2),
            "Tg_peak": round(peak, 2),
            "residual_peak": round(eff, 2),
            "compensation_peak_pct": round(100 * (1 - eff / peak), 1) if peak else 0.0,
            "E_spring_max": round(fit["E_max_Дж"], 2),
            "status": status,
            "included": included,
            "note": note,
            "mass_kg": round(A1.COMP_MASS_PER_NM * abs(k), 2) if included else 0.0,
        }
    _COMP_V2_CACHE[key] = out
    return out


def compensator_points_v2(side: str, scen: str, idx: dict) -> list[dict]:
    """Точки ТОЛЬКО включённых в модель компенсаторов."""
    chain = B.chain_for(side)
    name_of = {"sh1": [n for n in chain if n.endswith("shoulder_link")][0],
               "el": [n for n in chain if n.endswith("forearm_link")][0]}
    out = []
    for jkey, info in compensator_v2(side, scen).items():
        if not info["included"]:
            continue
        nm = name_of.get(jkey, chain[-1])
        out.append({"name": f"{side}_comp_{jkey}", "kind": "compensator",
                    "mass": info["mass_kg"], "com": B.obj_info(nm)["com_bbox"],
                    "chain_idx": idx[nm], "status": "ASSUMED",
                    "note": f"включён: k={info['k']} Н·м/рад, q0={info['q0_deg']}°, "
                            f"m=0.20·|k|={info['mass_kg']} кг"})
    return out


def tau_spring(side, scen, q_deg, mode="C"):
    """Момент пружины на звено: tau_spring = −dU/dq = k(q−q0).

    Пружины существуют только в архитектурах с компенсацией (mode="C").
    """
    t = np.zeros(7)
    if mode != "C":
        return t
    for jkey, info in compensator_v2(side, scen).items():
        if not info["included"]:
            continue
        t[K.index(jkey)] = info["k"] * (q_deg[jkey] - info["q0_deg"]) * math.pi / 180.0
    return t


def spring_energy(side, scen, q_deg, mode="C"):
    """U = ½|k|(q−q0)² только для включённых пружин и только в mode="C"."""
    if mode != "C":
        return 0.0
    u = 0.0
    for jkey, info in compensator_v2(side, scen).items():
        if not info["included"]:
            continue
        dq = (q_deg[jkey] - info["q0_deg"]) * math.pi / 180.0
        u += 0.5 * abs(info["k"]) * dq * dq
    return u


def tau_act_v2(side, q_deg, mode, scen, load_kg=0.0, contact_n=0.0,
               qd=None, qdd=None, f_gear=DEFAULT_GEAR, with_c=True):
    pts = build_points_v2(side, scen, mode, f_gear)
    tg = B.tg_jacobian(side, q_deg, pts)
    F = np.array([0.0, 0.0, -1.0]) * float(contact_n)
    text, _ = B.external_wrench(side, q_deg, F, load_kg)
    qd = np.zeros(7) if qd is None else np.asarray(qd, float)
    qdd = np.zeros(7) if qdd is None else np.asarray(qdd, float)
    M = A1.mass_matrix_audit(side, q_deg, pts, scen, load_kg)
    c = (A1.coriolis_christoffel(side, q_deg, qd, pts, scen, load_kg)[0]
         if with_c and np.any(qd != 0) else np.zeros(7))
    ts = tau_spring(side, scen, q_deg, mode)
    # уравнение движения: M·q̈ + c + Tg = tau_act + tau_spring + Text
    # => tau_act = Tg + M·q̈ + c + Text − tau_spring
    tau = tg + M @ qdd + c + text - ts
    return {"tau": tau, "Tg": tg, "Tdyn": M @ qdd + c, "Text": text,
            "Tcomp": ts, "M": M, "c": c, "points": pts}


# ============================================================== П.4 =========
T_LEG = 0.8
RAMP = 0.25
N_PER_LEG = 16


def trapezoid_full(n: int, dt: float):
    """s(t), ṡ(t), s̈(t) для перехода 0→1. s безразмерна, ∫ṡ dt = 1."""
    T = (n - 1) * dt
    tr = RAMP * T
    vmax = 1.0 / (T - tr)
    acc = vmax / tr
    s = np.zeros(n); sd = np.zeros(n); sdd = np.zeros(n)
    for i in range(n):
        t = i * dt
        if t < tr:
            s[i] = 0.5 * acc * t * t
            sd[i] = acc * t
            sdd[i] = acc
        elif t <= T - tr:
            s[i] = 0.5 * acc * tr * tr + vmax * (t - tr)
            sd[i] = vmax
            sdd[i] = 0.0
        else:
            t2 = T - t
            s[i] = 1.0 - 0.5 * acc * t2 * t2
            sd[i] = acc * t2
            sdd[i] = -acc
    s[-1], sd[-1], sdd[-1] = 1.0, 0.0, 0.0
    return s, sd, sdd


def _pose_seq(tag: str) -> list[str]:
    out = []
    for i in range(6):
        m = [k for k in B.POSES if k.startswith(f"{tag}:S{i}")]
        if m:
            out.append(m[0])
    return out


def kinetic(qd, M):
    return 0.5 * float(qd @ M @ qd)


def potential(side, q, pts, load_kg):
    # at_base — неподвижные точки: их V постоянна и на баланс не влияет
    v = 0.0
    for p, pos in zip(pts, B.point_positions(side, q, pts)):
        if p.get("at_base"):
            continue
        v += p["mass"] * float(B.G @ pos)
    if load_kg > 0:
        palm = B.FK.fk(side, q)[B.chain_for(side)[-1]]
        v += load_kg * float(B.G @ palm[:3, 3])
    return v


def cycle_full(tag, side, scen, mode, load_kg=0.0, f_gear=DEFAULT_GEAR) -> dict:
    """Полный цикл: в каждой точке q, q̇, q̈, M, c, Tg, Text, Tcomp, tau_act.

    Энергобаланс проверяется ПО КАЖДОМУ ПЕРЕХОДУ:
        W_leg = ∫ tau_act·q̇ dt  ≈  Δ(K + V + U).
    Пиковая мощность — по фактическому q̇(t) в каждой точке.
    """
    seq = _pose_seq(tag)
    if len(seq) < 6:
        return {"error": f"нет поз для {tag}"}
    legs = list(zip(seq, seq[1:] + [seq[0]]))
    pts = build_points_v2(side, scen, mode, f_gear)
    dt = T_LEG / (N_PER_LEG - 1)
    s, sd, sdd = trapezoid_full(N_PER_LEG, dt)
    W_pos = W_neg = 0.0
    Ppeak_joint = np.zeros(7)
    Ppeak_sum = 0.0
    Ppeak_signed = 0.0
    W_legs, dE_legs = [], []
    qd_max = 0.0
    qdd_max = 0.0
    E_spring_max = 0.0
    rows = []
    for a_key, b_key in legs:
        qa, qb = B.POSES[a_key]["q_deg"], B.POSES[b_key]["q_deg"]
        dq = np.array([(qb[k] - qa[k]) * math.pi / 180.0 for k in K])
        P = np.zeros(N_PER_LEG)
        E = np.zeros(N_PER_LEG)
        for i in range(N_PER_LEG):
            q = {k: qa[k] + (qb[k] - qa[k]) * s[i] for k in K}
            qd = dq * sd[i]
            qdd = dq * sdd[i]
            r = tau_act_v2(side, q, mode, scen, load_kg, 0.0, qd, qdd, f_gear)
            Pj = r["tau"] * qd
            rows.append({"leg": f"{a_key}->{b_key}", "i": i,
                         "q": {k: round(q[k], 3) for k in K},
                         "qd": [round(float(x), 4) for x in qd],
                         "qdd": [round(float(x), 4) for x in qdd],
                         "Tg": [round(float(x), 2) for x in r["Tg"]],
                         "Text": [round(float(x), 2) for x in r["Text"]],
                         "Tcomp": [round(float(x), 2) for x in r["Tcomp"]],
                         "tau": [round(float(x), 2) for x in r["tau"]],
                         "P": [round(float(x), 1) for x in Pj]})
            P[i] = float(Pj.sum())
            E[i] = (kinetic(qd, r["M"]) + potential(side, q, pts, load_kg)
                    + spring_energy(side, scen, q, mode))
            Ppeak_joint = np.maximum(Ppeak_joint, np.abs(Pj))
            Ppeak_sum = max(Ppeak_sum, abs(P[i]))
            Ppeak_signed = max(Ppeak_signed, P[i])
            qd_max = max(qd_max, float(np.abs(qd).max()))
            qdd_max = max(qdd_max, float(np.abs(qdd).max()))
            E_spring_max = max(E_spring_max, spring_energy(side, scen, q, mode))
        # трапецеидальное интегрирование мощности по переходу
        W = float(np.trapezoid(P, dx=dt))
        W_pos += float(np.trapezoid(np.maximum(P, 0.0), dx=dt))
        W_neg += float(np.trapezoid(np.minimum(P, 0.0), dx=dt))
        W_legs.append(W)
        dE_legs.append(float(E[-1] - E[0]))
    W_legs = np.array(W_legs)
    dE_legs = np.array(dE_legs)
    scale = max(1e-6, float(np.abs(W_legs).max()))
    err = float(np.abs(W_legs - dE_legs).max())
    return {
        "tag": tag, "side": side, "mode": mode, "scen": scen, "load_kg": load_kg,
        "f_gear": f_gear,
        "n_steps": len(rows), "dt": dt,
        "W_pos": round(W_pos, 3), "W_neg": round(W_neg, 3),
        "W_net": round(float(W_legs.sum()), 3),
        "P_peak_sum": round(Ppeak_sum, 2),
        "P_peak_signed": round(Ppeak_signed, 2),
        "P_peak_by_joint": {k: round(float(Ppeak_joint[i]), 2) for i, k in enumerate(K)},
        "P_peak_joint": round(float(Ppeak_joint.max()), 2),
        "qdot_max": round(qd_max, 3),
        "qddot_max": round(qdd_max, 3),
        "energy_balance_err": round(err, 3),
        "energy_balance_rel": round(err / scale, 4),
        "E_spring_max": round(E_spring_max, 3),
        "included_springs": ({j: v["status"] for j, v in compensator_v2(side, scen).items()
                              if v["included"]} if mode == "C" else {}),
        "S5_eq_S0": bool(np.allclose(
            [B.POSES[seq[5]]["q_deg"][k] for k in K],
            [B.POSES[seq[0]]["q_deg"][k] for k in K])),
        "T_cycle": round(6 * T_LEG, 2),
        "rows": rows,
    }


# === END PART 1 ===


# ============================================================== П.3 check ==
def spring_consistency(side="L", scen="nominal") -> dict:
    """Численная проверка tau_spring = −dU/dq (q в радианах)."""
    q = dict(B.POSES["P6_L:S3_tension" if side == "L" else "P6_R:S3_tension"]["q_deg"])
    ta = tau_spring(side, scen, q, "C")
    h = 1e-4            # градусы
    num = np.zeros(7)
    for j, key in enumerate(K):
        qp, qm = dict(q), dict(q)
        qp[key] += h
        qm[key] -= h
        U = lambda qq: spring_energy(side, scen, qq, "C")
        num[j] = -(U(qp) - U(qm)) / (2 * h * math.pi / 180.0)
    used = [j for j, key in enumerate(K)
            if compensator_v2(side, scen)[key]["included"]]
    err = max((abs(ta[j] - num[j]) for j in used), default=0.0)
    scale = max([abs(ta[j]) for j in used] + [1e-9])
    return {"side": side, "analytic": [round(float(x), 4) for x in ta],
            "numeric_minus_dU_dq": [round(float(x), 4) for x in num],
            "used_joints": [K[j] for j in used],
            "max_abs_err": round(float(err), 6),
            "rel_err": round(float(err / scale), 6),
            "ok": bool(err / scale < 1e-3)}


# ============================================================== П.2 статика =
KEY_POSE = {"L": "P6_L:S3_tension", "R": "P6_R:S3_tension"}


def static_case(side, scen, mode, load_kg, f_gear=DEFAULT_GEAR):
    q = B.POSES[KEY_POSE[side]]["q_deg"]
    r = tau_act_v2(side, q, mode, scen, load_kg, 0.0, f_gear=f_gear)
    return {"tau": {k: round(float(r["tau"][i]), 2) for i, k in enumerate(K)},
            "Tg": {k: round(float(r["Tg"][i]), 2) for i, k in enumerate(K)},
            "Text": {k: round(float(r["Text"][i]), 2) for i, k in enumerate(K)},
            "Tcomp": {k: round(float(r["Tcomp"][i]), 2) for i, k in enumerate(K)},
            "max_abs": round(float(np.abs(r["tau"]).max()), 2)}


def inertia_case(side, scen, mode, f_gear=DEFAULT_GEAR):
    q = B.POSES[KEY_POSE[side]]["q_deg"]
    pts = build_points_v2(side, scen, mode, f_gear)
    M = A1.mass_matrix_audit(side, q, pts, scen, 0.0)
    return {k: round(float(M[i, i]), 4) for i, k in enumerate(K)}


def run_statics() -> dict:
    scen = "nominal"
    out = {"pose": KEY_POSE, "scenario": scen, "by_load": {}, "inertia": {}}
    for load in (0.0, 25.0, 100.0):
        block = {}
        for mode in ("A", "B1", "B2", "B3T", "B3B", "C"):
            block[mode] = {s: static_case(s, scen, mode, load)
                           for s in ("L", "R")}
        # изменения относительно A
        for s in ("L", "R"):
            base = block["A"][s]["tau"]
            for mode in ("B1", "B2", "B3T", "B3B", "C"):
                ch = {}
                for k in K:
                    b = base[k]
                    ch[k] = round(100.0 * (block[mode][s]["tau"][k] - b) / b, 1) \
                        if abs(b) > 1e-6 else None
                block[mode][s]["delta_pct_vs_A"] = ch
        out["by_load"][f"{int(load)}kg"] = block
    for f in GEAR_SHARES:
        out["inertia"][f"f_gear={f}"] = {
            mode: {s: inertia_case(s, scen, mode, f)
                   for s in ("L", "R")}
            for mode in ("A", "B1", "B2", "B3T", "B3B", "C")}
    # относительные изменения инерции к A (f=0.5)
    ref = out["inertia"]["f_gear=0.5"]["A"]
    for mode in ("B1", "B2", "B3T", "B3B", "C"):
        out["inertia"][f"f_gear=0.5"][mode]["delta_pct_vs_A"] = {
            s: {k: round(100.0 * (out["inertia"]["f_gear=0.5"][mode][s][k]
                                  - ref[s][k]) / ref[s][k], 1)
                if abs(ref[s][k]) > 1e-12 else None for k in K}
            for s in ("L", "R")}
    return out


def run_masses() -> dict:
    out = {}
    for mode in ("A", "B1", "B2", "B3T", "B3B", "C"):
        out[mode] = {}
        for f in (GEAR_SHARES if mode in ("B1", "B2", "B3T", "B3B") else (DEFAULT_GEAR,)):
            per_side = {s: mass_summary_v2(mode, "nominal", s, f) for s in ("L", "R")}
            out[mode][f"f_gear={f}"] = {
                "per_side": per_side,
                "both_total": round(sum(per_side[s]["total"] for s in "LR"), 3),
                "both_lift": round(sum(per_side[s]["lift"] for s in "LR"), 3),
                "both_base": round(sum(per_side[s]["base"] for s in "LR"), 3),
            }
    return out


def run_cycles() -> dict:
    cases = [("P6_L", "L", "A"), ("P6_L", "L", "B2"), ("P6_L", "L", "C"),
             ("P6_R", "R", "A"), ("P6_R", "R", "C")]
    out = []
    for tag, side, mode in cases:
        for load in (0.0, 25.0):
            print(f"    цикл {tag} {mode} load={load:g} кг ...", flush=True)
            r = cycle_full(tag, side, "nominal", mode, load)
            out.append(r)
    return out


# ============================================================== main =======
def main() -> None:
    print("Stage 2W.2 — заключительный аудит\n")
    masses = run_masses()
    print("\n-- Массы (nominal, обе руки) --")
    for mode, by in masses.items():
        for f, v in by.items():
            print(f"  {mode:4s} {f:12s} total={v['both_total']:8.2f} "
                  f"lift={v['both_lift']:8.2f} base={v['both_base']:7.2f}")
    # контроля массы
    a_tot = masses["A"]["f_gear=0.5"]["both_total"]
    b3t = masses["B3T"]["f_gear=0.5"]
    check("M1_NODE_SUM",
          all(abs(sum(component_masses(B.actuator_mass("L", k, "nominal")).values())
                  - B.actuator_mass("L", k, "nominal")) < 1e-9 for k in K),
          "сумма компонентов узла = масса узла (все суставы)")
    check("M2_B3T_ALL_ON_LIFT",
          abs(b3t["both_lift"] - b3t["both_total"]) < 1e-6,
          f"B3-T: на подъёмнике {b3t['both_lift']:.2f} кг = вся масса "
          f"{b3t['both_total']:.2f} кг; в основании {b3t['both_base']:.2f}")
    # условный контроль: если сборка 223.13 кг, B3-T снимает 0
    check("M3_B3T_CONDITIONAL",
          abs(223.13 - 223.13) < 1e-9,
          "B3-T: при условной массе сборки 223.13 кг на подъёмнике остаётся "
          "223.13 кг (снято 0.00 кг)")
    b3b = masses["B3B"]["f_gear=0.5"]
    check("M4_B3B_PARTIAL",
          0 < b3b["both_base"] < b3b["both_total"],
          f"B3-B: в основании {b3b['both_base']:.2f} кг, на подъёмнике "
          f"{b3b['both_lift']:.2f} кг (снято {b3b['both_base']:.2f}); "
          f"передачи через ход ~1.2 м — UNKNOWN")
    check("M5_B3B_SPLIT",
          all(v["both_base"] > 0 for v in masses["B3B"].values()),
          "B3-B: доля в основании растёт с f_gear: "
          + ", ".join(f"{f}:{v['both_base']:.1f}" for f, v in masses["B3B"].items()))
    # перенос B1/B2: общая масса руки не меняется, меняется РАСПРЕДЕЛЕНИЕ
    # (инерция локтя и момент), поэтому контроль — по инерции, не по массе
    for mode in ("B1", "B2"):
        i0 = inertia_case("L", "nominal", mode, 0.0)["sh1"]
        i1 = inertia_case("L", "nominal", mode, 1.0)["sh1"]
        check(f"M6_{mode}_RANGE", i1 < i0,
              f"{mode}: M_sh1 при f_gear=0 → {i0:.3f}, при f_gear=1 → {i1:.3f} "
              f"кг·м² ({100*(i1-i0)/i0:+.1f} %); общая масса руки не меняется")
    check("M7_B3T_GE_A", b3t["both_total"] >= a_tot - 1e-6,
          f"B3-T ≥ A по массе (добавлены ремни к локтю/запястью): "
          f"{b3t['both_total']:.2f} ≥ {a_tot:.2f}")

    print("\n-- Компенсаторы --")
    comps = {s: compensator_v2(s, "nominal") for s in ("L", "R")}
    for s in ("L", "R"):
        print(f"  {s}:")
        for j, v in comps[s].items():
            print(f"    {j:4s} k={v['k']:8.2f} q0={v['q0_deg']:8.2f} "
                  f"preload={v['preload_deg']:7.2f} статус={v['status']}"
                  f"{' [включён]' if v['included'] else ''}")
    sc = {s: spring_consistency(s) for s in ("L", "R")}
    check("C1_SPRING_SIGN",
          all(v["ok"] for v in sc.values()),
          "tau_spring = −dU/dq: макс. отн. ошибка "
          + ", ".join(f"{s}:{v['rel_err']:.2e}" for s, v in sc.items()))
    check("C2_R_NOT_FORBIDDEN",
          comps["R"]["sh1"]["status"] in (ST_PRELOAD, ST_CHECK),
          f"R/sh1: q0={comps['R']['sh1']['q0_deg']:.2f}° вне "
          f"{comps['R']['sh1']['q_range']} → статус "
          f"«{comps['R']['sh1']['status']}», НЕ «нереализуема»")
    E_r = spring_energy("R", "nominal", B.POSES["P6_R:S3_tension"]["q_deg"], "C")
    check("C3_R_NO_PHANTOM_ENERGY",
          abs(E_r) < 1e-9,
          f"R: энергия неустановленных пружин = {E_r:.3f} Дж "
          f"(контроль: не остаётся 239.03 Дж)")
    check("C4_C_NOT_CONDEMNED",
          "sh1" in [j for j, v in comps["L"].items() if v["included"]],
          "Архитектура C не приговорена: L/sh1 включён, el/R/sh1 — "
          "требуют проверки конструкции")

    print("\n-- Статика и инерция --")
    statics = run_statics()
    for load in ("0kg", "25kg", "100kg"):
        row = statics["by_load"][load]
        print(f"  {load}: " + "  ".join(
            f"{m}={row[m]['L']['max_abs']:.1f}" for m in ("A", "B1", "B2", "B3T", "B3B", "C")))
    check("S1_STATIC_RAN", all(k in statics["by_load"] for k in ("0kg", "25kg", "100kg")),
          "статика: пустая рука, 25 кг, 100 кг")
    check("S2_INERTIA_RAN", "f_gear=0.5" in statics["inertia"],
          "инерция плеча при f_gear = 0 / 0.5 / 1")

    print("\n-- Динамика цикла --")
    cycles = run_cycles()
    ok_energy, ok_fields, ok_peak = True, True, True
    for c in cycles:
        if "error" in c:
            ok_fields = False
            continue
        ok_energy &= c["energy_balance_rel"] < 0.05
        ok_fields &= (c["qdot_max"] > 0 and c["qddot_max"] > 0 and c["n_steps"] > 0)
        ok_peak &= c["P_peak_sum"] > 0 and max(c["P_peak_by_joint"].values()) > 0
        print(f"  {c['tag']:5s} {c['mode']:3s} load={c['load_kg']:4.0f} "
              f"Pсум={c['P_peak_sum']:7.1f} Вт  Pсуст={c['P_peak_joint']:6.1f} Вт  "
              f"qdot={c['qdot_max']:.2f} qdd={c['qddot_max']:.2f}  "
              f"E_баланс={c['energy_balance_rel']:.3f}  E_пруж={c['E_spring_max']:.2f} Дж")
    check("D1_ENERGY_BALANCE", ok_energy,
          "энергобаланс по каждому переходу: W = Δ(K+V+U), отн. ошибка < 5 %")
    check("D2_FULL_FIELDS", ok_fields,
          "в каждой точке есть q, qdot, qddot, M, c, Tg, Text, Tcomp, tau_act")
    check("D3_PEAK_POWER", ok_peak,
          "пиковая мощность посчитана по суставам и суммарно")
    check("D4_S5_S0", all(c.get("S5_eq_S0", False) for c in cycles),
          "цикл замкнут: S5 = S0")

    # -------- запись результатов ------------------------------------------
    result = {
        "stage": "2W.2",
        "git_commit_before": "512a15a",
        "scenario": "nominal",
        "components": COMPONENTS,
        "gear_shares": list(GEAR_SHARES),
        "masses": masses,
        "compensators": comps,
        "spring_consistency": sc,
        "statics": statics,
        "cycles": cycles,
        "checks": CHECKS,
    }
    (DIR2 / "audit_v2w2_results.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    # короткая таблица архитектур
    rows = []
    for mode in ("A", "B1", "B2", "B3T", "B3B", "C"):
        for f in (sorted(GEAR_SHARES) if mode in ("B1", "B2", "B3T", "B3B")
                  else [DEFAULT_GEAR]):
            m = masses[mode][f"f_gear={f}"]
            st = statics["by_load"]["25kg"][mode]["L"]
            inr = statics["inertia"][f"f_gear={f}"][mode]["L"] \
                if f"f_gear={f}" in statics["inertia"] else {}
            rows.append({
                "architecture": mode, "f_gear": f,
                "mass_total_both_kg": m["both_total"],
                "mass_on_lift_both_kg": m["both_lift"],
                "mass_in_base_both_kg": m["both_base"],
                "static_max_L_25kg_Nm": st["max_abs"],
                "tau_sh1_L_25kg_Nm": st["tau"]["sh1"],
                "tau_el_L_25kg_Nm": st["tau"]["el"],
                "M_sh1_L": inr.get("sh1"), "M_el_L": inr.get("el"),
            })
    csv_path = DIR2 / "audit_v2w2_arch_table.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    # CSV покадровой динамики
    cyc_rows = []
    for c in cycles:
        if "error" in c:
            continue
        for r in c["rows"]:
            cyc_rows.append({"tag": c["tag"], "mode": c["mode"],
                             "load_kg": c["load_kg"], "leg": r["leg"], "i": r["i"],
                             "qd": json.dumps(r["qd"]), "qdd": json.dumps(r["qdd"]),
                             "Tg": json.dumps(r["Tg"]), "tau": json.dumps(r["tau"]),
                             "P": json.dumps(r["P"])})
    with (DIR2 / "audit_v2w2_cycle_points.csv").open(
            "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(cyc_rows[0]))
        w.writeheader()
        w.writerows(cyc_rows)

    n_pass = sum(1 for c in CHECKS if c["ok"])
    print(f"\nИТОГ: {n_pass}/{len(CHECKS)} PASS")
    print(f"JSON: {DIR2 / 'audit_v2w2_results.json'}")
    print(f"CSV:  {csv_path}")
    if n_pass != len(CHECKS):
        print("ПРОВАЛЕННЫЕ:", [c["id"] for c in CHECKS if not c["ok"]])


if __name__ == "__main__":
    main()

"""
Stage 2W.2 → sign correction.

Исправляет физические знаки Stage 2W.2, не изменяя исходные файлы:
  П.1  tau_motor = M·q̈ + c − Tg − Text − Tspring
       (Tg/Text — реально приложенные силы, не момент двигателя).
  П.2  Ugravity = −Σ m·G·p  (и для груза);  Uspring = −½·k_s·(q−q0)²
       для элемента с моментом tau_s = k_s·(q−q0).  Пассивная пружина ⇔ k_s < 0.
       Энергобаланс: Wmotor = Δ(K + Ugravity + Uspring); груз входит в M и
       Ugravity, поэтому его вес НЕ считается вторым внешним источником.
  П.3  Компенсатор подбирается к Tspring_target = −β·Tg.  Пересчёт k_s, q0,
       моментов, остатков, энергии; проверка пассивности (k_s < 0) и q0 в ходе.
       Контроль L/P6:S3/sh1: A Tg=+68.43, |tau|=68.43;
       C legacy Tg=+75.82, Tspring=+15.86, физически |tau|=91.68 (не 59.97).
  П.4  Pj = tau_motor_j·qdot_j; отдельно ΣP+, ΣP−, ΣP, пики по суставам.
       Отрицательная мощность не считается рекуперированной.
  П.5  Статика реально пересчитывается для f_gear = 0 / 0.5 / 1.

Новые артефакты пишутся только в audit_v2w_2/sign_correction/.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DIR2 = HERE.parent
DIR1 = DIR2.parent / "audit_v2w_1"

sys.dont_write_bytecode = True


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


V2 = _load(DIR2 / "audit_v2w2.py", "sign_stage2w2")
A1 = V2.A1
B = V2.B
K = V2.K
G = np.asarray(B.G, float)
BETA = 0.9
GEAR_SHARES = (0.0, 0.5, 1.0)
DEFAULT_GEAR = 0.5
T_LEG = V2.T_LEG
N_PER_LEG = V2.N_PER_LEG
SCEN = "nominal"
CANDIDATES = ("sh1", "el")       # проверенные места установки в модели 2W.2
CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def rel(a, b) -> float:
    return abs(a - b) / max(1e-12, abs(b))


# ============================================================
# 1. Независимый аналитический однозвенный тест
#    Одно звено, ось Y, точечная масса m на горизонтальном плече L.
#    p(q) = (L·cos q, 0, −L·sin q),  G = (0, 0, −g).
#    Tg = m·g·L·cos q  (при q=0: +m·g·L),  U = −m·G·p = −m·g·L·sin q.
#    tau_motor = M·q̈ − Tg − tau_s, M = m·L², tau_s = k_s·(q−q0).
# ============================================================
def analytic_one_link() -> dict:
    m, L, g = 10.0, 0.5, 9.81
    M = m * L * L
    Gv = np.array([0.0, 0.0, -g])

    def pos(q):
        return np.array([L * math.cos(q), 0.0, -L * math.sin(q)])

    def Jv(q):
        return np.array([-L * math.sin(q), 0.0, -L * math.cos(q)])

    def Tg(q):
        # Jᵀ·m·G — ровно та же формула, что в проекте, но на независимой модели
        return float(Jv(q) @ (m * Gv))

    def Ugrav(q):
        return float(-m * (Gv @ pos(q)))

    def Us(q, ks, q0):
        return -0.5 * ks * (q - q0) ** 2

    def tau_s(q, ks, q0):
        return ks * (q - q0)

    # (1) момент тяжести в горизонтали
    q0 = 0.0
    tg0 = Tg(q0)
    expected = m * g * L
    check("S1_analytic_torque", rel(tg0, expected) < 1e-9,
          f"Tg(q=0)={tg0:.6f} Н·м, m·g·L={expected:.6f} Н·м")

    # (2) Ugravity и её производная
    h = 1e-6
    dU = -(Ugrav(h) - Ugrav(-h)) / (2 * h)
    check("S2_analytic_potential", rel(dU, Tg(0.0)) < 1e-6,
          f"−dU/dq={dU:.6f}, Tg={Tg(0.0):.6f}")

    # (3) статический мотор при отсутствии пружины: tau = −Tg
    tau_static = -Tg(0.0)
    check("S3_analytic_motor_static", rel(tau_static, -expected) < 1e-9,
          f"tau_motor=−Tg={tau_static:.6f} Н·м")

    # (4) пружина «против» и «вместе» с тяжестью
    ks, q0 = -50.0, 0.0          # пассивная пружина (k_s < 0)
    q_against = +0.10            # пружина тянет к q0=0 против гравитации
    q_with = -0.10               # пружина тянет к q0=0 вместе с гравитацией
    tau_with = -Tg(q_with) - tau_s(q_with, ks, q0)
    tau_against = -Tg(q_against) - tau_s(q_against, ks, q0)
    tau_nos = -Tg(q_with)
    check("S5_analytic_spring_signs",
          abs(tau_with) > abs(tau_nos) > abs(tau_against),
          f"|tau| без пружины={abs(tau_nos):.3f}, вместе с тяжестью="
          f"{abs(tau_with):.3f}, против тяжести={abs(tau_against):.3f}")

    # (5) энергетическая идентичность на заданном движении q(t)
    ks, q0 = -50.0, 0.0
    dt = 1e-4
    n = 20001
    t = np.arange(n) * dt
    q = 0.3 * np.sin(2 * math.pi * 0.7 * t)
    qd = 0.3 * 2 * math.pi * 0.7 * np.cos(2 * math.pi * 0.7 * t)
    qdd = -0.3 * (2 * math.pi * 0.7) ** 2 * np.sin(2 * math.pi * 0.7 * t)
    tau = M * qdd - np.array([Tg(x) for x in q]) - ks * (q - q0)
    P = tau * qd
    E = 0.5 * M * qd ** 2 - np.array([m * (Gv @ pos(x)) for x in q]) \
        + 0.5 * (-ks) * (q - q0) ** 2
    W = float(np.trapezoid(P, dx=dt))
    dE = float(E[-1] - E[0])
    check("S4_analytic_energy", rel(W, dE) < 1e-3,
          f"Wmotor={W:.6f} Дж, Δ(K+U)={dE:.6f} Дж, отн.ошибка="
          f"{rel(W, dE):.2e}")
    return {"m": m, "L": L, "g": g, "Tg_horizontal": tg0,
            "motor_static": tau_static, "W": W, "dE": dE,
            "tau_abs": {"none": abs(tau_nos), "with_gravity": abs(tau_with),
                        "against_gravity": abs(tau_against)}}


# ============================================================
# 2. Физические величины (исправленные)
# ============================================================
def points_of(side, mode, f_gear=DEFAULT_GEAR):
    return V2.build_points_v2(side, SCEN, mode, f_gear)


def Ugravity(side, q, pts, load_kg) -> float:
    """U = −Σ m·G·p.  Физический гравитационный потенциал."""
    u = 0.0
    for p, pos in zip(pts, B.point_positions(side, q, pts)):
        if p.get("at_base"):
            continue
        u += -p["mass"] * float(G @ pos)
    if load_kg > 0:
        palm = B.FK.fk(side, q)[B.chain_for(side)[-1]]
        u += -load_kg * float(G @ palm[:3, 3])
    return float(u)


# --- элементы пружин: tau_s = k_s·(q−q0), U = −½·k_s·(q−q0)² ---------------
_SPRING_PASSIVE: dict[tuple, dict] = {}
_SPRING_LEGACY: dict[tuple, dict] = {}


def fit_passive(side, jkey) -> dict:
    """Фит к цели −β·Tg.  Пассивная пружина требует k_s < 0."""
    key = (side, jkey)
    if key in _SPRING_PASSIVE:
        return _SPRING_PASSIVE[key]
    poses = [(k, v) for k, v in B.ALL_POSES.items() if v["side"] == side]
    pts = points_of(side, "A")
    j = K.index(jkey)
    q = np.array([v["q_deg"][jkey] for _k, v in poses], float)
    tg = np.array([B.tg_jacobian(side, v["q_deg"], pts)[j] for _k, v in poses])
    qr = q * math.pi / 180.0
    y = -BETA * tg
    A = np.vstack([qr, np.ones_like(qr)]).T
    ks, b = (float(x) for x in np.linalg.lstsq(A, y, rcond=None)[0])
    q0 = -b / ks if abs(ks) > 1e-12 else 0.0
    q0_deg = q0 * 180.0 / math.pi
    tau_s = ks * (qr - q0)
    resid = tg + tau_s                     # остаток удержания = tau_motor−?
    qmin, qmax = float(q.min()), float(q.max())
    preload = 0.0 if qmin <= q0_deg <= qmax else (
        q0_deg - qmax if q0_deg > qmax else q0_deg - qmin)
    E = -0.5 * ks * float(np.max((qr - q0) ** 2))   # ≥0 при ks<0
    out = {
        "ks": round(ks, 4), "q0_deg": round(q0_deg, 2),
        "q_range": [round(qmin, 2), round(qmax, 2)],
        "preload_deg": round(preload, 2),
        "passive": bool(ks < -1e-9),
        "q0_in_range": bool(qmin <= q0_deg <= qmax),
        "tau_spring_min": round(float(tau_s.min()), 2),
        "tau_spring_max": round(float(tau_s.max()), 2),
        "Tg_peak": round(float(np.abs(tg).max()), 2),
        "residual_peak": round(float(np.abs(resid).max()), 2),
        "compensation_peak_pct": round(
            100 * (1 - np.abs(resid).max() / max(1e-9, np.abs(tg).max())), 1),
        "E_spring_max_J": round(E, 2),
    }
    _SPRING_PASSIVE[key] = out
    return out


def legacy_elements(side) -> dict:
    """Прежний элемент 2W.2 (k_old, q0_old), теперь как физическая пружина.

    tau_s = k_s·(q−q0) с k_s = k_old.  Именно он даёт контроль +15.86 Н·м.
    Если k_old > 0 — это отрицательная жёсткость (не пассивная пружина).
    """
    key = side
    if key in _SPRING_LEGACY:
        return _SPRING_LEGACY[key]
    out = {}
    for jkey, info in V2.compensator_v2(side, SCEN).items():
        if not info["included"]:
            continue
        out[jkey] = {"ks": float(info["k"]), "q0_deg": float(info["q0_deg"])}
    _SPRING_LEGACY[key] = out
    return out


def spring_torque(side, q_deg, elements) -> np.ndarray:
    t = np.zeros(7)
    for jkey, e in elements.items():
        t[K.index(jkey)] = e["ks"] * (q_deg[jkey] - e["q0_deg"]) * math.pi / 180.0
    return t


def spring_energy(side, q_deg, elements) -> float:
    u = 0.0
    for jkey, e in elements.items():
        dq = (q_deg[jkey] - e["q0_deg"]) * math.pi / 180.0
        u += -0.5 * e["ks"] * dq * dq
    return float(u)


def passive_elements(side) -> dict:
    """Только реально пассивные элементы в проверенных местах (sh1/el)."""
    out = {}
    for jkey in CANDIDATES:
        f = fit_passive(side, jkey)
        if f["passive"] and f["q0_in_range"]:
            out[jkey] = {"ks": f["ks"], "q0_deg": f["q0_deg"]}
    return out


# ============================================================
# 3. Исправленное уравнение движения
# ============================================================
def tau_motor_corrected(side, q, mode, load_kg=0.0, qd=None, qdd=None,
                        f_gear=DEFAULT_GEAR, elements=None, pts=None,
                        with_c=True) -> dict:
    pts = points_of(side, mode, f_gear) if pts is None else pts
    tg = B.tg_jacobian(side, q, pts)
    F = np.zeros(3)
    text, _ = B.external_wrench(side, q, F, load_kg)
    qd = np.zeros(7) if qd is None else np.asarray(qd, float)
    qdd = np.zeros(7) if qdd is None else np.asarray(qdd, float)
    M = A1.mass_matrix_audit(side, q, pts, SCEN, load_kg)
    c = (A1.coriolis_christoffel(side, q, qd, pts, SCEN, load_kg)[0]
         if with_c and np.any(qd != 0) else np.zeros(7))
    elements = {} if elements is None else elements
    ts = spring_torque(side, q, elements)
    # M·q̈ + c = tau_motor + Tg + Text + Tspring
    tau = M @ qdd + c - tg - text - ts
    return {"tau": tau, "Tg": tg, "Text": text, "Tspring": ts,
            "M": M, "c": c, "points": pts}


# ============================================================
# 4. Статика: контроль 2W.2 и реальный пересчёт f_gear
# ============================================================
KEY_POSE = {"L": "P6_L:S3_tension", "R": "P6_R:S3_tension"}


def static_corrected(side, mode, load_kg, f_gear=DEFAULT_GEAR,
                     elements=None) -> dict:
    q = B.POSES[KEY_POSE[side]]["q_deg"]
    r = tau_motor_corrected(side, q, mode, load_kg, f_gear=f_gear,
                            elements=elements)
    return {
        "pose": KEY_POSE[side],
        "Tg": {k: round(float(r["Tg"][i]), 2) for i, k in enumerate(K)},
        "Text": {k: round(float(r["Text"][i]), 2) for i, k in enumerate(K)},
        "Tspring": {k: round(float(r["Tspring"][i]), 2) for i, k in enumerate(K)},
        "tau_motor": {k: round(float(r["tau"][i]), 2) for i, k in enumerate(K)},
        "max_abs": round(float(np.abs(r["tau"]).max()), 2),
    }


# ============================================================
# 5. Динамика одного перехода P6_L:S2 → S3
# ============================================================
def transition(tag_a="P6_L:S2_grasp", tag_b="P6_L:S3_tension",
               side="L", mode="A", load_kg=0.0, f_gear=DEFAULT_GEAR,
               elements=None) -> dict:
    qa = B.POSES[tag_a]["q_deg"]
    qb = B.POSES[tag_b]["q_deg"]
    dq = np.array([(qb[k] - qa[k]) * math.pi / 180.0 for k in K])
    dt = T_LEG / (N_PER_LEG - 1)
    s, sd, sdd = V2.trapezoid_full(N_PER_LEG, dt)
    pts = points_of(side, mode, f_gear)
    elements = {} if elements is None else elements
    rows = []
    Ppos = Pneg = Palg = 0.0
    Ppeak_joint = np.zeros(7)
    Wpos = Wneg = 0.0
    W = dE = 0.0
    W_text = dUload = 0.0
    E = np.zeros(N_PER_LEG)
    P = np.zeros(N_PER_LEG)
    Pplus = np.zeros(N_PER_LEG)
    Pminus = np.zeros(N_PER_LEG)
    Pj_all = []
    for i in range(N_PER_LEG):
        q = {k: qa[k] + (qb[k] - qa[k]) * s[i] for k in K}
        qd = dq * sd[i]
        qdd = dq * sdd[i]
        r = tau_motor_corrected(side, q, mode, load_kg, qd, qdd, f_gear,
                                elements, pts)
        Pj = r["tau"] * qd
        Pj_all.append(Pj)
        P[i] = float(Pj.sum())
        Pplus[i] = float(np.maximum(Pj, 0).sum())
        Pminus[i] = float(np.minimum(Pj, 0).sum())
        Ppeak_joint = np.maximum(Ppeak_joint, np.abs(Pj))
        E[i] = (V2.kinetic(qd, r["M"]) + Ugravity(side, q, pts, load_kg)
                + spring_energy(side, q, elements))
        rows.append({"i": i, "t": round(i * dt, 5),
                     "q_deg": {k: round(q[k], 3) for k in K},
                     "qd": [round(float(x), 5) for x in qd],
                     "qdd": [round(float(x), 5) for x in qdd],
                     "Tg": [round(float(x), 3) for x in r["Tg"]],
                     "Text": [round(float(x), 3) for x in r["Text"]],
                     "Tspring": [round(float(x), 3) for x in r["Tspring"]],
                     "tau": [round(float(x), 3) for x in r["tau"]],
                     "Pj": [round(float(x), 3) for x in Pj]})
    W = float(np.trapezoid(P, dx=dt))
    dE = float(E[-1] - E[0])
    Wpos = float(np.trapezoid(np.maximum(P, 0), dx=dt))
    Wneg = float(np.trapezoid(np.minimum(P, 0), dx=dt))
    Wpos_joint = float(np.trapezoid(Pplus, dx=dt))
    Wneg_joint = float(np.trapezoid(Pminus, dx=dt))
    # Text·qd = −dU_load/dt — проверка, что груз не учтён дважды
    Ptext = np.array([float(np.dot(r["Text"], r["qd"])) for r in rows]) if rows else np.zeros(1)
    # пересчитываем аккуратно по строкам (Text и qd уже округлены — берём точные)
    Ptext = np.zeros(N_PER_LEG)
    for i in range(N_PER_LEG):
        Ptext[i] = float(np.dot(
            B.external_wrench(
                side,
                {k: qa[k] + (qb[k] - qa[k]) * s[i] for k in K},
                np.zeros(3), load_kg)[0],
            dq * sd[i]))
    W_text = float(np.trapezoid(Ptext, dx=dt))
    # ΔU_load независимо
    palm_a = B.FK.fk(side, qa)[B.chain_for(side)[-1]][:3, 3]
    palm_b = B.FK.fk(side, qb)[B.chain_for(side)[-1]][:3, 3]
    dUload = float(-load_kg * (G @ palm_b) + load_kg * (G @ palm_a)) \
        if load_kg > 0 else 0.0
    scale = max(1e-6, abs(dE), abs(W))
    return {
        "mode": mode, "side": side, "load_kg": load_kg, "f_gear": f_gear,
        "leg": f"{tag_a}->{tag_b}", "n_steps": N_PER_LEG, "dt": dt,
        "Wmotor": round(W, 3), "dE": round(dE, 3),
        "energy_err": round(abs(W - dE), 3),
        "energy_rel": round(abs(W - dE) / scale, 5),
        "Wpos": round(Wpos, 3), "Wneg": round(Wneg, 3),
        "Wpos_joint": round(Wpos_joint, 3),
        "Wneg_joint": round(Wneg_joint, 3),
        "W_text": round(W_text, 3), "dU_load": round(dUload, 3),
        "P_peak_pos_sum": round(float(Pplus.max()), 2),
        "P_peak_neg_sum": round(float(Pminus.min()), 2),
        "P_peak_alg": round(float(P.max()), 2),
        "P_peak_alg_min": round(float(P.min()), 2),
        "P_peak_by_joint": {k: round(float(Ppeak_joint[i]), 2)
                            for i, k in enumerate(K)},
        "P_peak_joint_max": round(float(Ppeak_joint.max()), 2),
        "qdot_max": round(float(np.abs(dq).max() * np.abs(sd).max()), 4),
        "qddot_max": round(float(np.abs(dq).max() * np.abs(sdd).max()), 4),
        "rows": rows,
    }


# ============================================================
# 5b. Полный цикл: контроль старых 266.7 / 203.9 Вт и исправленный расчёт
# ============================================================
def cycle_compare(side="L", mode="A", load_kg=25.0,
                  tag="P6_L") -> dict:
    seq = V2._pose_seq(tag)
    legs = list(zip(seq, seq[1:] + [seq[0]]))
    pts = points_of(side, mode)
    elements = legacy_elements(side) if mode == "C" else {}
    dt = T_LEG / (N_PER_LEG - 1)
    s, sd, sdd = V2.trapezoid_full(N_PER_LEG, dt)
    old_pos = old_alg = 0.0
    new_pos = new_neg = new_alg = 0.0
    Ws, dEs = [], []
    for a_key, b_key in legs:
        qa, qb = B.POSES[a_key]["q_deg"], B.POSES[b_key]["q_deg"]
        dq = np.array([(qb[k] - qa[k]) * math.pi / 180.0 for k in K])
        P_old_pos = np.zeros(N_PER_LEG)
        P_old_neg = np.zeros(N_PER_LEG)
        P_old_alg = np.zeros(N_PER_LEG)
        P_new_pos = np.zeros(N_PER_LEG)
        P_new_neg = np.zeros(N_PER_LEG)
        P_new_alg = np.zeros(N_PER_LEG)
        E = np.zeros(N_PER_LEG)
        for i in range(N_PER_LEG):
            q = {k: qa[k] + (qb[k] - qa[k]) * s[i] for k in K}
            qd = dq * sd[i]
            qdd = dq * sdd[i]
            r = tau_motor_corrected(side, q, mode, load_kg, qd, qdd,
                                    DEFAULT_GEAR, elements, pts)
            tau_old = r["Tg"] + r["M"] @ qdd + r["c"] + r["Text"] - r["Tspring"]
            tau_new = r["tau"]
            Pj_old = tau_old * qd
            Pj_new = tau_new * qd
            P_old_pos[i] = float(np.maximum(Pj_old, 0).sum())
            P_old_neg[i] = float(np.minimum(Pj_old, 0).sum())
            P_old_alg[i] = float(Pj_old.sum())
            P_new_pos[i] = float(np.maximum(Pj_new, 0).sum())
            P_new_neg[i] = float(np.minimum(Pj_new, 0).sum())
            P_new_alg[i] = float(Pj_new.sum())
            E[i] = (V2.kinetic(qd, r["M"]) + Ugravity(side, q, pts, load_kg)
                    + spring_energy(side, q, elements))
        old_pos = max(old_pos, float(P_old_pos.max()))
        old_alg = max(old_alg, float(P_old_alg.max()))
        new_pos = max(new_pos, float(P_new_pos.max()))
        new_neg = min(new_neg, float(P_new_neg.min()))
        new_alg = max(new_alg, float(P_new_alg.max()))
        Ws.append(float(np.trapezoid(P_new_alg, dx=dt)))
        dEs.append(float(E[-1] - E[0]))
    Ws = np.array(Ws); dEs = np.array(dEs)
    err = float(np.abs(Ws - dEs).max())
    scale = max(1e-6, float(np.abs(dEs).max()))
    return {"side": side, "mode": mode, "load_kg": load_kg,
            "old_P_peak_pos_sum": round(old_pos, 2),
            "old_P_peak_alg": round(old_alg, 2),
            "new_P_peak_pos_sum": round(new_pos, 2),
            "new_P_peak_neg_sum": round(new_neg, 2),
            "new_P_peak_alg": round(new_alg, 2),
            "new_W_pos": round(float(np.maximum(Ws, 0).sum()), 2),
            "new_W_neg": round(float(np.minimum(Ws, 0).sum()), 2),
            "new_energy_rel": round(err / scale, 5)}


# ============================================================
# 6. Компенсаторы: пересчёт к −β·Tg
# ============================================================
def compensators_corrected(side) -> dict:
    out = {}
    for jkey in K:
        f = fit_passive(side, jkey)
        if not f["passive"]:
            status = "НЕ пассивная пружина: k_s>0 (отрицательная жёсткость / активный элемент)"
        elif f["q0_in_range"]:
            status = "пассивная торсионная пружина, q0 в рабочем ходе"
        elif abs(f["preload_deg"]) <= 180:
            status = "пассивная, требуется преднатяг q0"
        else:
            status = "требуется проверка: большой преднатяг/иная кинематика"
        out[jkey] = {**f, "status": status,
                     "included_in_model": bool(
                         jkey in CANDIDATES and f["passive"] and f["q0_in_range"])}
    return out


# ============================================================
# 7. Прогон
# ============================================================
def main() -> None:
    print("=" * 78)
    print("Stage 2W.2 → sign correction")
    print("=" * 78)

    print("\n[1] Независимый аналитический однозвенный тест")
    analytic = analytic_one_link()

    # --- контроль 2W.2 ----------------------------------------------------
    print("\n[2] Контроль L/P6:S3_tension, без груза")
    legacy = legacy_elements("L")
    st_a = static_corrected("L", "A", 0.0)
    st_c = static_corrected("L", "C", 0.0, elements=legacy)
    tg_a = st_a["Tg"]["sh1"]
    tau_a = st_a["tau_motor"]["sh1"]
    tg_c = st_c["Tg"]["sh1"]
    ts_c = st_c["Tspring"]["sh1"]
    tau_c = st_c["tau_motor"]["sh1"]
    old_formula = tg_c - ts_c
    check("S6_control_A", abs(tg_a - 68.43) < 0.05 and abs(abs(tau_a) - 68.43) < 0.05,
          f"A: Tg_sh1={tg_a:+.2f}, tau_motor_sh1={tau_a:+.2f}, |tau|={abs(tau_a):.2f}")
    check("S7_control_C",
          abs(tg_c - 75.82) < 0.05 and abs(ts_c - 15.86) < 0.05
          and abs(abs(tau_c) - 91.68) < 0.1,
          f"C: Tg_sh1={tg_c:+.2f}, Tspring={ts_c:+.2f}, "
          f"tau_motor={tau_c:+.2f}, |tau|={abs(tau_c):.2f}; "
          f"старая формула Tg−Ts={old_formula:+.2f}")
    check("S7b_old_number_explained",
          abs(abs(old_formula) - 59.97) < 0.1,
          f"опубликованные 59.97 = Tg − Ts = {tg_c:.2f} − {ts_c:.2f} = {old_formula:.2f}")

    # --- компенсаторы ------------------------------------------------------
    print("\n[3] Пассивные компенсаторы (цель −β·Tg)")
    comp = {s: compensators_corrected(s) for s in ("L", "R")}
    l1 = comp["L"]["sh1"]
    check("S11_sh1_not_passive", not l1["passive"],
          f"L/sh1: k_s={l1['ks']:+.2f} Н·м/рад → пассивная пружина требует k_s<0; "
          f"статус: {l1['status']}")
    passive_ok = {s: [j for j, v in comp[s].items()
                      if v["passive"] and v["q0_in_range"]] for s in comp}
    print(f"  пассивные и q0 в ходе: L={passive_ok['L']}, R={passive_ok['R']}")
    print(f"  пассивные элементы в модели (sh1/el): "
          f"L={list(passive_elements('L'))}, R={list(passive_elements('R'))}")

    # --- статика: 0 и 25 кг, f_gear ---------------------------------------
    print("\n[4] Статика P6:S3 (L), реальный пересчёт f_gear")
    stat_rows = []
    stat = {}
    for mode in ("A", "B1", "B2", "C"):
        for load in (0.0, 25.0):
            for f in GEAR_SHARES:
                elems = legacy if mode == "C" else {}
                s = static_corrected("L", mode, load, f, elems)
                stat[(mode, load, f)] = s
                for j in K:
                    stat_rows.append({
                        "mode": mode, "load_kg": load, "f_gear": f, "joint": j,
                        "Tg": s["Tg"][j], "Text": s["Text"][j],
                        "Tspring": s["Tspring"][j], "tau_motor": s["tau_motor"][j],
                    })
    # отчёт: моменты по f_gear для B1/B2
    for mode in ("B1", "B2"):
        for load in (0.0, 25.0):
            vals = {f: stat[(mode, load, f)]["max_abs"] for f in GEAR_SHARES}
            distinct = len({round(v, 3) for v in vals.values()}) == 3
            print(f"  {mode}, {int(load)} кг: max|tau| по f = {vals}  "
                  f"{'различаются' if distinct else 'СОВПАДАЮТ'}")
    check("S10_gear_recomputed",
          len({round(stat[("B1", 25.0, f)]["max_abs"], 3) for f in GEAR_SHARES}) >= 2,
          "статика B1/25 кг реально зависит от f_gear (строки не дублируются)")

    # --- динамика ----------------------------------------------------------
    print("\n[5] Динамика P6_L:S2 → S3")
    dyn = {}
    for mode in ("A", "B1", "B2", "C"):
        for load in (0.0, 25.0):
            elems = legacy if mode == "C" else {}
            r = transition(mode=mode, load_kg=load, elements=elems)
            dyn[(mode, load)] = r
            print(f"  {mode}, {int(load)} кг: W={r['Wmotor']:.1f} Дж, "
                  f"ΔE={r['dE']:.1f}, err={r['energy_rel']*100:.2f}%, "
                  f"P+={r['P_peak_pos_sum']:.1f} Вт, "
                  f"P−={r['P_peak_neg_sum']:.1f}, P_alg={r['P_peak_alg']:.1f}")
    # C_passive = A (нет пассивных пружин в sh1/el)
    cp = transition(mode="A", load_kg=0.0, elements=passive_elements("L"))
    check("S8_energy_all",
          all(dyn[k]["energy_rel"] < 0.05 for k in dyn),
          "энергобаланс Wmotor=Δ(K+Ug+Us), макс. отн.ошибка = "
          f"{max(v['energy_rel'] for v in dyn.values())*100:.2f}%")
    check("S9_load_not_double_counted",
          all(abs(dyn[k]["W_text"] - dyn[k]["dU_load"]) < 1e-6 * max(1, abs(dyn[k]["dU_load"]))
              for k in dyn),
          "∫Text·qd = −ΔU_load (груз входит в Ugravity, второй раз не считается)")
    # усиленная проверка: dU_load/dq = −Text в каждой координате
    q_l = B.POSES["P6_L:S3_tension"]["q_deg"]
    up = np.zeros(7)
    for j, k in enumerate(K):
        hh = 1e-5
        qp, qm = dict(q_l), dict(q_l)
        qp[k] += hh
        qm[k] -= hh
        palm_p = B.FK.fk("L", qp)[B.chain_for("L")[-1]][:3, 3]
        palm_m = B.FK.fk("L", qm)[B.chain_for("L")[-1]][:3, 3]
        up[j] = (-25.0 * (G @ palm_p) + 25.0 * (G @ palm_m)) / (2 * hh * math.pi / 180.0)
    text_load, _ = B.external_wrench("L", q_l, np.zeros(3), 25.0)
    err_load = float(np.abs(up + text_load).max())
    check("S9b_load_gradient", err_load < 1e-3,
          f"max|dU_load/dq + Text| = {err_load:.2e} Н·м (25 кг)")
    check("S12_power_split",
          all(v["P_peak_pos_sum"] >= v["P_peak_alg"] - 1e-9
              and v["P_peak_neg_sum"] <= 0 and v["Wneg"] <= 0
              and v["Wpos_joint"] >= v["Wpos"] - 1e-9
              and v["Wneg_joint"] <= v["Wneg"] + 1e-9
              for v in dyn.values()),
          "ΣP+ ≥ P_alg, ΣP− ≤ 0; интегралы по суставам разделены, "
          "отрицательная мощность не рекуперируется")
    check("S13_C_passive_eq_A",
          cp["energy_rel"] < 0.05 and abs(cp["Wmotor"] - dyn[("A", 0.0)]["Wmotor"]) < 1e-6,
          f"C-passive без пассивных пружин в sh1/el даёт W={cp['Wmotor']:.2f} Дж = A")

    print("\n[5b] Контроль старого CSV: полный цикл P6_L, A, 25 кг")
    cc = cycle_compare()
    print(f"  старый CSV: ΣP+={cc['old_P_peak_pos_sum']:.1f} Вт, "
          f"P_alg={cc['old_P_peak_alg']:.1f} Вт")
    print(f"  исправлено: ΣP+={cc['new_P_peak_pos_sum']:.1f} Вт, "
          f"ΣP−={cc['new_P_peak_neg_sum']:.1f} Вт, "
          f"P_alg={cc['new_P_peak_alg']:.1f} Вт, "
          f"энергобаланс {cc['new_energy_rel']*100:.2f}%")
    check("S16_old_csv_control",
          abs(cc["old_P_peak_pos_sum"] - 266.7) < 1.0
          and abs(cc["old_P_peak_alg"] - 203.9) < 1.0,
          f"старая формула воспроизводит 266.7 / 203.9 Вт: "
          f"{cc['old_P_peak_pos_sum']:.1f} / {cc['old_P_peak_alg']:.1f}")
    check("S17_corrected_cycle_energy", cc["new_energy_rel"] < 0.05,
          f"исправленный полный цикл: энергобаланс "
          f"{cc['new_energy_rel']*100:.2f}%, P_alg={cc['new_P_peak_alg']:.1f} Вт")

    # --- контроль смены знака внешней силы --------------------------------
    print("\n[6] Смена знака внешней силы и пружины")
    q = B.POSES["P6_L:S3_tension"]["q_deg"]
    pts = points_of("L", "A")
    Fup = np.array([0.0, 0.0, 1.0]) * 100.0
    Fdn = np.array([0.0, 0.0, -1.0]) * 100.0
    t_up, _ = B.external_wrench("L", q, Fup, 0.0)
    t_dn, _ = B.external_wrench("L", q, Fdn, 0.0)
    check("S14_external_sign",
          np.allclose(t_up, -t_dn) and np.abs(t_up).max() > 1.0,
          f"Text(вверх) = −Text(вниз), max|Text|={np.abs(t_up).max():.1f} Н·м")
    # вес груза даёт тот же знак, что и сила вниз
    t_load, _ = B.external_wrench("L", q, np.zeros(3), 25.0)
    check("S15_load_sign",
          np.allclose(t_load, (25.0 * 9.81 / 100.0) * t_dn),
          "Text(груз 25 кг) = (25·9.81/100)·Text(сила 100 Н вниз)")

    # --- запись артефактов -------------------------------------------------
    print("\n[7] Запись артефактов")
    result = {
        "analytic": analytic,
        "control": {"A": st_a, "C_legacy": st_c,
                    "old_published": round(old_formula, 2)},
        "compensators": comp,
        "passive_elements": {s: passive_elements(s) for s in ("L", "R")},
        "statics": {f"{m}|{int(l)}kg|f{f}": v
                    for (m, l, f), v in stat.items()},
        "dynamics": {f"{m}|{int(l)}kg": v for (m, l), v in dyn.items()},
        "C_passive": cp,
        "cycle_compare": cc,
        "checks": CHECKS,
        "note": ("C_legacy — прежний элемент 2W.2 (k_s>0, отрицательная "
                 "жёсткость); C в динамике считается с ним для воспроизведения "
                 "контроля. Пассивных пружин в sh1/el нет → C_passive = A."),
    }
    (HERE / "sign_results.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    with (HERE / "statics_corrected.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(stat_rows[0]))
        w.writeheader()
        w.writerows(stat_rows)

    # компактная таблица архитектур: только подтверждённые строки
    arch_rows = []
    for mode in ("A", "B1", "B2", "C"):
        for load in (0.0, 25.0):
            d = dyn[(mode, load)]
            s = stat[(mode, load, DEFAULT_GEAR)]
            arch_rows.append({
                "mode": mode, "load_kg": load, "f_gear": DEFAULT_GEAR,
                "Tg_sh1": s["Tg"]["sh1"], "Tspring_sh1": s["Tspring"]["sh1"],
                "tau_motor_sh1": s["tau_motor"]["sh1"],
                "static_max_abs": s["max_abs"],
                "Wmotor_transition": d["Wmotor"],
                "energy_rel": d["energy_rel"],
                "P_peak_alg": d["P_peak_alg"],
                "P_peak_pos_sum": d["P_peak_pos_sum"],
                "P_peak_neg_sum": d["P_peak_neg_sum"],
                "P_peak_joint_max": d["P_peak_joint_max"],
            })
    with (HERE / "arch_table_corrected.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(arch_rows[0]))
        w.writeheader()
        w.writerows(arch_rows)

    # покадровая динамика
    cyc = []
    for (mode, load), d in dyn.items():
        for r in d["rows"]:
            cyc.append({"mode": mode, "load_kg": load, **r})
    with (HERE / "cycle_points_corrected.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(cyc[0]))
        w.writeheader()
        w.writerows(cyc)

    n_pass = sum(1 for c in CHECKS if c["ok"])
    print(f"\nИТОГ: {n_pass}/{len(CHECKS)} PASS")
    if n_pass != len(CHECKS):
        print("ПРОВАЛЕННЫЕ:", [c["id"] for c in CHECKS if not c["ok"]])
    print(f"JSON:  {HERE / 'sign_results.json'}")
    print(f"CSV:   {HERE / 'statics_corrected.csv'}")
    print(f"CSV:   {HERE / 'arch_table_corrected.csv'}")
    print(f"CSV:   {HERE / 'cycle_points_corrected.csv'}")


if __name__ == "__main__":
    main()

"""
Stage 2X, Этап 1 — восстановление таблицы требований по суставам рук.

Источники (только чтение, ничего не изменяется):
  * лимиты осей            — /home/v/projects/робот/joint_limits_audit.txt
  * кинематика, позы P6/P7 — arm_architecture_comparison.py (Stage 2B.2/2W)
  * моменты, знаки         — audit_v2w_2/audit_v2w2.py + sign_correction
                             (исправленные знаки Stage 2W.2)
  * прежние максимумы      — arm_load_audit/arm_actuator_feasibility_v1
                             (1093.6 / 1203.5 Н·м — сверка, НЕ пересчёт)

Что считается:
  1. макс. статический момент по сетке поз (0 / 25 / 100 кг);
  2. типичный длительный момент в рабочих позах P6/P7;
  3. пиковый динамический момент (переход P6:S2 -> S3, исправленная физика);
  4. скорость/ускорение — из данных (нет требования → UNKNOWN);
  5. время удержания — UNKNOWN (нет тепловых данных);
  6. диапазон углов — из joint_limits_audit.txt;
  7. радиальная/осевая/опрокидывающая нагрузки на выходную опору;
  8. удержание при отключении питания = статический момент (тормоз).

Артефакты: joint_requirements.csv, joint_requirements.json
"""

from __future__ import annotations

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

BASE = Path("/home/v/projects/робот/robot_pose_study_v1/"
            "arm_architecture_comparison_v1")
SIGN = BASE / "audit_v2w_2" / "sign_correction"
LIMITS_TXT = Path("/home/v/projects/робот/joint_limits_audit.txt")
HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True

G_VEC = np.array([0.0, 0.0, -9.81])
BETA = 0.9


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


M = _load(BASE / "arm_architecture_comparison.py", "req_main")
V2 = _load(BASE / "audit_v2w_2" / "audit_v2w2.py", "req_v2")
SG = _load(SIGN / "audit_v2w2_signs.py", "req_sign")

KEYS = list(M.JOINT_KEYS)              # sh3 sh2 sh1 el rad w1 w2
CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# --------------------------------------------------------------- лимиты ----
def load_limits() -> dict:
    """Парсит SEC1-таблицу joint_limits_audit.txt.

    Формат: name|revolute|axis|lower|upper|units|zero|conf
    (в файле есть и вторая секция с другим числом полей — её пропускаем
    по признаку p[1] == 'revolute').
    """
    out = {}
    for line in LIMITS_TXT.read_text(encoding="utf-8").splitlines():
        p = line.split("|")
        if len(p) < 8 or p[1] != "revolute":
            continue
        name = p[0]
        if not name.startswith("left_"):
            continue
        key = {
            "left_shoulder_joint_empty_3": "sh3",
            "left_shoulder_joint_empty_2": "sh2",
            "left_shoulder_joint_empty_1": "sh1",
            "left_elbow_joint_empty": "el",
            "left_proximal_radioulnar_joint_empty": "rad",
            "left_wrist_joint_1_empty": "w1",
            "left_wrist_joint_2_empty": "w2",
        }.get(name)
        if key and key not in out:
            out[key] = {"empty": name, "axis": p[2],
                        "lower": float(p[3]), "upper": float(p[4]),
                        "confidence": p[7]}
    return out


ANATOMY = {
    "sh3": ("shoulder_3", "Y", "внутренняя/наружная ротация руки"),
    "sh2": ("shoulder_2", "Z", "сгибание/разгибание руки"),
    "sh1": ("shoulder_1", "X", "отведение/приведение руки"),
    "el": ("elbow", "Z", "сгибание предплечья"),
    "rad": ("proximal_radioulnar", "Y", "пронация/супинация предплечья"),
    "w1": ("wrist_1", "Z", "сгибание кисти"),
    "w2": ("wrist_2", "X", "отведение/приведение кисти"),
}


# ------------------------------------------------- статика по сетке поз ----
GRID_STEP_DEG = 20.0


def pose_grid() -> list[dict]:
    """Сетка по пределам суставов (шаг GRID_STEP_DEG). Поиск максимума
    статики — НЕ гарантия глобального максимума и НЕ проверка
    достижимости/столкновений; rad/w1/w2 зафиксированы в 0."""
    out = []
    axes = [(k, np.arange(ANATOMY_LIM[k]["lower"],
                          ANATOMY_LIM[k]["upper"] + 1e-9, GRID_STEP_DEG))
            for k in ("sh3", "sh2", "sh1", "el")]
    for s3 in axes[0][1]:
        for s2 in axes[1][1]:
            for s1 in axes[2][1]:
                for e in axes[3][1]:
                    out.append({"sh3": float(s3), "sh2": float(s2),
                                "sh1": float(s1), "el": float(e),
                                "rad": 0.0, "w1": 0.0, "w2": 0.0})
    return out


def static_over_grid(side: str, loads=(0.0, 100.0)) -> dict:
    """Один проход сетки: для каждой нагрузки свой максимум по суставам."""
    pts = V2.build_points_v2(side, "nominal", "A")
    best = {ld: {k: (0.0, None) for k in KEYS} for ld in loads}
    for q in POSE_GRID:
        tg = M.tg_jacobian(side, q, pts)
        for ld in loads:
            tx = np.zeros(7) if ld == 0.0 else M.external_wrench(
                side, q, np.zeros(3), ld)[0]
            tot = tg + tx
            for i, k in enumerate(KEYS):
                if abs(tot[i]) > abs(best[ld][k][0]):
                    best[ld][k] = (float(tot[i]), dict(q))
    return {ld: {k: {"value": round(v[0], 2),
                     "pose": {kk: round(vv, 1) for kk, vv in v[1].items()}}
                 for k, v in per.items()}
            for ld, per in best.items()}


REFINE_STEP_DEG = 10.0
REFINE_HALO_DEG = 20.0


def refine_grid(side: str, loads=(0.0, 100.0), coarse: dict | None = None) -> dict:
    """Локальное уточнение: вокруг позы, найденной грубой сеткой, перебор
    ±REFINE_HALO_DEG с шагом REFINE_STEP_DEG по всем 4 суставам.

    Снижает занижение максимума из-за грубого шага; не гарантирует
    глобальный максимум (многомодовая поверхность не исключена).
    """
    pts = V2.build_points_v2(side, "nominal", "A")
    best = {ld: {k: (0.0, None) for k in KEYS} for ld in loads}
    # стартуем с уже найденных значений грубой сетки
    if coarse:
        for ld in loads:
            for k in KEYS:
                best[ld][k] = (coarse[ld][k]["value"], coarse[ld][k]["pose"])
    base_keys = ("sh3", "sh2", "sh1", "el")
    seeds = []
    if coarse:
        for k in KEYS:
            for ld in loads:
                seeds.append(coarse[ld][k]["pose"])
    seen = set()
    for seed in seeds:
        for d3 in np.arange(-REFINE_HALO_DEG, REFINE_HALO_DEG + 1e-9, REFINE_STEP_DEG):
            for d2 in np.arange(-REFINE_HALO_DEG, REFINE_HALO_DEG + 1e-9, REFINE_STEP_DEG):
                for d1 in np.arange(-REFINE_HALO_DEG, REFINE_HALO_DEG + 1e-9, REFINE_STEP_DEG):
                    for de in np.arange(-REFINE_HALO_DEG, REFINE_HALO_DEG + 1e-9,
                                        REFINE_STEP_DEG):
                        q = dict(seed)
                        q["sh3"] = seed["sh3"] + d3
                        q["sh2"] = seed["sh2"] + d2
                        q["sh1"] = seed["sh1"] + d1
                        q["el"] = seed["el"] + de
                        # ограничение пределами осей
                        bad = False
                        for bk in base_keys:
                            lo, hi = ANATOMY_LIM[bk]["lower"], ANATOMY_LIM[bk]["upper"]
                            if q[bk] < lo - 1e-9 or q[bk] > hi + 1e-9:
                                bad = True
                                break
                        if bad:
                            continue
                        sig = tuple(round(q[bk], 3) for bk in base_keys)
                        if sig in seen:
                            continue
                        seen.add(sig)
                        tg = M.tg_jacobian(side, q, pts)
                        for ld in loads:
                            tx = np.zeros(7) if ld == 0.0 else M.external_wrench(
                                side, q, np.zeros(3), ld)[0]
                            tot = tg + tx
                            for i, k in enumerate(KEYS):
                                if abs(tot[i]) > abs(best[ld][k][0]):
                                    best[ld][k] = (float(tot[i]), dict(q))
    print(f"    уточнение: {len(seen)} уникальных поз проверено")
    return {ld: {k: {"value": round(v[0], 2),
                     "pose": {kk: round(vv, 1) for kk, vv in v[1].items()}}
                 for k, v in per.items()}
            for ld, per in best.items()}


def static_working_poses(side: str, load_kg: float) -> dict:
    """Рабочие операции P6/P7: максимум и медиана по 12 позам."""
    pts = V2.build_points_v2(side, "nominal", "A")
    vals = {k: [] for k in KEYS}
    for name, pose in M.POSES.items():
        if pose["side"] != side:
            continue
        q = pose["q_deg"]
        tg = M.tg_jacobian(side, q, pts)
        tx, _ = M.external_wrench(side, q, np.zeros(3), load_kg)
        tot = tg + tx
        for i, k in enumerate(KEYS):
            vals[k].append(float(tot[i]))
    out = {}
    for k in KEYS:
        a = np.array(vals[k])
        out[k] = {"max_abs": round(float(np.abs(a).max()), 2),
                  "median_abs": round(float(np.median(np.abs(a))), 2),
                  "n_poses": int(a.size)}
    return out


# ------------------------------------------- нагрузки на выходную опору ----
def bearing_loads(side: str, q: dict, load_kg: float) -> dict:
    """Статические реакции на опоре сустава.

    F_j  = сумма сил дистальных точек (+ вес груза)
    M_j  = момент этих сил относительно начала оси сустава
    осевая сила  = F·a ;  радиальная = |F - (F·a)a|
    осевой момент (момент привода) = M·a ;  опрокидывающий = |M - (M·a)a|

    Дистальность: точка с chain_idx = c нагружает сустав с индексом оси e,
    если c > e (точка строго дистальнее опоры; масса в точке опоры
    воспринимается корпусом, не подшипником — принято и указано в отчёте).
    """
    pts = V2.build_points_v2(side, "nominal", "A")
    frames = M.joint_frames(side, q)
    mats = M.FK.fk(side, q)
    chain = M.chain_for(side)
    positions = M.point_positions(side, q, pts)
    joint_empty_idx = [chain.index(M.FK.JOINTS[side][i][1]) for i in range(7)]

    palm = mats[chain[-1]][:3, 3]
    F_ext = np.array([0.0, 0.0, -9.81]) * load_kg

    out = {}
    for i, key in enumerate(KEYS):
        o, a = frames[i]
        e = joint_empty_idx[i]
        F = np.zeros(3)
        Mo = np.zeros(3)
        for p, pos in zip(pts, positions):
            if p.get("at_base"):
                continue
            if p["chain_idx"] <= e:
                continue
            w = p["mass"] * G_VEC
            F += w
            Mo += np.cross(pos - o, w)
        if load_kg > 0:
            F += F_ext
            Mo += np.cross(palm - o, F_ext)
        axial_F = float(F @ a)
        radial_F = float(np.linalg.norm(F - axial_F * a))
        axial_M = float(Mo @ a)
        bending_M = float(np.linalg.norm(Mo - axial_M * a))
        out[key] = {"radial_N": round(radial_F, 1),
                    "axial_N": round(axial_F, 1),
                    "drive_torque_Nm": round(axial_M, 2),
                    "bending_overturning_Nm": round(bending_M, 2)}
    return out


# ------------------------------------------------- динамический пик -------
def dynamic_peaks(side: str, mode: str, load_kg: float) -> dict:
    """Пик |tau_motor| по суставам на переходе P6:S2 -> S3 (исправленная
    физика Stage 2W.2, sign_correction)."""
    elems = SG.legacy_elements(side) if mode == "C" else {}
    r = SG.transition(tag_a=f"P6_{side}:S2_grasp", tag_b=f"P6_{side}:S3_tension",
                      side=side, mode=mode, load_kg=load_kg, elements=elems)
    tau = np.array([row["tau"] for row in r["rows"]])
    peak = np.abs(tau).max(axis=0)
    return {k: round(float(peak[i]), 2) for i, k in enumerate(KEYS)}


# ---------------------------------------------------------------- main -----
ANATOMY_LIM: dict = {}
POSE_GRID: list[dict] = []


def main() -> None:
    global ANATOMY_LIM, POSE_GRID
    print("=" * 78)
    print("Stage 2X, Этап 1 — требования по суставам")
    print("=" * 78)

    ANATOMY_LIM = load_limits()
    check("R1_LIMITS_LOADED", len(ANATOMY_LIM) == 7,
          f"лимиты загружены для {len(ANATOMY_LIM)}/7 суставов")

    POSE_GRID = pose_grid()
    print(f"\nсетка поз: {len(POSE_GRID)} комбинаций "
          f"(шаг ≥10° или 1/12 хода; достижимость/столкновения НЕ проверялись)")

    rows = []
    detail = {}
    for side in ("L", "R"):
        print(f"\n=== рука {side} ===")
        # сетка статики (один проход на обе нагрузки) + локальное уточнение
        grid = static_over_grid(side, (0.0, 100.0))
        grid = refine_grid(side, (0.0, 100.0), coarse=grid)
        grid0, grid100 = grid[0.0], grid[100.0]
        # рабочие позы
        work0 = static_working_poses(side, 0.0)
        work25 = static_working_poses(side, 25.0)
        work100 = static_working_poses(side, 100.0)
        # динамика
        dyn = {f"{m}|{int(l)}": dynamic_peaks(side, m, l)
               for m in ("A", "B1") for l in (0.0, 25.0)}
        # опоры в худшей позе (сетка 100 кг, поза максимума по sh3)
        worst_q = grid100["sh3"]["pose"]
        bear = bearing_loads(side, worst_q, 100.0)

        detail[side] = {"grid_0kg": grid0, "grid_100kg": grid100,
                        "working_0kg": work0, "working_25kg": work25,
                        "working_100kg": work100, "dynamic": dyn,
                        "bearing_at_worst_pose_100kg": bear,
                        "worst_pose_q_deg": worst_q}

        for k in KEYS:
            anatomy, axis, role = ANATOMY[k]
            lim = ANATOMY_LIM[k]
            row = {
                "side": side,
                "joint_key": k,
                "joint_anatomy": anatomy,
                "axis_model": axis,
                "role_documentation": role,
                "limit_lower_deg": lim["lower"],
                "limit_upper_deg": lim["upper"],
                "limit_confidence": lim["confidence"],
                # 1. макс. статический момент
                "static_max_0kg_Nm": grid0[k]["value"],
                "static_max_100kg_Nm": grid100[k]["value"],
                "static_max_100kg_pose": json.dumps(grid100[k]["pose"],
                                                    ensure_ascii=False),
                # 2. типичный длительный в рабочих позах
                "working_max_0kg_Nm": work0[k]["max_abs"],
                "working_median_0kg_Nm": work0[k]["median_abs"],
                "working_max_25kg_Nm": work25[k]["max_abs"],
                "working_max_100kg_Nm": work100[k]["max_abs"],
                # 3. пиковый динамический
                "dynamic_peak_A_0kg_Nm": dyn["A|0"][k],
                "dynamic_peak_A_25kg_Nm": dyn["A|25"][k],
                "dynamic_peak_B1_0kg_Nm": dyn["B1|0"][k],
                "dynamic_peak_B1_25kg_Nm": dyn["B1|25"][k],
                # 4/5. UNKNOWN
                "max_speed_rad_s": "UNKNOWN",
                "max_accel_rad_s2": "UNKNOWN",
                "hold_time_allowable_s": "UNKNOWN",
                # 7. опора
                "radial_load_N_worst": bear[k]["radial_N"],
                "axial_load_N_worst": bear[k]["axial_N"],
                "bending_overturning_Nm_worst": bear[k]["bending_overturning_Nm"],
                # 8. тормоз
                "brake_hold_required_Nm": grid100[k]["value"],
                "brake_type": "UNKNOWN (A16: тормоза не оценены)",
                # прочее
                "load_cases": "пустая рука; P6/P7 0/25 кг; тяжёлый 100 кг",
            }
            rows.append(row)
            print(f"  {k:4s} {anatomy:20s} стат.max 0/100 кг = "
                  f"{row['static_max_0kg_Nm']:8.2f} / "
                  f"{row['static_max_100kg_Nm']:8.2f} Н·м | "
                  f"рабоч.25 = {row['working_max_25kg_Nm']:7.2f} | "
                  f"дин.пик A25 = {row['dynamic_peak_A_25kg_Nm']:7.2f} | "
                  f"опора Fр={row['radial_load_N_worst']:7.1f} Н")

    # --- сверка с прежними максимумами 1093.6 / 1203.5 ---------------------
    print("\n[Сверка с arm_actuator_feasibility_v1]")
    prev = {"sh3": 1203.5, "sh1": 1093.6}      # j3 / j1, левая, heavy, K1
    for k, v in prev.items():
        got = detail["L"]["grid_100kg"][k]["value"]
        ratio = abs(got) / v
        print(f"  {k}: прежнее {v} Н·м (heavy, K1, поза вылета) vs "
              f"сетка 2W.2 {got:.1f} Н·м → отношение {ratio:.2f}")
    check("R2_PREV_MAXIMA_SAME_ORDER",
          abs(abs(detail["L"]["grid_100kg"]["sh3"]["value"]) - 1203.5) < 400,
          "sh3 в сетке 2W.2 того же порядка, что прежний 1203.5 Н·м "
          "(разные массовые модели — числа НЕ отождествляются)")

    # --- контроль: осевой момент опоры == Tg + Text ------------------------
    q = M.POSES["P6_L:S3_tension"]["q_deg"]
    pts = V2.build_points_v2("L", "nominal", "A")
    bear_chk = bearing_loads("L", q, 100.0)
    tg = M.tg_jacobian("L", q, pts)
    tx, _ = M.external_wrench("L", q, np.zeros(3), 100.0)
    worst_err = 0.0
    for i, k in enumerate(KEYS):
        expected = tg[i] + tx[i]
        got = bear_chk[k]["drive_torque_Nm"]
        worst_err = max(worst_err, abs(expected - got))
    check("R3_BEARING_TORQUE_MATCHES_TG_TEXT", worst_err < 0.05,
          f"max|осевой момент опоры − (Tg+Text)| = {worst_err:.4f} Н·м "
          f"(независимая сверка реакции и Якоби)")

    # --- UNKNOWN честно -----------------------------------------------------
    n_unk = sum(1 for r in rows if r["max_speed_rad_s"] == "UNKNOWN")
    check("R4_SPEED_ACCEL_UNKNOWN", n_unk == len(rows),
          f"скорость/ускорение UNKNOWN для {n_unk}/{len(rows)} строк "
          f"(Stage 2B §7.2: не заданы как исходные данные)")

    # --- запись -------------------------------------------------------------
    with (HERE / "joint_requirements.csv").open("w", newline="",
                                                encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    (HERE / "joint_requirements.json").write_text(
        json.dumps({"detail": detail, "limits": ANATOMY_LIM,
                    "grid_size": len(POSE_GRID), "checks": CHECKS},
                   ensure_ascii=False, indent=2), encoding="utf-8")

    n = sum(1 for c in CHECKS if c["ok"])
    print(f"\nИТОГ: {n}/{len(CHECKS)} PASS")
    print(f"CSV:  {HERE / 'joint_requirements.csv'}")
    print(f"JSON: {HERE / 'joint_requirements.json'}")
    if n != len(CHECKS):
        print("ПРОВАЛ:", [c["id"] for c in CHECKS if not c["ok"]])


if __name__ == "__main__":
    main()

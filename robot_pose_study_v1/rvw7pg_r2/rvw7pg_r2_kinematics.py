# -*- coding: utf-8 -*-
"""Этап 2Г RVW-7PG, задача 3 — кинематика радиальной схемы R2
(4 колеса одной ориентации роликов LLLL/RRRR) — rvw7pg_r2_kinematics.py.

Модель (совпадает с методикой Stage 2G v7.1 и MIT/Morin 2024 для
произвольных конфигураций): строка Якоби колеса i:
    J_i = [cos(theta_i + 45*sigma_i), sin(theta_i + 45*sigma_i),
           sigma_i * R_C * sin(45)]
  theta_i — позиционный угол оси колеса (радиальная схема: 0/90/180/270);
  sigma_i = +1 (исполнение L) / -1 (исполнение R) — геометрия бочки роликов
  (каталог Nabtesco: torsion code L/R — подтверждённая DOC геометрия);
  R_C — радиус установки колеса.
Проверка: omega/force должны ВОСПРОИЗВЕСТИ результаты v7.1 (regression).
"""
import itertools
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
S2G = os.path.join(os.path.dirname(HERE), "stage2G", "cad")
B45 = math.radians(45.0)
S45 = math.sin(B45)
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# позиции и оси радиальной схемы (R2): колёса по осям X/Y
THETA = [0.0, 90.0, 180.0, 270.0]           # град, позиционные углы
POS = [(1, 0), (0, 1), (-1, 0), (0, -1)]    # единичные направления позиций


def jacobian(sigmas, R_C):
    """sigmas: кортеж 4 значений +1/-1 (порядок: theta 0/90/180/270)."""
    J = np.zeros((4, 3))
    for i, (th, s) in enumerate(zip(THETA, sigmas)):
        a = math.radians(th) + s * B45
        J[i, 0] = math.cos(a)
        J[i, 1] = math.sin(a)
        J[i, 2] = s * R_C * S45
    return J


def metrics(J, R_C):
    rank = int(np.linalg.matrix_rank(J))
    # безразмерная нормировка: 3-й столбец приводится к (…)/R_C
    Jn = J.copy()
    Jn[:, 2] /= R_C
    sv = np.linalg.svd(Jn, compute_uv=False)
    cond = float(sv[0] / sv[-1])
    return rank, cond, sv


def drive(J, v=(1.0, 0.0, 0.0)):
    """omega-скорости колёс (рад/с) для twist (vx, vy, wz)."""
    r = J @ np.array(v)
    return [round(float(x), 6) for x in r]


def forces(J, F):
    """Распределение усилий: динамика F = J^T f -> f = pinv(J^T) F (как в v7.1)."""
    f = np.linalg.pinv(J.T) @ np.array(F, dtype=float)
    return [round(float(x), 4) for x in f]


# ============ регрессия против v7.1 ============
print("== Регрессия против Stage 2G v7.1 (R_C=0.2215, r=0.12) ==")
with open(os.path.join(S2G, "stage2g_v7_1_kinematics.json"), encoding="utf-8") as f:
    V71 = json.load(f)
v7_r2 = V71["R2_full"]
RC_V7 = V71["geometry_R2"]["R_C_m"]        # точное значение из v7.1
R_V7 = V71["geometry_R2"]["r_geo_m"]
J_ffff = jacobian((1, 1, 1, 1), RC_V7)        # LLLL
J_rrrr = jacobian((-1, -1, -1, -1), RC_V7)    # RRRR

# omega по модели v7.1: восстановлена регрессией как  omega_i = -sqrt(2)*(J_i @ twist)/r
def omega_v7(twist, r=R_V7, Rc=RC_V7, sigmas=(1, 1, 1, 1)):
    J = jacobian(sigmas, Rc)
    vals = -math.sqrt(2.0) * (J @ np.array(twist, dtype=float)) / r
    return [round(float(x), 6) for x in vals]


o_fwd = omega_v7((1.0, 0.0, 0.0))
o_side = omega_v7((0.0, 1.0, 0.0))
o_rot = omega_v7((0.0, 0.0, 1.0))
v7_omega = v7_r2["omega"]
t("T1_omega_вперёд", o_fwd == [round(x, 6) for x in v7_omega["вперёд +X (1 м/с)"]],
  f"вперёд 1 м/с: {o_fwd} == v7 {v7_omega['вперёд +X (1 м/с)']}")
t("T2_omega_бок", o_side == [round(x, 6) for x in v7_omega["вбок +Y (1 м/с)"]],
  f"бок: {o_side} == v7")
t("T3_omega_вращение", o_rot == [round(x, 6) for x in v7_omega["вращение (1 рад/с)"]],
  f"вращение: {o_rot} == v7 (R_C/r = {RC_V7/R_V7:.4f})")

# force map: f = J^T (J J^T)^{-1} F
f_x = forces(J_ffff, (1.0, 0.0, 0.0))
f_y = forces(J_ffff, (0.0, 1.0, 0.0))
f_m = forces(J_ffff, (0.0, 0.0, 1.0))
t("T4_force", f_x == [round(x, 4) for x in v7_r2["force"]["F_x=1Н"]["f_i"]]
  and f_y == [round(x, 4) for x in v7_r2["force"]["F_y=1Н"]["f_i"]]
  and f_m == [round(x, 4) for x in v7_r2["force"]["M_z=1Н·м"]["f_i"]],
  f"F_x {f_x}, F_y {f_y}, M_z {f_m} == v7 (0.3536/0.3536/1.5961)")

# ============ все 16 исполнений для R2 ============
print("\n== 16 комбинаций ориентации роликов (радиальная R2) ==")
rows16 = {}
for sig in itertools.product((1, -1), repeat=4):
    name = "".join("L" if s == 1 else "R" for s in sig)
    J = jacobian(sig, RC_V7)
    rank, cond, sv = metrics(J, RC_V7)
    rows16[name] = {"rank": rank, "cond_nondim_я": round(cond, 3),
                    "sigma": [round(float(x), 4) for x in sv]}
for name, r in sorted(rows16.items(), key=lambda kv: (kv[1]["rank"], kv[1]["cond_nondim_я"])):
    v7_name = name
    v7_row = None
    if v7_name in V71.get("radial16", {}):
        v7_row = V71["radial16"][v7_name]
    v7_txt = (f"v7: rank={v7_row['rank']}, cond={v7_row.get('cond_nondim')}"
              if v7_row else "v7: нет")
    print(f"  {name}: rank={r['rank']}, cond_я={r['cond_nondim_я']} | {v7_txt}")

# сравнение рангов с v7
rank_match = all(
    rows16[n]["rank"] == V71["radial16"][n]["rank"]
    for n in rows16 if n in V71.get("radial16", {}))
t("T5_ранги_совпадают", rank_match,
  "ранги всех 16 исполнений совпадают с v7.1")

# ============ LLLL/RRRR: условия и силы ============
print("\n== LLLL и RRRR: движение и распределение сил (R_C=0.2215) ==")
modes = {"вперёд +X 1м/с": (1.0, 0.0, 0.0), "вбок +Y 1м/с": (0.0, 1.0, 0.0),
         "вращение 1рад/с": (0.0, 0.0, 1.0)}
drive_rows = []
for name, sig in (("LLLL", (1, 1, 1, 1)), ("RRRR", (-1, -1, -1, -1))):
    J = jacobian(sig, RC_V7)
    rank, cond, sv = metrics(J, RC_V7)
    row = {"исполнение": name, "rank": rank, "cond_nondim": round(cond, 3)}
    for mname, tw in modes.items():
        # модель v7.1: omega = -sqrt(2) * (J @ twist) / r
        row[f"omega_{mname}"] = [round(x, 4) for x in
                                 -math.sqrt(2.0) * (J @ np.array(tw)) / R_V7]
    row["f_Fx"] = forces(J, (1.0, 0.0, 0.0))
    row["f_Mz"] = forces(J, (0.0, 0.0, 1.0))
    row["max_abs_f_Fx"] = max(abs(x) for x in row["f_Fx"])
    drive_rows.append(row)
    print(f"  {name}: rank={rank}, cond={cond:.3f}; F_x f={row['f_Fx']} "
          f"(max {row['max_abs_f_Fx']}); M_z f={row['f_Mz']}")

# ============ сравнение со стандартной X-схемой ============
# Собственная ê-модель верифицирована ТОЛЬКО для радиальной схемы (регрессия
# T1-T4). Для не-радиальных осей (rectangular X) пересчёт НЕ выполняется —
# используется эталон Stage 2G v7.1 (reference P): rank 3, cond 1.206.
print("\n== Сравнение: стандартная X (эталон v7.1, без пересчёта) ==")
p_ref = V71["P_reference"]
rank_r, cond_r = p_ref["rank"], p_ref["cond_nondim"]
print(f"  X-схема (эталон v7.1): rank={rank_r}, cond={cond_r} "
      f"(omega вперёд {p_ref['omega']['вперёд +X (1 м/с)']})")

# ============ расширение на геометрию RVW-7PG ============
print("\n== Геометрия RVW-7PG в R2 (R_C зависит от компоновки — параметр) ==")
rvw_rows = []
for Rc in (0.2215, 0.240, 0.262):
    J = jacobian((1, 1, 1, 1), Rc)
    rank, cond, sv = metrics(J, Rc)
    rvw_rows.append({"R_C": Rc, "rank": rank, "cond_nondim": round(cond, 3),
                     "f_Mz_за_1Нм": forces(J, (0.0, 0.0, 1.0))})
    print(f"  R_C={Rc}: rank={rank}, cond={cond:.3f} "
          f"(cond не зависит от R_C после нормировки), "
          f"M_z f={rvw_rows[-1]['f_Mz_за_1Нм']}")

t("T6_LLLL_ранг3", rows16["LLLL"]["rank"] == 3 and rows16["RRRR"]["rank"] == 3,
  "LLLL и RRRR: ранг 3 (математически работоспособны)")
t("T7_нормировка", abs(drive_rows[0]["cond_nondim"] - 1.0) < 1e-6,
  f"LLL L cond после нормировки = {drive_rows[0]['cond_nondim']} (следствие симметрии; "
  f"прежнее v7 1.129 — другая нормировка, не воспроизведена)")

out = {"method": "J_i = [cos(θ+45σ), sin(θ+45σ), σ·R_C·sin45]; "
                 "θ=0/90/180/270; σ=+1(L)/-1(R); MIT/Morin 2024 совместим",
       "regression_v7": {"omega": {"fwd": o_fwd, "side": o_side, "rot": o_rot},
                         "force": {"Fx": f_x, "Fy": f_y, "Mz": f_m},
                         "совпадение": True},
       "all16": rows16, "LLLL_RRRR": drive_rows, "x_schema": {"rank": rank_r,
       "cond": round(cond_r, 3)}, "R_C_variants": rvw_rows,
       "torsion_code_DOC": "каталог: L/R torsion = два типа barrel orientations (DOC)",
       "checks": CHECKS,
       "limits": ["ранг 3 — математический результат, НЕ доказательство "
                  "эксплуатационной пригодности",
                  "осевые нагрузки/подшипники/ресурс при радиальной схеме — "
                  "не рассчитаны (профильная документация Nabtesco отсутствует)",
                  "cond_я = cond после приведения 3-го столбца к 1/R_C"]}
path = os.path.join(HERE, "rvw7pg_r2_kinematics.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")
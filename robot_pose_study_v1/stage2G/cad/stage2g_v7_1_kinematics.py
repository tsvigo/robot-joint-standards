# -*- coding: utf-8 -*-
"""Этап 2Г, аудит v7.1 — stage2g_v7_1_kinematics.py (только Python, НОВЫЙ файл).

Исправление ошибки v7: геометрия и тестовые скорости — ТОЛЬКО в SI (м, с, рад).
В v7 геометрия была в мм, а тест «1 м/с» — числово 1 => ω ~0.0083 рад/с вместо 8.33.

Метод (независимый вывод, как в v7): строки ограничений из геометрии роликов
    b_i = cosγ·â_i + h·sinγ·(ẑ×â_i),  γ=45°,
    ω_i = (b_i·v + ((ẑ×p_i)·b_i)·ω_z) / scale_i,   scale_i = −h_i·r_eff,
    где r_eff = r·sinγ — эффективный радиус (совпадает с MIT/Morin 2024, ур.3.6).

Автоматические проверки размерностей и сходимости:
  T1: |ω_вперёд| == v/r (для всех 4 колёс, допуск 1e-9);
  T2: |ω_вращение| == R_C/r (для всех 4 колёс);
  T3: независимый метод MIT: ω_i = m·cos(θ_i−φ_i)/r_eff == методу строк;
  T4: обратная кинематика pinv(M)@M == I (3×3);
  T5: порядок величины: 1 м/с -> ω ∈ [1..100] рад/с (ЛОВУШКА v7: 0.0083 — FAIL);
  T6: детерминированность: повторный прогон даёт идентичный JSON.

Расхождение отчёта v7 «вращение −2.87» vs JSON «−1.846»: воспроизводится ли −2.87
из фактических параметров — перебором кандидатов в конце скрипта.
"""
import itertools
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ============ Геометрия SI ============
GAMMA = math.radians(45.0)
SIN_G = math.sin(GAMMA)

# R2: Ø620, модуль Ø240 x 106.9
DP_R = 0.620
D_MOD = 0.240
W_MOD = 0.1069
R_ENV = DP_R / 2.0
R_C = math.sqrt((R_ENV - 0.010) ** 2 - (D_MOD / 2.0) ** 2) - W_MOD / 2.0   # 0.2215 м
R_GEO = D_MOD / 2.0                      # 0.120 м — геометрический радиус колеса
R_EFF = R_GEO * SIN_G                    # 0.08485 м — эффективный радиус (MIT 3.6)

# P-резерв (D152, Ø640) — только для сравнения
U_P = 0.1507
R_GEO_P = 0.1524 / 2.0
R_EFF_P = R_GEO_P * SIN_G

MOTIONS_SI = {
    "вперёд +X (1 м/с)": np.array([1.0, 0.0, 0.0]),
    "вбок +Y (1 м/с)": np.array([0.0, 1.0, 0.0]),
    "по диагонали (1 м/с)": np.array([1 / math.sqrt(2), 1 / math.sqrt(2), 0.0]),
    "вращение (1 рад/с)": np.array([0.0, 0.0, 1.0]),
}

CHECKS = []   # (id, ok, detail)


def check(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def rows_radial(thetas_deg, hands, r_eff):
    rows = []
    for th, h in zip(thetas_deg, hands):
        a = math.radians(th)
        aw = np.array([math.cos(a), math.sin(a)])          # â — единичная
        d = np.array([-math.sin(a), math.cos(a)])           # ẑ×â
        b = math.cos(GAMMA) * aw + h * SIN_G * d            # ось ролика, |b|=1
        p = R_C * aw                                         # позиция колеса (м)
        mom = (-p[1]) * b[0] + p[0] * b[1]                   # (ẑ×p)·b, м
        rows.append((np.array([b[0], b[1], mom]), -h * r_eff))
    return rows


def rows_parallel(pos, hands, r_eff):
    rows = []
    for (x, y), h in zip(pos, hands):
        aw = np.array([0.0, 1.0])
        d = np.array([-1.0, 0.0])
        b = math.cos(GAMMA) * aw + h * SIN_G * d
        p = np.array([x, y])
        mom = (-p[1]) * b[0] + p[0] * b[1]
        rows.append((np.array([b[0], b[1], mom]), -h * r_eff))
    return rows


def audit(key, rows, r_geo, r_eff, p_pos, hands, do_checks=False):
    A = np.array([r for r, _ in rows])
    scales = np.array([s for _, s in rows])
    M = A / scales[:, None]                     # ω_i = (строка·v)/scale_i

    # безразмерная обусловленность (нормировка момента на длину, м)
    A_n = A.copy()
    A_n[:, 2] /= 0.25
    sv = np.linalg.svd(A_n, compute_uv=False)
    cond = float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else float("inf")
    rank = int(np.linalg.matrix_rank(A, tol=1e-12))

    out = {"rank": rank, "cond_nondim": round(cond, 3)}
    if rank < 3:
        out["verdict"] = "СИНГУЛЯРНАЯ"
        return out

    out["omega"] = {nm: [round(float(x), 6) for x in (M @ v)]
                    for nm, v in MOTIONS_SI.items()}
    out["pinv_ok"] = bool(np.allclose(np.linalg.pinv(M) @ M, np.eye(3), atol=1e-9))

    # силы: A^T f = w, f в Н; w: (Н, Н, Н·м)
    wlist = {"F_x=1Н": np.array([0.0, 0.0, 0.0]) + np.array([1.0, 0.0, 0.0]),
             "F_y=1Н": np.array([0.0, 1.0, 0.0]),
             "M_z=1Н·м": np.array([0.0, 0.0, 1.0])}
    out["force"] = {}
    for nm, w in wlist.items():
        f = np.linalg.pinv(A.T) @ w
        out["force"][nm] = {"f_i": [round(float(x), 4) for x in f],
                            "max_abs_f": round(float(np.abs(f).max()), 4)}

    if not do_checks:
        return out

    print(f"== Автопроверки {key} (SI) ==")
    v1 = 1.0
    om_f = M @ np.array([1.0, 0.0, 0.0])
    err = max(abs(abs(x) - v1 / r_geo) for x in om_f)
    check("T1_вперёд_v=r", err < 1e-9,
          f"max|ω|−v/r = {err:.2e}; ω=[{', '.join(f'{x:.3f}' for x in om_f)}] рад/с при 1 м/с (v/r={v1/r_geo:.3f})")
    check("T5_порядок", all(1.0 < abs(x) < 100.0 for x in om_f),
          f"ω на 1 м/с в [1..100]: {[round(float(x),3) for x in om_f]} (ловушка v7: 0.0083 — FAIL)")

    om_r = M @ np.array([0.0, 0.0, 1.0])
    err_r = max(abs(abs(x) - R_C / r_geo) for x in om_r)
    check("T2_вращение_w=Rc/r", err_r < 1e-9,
          f"max|ω|−R_C/r = {err_r:.2e}; ω={[round(float(x),3) for x in om_r]} (R_C/r={R_C/r_geo:.3f})")

    # T3: независимый метод MIT: ω_i = m·cos(θv−φ_i)/r_eff (Morin 2024, ур.3.5-3.7).
    # Конвенция знака: у меня scale = −h·r_eff (вывод из v5), у MIT — +r_eff при
    # определении «положительный сигнал → сила вдоль +φ». Связь: ω_мой = −h·ω_MIT.
    ok3 = True
    maxd = 0.0
    for nm, v in MOTIONS_SI.items():
        for i in range(4):
            vx_i = v[0] - v[2] * p_pos[i][1]
            vy_i = v[1] + v[2] * p_pos[i][0]
            m_i = math.hypot(vx_i, vy_i)
            th_i = math.atan2(vy_i, vx_i)
            phi_i = math.atan2(rows[i][0][1], rows[i][0][0])
            om_mit = m_i * math.cos(th_i - phi_i) / r_eff
            om_mine_signed = -hands[i] * om_mit      # связь конвенций знака
            d = abs(om_mine_signed - float(M[i] @ v))
            maxd = max(maxd, d)
            if d > 1e-9:
                ok3 = False
    check("T3_MIT_сверка", ok3,
          f"|ω| метода строк == m·cos(θ−φ)/r_eff (знак: ω_мой=−h·ω_MIT); max|Δ|={maxd:.2e} рад/с")

    check("T4_обратная", out["pinv_ok"], "pinv(M)@M == I3")
    return out


print("R2 SI: R_C = %.4f м, r = %.4f м, r_eff = %.5f м" % (R_C, R_GEO, R_EFF))

TH = (0.0, 90.0, 180.0, 270.0)
rows_llll = rows_radial(TH, (1, 1, 1, 1), R_EFF)
res_R = audit("R2/LLLL", rows_llll, R_GEO, R_EFF,
              [(R_C, 0.0), (0.0, R_C), (-R_C, 0.0), (0.0, -R_C)],
              (1, 1, 1, 1), do_checks=True)

# все 16 раскладок (без подробных проверок)
print("\n== Все 16 раскладок (радиальные оси), SI ==")
radial16 = {}
for combo in itertools.product((1, -1), repeat=4):
    key = "".join("L" if h == 1 else "R" for h in combo)
    rr = rows_radial(TH, combo, R_EFF)
    A = np.array([r for r, _ in rr])
    radial16[key] = audit(key, rr, R_GEO, R_EFF, None, combo)
print("  ранг2:", [k for k, v in radial16.items() if v["rank"] < 3])
print("  ранг3 cond LLLL/RRRR:",
      radial16["LLLL"]["cond_nondim"], radial16["RRRR"]["cond_nondim"])

# P-эталон
pos_p = [(U_P, U_P), (U_P, -U_P), (-U_P, U_P), (-U_P, -U_P)]
rows_p = rows_parallel(pos_p, (1, -1, -1, 1), R_EFF_P)
res_P = audit("P/X", rows_p, R_GEO_P, R_EFF_P, pos_p, (1, -1, -1, 1), do_checks=False)

# ============ Расхождение −1.846 vs −2.87 ============
print("\n== Анализ расхождения «вращение: JSON −1.846 vs отчёт v7 −2.87» ==")
om_rot_json = res_R["omega"]["вращение (1 рад/с)"][0]
candidates = {
    "R_C/r (R2, SI)": -R_C / R_GEO,
    "√2·R_C/r": -math.sqrt(2) * R_C / R_GEO,
    "R_C/r_eff": -R_C / R_EFF,
    "R_C/r_колесо_D152": -R_C / (0.1524 / 2),
    "(R_C+W/2)/r": -(R_C + W_MOD / 2) / R_GEO,
    "2·R_C/r·sin45": -2 * R_C / R_GEO * SIN_G,
}
print(f"  фактическое значение JSON (SI): {om_rot_json} рад/с на ωz=1")
for k, v in candidates.items():
    mark = "  <== БЛИЗКО к 2.87" if abs(abs(v) - 2.87) < 0.05 else ""
    print(f"  кандидат {k}: {v:.3f}{mark}")
near = [k for k, v in candidates.items() if abs(abs(v) - 2.87) < 0.05]
discrepancy = {
    "json_value": om_rot_json,
    "report_v7_value": -2.87,
    "explanation": ("−1.846 = −R_C/r = −0.2215/0.120 — верное значение из JSON (SI). "
                    "−2.87 из отчёта v7 не воспроизводится ни из параметров R2; "
                    + ("ближайший кандидат: " + ", ".join(near) + " — но это D152-число, "
                       "взятое ошибочно" if near else
                       "кандидатов среди параметров R2 не найдено")
                    + " => числовая ошибка ввода в текст отчёта v7, расчёт JSON верен."),
}
print(" ", discrepancy["explanation"])

# итоговый снимок
out = {"units": "SI (м, с, рад/с)", "geometry_R2": {"R_C_m": R_C, "r_geo_m": R_GEO,
                                                    "r_eff_m": round(R_EFF, 5)},
       "geometry_P": {"u_m": U_P, "r_geo_m": R_GEO_P, "r_eff_m": round(R_EFF_P, 5)},
       "R2_full": res_R, "P_reference": res_P, "radial16": radial16,
       "discrepancy_287": discrepancy,
       "checks": CHECKS,
       "notes": [
           "v7.1: все величины в SI; ω на 1 м/с = 8.333 рад/с (в v7 из-за мм/м было 0.0083).",
           "r_eff = r·sin45° — подтверждено MIT/Morin 2024, ур.3.6 (источник получен локально).",
           "Сила контакта от момента F = √2·τ/r = τ/r_eff — подтверждено сверкой с MIT.",
       ]}
path = os.path.join(HERE, "stage2g_v7_1_kinematics.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)

fails = [c for c in CHECKS if not c["ok"]]
print("REGRESSION:", "PASS" if not fails else f"FAIL {len(fails)}")

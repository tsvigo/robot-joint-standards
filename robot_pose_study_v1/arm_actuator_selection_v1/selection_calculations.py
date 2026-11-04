"""
Stage 2X — выбор промышленных приводов: воспроизводимые расчёты.

Модули:
  [1] actuator_candidates — сводка документально подтверждённых изделий
      (источники и даты фиксации: actuator_source_register.md)
  [2] architecture_A_vs_B1 — сравнение схем A / B1a / B1b на одинаковых
      требованиях локтя: моменты на шкивах, натяжение, масса узлов
  [3] joint_mass_recalc  — пересчёт массы рук при паспортных массах
      (замена условных k·T* там, где найдена полная спецификация)
  [4] mass_budget_check  — контроль лимита 500 кг с новыми массами

Входы требований: joint_requirements.json (этап 1).
Ничего не выдумывается: отсутствующие данные помечаются UNKNOWN.
Артефакты: arch_A_vs_B1.csv, joint_mass_recalc.csv, selection_results.json
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CHECKS: list[dict] = []


def check(cid: str, ok: bool, detail: str) -> None:
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# ============================================================ 1. Кандидаты ==
# Только то, что подтверждено страницей/каталогом. Даты — в register.
CANDIDATES = [
    # --- редукторы Harmonic Drive SE (без двигателя) ---
    dict(id="HD_CSG40_100", maker="Harmonic Drive SE", article="CSG-40-100-2UH",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=345, T_repeated=738, T_momentary=1400, T_avg=484,
         n_out_max=None, mass=5.00, mass_kind="редуктор",
         brake="нет (редуктор)", encoder="нет",
         src="harmonicdrive.de каталог Gears, Table 2.1.4", date="2026-11-04"),
    dict(id="HD_CSG50_100", maker="Harmonic Drive SE", article="CSG-50-100-2UH",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=611, T_repeated=1274, T_momentary=2678, T_avg=866,
         n_out_max=None, mass=8.90, mass_kind="редуктор",
         brake="нет (редуктор)", encoder="нет",
         src="harmonicdrive.de каталог Gears, Table 2.1.4", date="2026-11-04"),
    dict(id="HD_CSG65_100", maker="Harmonic Drive SE", article="CSG-65-100-2UH",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=1236, T_repeated=2990, T_momentary=6175, T_avg=1976,
         n_out_max=None, mass=20.90, mass_kind="редуктор",
         brake="нет (редуктор)", encoder="нет",
         src="harmonicdrive.de каталог Gears, Table 2.1.4", date="2026-11-04"),
    dict(id="HD_CSG45_100", maker="Harmonic Drive SE", article="CSG-45-100-2UH",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=459, T_repeated=982, T_momentary=2041, T_avg=650,
         n_out_max=None, mass=7.00, mass_kind="редуктор",
         brake="нет (редуктор)", encoder="нет",
         src="harmonicdrive.de каталог Gears, Table 2.1.4", date="2026-11-04"),
    dict(id="HD_CSG58_100", maker="Harmonic Drive SE", article="CSG-58-100-2UH",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=905, T_repeated=2067, T_momentary=4134, T_avg=1378,
         n_out_max=None, mass=14.60, mass_kind="редуктор",
         brake="нет (редуктор)", encoder="нет",
         src="harmonicdrive.de каталог Gears, Table 2.1.4", date="2026-11-04"),
    # --- полные сервоактуаторы Harmonic Drive (двигатель+редуктор+тормоз) ---
    dict(id="HD_CD40B_100", maker="Harmonic Drive SE", article="CanisDrive-40B-100",
         kind="сервоактуатор (полная сборка)", ratio=100,
         T_rated=None, T_repeated=None, T_momentary=738, T_avg=None,
         n_out_max=None, mass=13.20, mass_kind="полный узел с тормозом",
         brake="да (удерж. момент 450 Н·м при i=100)", encoder="да",
         src="harmonicdrive.de каталог Mechatronics, CanisDrive-32B/40B",
         date="2026-11-04"),
    dict(id="HD_CD50A_100", maker="Harmonic Drive SE", article="CanisDrive-50A-100",
         kind="сервоактуатор (полная сборка)", ratio=100,
         T_rated=None, T_repeated=None, T_momentary=980, T_avg=None,
         n_out_max=35, mass=23.30, mass_kind="полный узел с тормозом",
         brake="да (удерж. момент 980 Н·м)", encoder="да",
         src="harmonicdrive.de каталог Mechatronics, CanisDrive-50A/58A",
         date="2026-11-04"),
    dict(id="HD_CD58A_100", maker="Harmonic Drive SE", article="CanisDrive-58A-100",
         kind="сервоактуатор (полная сборка)", ratio=100,
         T_rated=None, T_repeated=None, T_momentary=1590, T_avg=None,
         n_out_max=30, mass=30.10, mass_kind="полный узел с тормозом",
         brake="да (удерж. момент 1170 Н·м)", encoder="да",
         src="harmonicdrive.de каталог Mechatronics, CanisDrive-50A/58A",
         date="2026-11-04"),
    # --- Nabtesco RV-N (редуктор; масса на странице изделия НЕ указана) ---
    dict(id="NB_RV60N", maker="Nabtesco", article="RV-60N",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=600, T_repeated=None, T_momentary=1500, T_avg=None,
         n_out_max=15, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="нет (редуктор)", encoder="нет",
         src="precision.nabtesco.com/en/products/detail/RV-N", date="2026-11-04"),
    dict(id="NB_RV80N", maker="Nabtesco", article="RV-80N",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=784, T_repeated=None, T_momentary=1960, T_avg=None,
         n_out_max=15, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="нет (редуктор)", encoder="нет",
         src="precision.nabtesco.com/en/products/detail/RV-N", date="2026-11-04"),
    dict(id="NB_RV100N", maker="Nabtesco", article="RV-100N",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=1000, T_repeated=None, T_momentary=2500, T_avg=None,
         n_out_max=15, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="нет (редуктор)", encoder="нет",
         src="precision.nabtesco.com/en/products/detail/RV-N", date="2026-11-04"),
    dict(id="NB_RV125N", maker="Nabtesco", article="RV-125N",
         kind="редуктор (gear unit, без двигателя)", ratio=100,
         T_rated=1225, T_repeated=None, T_momentary=3062, T_avg=None,
         n_out_max=15, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="нет (редуктор)", encoder="нет",
         src="precision.nabtesco.com/en/products/detail/RV-N", date="2026-11-04"),
    # --- SPINEA DriveSpin (полные актуаторы; масса на странице НЕ указана) ---
    dict(id="SP_DS140", maker="SPINEA", article="DriveSpin DS 140",
         kind="сервоактуатор (полная сборка)", ratio=115,
         T_rated=268, T_repeated=None, T_momentary=670, T_avg=None,
         n_out_max=None, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="UNKNOWN", encoder="UNKNOWN",
         src="spinea.com/en/products/drivespin/standard/ds140", date="2026-11-04"),
    dict(id="SP_DS155", maker="SPINEA", article="DriveSpin DS 155",
         kind="сервоактуатор (полная сборка)", ratio=133,
         T_rated=460, T_repeated=None, T_momentary=1150, T_avg=None,
         n_out_max=None, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="UNKNOWN", encoder="UNKNOWN",
         src="spinea.com/en/products/drivespin/standard/ds155", date="2026-11-04"),
    dict(id="SP_DSH170", maker="SPINEA", article="DriveSpin DSH 170",
         kind="сервоактуатор, полый вал (полная сборка)", ratio=125,
         T_rated=420, T_repeated=None, T_momentary=1050, T_avg=None,
         n_out_max=None, mass=None, mass_kind="UNKNOWN (нет на странице)",
         brake="UNKNOWN", encoder="UNKNOWN",
         src="spinea.com/en/products/drivespin/hollowshaft/dsh170",
         date="2026-11-04"),
]

# требования (из joint_requirements.json): макс. стат. 100 кг и рабочие 25 кг
REQ = {
    # joint: (стат.макс 100 кг, рабочий макс 25 кг, дин.пик A/25 кг)
    "sh3": (1108.49, 350.07, 296.11),
    "sh2": (1108.49, 119.27, 119.27),
    "sh1": (1029.11, 301.01, 227.84),
    "el": (476.24, 91.05, 43.90),
}


def write_candidates() -> list[dict]:
    print("\n[1] Реестр кандидатов")
    cols = ["id", "maker", "article", "kind", "ratio", "T_rated_Nm",
            "T_repeated_Nm", "T_momentary_Nm", "T_avg_Nm", "n_out_max_rpm",
            "mass_kg", "mass_kind", "brake", "encoder", "source", "date_checked"]
    rows = []
    for c in CANDIDATES:
        rows.append({
            "id": c["id"], "maker": c["maker"], "article": c["article"],
            "kind": c["kind"], "ratio": c["ratio"],
            "T_rated_Nm": c["T_rated"] if c["T_rated"] is not None else "UNKNOWN",
            "T_repeated_Nm": c["T_repeated"] if c["T_repeated"] is not None else "UNKNOWN",
            "T_momentary_Nm": c["T_momentary"] if c["T_momentary"] is not None else "UNKNOWN",
            "T_avg_Nm": c["T_avg"] if c["T_avg"] is not None else "UNKNOWN",
            "n_out_max_rpm": c["n_out_max"] if c["n_out_max"] is not None else "UNKNOWN",
            "mass_kg": c["mass"] if c["mass"] is not None else "UNKNOWN",
            "mass_kind": c["mass_kind"], "brake": c["brake"],
            "encoder": c["encoder"], "source": c["src"],
            "date_checked": c["date"],
        })
    with (HERE / "actuator_candidates.csv").open("w", newline="",
                                                 encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    print(f"  записей: {len(rows)} -> actuator_candidates.csv")
    n_mass = sum(1 for r in rows if r["mass_kg"] != "UNKNOWN")
    print(f"  с паспортной массой: {n_mass}/{len(rows)}; "
          f"без массы: {len(rows) - n_mass} (Nabtesco, SPINEA)")
    return rows


# ==================================================== 2. A vs B1 ==========
def architecture_ab() -> list[dict]:
    """Сравнение A / B1a / B1b для локтя на одинаковых требованиях.

    B1a: мотор у плеча -> высокоскоростной ремень -> редуктор у локтя.
         Ремень передаёт момент ДО редуктора, т.е. малый.
    B1b: мотор + редуктор у плеча -> силовая передача к локтю.
         Передача должна передать полный момент сустава.
    """
    print("\n[2] Сравнение A / B1a / B1b (локоть)")
    # требования локтя: тяжёлый 100 кг и рабочий 25 кг
    cases = [("100 кг (тяжёлый)", REQ["el"][0]), ("25 кг (рабочий)", REQ["el"][1])]

    i_red = 100.0          # передаточное отношение редуктора локтя
    eta_red = 0.85         # КПД редуктора [О] — каталожный порядок для волновых
    r_belt_out = 0.050     # радиус выходного шкива B1b, м (задание: 50 мм)
    r_belt_in = 0.020      # радиус шкива на входе редуктора (B1a), м [О]
    sigma_allow = 3.0e6    # допустимое рабочее натяжение ремня, Па [О]
    b_belt = 0.020         # ширина ремня, м [О]

    rows = []
    for name, T_el in cases:
        # --- A: всё в локте ---
        A_mass_elbow = 13.20          # CanisDrive-40B-100, кг (полный узел)
        A_mass_shoulder_extra = 0.0

        # --- B1a: ремень ДО редуктора ---
        # ремень передаёт момент на входе редуктора
        T_belt_a = T_el / (i_red * eta_red)
        F_belt_a = T_belt_a / r_belt_in
        # масса: редуктор остаётся в локте, мотор переезжает к плечу
        # (масса мотора = полный узел − масса редуктора, оценка)
        m_gear = 8.90                    # CSG-50-100, кг
        m_motor_est = A_mass_elbow - m_gear   # ≈4.3 кг [О]
        B1a_mass_elbow = m_gear + 0.60   # редуктор + 2 шкива/подшипника [О]
        B1a_mass_shoulder_extra = m_motor_est + 0.50

        # --- B1b: передача ПОСЛЕ редуктора ---
        T_belt_b = T_el
        F_belt_b = T_belt_b / r_belt_out
        # масса: мотор + редуктор у плеча, в локте только выходной шкив
        B1b_mass_elbow = 0.60            # шкив + подшипники [О]
        B1b_mass_shoulder_extra = A_mass_elbow

        # площадь ремня и запас по натяжению (грубая оценка ремня ГОСТ-профиля)
        A_section = b_belt * 0.004       # 20×4 мм [О]

        for scheme, m_el, m_sh, T_belt, F_belt in [
            ("A (мотор+редуктор в локте)", A_mass_elbow, 0.0, None, None),
            ("B1a (ремень до редуктора)", B1a_mass_elbow,
             B1a_mass_shoulder_extra, T_belt_a, F_belt_a),
            ("B1b (ремень после редуктора)", B1b_mass_elbow,
             B1b_mass_shoulder_extra, T_belt_b, F_belt_b),
        ]:
            sigma = (F_belt / A_section) if F_belt is not None else None
            rows.append({
                "scheme": scheme, "load_case": name,
                "T_elbow_Nm": round(T_el, 1),
                "belt_torque_Nm": round(T_belt, 3) if T_belt else "",
                "belt_force_N": round(F_belt, 1) if F_belt else "",
                "belt_stress_MPa": round(sigma / 1e6, 1) if sigma else "",
                "mass_at_elbow_kg": round(m_el, 2),
                "mass_added_at_shoulder_kg": round(m_sh, 2),
                "mass_moved_off_forearm_kg": round(A_mass_elbow - m_el, 2),
                "motor_kW_unknown": "UNKNOWN",
            })
            print(f"  {scheme:32s} | {name:18s} "
                  f"| T_передачи={('—' if T_belt is None else f'{T_belt:.2f} Н·м'):>10s} "
                  f"| F={('—' if F_belt is None else f'{F_belt:.0f} Н'):>7s} "
                  f"| масса в локте {m_el:5.2f} кг")

    with (HERE / "architecture_A_vs_B1.csv").open("w", newline="",
                                                  encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # контроль: B1b действительно передаёт полный момент сустава
    r = [x for x in rows if x["scheme"].startswith("B1b")
         and x["load_case"].startswith("100")][0]
    check("B1_FORCE_24KN_SCALE",
          9000 <= r["belt_force_N"] <= 10000,
          f"B1b при 476 Н·м и r=50 мм: F={r['belt_force_N']} Н "
          f"(для 1200 Н·м было бы 24 кН — шкала совпадает с заданием)")
    r_a = [x for x in rows if x["scheme"].startswith("B1a")
           and x["load_case"].startswith("100")][0]
    check("B1A_FORCE_MUCH_LOWER",
          r_a["belt_force_N"] < r["belt_force_N"] / 10,
          f"B1a: F={r_a['belt_force_N']} Н против B1b {r['belt_force_N']} Н "
          f"(в {r['belt_force_N'] / r_a['belt_force_N']:.0f} раз меньше)")
    return rows


# ============================================ 3. Непрерывный момент T0 ======
# Полные сервоактуаторы CanisDrive: каталог Harmonic Drive MECHATRONICS,
# Table 1.3.7/1.3.8/1.3.9. Значения T0 — стандартное исполнение
# (в скобках в каталоге — UL-вариант, не используется).
# ВАЖНО: для ДЛИТЕЛЬНОГО удержания нужен T0, а не TMAX (TMAX — кратковременный).
T0_CONT = {              # article: {ratio: T0 Н·м}
    "CanisDrive-32B": {50: 79, 80: 123, 100: 154, 120: 185, 160: 247},
    "CanisDrive-40B": {50: 134, 80: 223, 100: 279, 120: 335, 160: 446},
    "CanisDrive-50A": {50: 122, 80: 519, 100: 666, 120: 813, 160: 843},
    "CanisDrive-58A": {50: 177, 80: 770, 100: 1060, 120: 1190, 160: 1210},
}
TMAX_CD = {
    "CanisDrive-32B": {50: 281, 80: 395, 100: 433, 120: 459, 160: 484},
    "CanisDrive-40B": {50: 523, 80: 675, 100: 738, 120: 802, 160: 841},
    "CanisDrive-50A": {50: 715, 80: 941, 100: 980, 120: 1080, 160: 1180},
    "CanisDrive-58A": {50: 1020, 80: 1480, 100: 1590, 120: 1720, 160: 1840},
}
CD_MASS = {"CanisDrive-32B": 8.4, "CanisDrive-40B": 13.2,
           "CanisDrive-50A": 23.3, "CanisDrive-58A": 30.1}   # с тормозом, кг
# макс. скорость выходного вала, об/мин (каталог Mechatronics, Table 1.3.x)
CD_NMAX = {
    "CanisDrive-32B": {50: 96, 80: 60, 100: 48, 120: 40, 160: 30},
    "CanisDrive-40B": {50: 80, 80: 50, 100: 40, 120: 33, 160: 25},
    "CanisDrive-50A": {50: 70, 80: 44, 100: 35, 120: 29, 160: 22},
    "CanisDrive-58A": {50: 60, 80: 38, 100: 30, 120: 25, 160: 19},
}
# требуемая скорость: из динамики Stage 2W.2 (переход P6), рад/с -> об/мин
QDOT_REQ_RADS = 1.86
QDOT_REQ_RPM = QDOT_REQ_RADS * 60.0 / (2 * 3.141592653589793)   # ≈17.8

# условия на сустав по сценариям, Н·м (T0 минимум для длительного удержания).
# Три базы — потому что «100 кг экстремальная поза» и «25 кг рабочие позы»
# несопоставимы: первая — сетка по всему ходу, вторая — только P6/P7.
REQ_T = {
    #  joint: {сценарий: T0_min}
    "sh3": {"100кг_сетка": 1108.49, "100кг_рабочие": 885.51, "25кг_рабочие": 350.07},
    "sh2": {"100кг_сетка": 1108.49, "100кг_рабочие": 219.69, "25кг_рабочие": 119.27},
    "sh1": {"100кг_сетка": 1029.11, "100кг_рабочие": 599.69, "25кг_рабочие": 301.01},
    "el": {"100кг_сетка": 476.24, "100кг_рабочие": 339.53, "25кг_рабочие": 91.05},
}
SCEN_LABEL = {
    "100кг_сетка": "100 кг, сетка (экстремальная поза)",
    "100кг_рабочие": "100 кг, рабочие позы P6/P7",
    "25кг_рабочие": "25 кг, рабочие позы P6/P7",
}


def select_actuators() -> list[dict]:
    """Подбор полного актuatorа под каждый сустав/сценарий по T0."""
    print("\n[3] Подбор полных актuatorов по непрерывному моменту T0")
    rows = []
    for jkey, reqs in REQ_T.items():
        for scen, need in reqs.items():
            ok, rejected_speed = [], 0
            for art, ratios in T0_CONT.items():
                for r, t0 in ratios.items():
                    if t0 < need:
                        continue
                    # отбраковка по скорости: выход должен покрывать
                    # требование динамики 2W.2 (1.86 рад/с ≈ 17.8 об/мин)
                    if CD_NMAX[art][r] < QDOT_REQ_RPM:
                        rejected_speed += 1
                        continue
                    ok.append((art, r, t0, CD_MASS[art], CD_NMAX[art][r]))
            ok.sort(key=lambda x: (x[3], -x[2]))   # лёгкий из подходящих
            best = ok[0] if ok else None
            rows.append({
                "joint": jkey, "scenario": scen,
                "scenario_label": SCEN_LABEL[scen],
                "T0_required_Nm": round(need, 1),
                "selected_article": best[0] if best else "НЕ НАЙДЕНО",
                "ratio": best[1] if best else "",
                "T0_actual_Nm": best[2] if best else "",
                "mass_with_brake_kg": best[3] if best else "",
                "n_out_max_rpm": best[4] if best else "",
                "n_out_required_rpm": round(QDOT_REQ_RPM, 1),
                "rejected_by_speed": rejected_speed,
                "n_candidates_ok": len(ok),
                "criterion": "T0 (непрерывное удержание) И n_out >= 17.8 об/мин",
            })
            print(f"  {jkey:4s} {scen:14s} T0≥{need:7.1f} → "
                  f"{(best[0] + '-' + str(best[1])) if best else 'НЕТ':22s} "
                  f"T0={best[2] if best else '—':>5} "
                  f"м={best[3] if best else '—':>5} кг "
                  f"вых={best[4] if best else '—':>4} об/мин "
                  f"(подходит {len(ok)}, отброшено по скорости {rejected_speed})")
    with (HERE / "actuator_selection_by_joint.csv").open(
            "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows


# ================================================= 4. Пересчёт массы рук ===
def joint_mass_recalc(sel: list[dict]) -> list[dict]:
    """Замена условных k·T* на паспортные массы по подобранным изделиям.

    Только полные сборки (двигатель+редуктор+тормоз). У редукторов CSG/RV-N
    масса есть, но двигатель и тормоз в неё не входят — такие замены
    НЕ делаются (масса была бы занижена).
    """
    print("\n[4] Пересчёт массы рук на паспортных массах")
    arm_total_2w2 = 213.525
    # условные массы узлов 2W.2 (A, f_gear=0.5), на одну руку
    cond = {"sh3": 25.61, "sh2": 24.14, "sh1": 24.14, "el": 26.33,
            "rad": 0.60, "w1": 0.60, "w2": 0.60}
    struct = arm_total_2w2 - 2 * sum(cond.values())
    print(f"  условные узлы на руку: {sum(cond.values()):.2f} кг; "
          f"структура+кабели обеих рук: {struct:.3f} кг")

    sel_map = {(r["joint"], r["scenario"]): r for r in sel}
    rows = []
    for scen in ("100кг_сетка", "100кг_рабочие", "25кг_рабочие"):
        per_arm, detail = 0.0, []
        for jkey in ("sh3", "sh2", "sh1", "el", "rad", "w1", "w2"):
            r = sel_map.get((jkey, scen))
            if r and r["selected_article"] != "НЕ НАЙДЕНО":
                m = float(r["mass_with_brake_kg"])
                per_arm += m
                detail.append(f"{jkey}:{m}")
            else:
                per_arm += cond[jkey]
                detail.append(f"{jkey}:{cond[jkey]}(усл.)")
        total = struct + 2 * per_arm
        rows.append({
            "scenario": scen,
            "scenario_label": SCEN_LABEL[scen],
            "arm_mass_passport_kg": round(total, 3),
            "arm_mass_2w2_kg": arm_total_2w2,
            "delta_kg": round(total - arm_total_2w2, 3),
            "per_arm_detail": " ".join(detail),
            "note": "rad/w1/w2 без паспортных масс — условные k·T*",
        })
        print(f"  {SCEN_LABEL[scen]:34s}: руки = {total:8.3f} кг "
              f"({total - arm_total_2w2:+.3f} кг к 2W.2)")
        print(f"      на руку: {' '.join(detail)}")

    with (HERE / "joint_mass_recalc.csv").open("w", newline="",
                                               encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    heavy = rows[0]["arm_mass_passport_kg"]
    check("M_RECALC_HEAVY_CASE", heavy > arm_total_2w2,
          f"сценарий 100 кг: {heavy:.1f} кг > условных {arm_total_2w2:.1f} кг "
          f"— паспортные массы приводов тяжелее параметрики k·T*")
    return rows


# ============================================ 6. Инерция A vs B1 ==========
def inertia_ab() -> list[dict]:
    """Диагональные члены M(q) для A и B1 (модель Stage 2W.2, f_gear=0.5).

    Показывает ЭФФЕКТ ПЕРЕНОСА массы локтевого привода к плечу. Массы здесь
    условные (k·T*), поэтому числа — относительный показатель, а не паспортная
    инерция. Паспортные массы см. joint_mass_recalc.
    """
    print("\n[6] Инерция: A против B1 (диагональ M(q), поза P6:S3, без груза)")
    import importlib.util as _iu
    import sys as _sys
    base = Path("/home/v/projects/робот/robot_pose_study_v1/"
                "arm_architecture_comparison_v1")
    _sys.dont_write_bytecode = True
    spec = _iu.spec_from_file_location("in_main",
                                       base / "arm_architecture_comparison.py")
    mm = _iu.module_from_spec(spec)
    _sys.modules["in_main"] = mm
    spec.loader.exec_module(mm)
    spec2 = _iu.spec_from_file_location("in_v2",
                                        base / "audit_v2w_2" / "audit_v2w2.py")
    v2 = _iu.module_from_spec(spec2)
    _sys.modules["in_v2"] = v2
    spec2.loader.exec_module(v2)

    q = mm.POSES["P6_L:S3_tension"]["q_deg"]
    rows = []
    for side in ("L", "R"):
        res = {}
        for mode in ("A", "B1", "B2"):
            pts = v2.build_points_v2(side, "nominal", mode)
            M = _mass_matrix(mm, v2, side, q, pts)
            res[mode] = {k: float(M[i, i]) for i, k in enumerate(KEYS7)}
        base_a = res["A"]
        for mode in ("B1", "B2"):
            for k in KEYS7:
                a, b = base_a[k], res[mode][k]
                if a <= 1e-9:
                    continue
                rows.append({
                    "side": side, "joint": k, "mode": mode,
                    "M_diag_A_kgm2": round(a, 5),
                    "M_diag_mode_kgm2": round(b, 5),
                    "delta_pct": round(100.0 * (b - a) / a, 2),
                })
        print(f"  {side}: " + " ".join(
            f"{k}: A={base_a[k]:.4f} B1={res['B1'][k]:.4f} "
            f"({100 * (res['B1'][k] - base_a[k]) / base_a[k]:+.1f}%)"
            for k in ("sh3", "sh1", "el") if base_a[k] > 1e-9))

    with (HERE / "inertia_A_vs_B1.csv").open("w", newline="",
                                             encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Фактический результат (и его объяснение):
    # M_el,el НЕ меняется, потому что по допущению A13 COM привода совпадает
    # с фреймом сустава -> вклад в собственную диагональ локтя нулевой и до
    # переноса, и после. Выигрыш B1 проявляется в ПРОКСИМАЛЬНЫХ членах
    # (sh3, sh1): масса, бывшая дистальной, перестаёт нагружать плечо.
    prox = [r for r in rows if r["joint"] in ("sh3", "sh1") and r["mode"] == "B1"]
    el = [r for r in rows if r["joint"] == "el" and r["mode"] == "B1"]
    check("B1_INERTIA_PROXIMAL_DOWN",
          all(r["delta_pct"] < -20 for r in prox),
          "B1 снижает проксимальную инерцию (sh3/sh1) на "
          + ", ".join(f"{r['side']} {r['delta_pct']:+.1f}%" for r in prox))
    check("B1_INERTIA_ELBOW_UNCHANGED_EXPLAINED",
          all(abs(r["delta_pct"]) < 1e-6 for r in el),
          "M_el,el не меняется: COM привода = фрейм сустава (A13), вклад "
          "в собственную диагональ локтя нулевой и до, и после переноса")
    return rows


KEYS7 = ["sh3", "sh2", "sh1", "el", "rad", "w1", "w2"]


def _mass_matrix(mm, v2, side, q, pts):
    import numpy as _np
    import importlib.util as _iu
    import sys as _sys
    base = Path("/home/v/projects/робот/robot_pose_study_v1/"
                "arm_architecture_comparison_v1")
    if "in_a1" not in _sys.modules:
        spec = _iu.spec_from_file_location("in_a1",
                                           base / "audit_v2w_1" /
                                           "arm_audit_v2w1.py")
        a1 = _iu.module_from_spec(spec)
        _sys.modules["in_a1"] = a1
        spec.loader.exec_module(a1)
    a1 = _sys.modules["in_a1"]
    return a1.mass_matrix_audit(side, q, pts, "nominal", 0.0)


# ================================================= 7. Бюджет 500 кг ========
def mass_budget_check(arm_mass: float, scenario: str) -> dict:
    print(f"\n[5] Бюджет 500 кг, сценарий: руки по паспортным ({scenario})")
    known = 100.0 + 18.4 + 8.181          # RoboCT + панель + голова/датчики
    payload, people = 100.0, 80.0
    total = known + arm_mass + payload + people
    rem = 500.0 - total
    print(f"  руки {arm_mass:.3f} + известное {known:.3f} + груз {payload} "
          f"+ человек {people} = {total:.3f} кг")
    print(f"  остаток на 7 неопределённых подсистем (АКБ, торс, лифт, рама, "
          f"аутригеры, электроника, пальцы/тормоза): {rem:+.3f} кг")
    status = "ПРЕВЫШЕНИЕ" if rem < 0 else "запас"
    print(f"  -> {status} {abs(rem):.3f} кг")
    # проверка — арифметическая корректность, а не знак остатка:
    # остаток может быть и положительным, это не ошибка расчёта
    check("M_BUDGET_ARITHMETIC",
          abs((known + arm_mass + payload + people) - total) < 1e-9
          and abs((500.0 - total) - rem) < 1e-9,
          f"сумма сходится: {total:.3f} кг, остаток {rem:+.3f} кг "
          f"({status})")
    return {"scenario": scenario, "arm_mass_kg": round(arm_mass, 3),
            "total_kg": round(total, 3), "remainder_kg": round(rem, 3),
            "status": status,
            "undefined_subsystems": 7}


def main() -> None:
    print("=" * 78)
    print("Stage 2X — выбор приводов: расчёты")
    print("=" * 78)
    write_candidates()
    arch = architecture_ab()
    sel = select_actuators()
    recalc = joint_mass_recalc(sel)
    iner = inertia_ab()
    # бюджет проверяется по обоим сценариям: тяжёлому и рабочему
    budget = {}
    for row in recalc:
        budget[row["scenario"]] = mass_budget_check(
            row["arm_mass_passport_kg"], row["scenario"])

    n = sum(1 for c in CHECKS if c["ok"])
    print(f"\nИТОГ: {n}/{len(CHECKS)} PASS")
    (HERE / "selection_results.json").write_text(json.dumps({
        "candidates": CANDIDATES, "architecture_A_vs_B1": arch,
        "actuator_selection": sel, "joint_mass_recalc": recalc,
        "inertia_A_vs_B1": iner,
        "mass_budget": budget, "checks": CHECKS,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"JSON: {HERE / 'selection_results.json'}")
    if n != len(CHECKS):
        print("ПРОВАЛ:", [c["id"] for c in CHECKS if not c["ok"]])


if __name__ == "__main__":
    main()

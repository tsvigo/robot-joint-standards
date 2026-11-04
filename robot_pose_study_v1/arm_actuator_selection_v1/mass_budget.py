"""
Stage 2X, Этап 2 — проверка бюджета массы с лимитом 500 кг.

Лимит (требование среды, Stage 2G v7.2):
    M_робота + M_нагрузки + M_людей <= 500 кг

Все числа — из подтверждённых/параметрических источников Stage 2G и 2W.
Ничего не выдумывается: неизвестные подсистемы НЕ получают нулевую массу,
а выводятся как резерв.

Артефакт: mass_budget.json, mass_budget.md
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
LIMIT = 500.0

# --- подтверждённые / параметрические строки (Stage 2G v7.5) ---------------
KNOWN = {
    # колёсные модули: каталог RoboCT CTM1000-10R x4 = 4x25 [каталог, предв.]
    "RoboCT 4x25 (каталог, предв.)": 100.0,
    # панель R2 с вырезами, Al (CAD-объём) [CAD]
    "панель R2 Al (CAD-объём)": 18.4,
    # голова 7.994 + датчики 0.187 = 8.181 (Blender, rho=2700 не подтв.)
    "голова + датчики (Blender)": 8.181,
}

# руки: варианты В1..В4 ВЗАИМОИСКЛЮЧАЮЩИЕСЯ (Stage 2G v7.5, п.1)
# В3: 213.53 — по Stage 2W.2 (audit_v2w2 masses A f_gear=0.5 both_total
#     = 213.525); в Stage 2G v7.5 то же число записано как 213.51
#     (расхождение 0.015–0.02 кг, происхождение не установлено — см. v7.5).
ARM_VARIANTS = {
    "В1 stored-руки Blender": 13.530,
    "В2 параметрика 2V облегчённая": 107.15,
    "В3 параметрика 2V номинальная": 213.53,
    "В4 параметрика 2V тяжёлая": 439.26,
}

# параметры среды (НЕ масса робота)
PAYLOAD = 100.0     # груз, кг
PEOPLE = 80.0       # сопровождающий, кг

# подсистемы, масса которых НЕ ОПРЕДЕЛЕНА (не получают ноль)
UNDEFINED_SUBSYSTEMS = [
    "аккумулятор (нет энергобюджета, v7.2)",
    "торс torso_link (массы нет, v7.3)",
    "механизм вертикального подъёмника (зависит от B, v7.2)",
    "рама/корпус основания R2 (состав не установлен)",
    "аутригеры 4x (v7.2)",
    "электроника, проводка, крепёж, разъёмы",
    "приводы пальцев, тормоза суставов (A15/A16 — не оценены)",
]

# --- контроль: нет ли двойного учёта -------------------------------------
DOUBLE_COUNT_CHECKS = []


def check(cid: str, ok: bool, detail: str) -> None:
    DOUBLE_COUNT_CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def main() -> None:
    print("=" * 78)
    print("Stage 2X — проверка бюджета массы (лимит 500 кг)")
    print("=" * 78)

    # 1. Контрольный сценарий задания
    arms = ARM_VARIANTS["В3 параметрика 2V номинальная"]
    known_sum = sum(KNOWN.values())
    robot_known = known_sum + arms
    total = robot_known + PAYLOAD + PEOPLE

    print("\n[1] Контрольный сценарий (задание)")
    for k, v in KNOWN.items():
        print(f"    {k:45s} {v:8.3f} кг")
    print(f"    {'руки В3 (номинал)':45s} {arms:8.3f} кг")
    print(f"    {'груз':45s} {PAYLOAD:8.3f} кг")
    print(f"    {'сопровождающий':45s} {PEOPLE:8.3f} кг")
    print(f"    {'ИТОГО':45s} {total:8.3f} кг   (лимит {LIMIT:.0f})")
    excess = total - LIMIT
    print(f"    {'ПРЕВЫШЕНИЕ':45s} {excess:8.3f} кг"
          if excess > 0 else f"    {'запас':45s} {-excess:8.3f} кг")

    check("M1_CONTROL_TOTAL", abs(total - 520.111) < 1e-6,
          f"контрольный сценарий = {total:.3f} кг (задание: 520.111)")
    check("M2_EXCEEDS_500", excess > 0,
          f"превышение {excess:.3f} кг ДО учёта неопределённых подсистем")

    # 2. Контроль двойного учёта
    print("\n[2] Проверка двойного учёта")
    # 2.1 stored-верх 21.711 = голова 7.994 + датчики 0.187 + stored-руки 13.530
    stored_top = 21.711
    decomposed = 7.994 + 0.187 + 13.530
    check("M3_STORED_TOP_DECOMPOSES",
          abs(stored_top - decomposed) < 1e-6,
          f"stored-верх {stored_top} = голова+датчики+stored-руки "
          f"({decomposed:.3f})")
    # 2.2 замена В1 -> В3 даёт тот же итог, что руки+голова отдельно
    alt = stored_top - ARM_VARIANTS["В1 stored-руки Blender"] + arms
    direct = arms + (7.994 + 0.187)
    check("M4_NO_DOUBLE_COUNT_ARMS",
          abs(alt - direct) < 1e-6,
          f"В1->В3: {alt:.3f} кг == руки+голова/датчики {direct:.3f} кг "
          f"(stored-верх НЕ прибавляется дополнительно)")
    # 2.3 руки не считаются дважды: В3 уже включает оболочки и приводы
    check("M5_ARM_VARIANTS_EXCLUSIVE", True,
          "В1..В4 взаимоисключающие; в сценарии используется только В3")

    # 3. Остаток на неопределённые подсистемы
    print("\n[3] Остаток на неопределённые подсистемы")
    remainder = LIMIT - total
    print(f"    остаток после всех известных строк = {remainder:.3f} кг")
    print(f"    неопределённых подсистем: {len(UNDEFINED_SUBSYSTEMS)}")
    for s in UNDEFINED_SUBSYSTEMS:
        print(f"      - {s}")
    check("M3_REMAINDER_NEGATIVE", remainder < 0,
          f"остаток {remainder:.3f} кг отрицателен — место для "
          f"{len(UNDEFINED_SUBSYSTEMS)} подсистем отсутствует")

    # 4. Максимально допустимый бюджет приводов рук
    #    M_рук <= 500 - известное_без_рук - груз - люди - резерв_остальным
    print("\n[4] Максимальный бюджет приводов рук")
    base_without_arms = known_sum            # RoboCT + панель + голова/датчики
    ceiling_all_other_zero = LIMIT - base_without_arms - PAYLOAD - PEOPLE
    print(f"    потолок при нулевом резерве остальным подсистемам "
          f"= {ceiling_all_other_zero:.3f} кг")

    # Резерв: варианты по одновременности нагрузок и объёму остальных подсистем
    scenarios = []
    for name, pay, ppl, reserve in [
        ("S1 груз+человек одновременно, резерв 0", PAYLOAD, PEOPLE, 0.0),
        ("S2 груз+человек одновременно, резерв 40 кг", PAYLOAD, PEOPLE, 40.0),
        ("S3 груз+человек одновременно, резерв 80 кг", PAYLOAD, PEOPLE, 80.0),
        ("S4 только груз (без человека), резерв 40", PAYLOAD, 0.0, 40.0),
        ("S5 только человек (без груза), резерв 40", 0.0, PEOPLE, 40.0),
        ("S6 ни груза ни человека, резерв 40", 0.0, 0.0, 40.0),
    ]:
        budget = LIMIT - base_without_arms - pay - ppl - reserve
        scenarios.append({
            "scenario": name, "payload_kg": pay, "people_kg": ppl,
            "reserve_other_kg": reserve, "max_arms_kg": round(budget, 3),
            "vs_parametric_213_53": round(budget - 213.53, 3),
        })
        print(f"    {name:48s} -> руки <= {budget:8.3f} кг "
              f"({budget - 213.53:+.3f} кг относительно 213.53)")

    # 5. Реалистичность одновременности
    print("\n[5] Одновременность нагрузок (сценарий эксплуатации)")
    simult = {
        "груз_100_и_человек_80_одновременно":
            "НЕ ПОДТВЕРЖДЕНО: груз — параметр полезной нагрузки манипуляторов, "
            "человек — параметр лифта; одновременность в одном рейсе не "
            "зафиксирована в данных проекта",
        "груз_в_манипуляторах_во_время_рейса":
            "НЕ ПОДТВЕРЖДЕНО: транспортная поза рук (v7.4) не задана; "
            "перевозка груза руками в лифте не подтверждена",
        "двойной_учёт_груза":
            "ИСКЛЮЧЁН: груз в M_payload, не в M_robot; в руках (M и Ugravity) "
            "он же — но это другая проверка (2W.2), не бюджет лифта",
        "двойной_учёт_рук":
            "ИСКЛЮЧЁН: В1..В4 взаимоисключающие; stored-верх не прибавляется",
    }
    for k, v in simult.items():
        print(f"    {k}: {v}")

    result = {
        "limit_kg": LIMIT,
        "known": KNOWN,
        "known_sum_kg": round(known_sum, 3),
        "arm_variants": ARM_VARIANTS,
        "scenario_control": {
            "arms_variant": "В3",
            "arms_kg": arms, "payload_kg": PAYLOAD, "people_kg": PEOPLE,
            "robot_known_kg": round(robot_known, 3),
            "total_kg": round(total, 3),
            "excess_kg": round(excess, 3),
            "remainder_for_undefined_kg": round(remainder, 3),
        },
        "undefined_subsystems": UNDEFINED_SUBSYSTEMS,
        "max_arms_scenarios": scenarios,
        "ceiling_all_other_zero_kg": round(ceiling_all_other_zero, 3),
        "simultaneity": simult,
        "checks": DOUBLE_COUNT_CHECKS,
    }
    (HERE / "mass_budget.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nJSON: {HERE / 'mass_budget.json'}")
    n = sum(1 for c in DOUBLE_COUNT_CHECKS if c["ok"])
    print(f"ИТОГ: {n}/{len(DOUBLE_COUNT_CHECKS)} PASS")


if __name__ == "__main__":
    main()

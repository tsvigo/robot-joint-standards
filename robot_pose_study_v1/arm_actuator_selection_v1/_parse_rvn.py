"""Парсит таблицу RV-N по классам <td> из sources/rvn_page.html.

Источник: https://precision.nabtesco.com/en/products/detail/RV-N
Дата фиксации: см. actuator_source_register.md
"""
from __future__ import annotations

import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "sources" / "rvn_page.html"

# класс ячейки -> каноническое имя поля
FIELD = {
    "kw_number": "motor_rated_capacity_kw",
    "ratio": "ratio",
    "actual_ratio": "actual_ratio",
    "rated_torque": "rated_torque_Nm",
    "Maximum_torque": "maximum_torque_Nm",
    "allowable_acceleration_deceleration_torque":
        "allowable_accel_decel_torque_Nm",
    "momentary_maximum_allowable_torque": "momentary_max_torque_Nm",
    "rated_output_speed": "rated_output_speed_rpm",
    "rated_life": "rated_life_h",
    "backlash": "backlash_arcmin",
    "lost_motion": "lost_motion_arcmin",
    "torsional_regidity": "torsional_rigidity_Nm_per_arcmin",
    "moment_rejidity": "moment_rigidity_Nm_per_arcmin",
    "start-up_effciency": "startup_efficiency_pct",
    "moment_of_inertia": "moment_of_inertia_kgm2",
    "mass": "mass_kg",
    "allowable_moment_1": "allowable_moment_1_Nm",
    "allowance_Thrust_1": "allowance_thrust_1_N",
    "allowable_moment_2": "allowable_moment_2_Nm",
    "allowance_Thrust_2": "allowance_thrust_2_N",
    "brake_holding_torque": "brake_holding_torque_Nm",
    "Motor_rated_capacity": "motor_rated_capacity_kw_alt",
    "Motor_brake_rated_voltage": "motor_brake_voltage_V",
    "Actuator_brake_rated_voltage": "actuator_brake_voltage_V",
}


def main() -> None:
    html = SRC.read_text(encoding="utf-8", errors="replace")
    models = re.findall(r"<th class='model greasespec_Model_Code'>"
                        r"([^<]+)</th>", html)
    tbodies = [m.start() for m in re.finditer(r"<tbody>", html)]
    body = html[tbodies[1]:] if len(tbodies) > 1 else html
    rows_html = re.findall(r"<tr>(.*?)</tr>", body, re.S)
    print(f"моделей: {len(models)}, строк данных: {len(rows_html)}")

    recs = []
    for mi, row in enumerate(rows_html):
        cells = re.findall(r"<(?:td|th) class='([^']*)'>(.*?)</(?:td|th)>",
                           row, re.S)
        rec = {"model": models[mi] if mi < len(models) else ""}
        for cls, val in cells:
            if cls in FIELD:
                txt = re.sub(r"<[^>]+>", "", val).strip()
                rec[FIELD[cls]] = txt
        recs.append(rec)

    # сводка по заполненности
    keys = sorted({k for r in recs for k in r})
    print("\nзаполненность полей:")
    for k in keys:
        n = sum(1 for r in recs if r.get(k))
        print(f"  {k:38s} {n:3d}/{len(recs)}")

    # интересующие модели
    print("\n=== RV-N: строки для интересующих размеров ===")
    for r in recs:
        if r["model"] in ("RV-25N", "RV-42N", "RV-60N", "RV-80N",
                          "RV-100N", "RV-125N", "RV-160N"):
            print(" ", json.dumps(r, ensure_ascii=False))

    out = {"source_url": "https://precision.nabtesco.com/en/products/detail/RV-N",
           "rows": recs}
    (HERE / "sources" / "rvn_specs.json").write_text(
        json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n-> {HERE / 'sources' / 'rvn_specs.json'}")


if __name__ == "__main__":
    main()

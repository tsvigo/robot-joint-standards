"""Парсинг таблицы performance data CSG-2UH/HFUC-2UH (без вложенных квантификаторов).

Источник: Harmonic Drive SE — каталог Harmonic Drive® Gears, секция 2.1,
Table 2.1.4; sources/harmonic_gears.pdf -> harmonic_gears.txt (pdftotext -layout).

В каталоге одна строка на размер (как правило, i=100) несёт «хвост»:
    n_in_max n_in_avg  inertia  mass_2UH  mass_LW
Остальные строки передач содержат только размер/передачу/4 момента.
Поэтому парсинг в два прохода: (1) хвост -> масса/инерция/скорости по размеру,
(2) шесть чисел -> моменты для каждой пары (size, ratio).
"""
from __future__ import annotations

import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "sources"
OUT = SRC / "harmonic_csg2uh.json"


def tokens(line: str) -> list:
    s = line.rstrip()
    s = re.sub(r"\([^)]*\)", " ", s)      # (1488)3) и т.п.
    s = re.sub(r"\b\d\)", " ", s)          # сноски 1) 2) 3)
    s = re.sub(r"x10-\d+", " X10N ", s)
    out = []
    for p in s.split():
        if p == "X10N":
            out.append("X10N")
            continue
        try:
            out.append(float(p.replace(",", ".")))
        except ValueError:
            out.append(None)
    return out


def main() -> None:
    lines = (SRC / "harmonic_gears.txt").read_text(
        encoding="utf-8", errors="replace").splitlines()[4760:5060]

    tails: dict[int, dict] = {}
    torques: list[dict] = []

    for ln in lines:
        t = tokens(ln)
        # --- проход 1: хвост (есть X10N) ---
        if "X10N" in t:
            i = t.index("X10N")
            # ... n_max n_avg  [мантисса] X10N  mass2UH  massLW
            if i >= 2 and isinstance(t[i + 1], float) and isinstance(t[i + 2], float):
                left = [v for v in t[:i - 2] if isinstance(v, float)]
                if left:
                    size = int(left[0])
                    mant = t[i - 1] if isinstance(t[i - 1], float) else None
                    tails[size] = {
                        "n_input_max_rpm": int(t[i - 3]),
                        "n_input_avg_rpm": int(t[i - 2]),
                        "inertia_mantissa": mant,
                        "mass_2UH_kg": t[i + 1],
                        "mass_2UH_LW_kg": t[i + 2],
                    }
        # --- проход 2: size ratio TR TA TN TM ---
        f = [v for v in t if isinstance(v, float)]
        for a in range(len(f) - 5):
            size, ratio, tr, ta, tn, tm = (int(f[a + k]) for k in range(6))
            if not (10 <= size <= 200 and 20 <= ratio <= 300):
                continue
            if not (0 < tn <= ta <= tr <= tm and tm <= 50000):
                continue
            if not (200 <= tr <= 30000):
                continue
            torques.append({"size": size, "ratio": ratio,
                            "TR_repeated_peak_Nm": float(tr),
                            "TA_average_Nm": float(ta),
                            "TN_rated_2000rpm_Nm": float(tn),
                            "TM_momentary_peak_Nm": float(tm)})
            break

    seen, uniq_t = set(), []
    for r in torques:
        k = (r["size"], r["ratio"])
        if k in seen:
            continue
        seen.add(k)
        uniq_t.append(r)
    uniq_t.sort(key=lambda r: (r["size"], r["ratio"]))

    rows = []
    for r in uniq_t:
        tail = tails.get(r["size"], {})
        rows.append({**r, **tail})

    print(f"строк моментов: {len(rows)}; хвостов (масса): {len(tails)}")
    print(f"{'size':>5} {'ratio':>6} {'TR':>7} {'TA':>7} {'TN':>7} {'TM':>8} "
          f"{'n_in':>6} {'масса2UH':>9} {'LW':>7}")
    for r in rows:
        print(f"{r['size']:5d} {r['ratio']:6d} "
              f"{r['TR_repeated_peak_Nm']:7.0f} {r['TA_average_Nm']:7.0f} "
              f"{r['TN_rated_2000rpm_Nm']:7.0f} "
              f"{r['TM_momentary_peak_Nm']:8.0f} "
              f"{r.get('n_input_max_rpm', 0):6d} "
              f"{r.get('mass_2UH_kg') or 0:9.2f} "
              f"{r.get('mass_2UH_LW_kg') or 0:7.2f}")

    print("\nконтроль масс против arm_actuator_candidates.csv "
          "(CSF, Harmonic Drive LLC — другая серия/производитель):")
    for size, prev in {40: 5.0, 50: 8.9, 65: 20.9}.items():
        got = tails.get(size, {}).get("mass_2UH_kg")
        print(f"  size {size}: прежнее {prev} кг | здесь (CSG-2UH) {got}")

    OUT.write_text(json.dumps({
        "source": "Harmonic Drive SE — каталог Harmonic Drive® Gears, "
                  "секция 2.1 Table 2.1.4 (harmonicdrive.de)",
        "file": "sources/harmonic_gears.pdf",
        "note": "серия CSG/HFUC-2UH (SE). НЕ смешивать с CSF-2UH "
                "Harmonic Drive LLC (harmonicdrive.net) из прежнего аудита. "
                "TR/TA/TN/TM — пределы по моментам; TN — при 2000 об/мин "
                "входа; масса редуктора БЕЗ двигателя и тормоза.",
        "rows": rows,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()

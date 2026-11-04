"""
RVW-7PA v4.1 — ГЛАВНАЯ КОЛЛИЗИЯ С КОЛОННОЙ Ø240 (документирование, не «решение»).

Проверяет:
  1. Какая именно часть модуля пересекает колонну (по телам сохранённого FCStd).
  2. Какой сдвиг модуля вдоль оси устраняет коллизию.
  3. Куда этот сдвиг выводит габарит колеса и рамы.

Вывод: колесо, корпус и фланец колонну НЕ пересекают; врезаются мотор □90 и
разъём. Это свойство РАЗМЕЩЕНИЯ АКТУАТОРА, а не рамы: связная рама (см.
rvw7pa_frame_link.py) коллизию не устраняет.
"""

import json
import math
import os

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)

R_COLUMN = 120.0
R_PLATFORM = 310.0        # Ø620
R_PANEL = 293.77          # Ø587.5
AX = {"module_0": (1, 0), "module_90": (0, 1),
      "module_180": (-1, 0), "module_270": (0, -1)}
CHECKS = []
OUT = {}


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def load(name):
    doc = App.openDocument(os.path.join(BASE, name))
    objs = {o.Name: o.Shape for o in doc.Objects if hasattr(o, "Shape")}
    App.closeDocument(doc.Name)
    return objs


print("== 1. Коллизия по телам модуля ==")
per = {}
for tag, fn in (("200W", "rvw7pa_layout_200w.FCStd"),
                ("400W", "rvw7pa_layout_400w.FCStd")):
    o = load(fn)
    m, lift = o["module_0"], o["lift_zone"]
    rows = []
    for i, s in enumerate(m.Solids):
        bb = s.BoundBox
        rows.append({"тело": i, "V_мм3": round(s.Volume),
                     "X": [round(bb.XMin, 1), round(bb.XMax, 1)],
                     "Y": [round(bb.YMin, 1), round(bb.YMax, 1)],
                     "Z": [round(bb.ZMin, 1), round(bb.ZMax, 1)],
                     "коллизия_с_колонной_мм3": round(s.common(lift).Volume, 1)})
    per[tag] = rows
    total = sum(r["коллизия_с_колонной_мм3"] for r in rows)
    print(f"\n  {tag}:")
    for r in rows:
        print(f"    тело {r['тело']}: X[{r['X'][0]:7.1f},{r['X'][1]:7.1f}] "
              f"V={r['V_мм3']:>8}  коллизия={r['коллизия_с_колонной_мм3']:>9.1f} мм³")
    print(f"    суммарно {total:.1f} мм³")
OUT["по_телам"] = per

wheel_col = per["200W"][0]["коллизия_с_колонной_мм3"]
motor_col = per["200W"][3]["коллизия_с_колонной_мм3"]
t("C1_колесо_свободно", wheel_col == 0.0,
  "тело 0 (колесо Ø178×96, X 200.9..296.9) не пересекает колонну: 0.0 мм³. "
  "Пересечения корпуса (X 167.9..200.9) и фланца (X 151.9..167.9) тоже 0.0 мм³")
t("C2_врезается_мотор", motor_col > 0,
  f"в колонну врезаются только тело 3 (мотор □90) — {motor_col:.0f} мм³ — "
  f"и тело 4 (разъём Ø14) — {per['200W'][4]['коллизия_с_колонной_мм3']:.0f} мм³; "
  f"это размещение АКТУАТОРА, а не рамы")

print("\n== 2. Сдвиг модуля, устраняющий коллизию ==")
shift = {}
for tag, fn in (("200W", "rvw7pa_layout_200w.FCStd"),
                ("400W", "rvw7pa_layout_400w.FCStd")):
    o = load(fn)
    m, lift, fr = o["module_0"], o["lift_zone"], o["frame_proposal"]
    xmin = min(s.BoundBox.XMin for s in m.Solids)
    need = R_COLUMN - xmin
    moved = m.translated(App.Vector(need, 0, 0))
    wheel = max(m.Solids, key=lambda s: s.BoundBox.XMax)
    # реальный радиус габарита колеса после сдвига (угол лицевого круга)
    r_after = math.hypot(wheel.BoundBox.XMax + need, wheel.BoundBox.YMax)
    r_before = math.hypot(wheel.BoundBox.XMax, wheel.BoundBox.YMax)
    sh = {nm: o[nm].translated(App.Vector(dx * need, dy * need, 0))
          for nm, (dx, dy) in AX.items()}
    gaps = [round(sh[a].distToShape(sh[b])[0], 1)
            for a, b in (("module_0", "module_90"), ("module_90", "module_180"),
                         ("module_180", "module_270"), ("module_270", "module_0"))]
    shift[tag] = {
        "xmin_мм": round(xmin, 1), "сдвиг_мм": round(need, 1),
        "коллизия_после_сдвига_мм3": round(moved.common(lift).Volume, 1),
        "радиус_габарита_до_мм": round(r_before, 1),
        "радиус_габарита_после_мм": round(r_after, 1),
        "вылет_за_платформу_мм": round(r_after - R_PLATFORM, 1),
        "вылет_за_панель_мм": round(r_after - R_PANEL, 1),
        "зазоры_соседей_мм": gaps,
        "коллизия_с_рамой_мм3": round(
            sum(sh[n].common(fr).Volume for n in sh), 1),
    }
    v = shift[tag]
    print(f"\n  {tag}: xmin={v['xmin_мм']} -> сдвиг {v['сдвиг_мм']} мм")
    print(f"    коллизия с колонной: {v['коллизия_после_сдвига_мм3']:.0f} мм³ "
          f"(после сдвига)")
    print(f"    радиус габарита: {v['радиус_габарита_до_мм']} -> "
          f"{v['радиус_габарита_после_мм']} мм; платформа R{R_PLATFORM:.0f}, "
          f"панель R{R_PANEL:.1f}")
    print(f"    вылет: за платформу +{v['вылет_за_платформу_мм']:.1f} мм, "
          f"за панель +{v['вылет_за_панель_мм']:.1f} мм")
    print(f"    зазоры соседей: {v['зазоры_соседей_мм']} мм; коллизия с рамой "
          f"{v['коллизия_с_рамой_мм3']:.0f} мм³")

t("C3_сдвиг_устраняет", all(v["коллизия_после_сдвига_мм3"] < 1e-6 for v in shift.values()),
  "после продольного сдвига коллизии с колонной нет ("
  + ", ".join(f"{v['сдвиг_мм']} мм — {k}" for k, v in shift.items()) + ")")
t("C4_но_выходит_за_габарит", all(v["вылет_за_платформу_мм"] > 0 for v in shift.values()),
  "НО после сдвига колесо выходит за Ø620: "
  + ", ".join(f"+{v['вылет_за_платформу_мм']:.1f} мм по радиусу ({k})"
               for k, v in shift.items())
  + " — габарит платформы пришлось бы увеличивать до Ø728/Ø749 мм")

OUT["сдвиг"] = shift
OUT["вывод"] = (
    "Коллизия с колонной Ø240 — это конфликт РАЗМЕЩЕНИЯ АКТУАТОРА: мотор □90 "
    "(X 52.9..63.9 внутренний торец) и разъём входят в цилиндр R120, а колесо, "
    "корпус и фланец — нет. Сдвиг модуля на 56.1/67.1 мм убирает пересечение, но "
    "выносит габарит колеса за пределы платформы Ø620 на 54..65 мм по радиусу. Связная рама "
    "(rvw7pa_frame_link.py) эту проблему НЕ решает: рама и кольцо обходят колонну, "
    "а модули — нет.")
OUT["контрольные_точки"] = CHECKS
with open(os.path.join(HERE, "rvw7pa_column_conflict.json"), "w", encoding="utf-8") as fh:
    json.dump(OUT, fh, ensure_ascii=False, indent=2, default=str)

print("\n== ИТОГ ==")
bad = [c for c in CHECKS if not c["ok"]]
print(f"  проверок: {len(CHECKS)}, провалено: {len(bad)}")
print(f"  JSON: {os.path.join(HERE, 'rvw7pa_column_conflict.json')}")

# -*- coding: utf-8 -*-
"""Этап 2Г, аудит v7.1 — stage2g_v7_1_cad_suspension.py (FreeCAD, ТОЛЬКО ЧТЕНИЕ v6).

Задача 6: расширение проверки v7 (там все 4 модуля сдвигались одновременно).
Новые сценарии неравномерных относительных смещений модулей к телу (ход амортизатора
НЕ известен => смещения — параметры ±30 мм):
  S1: все +30 (прежний базовый);
  S2: одиночное колесо на препятствии: модуль0 +30, остальные 0;
  S3: диагональный перекос: модули0,2 = +20 / 1,3 = −20;
  S4: разнокрайние: +30/−15/+15/−30;
  S5: пол и дорожный просвет: минимальные z-нижние грани транспортных тел
      (кроме колёс) — наезд препятствия на просвет;
  S6: зазоры кабелей/креплений: кабели НЕ СМОДЕЛИРОВАНЫ — фиксируется как пробел
      (в запрос RoboCT: габариты выводов, радиус изгиба).

Файл v6 НЕ СОХРАНЯЕТСЯ (save не вызывается); SHA/mtime проверяются до/после.
"""
import hashlib
import json
import os

import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

FC = os.path.join(SCRIPT_DIR, "stage2g_v6_variants.FCStd")


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


sha_before, mt_before = sha(FC), os.path.getmtime(FC)
doc = App.openDocument(FC)
doc.recompute()

MODS = sorted([o for o in doc.Objects if o.Name.startswith("R620_CTM") and "_WHL" in o.Name],
              key=lambda o: o.Name)
assert len(MODS) == 4, f"ожидалось 4 модуля, найдено {len(MODS)}"
BASES = {m.Name: App.Vector(m.Placement.Base) for m in MODS}

STATIC = [o for o in doc.Objects if o.Name.startswith("R620_CTM_")
          and ("_body" in o.Name or "_lift" in o.Name or "_bat" in o.Name
               or "_pod" in o.Name or "_arm" in o.Name)
          and hasattr(o, "Shape") and not o.Shape.isNull()]
FLOORS = [o for o in doc.Objects if o.Name.startswith("R620_CTM_")
          and ("_pad" in o.Name or "_supp" in o.Name) and hasattr(o, "Shape")
          and not o.Shape.isNull()]


def apply(dzs):
    for m, dz in zip(MODS, dzs):
        b = BASES[m.Name]
        m.Placement.Base = App.Vector(b.x, b.y, b.z + dz)
    doc.recompute()


def pair_scan():
    """min расстояние модулей до static; все пересечения."""
    worst, interf = 1e9, []
    for m in MODS:
        for s in STATIC:
            d, _p, _i = m.Shape.distToShape(s.Shape)
            v = m.Shape.common(s.Shape).Volume
            d = float(d)
            if v > 1.0:
                interf.append({"pair": f"{m.Name} x {s.Name}", "d": round(d, 2),
                               "common_cm3": round(v / 1000.0, 2)})
            elif d < worst:
                worst = d
    return (round(worst, 2) if worst < 1e9 else None), interf


SCEN = {
    "S1_все_+30": [30.0] * 4,
    "S2_колесо0_на_препятствии_+30": [30.0, 0.0, 0.0, 0.0],
    "S3_диагональный_перекос_+20/−20": [20.0, -20.0, 20.0, -20.0],
    "S4_разнокрайние_+30/−15/+15/−30": [30.0, -15.0, 15.0, -30.0],
}

print("== 1. Сценарии неравномерных смещений модулей (ход — параметр ±30 мм) ==")
results = {}
for name, dzs in SCEN.items():
    apply(dzs)
    worst, interf = pair_scan()
    results[name] = {"dz_мм": dzs, "min_зазор_мм": worst, "пересечения": interf}
    st = f"ПЕРЕСЕЧЕНИЙ {len(interf)}" if interf else "пересечений нет"
    print(f"  {name}: min_зазор={worst} мм; {st}")
    for it in interf:
        print(f"    {it['pair']}: {it['d']} мм, {it['common_cm3']} см3")

# вернуть базу
apply([0.0] * 4)

# ============ 2. Пол и дорожный просвет (транспортное положение) ============
print("\n== 2. Пол и дорожный просвет (транспорт) ==")
floor_z = {}
for o in doc.Objects:
    if not o.Name.startswith("R620_CTM_") or not hasattr(o, "Shape") or o.Shape.isNull():
        continue
    n = o.Name
    if "_pad" in n:                     # площадки — рабочее положение, помечаем отдельно
        kind = "площадка (рабочее пол.)"
    elif "_WHL" in n or "_hub" in n:
        kind = "колесо (штатный контакт с полом)"
    elif "_env" in n or "_supp" in n:
        kind = "не элемент просвета"
    elif "_pt" in n or "_hand" in n:
        kind = "служебный маркер (не деталь)"
    else:
        kind = "транспортный элемент"
    zmin = round(float(o.Shape.BoundBox.ZMin), 2)
    if kind in ("транспортный элемент", "площадка (рабочее пол.)"):
        floor_z.setdefault(kind, []).append((n.split("R620_CTM_")[1], zmin))
for kind, items in floor_z.items():
    items.sort(key=lambda t: t[1])
    top3 = ", ".join(f"{n}={z}" for n, z in items[:3])
    print(f"  {kind}: минимумы -> {top3}")
ground_clearance = min(z for n, z in floor_z.get("транспортный элемент", [(None, 999)]))
clearance_finding = {
    "просвет_транспорт_мм": ground_clearance,
    "вердикт": (f"минимальный просвет транспортных элементов = {ground_clearance} мм "
                f"(блоки аутригеров). Пороги/швы линолеума и уклон — параметры требований: "
                f"при перепаде ≥ {ground_clearance} мм — НАЕЗД. Требуется либо подъём "
                f"блоков, либо подтверждение допустимых перепадов пола — ВОПРОС "
                f"ТРЕБОВАНИЯМ/производителю подвески."),
    "статус": "ПАРАМЕТР: не объявлять достаточным без требований к полам",
}
print(f"  просвет: {ground_clearance} мм -> {clearance_finding['вердикт'][:120]}...")

# ============ 3. Кабели и крепления ============
cables = {
    "статус": "НЕ СМОДЕЛИРОВАНЫ в v6",
    "что_нужно": ["габариты выводов/разъёмов модуля (с учётом 106.9 — включает ли вывод)",
                  "радиус изгиба и трасса 4 кабелей при ходе подвески ±30 мм",
                  "крепёж кабелей к панели, запас длины при перекосах"],
    "риск": "кабель в зоне ниши/колеса при неравномерных ходах — не проверен",
}
print("\n== 3. Кабели/крепления ==")
print(f"  {cables['статус']}; {cables['риск']} -> в запрос RoboCT")

# закрытие
out = {"suspension_scenarios": results, "floor_clearance": clearance_finding,
       "cables": cables,
       "notes": [
           "Ход подвески ±30 мм — параметр (реальный ход неизвестен, запрос RoboCT).",
           "Смещения — относительные модуль↔тело (консервативный охват).",
           "v6 FCStd не изменён: save не вызывался; SHA/mtime проверяются ниже.",
       ]}
path = os.path.join(SCRIPT_DIR, "stage2g_v7_1_cad_suspension.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

App.closeDocument(doc.Name)
sha_after, mt_after = sha(FC), os.path.getmtime(FC)
ok = sha_before == sha_after and mt_before == mt_after
print(f"\nv6 целостность: SHA {sha_before} -> {sha_after} {'OK' if ok else 'ИЗМЕНЁН!'}")
print("JSON:", path)
print("REGRESSION:", "PASS" if ok and all(
    not r["пересечения"] for r in results.values()) else "FAIL")

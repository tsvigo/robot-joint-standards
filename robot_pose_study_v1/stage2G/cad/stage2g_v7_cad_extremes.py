# -*- coding: utf-8 -*-
"""Этап 2Г, задача 5 — stage2g_v7_cad_extremes.py (FreeCAD, ТОЛЬКО ЧТЕНИЕ v6).

1. Крайние положения подвески: сдвиг всех модулей по Z на Δz ∈ {+30,+15,0,−15,−30} мм
   (ход амортизатора НЕ подтверждён — параметр) + пересчёты 3D-расстояний.
2. Чувствительность общей ширины модуля W (с амортизатором/креплением/кабелем)
   до зоны лифта и контура — замкнутая формула.
3. Силовой путь «лифт → панель → узлы колёс»: порядковая оценка изгиба панели
   (не считать 40 мм автоматически достаточными — даётся оценка с допущениями).

Файл v6 НЕ СОХРАНЯЕТСЯ: save не вызывается; mtime/SHA проверяются до/после.
"""
import hashlib
import json
import math
import os
import time

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


def mtime(path):
    return os.path.getmtime(path)


sha_before, mt_before = sha(FC), mtime(FC)

doc = App.openDocument(FC)
doc.recompute()

MODS = [o for o in doc.Objects if "_WHL" in o.Name and o.Name.startswith("R620_CTM")]
assert len(MODS) == 4, f"ожидалось 4 модуля, найдено {len(MODS)}"

STATIC = [o for o in doc.Objects if o.Name.startswith("R620_CTM_")
          and ("_body" in o.Name or "_lift" in o.Name or "_bat" in o.Name
               or "_pod" in o.Name or "_arm" in o.Name)
          and hasattr(o, "Shape") and not o.Shape.isNull()]


def pair_dist(a, b):
    d, _p, _i = a.Shape.distToShape(b.Shape)
    v = a.Shape.common(b.Shape).Volume
    return round(float(d), 2), round(v / 1000.0, 3)


# ============ 1. Крайние положения подвески ============
print("== 1. Крайние положения подвески (Δz по модулям, ход параметр) ==")
extremes = {}
for dz in (0.0, 15.0, 30.0, -15.0, -30.0):
    for m in MODS:
        m.Placement.Base = App.Vector(m.Placement.Base.x, m.Placement.Base.y,
                                      m.Placement.Base.z)  # база хранится ниже
    # чистый сдвиг: запоминаем исходные базы один раз
    if dz == 0.0:
        bases = {m.Name: App.Vector(m.Placement.Base) for m in MODS}
    for m in MODS:
        b = bases[m.Name]
        m.Placement.Base = App.Vector(b.x, b.y, b.z + dz)
    doc.recompute()
    rows = []
    worst = 1e9
    for m in MODS:
        for s in STATIC:
            if "_arm" in s.Name or "_pod" in s.Name:
                pass  # проверяем все
            d, v = pair_dist(m, s)
            if v > 1.0:
                rows.append({"pair": f"{m.Name} x {s.Name}", "d": d, "common_cm3": v,
                             "verdict": "ПЕРЕСЕЧЕНИЕ"})
            elif d < worst:
                worst = d
    interf = [r for r in rows if r["verdict"] == "ПЕРЕСЕЧЕНИЕ"]
    extremes[str(dz)] = {"min_dist_мм": (round(worst, 2) if worst < 1e9 else None),
                         "intersections": interf}
    status = f"ПЕРЕСЕЧЕНИЙ {len(interf)}" if interf else "пересечений нет"
    print(f"  Δz={dz:+.0f} мм: мин.расстояние модуля до static = {worst:.2f} мм; {status}")
    for r in interf:
        print(f"    {r['pair']}: {r['d']} мм, {r['common_cm3']} см3")

# вернуть базы
for m in MODS:
    b = bases[m.Name]
    m.Placement.Base = b
doc.recompute()

# ============ 2. Чувствительность ширины W модуля ============
print("\n== 2. Общая ширина модуля W (с амортизатором/креплением/кабелем) ==")
# r_c = sqrt((R_env−10)² − 120²) − W/2; внутр. кромка = sqrt(...) − W
# зазор до лифта (R=120 + проектные 10): sqrt((R−10)²−120²) − W − 130 ≥ 0
# контур: кромка по D/2 (радиальная) не зависит от W; кромка по ширине: растёт W/2 внутрь
w_sens = {}
for Dp in (620.0, 640.0):
    Rl = Dp / 2.0 - 10.0
    base = math.sqrt(Rl ** 2 - 120.0 ** 2)      # = r_c + W/2 (внешняя осевая кромка)
    # лимит по лифту: base − W − 130 ≥ 0
    w_lim_lift = base - 130.0
    # лимит по контуру: модуль не выходит за R_env (осевая кромка уже в р_c) —
    # внутренняя кромка base − W ≥ 10 (до оси... нет, до лифта выше); проверка угловой:
    w_sens[f"Dp{Dp:.0f}"] = {
        "r_c_при_W106.9": round(base - 106.9 / 2, 1),
        "W_max_до_нарушения_зазора_лифт_10мм": round(w_lim_lift, 1),
        "зазор_лифт_при_W106.9": round(base - 106.9 - 120.0 - 10.0, 1),
        "зазор_лифт_при_W130": round(base - 130.0 - 120.0 - 10.0, 1),
    }
    print(f"  Ø{Dp:.0f}: r_c(W=106.9)={w_sens[f'Dp{Dp:.0f}']['r_c_при_W106.9']} мм; "
          f"W_макс до зазора 10 мм к лифту = {w_lim_lift:.1f} мм; "
          f"зазор при W=106.9: {w_sens[f'Dp{Dp:.0f}']['зазор_лифт_при_W106.9']} мм; "
          f"при W=130: {w_sens[f'Dp{Dp:.0f}']['зазор_лифт_при_W130']} мм")

# ============ 3. Силовой путь: порядковая оценка панели ============
print("\n== 3. Силовой путь «лифт → панель → узлы колёс»: оценка изгиба панели ==")
F_LIFT = 500.0 * 9.81            # бюджет лифта 500 кг (из v3, не подтверждён) Н
L_SPAN = 0.2215                  # м: центр → узел колеса (r_c)
W_STRIP = 0.15                   # м: ширина «полки» распределения (допущение)
T_PANEL = 0.040                  # м
for factor, tag in ((1.0, "статика 1×"), (3.0, "с запасом/динамика 3×")):
    F_spoke = F_LIFT / 4.0 * factor
    M = F_spoke * L_SPAN / 4.0           # балка на двух опорах, нагрузка посередине
    Wz = W_STRIP * T_PANEL ** 2 / 6.0
    sigma = M / Wz / 1e6                 # МПа
    print(f"  {tag}: M≈{M:.0f} Н·м, W_z={Wz*1e6:.0f} см³, σ_изг≈{sigma:.1f} МПа "
          f"(сталь 235 / алюм 276 МПа)")
panel_est = {
    "sigma_static_MPa": round(F_LIFT / 4 * L_SPAN / 4 / (W_STRIP * T_PANEL ** 2 / 6) / 1e6, 1),
    "sigma_x3_MPa": round(3 * F_LIFT / 4 * L_SPAN / 4 / (W_STRIP * T_PANEL ** 2 / 6) / 1e6, 1),
    "verdict": ("Порядковая оценка: статический изгиб при 500 кг ПОРОГОВО мал — "
                "НО не проверены: местный разнос от фланца лифта, вырезы ниш "
                "(остаётся 168 мм полной дисковой зоны), боковой момент руки, "
                "усталость, крепёж модуля (схема крепления неизвестна), материал. "
                "СТАТУС: НЕ ПОДТВЕРЖДЕНО, требуется FEA после выбора материала."),
    "load_path": ("лифт → фланец (Ø240, z90) → диск панели (сплошная зона r<168 мм) "
                  "→ кольцо/спицы между нишами → узлы крепления модулей на торцах ниш "
                  "→ модуль → колесо. Схема крепления модуля НЕ ПОДТВЕРЖДЕНА (чертеж)."),
}
print("  вердикт: НЕ ПОДТВЕРЖДЕНО — порядково мал, но нужны материал/FEA/схема крепления")

# ============ 4. Доступ/обслуживание ============
access = {
    "модуль_с_амортизатором": ("габарит 106.9 мм указан как «outer space between two sides» — "
                               "включает ли амортизатор/крепление/кабель — НЕЯСНО (запрос)"),
    "монтаж_модуля": "радиальный ввод в нишу изнутри/снизу — при условии съёмной панели или открытого борта ниши; схема крепления неизвестна",
    "ход_подвески": "не подтверждён (±30 мм — параметр); при −30 колесо уходит ниже z=0 — тело на 30 мм выше (кинематика, не столкновение)",
    "кабели": "не моделированы — требуются габариты выводов и радиус изгиба (запрос)",
    "доступ_к_роликам": "замена роликов снаружи (заявлено производителем: «менять ролики, не колёса»)",
}
print("\n== 4. Доступ/обслуживание ==")
for k, v in access.items():
    print(f"  {k}: {v}")

# ============ Сохранение (только JSON; v6 НЕ сохраняется) ============
out = {"suspension_extremes": extremes, "width_sensitivity": w_sens,
       "panel_strength": panel_est, "access": access,
       "notes": [
           "Ход подвески ±30 мм — параметр; реальный ход/ось хода запросить у RoboCT.",
           "P_pairs — модуль↔панель/лифт/АКБ/аутригеры при сдвигах Δz.",
           "v6 FCStd не изменён: save не вызывался; SHA/mtime проверяются ниже.",
       ]}
path = os.path.join(SCRIPT_DIR, "stage2g_v7_cad_extremes.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

App.closeDocument(doc.Name)
sha_after, mt_after = sha(FC), mtime(FC)
print(f"\nv6 целостность: SHA {sha_before} -> {sha_after} "
      f"{'OK' if sha_before == sha_after else 'ИЗМЕНЁН!'}; mtime "
      f"{'OK' if mt_before == mt_after else 'ИЗМЕНЁН!'}")
print("JSON:", path)

# -*- coding: utf-8 -*-
"""Этап 2Г v7.4, задача 2 — stage2g_v7_4_transport.py (только Python, read-only).

Транспортная поза робота R2: кандидаты и перечень исходных данных (поза НЕ УТВЕРЖДЕНА).
Проверка полного транспортного габарита СБОРКИ (не только базы):
 дверь 700 мм; высота 2000 мм; опасные выступы; сложенные аутригеры; влияние на ЦМ.

Источники: stage2v_blender_mass_dump.json (182 объекта, origin/mass),
model_key_facts.json (visual_bbox, высоты, ход лифта, опорный контур),
stage2A/control_pose_gt.json (определения рабочих поз, включая folded/lowered_sym),
stage2A/control_pose_table.md, геометрия R2/v6 (аутригеры, контур).
Blender не изменяется.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DUMP_P = os.path.join(ROOT, "stage2V", "stage2v_blender_mass_dump.json")
KF_P = os.path.join(ROOT, "model_key_facts.json")
GT_P = os.path.join(ROOT, "stage2A", "control_pose_gt.json")

with open(DUMP_P, encoding="utf-8") as f:
    D = json.load(f)
with open(KF_P, encoding="utf-8") as f:
    KF = json.load(f)
with open(GT_P, encoding="utf-8") as f:
    GT = json.load(f)

OBJS = D["objects"]
FLOOR = -5.6086
OX, OY = 3.608398, -1.86476
r2 = lambda p: (p[0] - OX, p[1] - OY, p[2] - FLOOR)

# ============ 1. Габарит исходной (P0) сборки из Blender ============
vb = KF["geometry"]["visual_bbox"]
vb_r2 = {"x": [round(vb["min"][0] - OX, 3), round(vb["max"][0] - OX, 3)],
         "y": [round(vb["min"][1] - OY, 3), round(vb["max"][1] - OY, 3)],
         "z": [round(vb["min"][2] - FLOOR, 3), round(vb["max"][2] - FLOOR, 3)]}
print("== 1. Габарит исходной сборки Blender (поза P0, руки протянуты) ==")
print(f"  visual bbox СК R2: X {vb_r2['x']} ({(vb_r2['x'][1]-vb_r2['x'][0])*1000:.0f} мм), "
      f"Y {vb_r2['y']} ({(vb_r2['y'][1]-vb_r2['y'][0])*1000:.0f} мм), "
      f"Z {vb_r2['z']} (верх {vb_r2['z'][1]:.3f} м)")

# суставы и origin рук
def p(name):
    v = OBJS.get(name, {})
    return r2(v["matrix_world_translation"]) if v.get("matrix_world_translation") else None

joints = {n: p(n) for n in ("left_shoulder_joint_empty_1", "right_shoulder_joint_empty_1",
                            "left_elbow_joint_empty", "right_elbow_joint_empty")}
arms = {n: p(n) for n in ("left_shoulder_link", "left_forearm_link", "left_palm_link",
                           "right_shoulder_link", "right_forearm_link", "right_palm_link")}
print("  плечевые суставы (СК R2):")
for n, v in joints.items():
    print(f"    {n}: {v}")
print("  origin звеньев рук (СК R2):")
for n, v in arms.items():
    print(f"    {n}: {v}")
sh_ys = [abs(p(n)[1]) for n in ("left_shoulder_joint_empty_1", "right_shoulder_joint_empty_1")]
el_ys = [abs(p(n)[1]) for n in ("left_elbow_joint_empty", "right_elbow_joint_empty")]
shoulder_half_width = max(sh_ys)      # ПЛЕЧИ (не локти!)
elbow_half_width = max(el_ys)         # локти в позе P0 (протянутые руки)
print(f"  полушырина по ПЛЕЧЕВЫМ суставам: {shoulder_half_width:.3f} м -> "
      f"мин. ширина по суставам {2*shoulder_half_width*1000:.0f} мм (< 700 ✓)")
print(f"  локти в позе P0: ±{elbow_half_width:.3f} м (вылет {2*elbow_half_width*1000:.0f} мм) "
      f"— поза P0 для двери неприемлема")

# ============ 2. Кандидаты транспортной позы ============
poses_def = GT.get("pose_definitions", {})
candidates = [
    {"поза": "К0: P0 как в модели (NE PAS — контрольная)",
     "определение": "исходная Blender: руки протянуты по ±Y, ладони на ±1.13–1.15",
     "габарит_по_Y": round(vb_r2["y"][1] - vb_r2["y"][0], 3),
     "дверь_700": "НЕ ПРОХОДИТ (2.7 м)",
     "статус": "текущее состояние модели, НЕ транспортная поза"},
    {"поза": "К1: folded (складная, Stage 2A)",
     "определение": "control_pose_gt: el на верхнем лимите (обе руки); описание "
                    "«Складная; столкновения не проверены»",
     "габарит_по_Y": "не вычислен (FK/габарит звеньев не посчитаны)",
     "дверь_700": "Кандидат: требует расчёта габарита и проверки столкновений",
     "статус": "КАНДИДАТ (определение суставов есть, габарит не вычислен)"},
    {"поза": "К2: lowered_sym (симметричное опускание, Stage 2A)",
     "определение": "L sh1 −21 / R sh1 +21, ладони ±X, palm_z ≈0.70",
     "габарит_по_Y": "не вычислен; плечи в протянутой pозе ±0.49 — при опускании "
                     "вбок габарит определяется траекторией плеч",
     "дверь_700": "Кандидат: требует расчёта",
     "статус": "КАНДИДАТ"},
    {"поза": "К3: руки вдоль тела/прижаты (не определена в 2A)",
     "определение": "требует IK/конфигурации суставов — в 2A отсутствует",
     "габарит_по_Y": "теоретически ≥2×(полуплечи + толщина руки)",
     "дверь_700": "Кандидат: нужен расчёт; минимальная ширина ограничена "
                  "положением плечевых суставов",
     "статус": "КАНДИДАТ (требует новых IK-решений)"},
]
print("\n== 2. Кандидаты транспортной позы (НЕ утверждены) ==")
for c in candidates:
    print(f"  {c['поза']}: {c['статус']}; дверь 700: {c['дверь_700']}")

# ============ 3. Проверки габарита ============
print("\n== 3. Проверка полного транспортного габарита ==")
checks = {}

# дверь 700
base_w = KF["geometry"]["base"]["dims"][0]            # 0.67 Blender-база
checks["дверь_700_база_Blender"] = {
    "ширина": f"{base_w*1000:.0f} мм", "зазор": f"{(0.7-base_w)*1000:.0f} мм на проём",
    "статус": "проходит, но зазор всего 30 мм (R2-база Ø620 даёт 80 мм; "
              "проектный зазор на проём НЕ ЗАДАН)"}
checks["дверь_700_руки_P0"] = {"статус": "НЕ ПРОХОДИТ (Y 2.725 м) — поза P0 непригодна"}
checks["дверь_700_суставы"] = {
    "плечи_полушырина": f"{shoulder_half_width:.3f} м",
    "мин_ширина_по_плечам": f"≈{2*shoulder_half_width*1000:.0f} мм < 700 ✓ (с запасом "
                            f"на габарит звеньев — он НЕ ИЗВЕСТЕН, origin ≠ bbox)",
    "локти_в_P0": f"±{elbow_half_width:.3f} м (вылет {2*elbow_half_width*1000:.0f} мм) "
                  "— поза P0 неприемлема; локти обязательно в транспортной позе",
    "статус": "проход 700 ВОЗМОЖЕН при позе рук, сводящей локти внутрь; "
              "подтверждение требует габаритов звеньев и FK транспортной позы",
    "статус_флаг": "Кандидат"}
checks["дверь_700_торс_и_голова"] = {"торс": "0.32×0.26 м", "голова": "0.144 м",
                                     "статус": "в пределах 700 ✓"}
# высота 2000
h_guides = KF["geometry"]["height_guides_m"]
h_head = KF["geometry"]["height_head_m"]
lift_up = KF["lift"]["travel_upper_mm"] / 1000.0
checks["высота_2000"] = {
    "верх_направляющих": round(h_guides, 3),
    "голова_верх_при_подъёме_лифта": round(h_head + lift_up, 3),
    "запас_до_2000": round(2.0 - max(h_guides, h_head + lift_up), 3),
    "статус": f"в текущих позах ОК (макс {max(h_guides, h_head+lift_up):.3f} м); "
              "руки/инвентарь при подъёме вверх — НЕ ОЦЕНЕНЫ (поза не задана)"}
print(f"  дверь 700: база Blender {base_w*1000:.0f} мм (зазор 30), R2-база 620 (зазор 80) "
      f"— проектный зазор НЕ ЗАДАН")
print(f"  суставы: плечи ±{shoulder_half_width:.3f} (ширина {2*shoulder_half_width*1000:.0f} <700 ✓), "
      f"локти в P0 ±{elbow_half_width:.3f} (вылет {2*elbow_half_width*1000:.0f} — позу складывать)")
print(f"  высота: направляющие {h_guides} м, голова+подъём {h_head+lift_up:.3f} м -> "
      f"запас до 2000 = {2.0 - max(h_guides, h_head+lift_up):.3f} м ✓ (без учёта рук)")
# опасные выступы
checks["опасные_выступы"] = {
    "по_X": f"{(vb_r2['x'][1]-vb_r2['x'][0])*1000:.0f} мм (в пределах базы)",
    "по_Y": "2725 мм — руки в P0; в транспортной позе должны быть убраны",
    "по_Z": "верх 1.737 м; направляющие лифта и кабельные цепи — нужен облёт "
            "в транспортной позе (не проверялся)",
    "статус": "кроме рук — в пределах базы; выступы рук = главный риск"}
checks["аутригеры_сложены"] = {
    "блоки": "r 0.18–0.28 м, z 0.008–0.042 м (v6) — в пределах контура Ø620",
    "площадки": "втянуты (рабочее положение показывается отдельно)",
    "статус": "по XY в габарите ✓; просвет 8 мм остаётся ОТКРЫТЫМ требованием "
              "(v7.1, риск для порогов) — безопасное движение НЕ подтверждено"}
print("  опасные выступы: главный риск — руки в P0 (Y 2.7 м); прочее в пределах базы")
print("  аутригеры сложены: в пределах Ø620 ✓; просвет 8 мм — открытое требование")

# ============ 4. Влияние транспортной позы на ЦМ ============
m_lift = 21.711                     # подвижная сборка stored (голова+руки+датчики)
print("\n== 4. Влияние транспортной позы на ЦМ (параметрически) ==")
cm_pose = {}
for tag, dz in (("P0 (протянутые, z≈1.05)", 0.0),
                ("lowered_sym (ладони 0.70, Δz≈−0.20)", -0.20),
                ("руки подняты (Δz≈+0.30 — гипотеза)", 0.30)):
    dz_total = dz * (13.7 / m_lift)   # доля рук в подвижной сборке ~13.5/21.7
    cm_pose[tag] = {"Δz_ЦМ_м_при_м_робота": {
        "176кг": round(m_lift * dz * 0.626 / 176, 4),   # руки ≈ 13.53 кг из 21.7
        "400кг": round(m_lift * dz * 0.626 / 400, 4)}}
    print(f"  {tag}: Δz_ЦМ ≈ {cm_pose[tag]['Δz_ЦМ_м_при_м_робота']['400кг']:.4f} м "
          f"при 400 кг; {cm_pose[tag]['Δz_ЦМ_м_при_м_робота']['176кг']:.4f} м при 176 кг")
cm_pose["примечание"] = ("высота ЦМ рук в позе НЕ определена (FK не считался) — "
                         "значения Δz оценочные, параметры; несимметричная поза "
                         "добавила бы и сдвиг по Y")
out = {"visual_bbox_R2": vb_r2, "joints_R2": joints, "arms_origin_R2": arms,
       "candidates": candidates, "checks": checks, "cm_pose": cm_pose,
       "notes": ["Транспортная поза НЕ УТВЕРЖДЕНА — только кандидаты.",
                 "Габариты К1–К3 требуют FK/IK по Blender-иерархии.",
                 "Blender не изменялся (только чтение)."]}
path = os.path.join(HERE, "stage2g_v7_4_transport.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)

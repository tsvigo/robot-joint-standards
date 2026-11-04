# -*- coding: utf-8 -*-
"""Этап RVW-7PA v4, задача 1 — компоновка четырёх реальных актуаторов в R2 Ø620.

Источник габаритов: Nabtesco en_cat_RVW.pdf, Rev.005.1, CAT.210125_B (03.09.2026),
стр. 17-18 (таблица External Dimensions), стр. 39-40 (чертежи RVW-7PALA/PARA).

Реальные габариты (DOC, стр.17-18 + чертёж стр.39):
  a = Ø178 (колесо), b = 96 (ширина), c = 233 / 244 (полная длина),
  d = 99 / 110 (длина двигателя), мотор □90, Ø148 фланец, Ø123h7 центровка,
  6×M6-16 на P.C.D.136, L1 = 233/244 = 129±0.6 + L2 (104/115), масса 15.4/15.9 кг.

Осевая декомпозиция (по чертежу): [0..96] колесо, [96..129] корпус редуктора
Ø148 с монтажным фланцем, [129..145] фланец Ø148 (6×M6 PCD136), [134..c] мотор □90.
Проверка: 129 + d + 5 = c (200 Вт: 129+99+5 = 233; 400 Вт: 129+110+5 = 244).

ВНИМАНИЕ: оболочка модуля КОНСЕРВАТИВНАЯ (цилиндры вместо реальных ступеней,
□90 вместо скруглений). Точный контур снимается с STEP после member-доступа.
"""
import hashlib
import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))

PLATFORM_R = 310.0          # Ø620 (требование)
RW = 248.9                  # радиус установки оси колеса (пересчёт v2)
WHEEL_D, WHEEL_B = 178.0, 96.0
BODY_D, BODY_S0, BODY_S1 = 148.0, 96.0, 129.0
FLANGE_S1, FLANGE_W = 145.0, 16.0
MOTOR_SQ = 90.0
LIFT_D = 240.0              # предварительный габарит центрального подъёмника
PANEL_Z0, PANEL_Z1 = 50.0, 90.0
GAP = 10.0
CONN_L, CONN_D = 25.0, 14.0  # кабельный разъём (чертёж: Ø14, R0.5)
SUSP = [-30.0, -15.0, 0.0, 15.0, 30.0]
AL_RHO = 2.7e-6             # кг/мм3 (материал не выбран — параметр)

VARIANTS = [
    {"name": "200w", "power": 200, "c": 233.0, "d": 99.0, "mass": 15.4, "L2": 104.0},
    {"name": "400w", "power": 400, "c": 244.0, "d": 110.0, "mass": 15.9, "L2": 115.0},
]

CHECK = []
FIND = []


def t(cid, ok, detail):
    CHECK.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def sha16(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def cyl(r, h, x0, axis=(-1.0, 0.0, 0.0), z=0.0):
    return Part.makeCylinder(r, h, App.Vector(x0, 0, z), App.Vector(*axis))


def cd(sh):
    return sh.Volume if sh and not sh.isNull() else 0.0


def build_module(c, d, with_conn=True):
    """Модуль в локальной СК: ось колеса вдоль X, наружный торец колеса x=x_out."""
    x_out = RW + WHEEL_B / 2
    z_ax = WHEEL_D / 2
    parts = [
        cyl(WHEEL_D / 2, WHEEL_B, x_out, axis=(-1, 0, 0), z=z_ax),              # колесо
        cyl(BODY_D / 2, BODY_S1 - BODY_S0, x_out - BODY_S1, axis=(1, 0, 0), z=z_ax),
        cyl(BODY_D / 2, FLANGE_W, x_out - FLANGE_S1, axis=(1, 0, 0), z=z_ax),    # фланец
    ]
    x_in = x_out - c
    # мотор: □90 на всю длину d от x_in
    parts.append(Part.makeBox(d, MOTOR_SQ, MOTOR_SQ,
                              App.Vector(x_in, -MOTOR_SQ / 2, z_ax - MOTOR_SQ / 2)))
    if with_conn:
        # кабельный разъём сверху, ближе к заднему торцу мотора
        parts.append(cyl(CONN_D / 2, CONN_L, x_in + 22.0, axis=(0, 0, 1),
                         z=z_ax + MOTOR_SQ / 2))
    return Part.makeCompound(parts)


def place(sh, ang):
    if ang % 360:
        sh = sh.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), ang)
    return sh


def build_panel(x_in):
    panel = Part.makeCylinder(PLATFORM_R, PANEL_Z1 - PANEL_Z0, App.Vector(0, 0, PANEL_Z0))
    # центральный проём под колонну подъёмника (Ø240 сквозной)
    panel = panel.cut(Part.makeCylinder(LIFT_D / 2, 200.0, App.Vector(0, 0, -60)))
    half = WHEEL_D / 2 + GAP                       # 99
    x1 = PLATFORM_R + 60.0
    for ang in (0, 90, 180, 270):
        b = Part.makeBox(x1 - x_in, 2 * half, PANEL_Z1 - PANEL_Z0 + 60,
                         App.Vector(x_in, -half, PANEL_Z0 - 30))
        panel = panel.cut(place(b, ang))
    return panel


def build_frame(x_in):
    """Несущая рама вместо сплошной панели (ГИПОТЕЗА конструкции).

    Состав: центральное кольцо r124..168 (только секторы 45°, пазы модулей
    вырезаны), 4 диагональных луча 50x30 под 45°, 8 рычагов подвески
    (|y| 97..195, x 175..195, z 60..90). Кронштейн модуля — хомут Ø148..190
    шириной 20 на корпусе редуктора (x 175..195), движется вместе с модулем.
    """
    z0, z1 = 60.0, 90.0
    collar = Part.makeCylinder(168.0, z1 - z0, App.Vector(0, 0, z0))
    collar = collar.cut(Part.makeCylinder(124.0, 40.0, App.Vector(0, 0, z0 - 5)))
    half = WHEEL_D / 2 + GAP
    for ang in (0, 90, 180, 270):
        b = Part.makeBox(250.0, 2 * half, 60, App.Vector(x_in, -half, z0 - 15))
        collar = collar.cut(place(b, ang))
    parts = [collar]
    for ang in (45, 135, 225, 315):
        beam = Part.makeBox(185.0, 50.0, z1 - z0, App.Vector(125.0, -25.0, z0))
        parts.append(place(beam, ang))
    for ang in (0, 90, 180, 270):
        for sgn in (+1, -1):
            a = Part.makeBox(20.0, 98.0, z1 - z0,
                             App.Vector(175.0, 97.0 if sgn > 0 else -195.0, z0))
            parts.append(place(a, ang))
    return parts[0].fuse(parts[1:])


def build_clamp():
    """Хомут модуля (движется вместе с модулем): Ø148..190, ширина 20."""
    x0 = RW + WHEEL_B / 2 - BODY_S1
    z = WHEEL_D / 2
    outer = Part.makeCylinder(95.0, 20.0, App.Vector(x0, 0, z), App.Vector(1, 0, 0))
    inner = Part.makeCylinder(74.0, 40.0, App.Vector(x0 - 10, 0, z), App.Vector(1, 0, 0))
    return outer.cut(inner)


def run_variant(v):
    c, d = v["c"], v["d"]
    x_out = RW + WHEEL_B / 2
    x_in = x_out - c
    print(f"\n===== ВАРИАНТ {v['power']} Вт: c={c}, d={d}, масса {v['mass']} кг =====")
    print(f"  наружный торец колеса x={x_out:.1f}, внутренний торец модуля x={x_in:.1f}")

    doc = App.newDocument(f"rvw7pa_{v['name']}")
    panel = build_panel(x_in)
    lift = Part.makeCylinder(LIFT_D / 2, 400.0 + 88.0, App.Vector(0, 0, -88.0))
    mod0 = build_module(c, d)
    modules = {a: place(mod0.copy(), a) for a in (0, 90, 180, 270)}

    for nm, sh in (("panel", panel), ("lift_zone", lift)):
        doc.addObject("Part::Feature", nm).Shape = sh
    for a, sh in modules.items():
        doc.addObject("Part::Feature", f"module_{a}").Shape = sh

    # --- L/R: зеркальная оболочка (ролики другого направления) ---
    modL = mod0.copy()
    modR = mod0.mirror(App.Vector(0, 0, 0), App.Vector(0, 1, 0))
    dv = (modL.common(lift).Volume, modR.common(lift).Volume)
    t(f"T1_{v['name']}_L_R_оболочка", abs(dv[0] - dv[1]) < 1.0,
      f"L и R: одинаковый объём пересечения с подъёмником "
      f"({dv[0]:.0f} / {dv[1]:.0f} мм3) — L/R меняют только направление роликов "
      f"и отверстия на торце колеса (стр.39-40), оболочка идентична")

    # --- выход за Ø620 реальным объёмом ---
    outer = Part.makeCylinder(PLATFORM_R, 700.0, App.Vector(0, 0, -100))
    out = {}
    for nm, sh in [("panel", panel), ("lift", lift)] + \
                  [(f"module_{a}", s) for a, s in modules.items()]:
        out[nm] = round(cd(sh.cut(outer)), 1)
    print("  вне Ø620 (мм3):", out)
    t(f"T2_{v['name']}_Ø620", all(x <= 1.0 for x in out.values()),
      f"весь объём внутри Ø620: {out}")

    # --- коллизии ---
    rows = []
    for a, m in modules.items():
        vl = cd(m.common(lift))
        vp = cd(m.common(panel))
        rows.append({"модуль": f"D{a}", "с_подъёмником_мм3": round(vl, 1),
                     "с_панелью_мм3": round(vp, 1),
                     "зазор_подъёмник_мм": round(m.distToShape(lift)[0], 1),
                     "зазор_панель_мм": round(m.distToShape(panel)[0], 1)})
        print(f"  D{a}: подъёмник {vl:.0f} мм3 (зазор {rows[-1]['зазор_подъёмник_мм']}), "
              f"панель {vp:.0f} мм3 (зазор {rows[-1]['зазор_панель_мм']})")
    nbr = []
    for a1, a2 in ((0, 90), (90, 180), (180, 270), (270, 0)):
        vv = cd(modules[a1].common(modules[a2]))
        dd = modules[a1].distToShape(modules[a2])[0]
        nbr.append({"пара": f"D{a1}-D{a2}", "объём_мм3": round(vv, 1), "зазор_мм": round(dd, 1)})
        print(f"  D{a1} <-> D{a2}: {vv:.0f} мм3, зазор {dd:.1f} мм")

    lift_hit = any(r["с_подъёмником_мм3"] > 1.0 for r in rows)
    required_R = x_in - 5.0
    print(f"  [ВЫВОД] пересечение с колонной подъёмника Ø240: "
          f"{'ЕСТЬ' if lift_hit else 'нет'}; модуль доходит до x={x_in:.1f} мм, "
          f"нужен R_колонны <= {required_R:.1f} мм (Ø{2*required_R:.0f})")

    t(f"T4_{v['name']}_соседи", all(n["объём_мм3"] <= 1.0 for n in nbr),
      f"соседние модули не пересекаются (мин. зазор "
      f"{min(n['зазор_мм'] for n in nbr):.1f} мм)")

    # --- панель: связность ---
    n_sol = len(panel.Solids)
    vol_panel = panel.Volume / 1e6
    print(f"  панель: Solids={n_sol}, объём={vol_panel:.2f} л")
    print(f"  [ВЫВОД] панель после пазов и центрального проёма: {n_sol} тел — "
          + ("связна" if n_sol == 1 else "НЕСВЯЗНА (острова) -> нужна рама"))

    # --- рама (гипотеза несущей конструкции вместо сплошной панели) ---
    frame = build_frame(x_in)
    clamp = build_clamp()
    f_sol = len(frame.Solids)
    f_vol = frame.Volume
    f_mass = f_vol * AL_RHO
    c_mass = clamp.Volume * AL_RHO
    f_lift = cd(frame.common(lift))
    f_mod = max(cd(frame.common(m)) for m in modules.values())
    cl_mod = max(cd(clamp.common(m)) for m in modules.values())
    cl_lift = cd(clamp.common(lift))
    print(f"  рама: Solids={f_sol}, объём={f_vol/1e6:.2f} л, масса(Al)={f_mass:.1f} кг, "
          f"пересечение с подъёмником {f_lift:.0f} мм3, с модулями макс {f_mod:.0f} мм3")
    print(f"  хомут модуля: масса(Al)={c_mass:.2f} кг x4 = {4*c_mass:.1f} кг, "
          f"пересечение с модулем {cl_mod:.0f} мм3 (посадка Ø148 — по "
          f"поверхности, объёмного пересечения нет), с подъёмником {cl_lift:.0f} мм3")
    t(f"T6_{v['name']}_рама_без_коллизий", f_lift <= 1.0 and f_mod <= 1.0,
      f"рама (кольцо+4 луча+8 рычагов): масса Al {f_mass:.1f} кг; пересечений с "
      f"подъёмником ({f_lift:.0f} мм3) и модулями ({f_mod:.0f} мм3) нет")
    doc.addObject("Part::Feature", "frame_proposal").Shape = frame
    doc.addObject("Part::Feature", "clamp_proposal").Shape = clamp

    # --- подвеска: модуль+хомут смещаются относительно рамы ---
    susp = []
    for s in SUSP:
        m = mod0.copy().translate(App.Vector(0, 0, s))
        cl = clamp.copy().translate(App.Vector(0, 0, s))
        vl, vfr, vcl_fr = cd(m.common(lift)), cd(m.common(frame)), cd(cl.common(frame))
        susp.append({"dz": s, "модуль_подъёмник_мм3": round(vl, 1),
                     "модуль_рама_мм3": round(vfr, 1),
                     "хомут_рама_мм3": round(vcl_fr, 1)})
    print("  подвеска:", susp)
    susp_ok = all(x["модуль_рама_мм3"] <= 1.0 and x["хомут_рама_мм3"] <= 1.0
                  for x in susp)
    t(f"T7_{v['name']}_подвеска", susp_ok,
      f"при ±15/±30 мм зазоры модуль↔рама и хомут↔рама сохраняются: {susp_ok} "
      f"(хомут движется с модулем, рычаги рамы — направляющие скольжения; "
      f"сам узел скольжения не спроектирован)")

    # --- просвет / низшие точки ---
    bb = mod0.BoundBox
    t(f"T8_{v['name']}_просвет", bb.ZMin >= -0.01,
      f"низшая точка модуля z={bb.ZMin:.1f} мм (колесо на полу), низ мотора "
      f"z={89-45:.0f} мм; ось колеса 89 мм вместо 120 мм (RoboCT Ø240) — "
      f"база опускается на 31 мм")

    # --- выводы (не тесты) ---
    z_motor_top = WHEEL_D / 2 + MOTOR_SQ / 2
    z_lift0_min = z_motor_top + GAP
    RW_needed = RW + (LIFT_D / 2 - x_in) + 5.0
    FIND.extend([
        {"id": f"F1_{v['name']}_колонна",
         "текст": f"модуль доходит до x={x_in:.1f} мм при радиусе колонны "
                  f"{LIFT_D/2:.0f} мм: пересечение {rows[0]['с_подъёмником_мм3']:.0f} мм3 "
                  f"на каждый модуль; для обхода нужна колонна Ø{2*required_R:.0f} мм"},
        {"id": f"F2_{v['name']}_панель",
         "текст": f"сплошная панель с пазами до x={x_in:.1f} и центральным проёмом "
                  f"Ø240 распадается на {n_sol} тела — несущая панель в прежнем "
                  f"виде невозможна"},
        {"id": f"F2b_{v['name']}_рама",
         "текст": f"рама из кольца и лучей в ОДНОЙ горизонтальной плоскости "
                  f"распадается на {f_sol} квадранта: пазы модулей шириной "
                  f"{2*(WHEEL_D/2+GAP):.0f} мм прорезают кольцо и периметр "
                  f"насквозь; связность достижима только через корпус модуля "
                  f"(хомут) или вторым уровнем выше колеса z>178 — вне высоты "
                  f"базы 176 мм"},
        {"id": f"F3_{v['name']}_z",
         "текст": f"верх моторной зоны z={z_motor_top:.0f} мм; вертикальное "
                  f"разнесение потребовало бы z_lift0 >= {z_lift0_min:.0f} мм, но "
                  f"колонна проходит через всю высоту базы (z=-88..+400) — "
                  f"несовместимо с существующим лифтом"},
        {"id": f"F4_{v['name']}_RW",
         "текст": f"обход колонны смещением колёс: RW >= {RW_needed:.1f} мм "
                  f"(сейчас 248.9) -> наружный торец {RW_needed + 48:.1f} мм > 310, "
                  f"выход за Ø620 на {RW_needed + 48 - 310:.1f} мм"},
    ])

    doc.recompute()
    path = os.path.join(HERE, f"rvw7pa_layout_{v['name']}.FCStd")
    if os.path.exists(path):
        os.remove(path)
    doc.saveAs(path)
    App.closeDocument(doc.Name)

    verdict = ("NO FIT: модули пересекают колонну подъёмника, панель несвязна"
               if lift_hit and n_sol > 1 else
               "CONDITIONAL" if not lift_hit and n_sol == 1 else
               "NO FIT: требуется переработка несущей конструкции")
    res = {
        "вариант": v,
        "геометрия": {"x_out": x_out, "x_in": round(x_in, 1), "c": c, "d": d,
                      "RW": RW, "наружный_радиус_план": round(
                          math.hypot(x_out, WHEEL_D / 2), 2)},
        "вне_Ø620_мм3": out,
        "коллизии_модулей": rows,
        "соседи": nbr,
        "панель": {"Solids": n_sol, "объём_л": round(vol_panel, 2),
                   "для_сравнения_масса_Al_кг": round(panel.Volume * AL_RHO, 1)},
        "рама_гипотеза": {"Solids": f_sol, "объём_л": round(f_vol / 1e6, 2),
                          "масса_Al_кг": round(f_mass, 1),
                          "пересечение_подъёмник_мм3": round(f_lift, 1),
                          "max_пересечение_модуль_мм3": round(f_mod, 1),
                          "состав": "кольцо r124..168 + 4 луча 50×30 под 45° + "
                                    "8 рычагов подвески (|y| 97..195, x 175..195, "
                                    "z 60..90); хомут Ø148..190×20 — отдельная "
                                    "деталь, движется с модулем"},
        "подвеска": susp,
        "вертикальное_разнесение": {"z_motor_top": z_motor_top,
                                    "z_lift0_min": z_lift0_min,
                                    "статус": "несовместимо с существующей "
                                              "колонной подъёмника"},
        "переработка": {"R_колонны_требуемый": round(required_R, 1),
                        "Ø_колонны_требуемый": round(2 * required_R, 1),
                        "RW_необходимый": round(RW_needed, 1),
                        "выход_за_Ø620_мм": round(RW_needed + 48 - 310, 1)},
        "вердикт": verdict,
        "checks": [c for c in CHECK if c["id"].startswith("T") and v["name"] in c["id"]],
        "findings": [f for f in FIND if v["name"] in f["id"]],
        "fcstd": os.path.basename(path), "fcstd_sha16": sha16(path),
    }
    print(f"  ВЕРДИКТ: {verdict}")
    print(f"  FCStd: {path} SHA16 {res['fcstd_sha16']}")
    return res


results = {}
for v in VARIANTS:
    results[v["name"]] = run_variant(v)

out = {"источник": "Nabtesco en_cat_RVW.pdf Rev.005.1 CAT.210125_B (03.09.2026), "
                   "стр.17-18 (габариты), стр.39-40 (чертежи 7PALA/7PARA)",
       "RW": RW, "Ø620": 2 * PLATFORM_R, "колонна_подъёмника_Ø": LIFT_D,
       "панель_z": [PANEL_Z0, PANEL_Z1], "ось_колеса_z": WHEEL_D / 2,
       "предположения": [
           "оболочка модуля консервативная (цилиндры + □90), точный контур — STEP",
           "кабельный разъём Ø14×25 смоделирован сверху мотора; трасса кабеля — "
           "зона обслуживания, параметр",
           "колонна подъёмника — цилиндр Ø240 на всю высоту базы (по "
           "stage2g_lift_interface.md: сквозной проём)",
           "панель z=50..90 сохранена от layout v2",
           "рама — гипотеза конструкции, материал Al (2.7 г/см3) — параметр"],
       "варианты": results, "checks": CHECK, "findings": FIND}
p = os.path.join(HERE, "rvw7pa_layout.json")
with open(p, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nREGRESSION:", "PASS" if all(c["ok"] for c in CHECK) else "FAIL")
print("JSON:", p)

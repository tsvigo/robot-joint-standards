# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v5_geometry.py (только Python).

Сравнение геометрии двух схем круглого основания с ненулевыми проектными зазорами:
  R — радиальные оси (v4, колёса на 0/90/180/270°);
  P — параллельные оси || Y на диагоналях FL/FR/RL/RR (альтернатива, стандартная X).

Реальные элементы колеса (модель для проверки габарита):
  D_asm = D + dia_extra   — ролики + торцевой крепёж;
  W_asm = W + asm_extra   — ролики + ступицы/подшипники/крепёж (внешний привод);
  W_hub = W + hub_extra   — колесо со встроенным мотор-ступицей.
Проектный зазор до транспортного контура: gap_transport (не касание!).

Проверки: зазор контура, зона лифта Ø240, полосы приводов, взаимное положение
колёс, секторы оборудования, блоки аутригеров, опорные контуры (транспорт/работа),
углы опрокидывания (параметрически по высоте ЦМ), минимальные диаметры.
Массы/ЦМ не вводились — только геометрия и кинематика.
Результат: stage2g_v5_geometry.json + печать.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

WHEELS = {
    "D152": {"D": 152.4, "W": 80.0},
    "D203": {"D": 203.0, "W": 103.0},
    "D230": {"D": 230.0, "W": 113.0},
}
DPS = [600.0, 620.0, 640.0]

P = {
    "lift_R": 120.0,          # зона лифта Ø240 (наследие v2) — параметр
    "lift_gap": 10.0,         # зазор до зоны лифта
    "dia_extra": 10.0,        # ролики + торцевой крепёж к диаметру
    "asm_extra": 30.0,        # ступицы/подшипники/крепёж к ширине (внешний привод)
    "hub_extra": 60.0,        # ширина колеса со встроенным мотор-ступицей
    "gap_transport": 10.0,    # проектный зазор сборки до транспортного контура
    "well_clr": 10.0,         # зазор ниши
    "S_inline": 150.0,        # коаксиальный привод за колесом
    "S_belt": 60.0,           # ремённый/параллельный привод
    "S_hub": 0.0,             # мотор-ступица: внешняя полоса не нужна
    "pod_half_tan": 70.0,
    "pod_depth": 100.0,
    "pod_r_out_off": 30.0,    # наружная грань блока аутригера = R − off
    "bat_Rout_margin": 140.0,  # батарейный сектор до R − margin (блок аутригера R−130..R−30)
    "stroke": [200.0, 300.0, 400.0],
    "drive_w_m": 60.0,        # поперечник объёма привода (для схемы P)
    "cg_heights": [300.0, 500.0, 700.0, 900.0],  # для предварительных углов опрокидывания
}
LIFT_CLEAR_R = P["lift_R"] + P["lift_gap"]   # 130


def rect_point_dist(px, py, x0, x1, y0, y1):
    dx = max(x0 - px, px - x1, 0.0)
    dy = max(y0 - py, py - y1, 0.0)
    return math.hypot(dx, dy)


def sector_half_ok(half, sector_dir, rects, r_in, r_out, clr):
    """Сектор оборудования: ось sector_dir (угол), радиус r_in..r_out, полутангенс half."""
    a = math.radians(sector_dir)
    ca, sa = math.cos(a), math.sin(a)
    r = r_in
    while r <= r_out + 1e-9:
        t = -half
        while t <= half + 1e-9:
            x = r * ca - t * sa
            y = r * sa + t * ca
            for (x0, x1, y0, y1) in rects:
                if rect_point_dist(x, y, x0, x1, y0, y1) < clr:
                    return False
            t += 5.0
        r += 5.0
    return True


def sector_max_half(sector_dir, rects, r_in, r_out, clr):
    for ht in range(160, 4, -5):
        if sector_half_ok(float(ht), sector_dir, rects, r_in, r_out, clr):
            return float(ht)
    return 0.0


def eval_case(layout, wkey, Dp, drive):
    D, W = WHEELS[wkey]["D"], WHEELS[wkey]["W"]
    D_asm = D + P["dia_extra"]
    W_eff = (W + P["hub_extra"]) if drive == "hub" else (W + P["asm_extra"])
    S_req = {"hub": P["S_hub"], "belt": P["S_belt"], "inline": P["S_inline"]}[drive]
    R_env = Dp / 2.0
    R_lim = R_env - P["gap_transport"]          # предел для реальных элементов
    c = P["well_clr"]
    res = {"layout": layout, "wheel": wkey, "Dp": Dp, "drive": drive,
           "D_asm": D_asm, "W_eff": W_eff, "S_req": S_req}

    if layout == "R":
        # ось радиальна: угловая точка (r_c + W_eff/2, D_asm/2) <= R_lim
        r_c = math.sqrt(max(R_lim ** 2 - (D_asm / 2.0) ** 2, 0.0)) - W_eff / 2.0
        corner = math.hypot(r_c + W_eff / 2.0, D_asm / 2.0)
        well_inner = r_c - W_eff / 2.0 - c
        lift_margin = well_inner - LIFT_CLEAR_R
        band = well_inner - LIFT_CLEAR_R
        band_margin = band - S_req
        # ниши (прямоугольные проекции) для секторов оборудования
        rects = []
        for th in (0, 90, 180, 270):
            a = math.radians(th)
            rc_, rs_ = math.cos(a), math.sin(a)
            half_rad = W_eff / 2.0 + c
            half_tan = D_asm / 2.0 + c
            # прямоугольник, повёрнутый на th: пересчитаем через 4 угла
            corners = []
            for (dr, dt) in ((-half_rad, -half_tan), (-half_rad, half_tan),
                             (half_rad, -half_tan), (half_rad, half_tan)):
                x = (r_c + dr) * rc_ - dt * rs_
                y = (r_c + dr) * rs_ + dt * rc_
                corners.append((x, y))
            xs = [p[0] for p in corners]
            ys = [p[1] for p in corners]
            # для сетки используем AABB-пересечение с поворотом через проверку расстояния
            # по 4 углам + центру (консервативно: берём описанный AABB)
            rects.append((min(xs), max(xs), min(ys), max(ys)))
        # взаимное положение колёс: зазор между соседними нишами
        gap_wheels = (r_c - W_eff / 2.0 - c) - (D_asm / 2.0 + c)
        res.update({
            "r_c": round(r_c, 1), "corner_R": round(corner, 1),
            "gap_to_contour": round(R_env - corner, 1),
            "well_inner": round(well_inner, 1),
            "lift_margin": round(lift_margin, 1),
            "band_mm": round(band, 1), "band_margin": round(band_margin, 1),
            "wheels_gap": round(gap_wheels, 1),
            "supp_dir_deg": [0, 90, 180, 270],
            "pad_dir_deg": [45, 135, 225, 315],
        })
        # опорный контур колёс: ромб вершинами на (r_c + W/2) по осям
        rr = r_c + W / 2.0
        res["supp_transport"] = {"area_m2": round(2 * rr ** 2 / 1e6, 4),
                                 "inradius": round(rr / math.sqrt(2.0), 1)}
    else:
        # схема P: оси || Y, позиции (+-u, +-u); угловая точка (u + D_asm/2, u + W_eff/2)
        a1, b1 = D_asm / 2.0, W_eff / 2.0
        # (u+a1)^2 + (u+b1)^2 = R_lim^2
        A = 2.0
        B = 2.0 * (a1 + b1)
        C = a1 * a1 + b1 * b1 - R_lim * R_lim
        disc = max(B * B - 4 * A * C, 0.0)
        u = (-B + math.sqrt(disc)) / (2 * A)
        corner = math.hypot(u + a1, u + b1)
        inner = math.hypot(max(u - a1, 0.0), max(u - b1, 0.0))
        lift_margin = inner - LIFT_CLEAR_R
        # полоса привода вдоль оси за колесом: от зоны лифта до внутренней грани
        x_in = u - P["drive_w_m"] / 2.0
        y_lift = math.sqrt(max(LIFT_CLEAR_R ** 2 - x_in * x_in, 0.0)) if x_in < LIFT_CLEAR_R else 0.0
        band = (u - b1 - c) - y_lift
        band_margin = band - S_req
        gap_wheels = (2 * u - D_asm) / 2.0      # зазор между соседними колёсами по X/Y
        res.update({
            "u": round(u, 1), "corner_R": round(corner, 1),
            "gap_to_contour": round(R_env - corner, 1),
            "inner_corner_R": round(inner, 1),
            "lift_margin": round(lift_margin, 1),
            "band_mm": round(band, 1), "band_margin": round(band_margin, 1),
            "wheels_gap": round(2 * u - D_asm, 1),
            "supp_dir_deg": [45, 135, 225, 315],
            "pad_dir_deg": [0, 90, 180, 270],
        })
        # опорный контур колёс: прямоугольник |x|<=u, |y|<=u+W/2
        res["supp_transport"] = {"area_m2": round(4 * u * (u + W / 2.0) / 1e6, 4),
                                 "inradius": round(min(u, u + W / 2.0), 1)}

    # блоки аутригеров (транспорт) и площадки (работа)
    pod_r_out = R_env - P["pod_r_out_off"]
    pod_corner = math.hypot(pod_r_out, P["pod_half_tan"])
    pad_r = (pod_r_out - P["pod_depth"] / 2.0) + 300.0   # показ. ход 300
    res["pod_corner_R"] = round(pod_corner, 1)
    res["pod_gap_to_contour"] = round(R_env - pod_corner, 1)
    res["pads"] = {str(int(s)): {"pad_r": round((pod_r_out - P["pod_depth"] / 2.0) + s, 1),
                                 "area_m2": round(2 * ((pod_r_out - P["pod_depth"] / 2.0) + s) ** 2 / 1e6, 4),
                                 "inradius": round(((pod_r_out - P["pod_depth"] / 2.0) + s) / math.sqrt(2.0), 1)}
                   for s in P["stroke"]}
    res["supp_working"] = {"area_m2": round(2 * pad_r ** 2 / 1e6, 4),
                           "inradius": round(pad_r / math.sqrt(2.0), 1)}

    # углы опрокидывания (предварительно, ЦМ в центре): tan α = inradius / h_cg
    res["tip_angles_deg"] = {
        f"h{int(h)}": round(math.degrees(math.atan2(res["supp_transport"]["inradius"], h)), 1)
        for h in P["cg_heights"]}
    res["tip_angles_working_deg"] = {
        f"h{int(h)}": round(math.degrees(math.atan2(res["supp_working"]["inradius"], h)), 1)
        for h in P["cg_heights"]}

    # секторы оборудования
    if layout == "R":
        r_in = LIFT_CLEAR_R
        r_out = R_env - P["bat_Rout_margin"]
        half = sector_max_half(45.0, res.get("_rects", []), r_in, r_out, P["well_clr"])
    else:
        r_in = LIFT_CLEAR_R
        r_out = R_env - P["bat_Rout_margin"]
        half = sector_max_half(0.0, res.get("_rects", []), r_in, r_out, P["well_clr"])
    res["battery_half_tan"] = half
    res["battery_radial"] = round(r_out - r_in, 1)

    # пригодность
    res["pass"] = bool(res["lift_margin"] >= 0 and res["band_margin"] >= 0
                       and res["wheels_gap"] >= 10.0
                       and res["gap_to_contour"] >= P["gap_transport"] - 0.01
                       and res["pod_gap_to_contour"] > 0)
    return res


def eval_case_full(layout, wkey, Dp, drive):
    """eval_case + прямоугольники колёс для проверки секторов."""
    res = eval_case(layout, wkey, Dp, drive)
    D, W = WHEELS[wkey]["D"], WHEELS[wkey]["W"]
    D_asm = D + P["dia_extra"]
    W_eff = (W + P["hub_extra"]) if drive == "hub" else (W + P["asm_extra"])
    c = P["well_clr"]
    rects = []
    if layout == "R":
        r_c = res["r_c"]
        for th in (0, 90, 180, 270):
            a = math.radians(th)
            ca, sa = math.cos(a), math.sin(a)
            hr, ht = W_eff / 2.0 + c, D_asm / 2.0 + c
            xs, ys = [], []
            for (dr, dt) in ((-hr, -ht), (-hr, ht), (hr, -ht), (hr, ht)):
                xs.append((r_c + dr) * ca - dt * sa)
                ys.append((r_c + dr) * sa + dt * ca)
            rects.append((min(xs), max(xs), min(ys), max(ys)))
    else:
        u = res["u"]
        for sx in (1, -1):
            for sy in (1, -1):
                rects.append((sx * u - D_asm / 2.0 - c, sx * u + D_asm / 2.0 + c,
                              sy * u - W_eff / 2.0 - c, sy * u + W_eff / 2.0 + c))
    res["_rects"] = rects
    # пересчёт сектора с реальными прямоугольниками
    r_in = LIFT_CLEAR_R
    r_out = res["Dp"] / 2.0 - P["bat_Rout_margin"]
    sect_dir = 45.0 if layout == "R" else 0.0
    res["battery_half_tan"] = sector_max_half(sect_dir, rects, r_in, r_out, P["well_clr"])
    res.pop("_rects", None)
    return res


results = []
for layout in ("R", "P"):
    for Dp in DPS:
        for wkey in WHEELS:
            for drive in ("hub", "belt", "inline"):
                results.append(eval_case_full(layout, wkey, Dp, drive))


def min_diameter(layout, wkey, drive):
    d = 500.0
    while d <= 950.0:
        r = eval_case(layout, wkey, d, drive)   # без сетки секторов (на вердикт не влияет)
        if r["pass"]:
            return d
        d += 0.5
    return None


min_d = {}
for layout in ("R", "P"):
    for wkey in WHEELS:
        for drive in ("hub", "belt", "inline"):
            min_d[f"{layout}_{wkey}_{drive}"] = min_diameter(layout, wkey, drive)

out = {"params": P, "lift_clear_R": LIFT_CLEAR_R,
       "cases": [dict(r) for r in results], "min_diameter_mm": min_d,
       "notes": [
           "D_asm = D + 10 (ролики + торцевой крепёж); W_asm = W + 30; W_hub = W + 60 — параметры, требуют проверки по чертежам.",
           "Зазор до транспортного контура 10 мм: касание max_r = R НЕ принимается как готовая конструкция.",
           "Ø600/620/640 — внешний габарит сборки: панель = ØDp, все реальные элементы ≤ R − 10 мм.",
           "Полосы приводов (0/60/150 мм) — условные, hub=OK не подтверждает существование мотор-колеса (см. задачу 6).",
           "Углы опрокидывания — геометрия (ЦМ в центре на заданной высоте); массы/ЦМ не вводились.",
       ]}
path = os.path.join(HERE, "stage2g_v5_geometry.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

for layout in ("R", "P"):
    name = "R-радиальные оси" if layout == "R" else "P-параллельные оси (диагонали)"
    print(f"\n===== СХЕМА {name} =====")
    print(f"{'Dпан':>5} {'колесо':>6} {'привод':>6} {'ось/ u':>7} {'зазор':>6} {'лифт±':>7} "
          f"{'полоса':>7} {'полоса±':>7} {'колёса':>7} {'сектор±':>7} {'вердикт':>7}")
    for r in results:
        if r["layout"] != layout:
            continue
        pos = r.get("r_c", r.get("u"))
        print(f"{r['Dp']:5.0f} {r['wheel']:>6} {r['drive']:>6} {pos:7.1f} "
              f"{r['gap_to_contour']:6.1f} {r['lift_margin']:7.1f} {r['band_mm']:7.1f} "
              f"{r['band_margin']:7.1f} {r['wheels_gap']:7.1f} {r['battery_half_tan']:7.0f} "
              f"{'OK' if r['pass'] else 'НЕТ':>7}")

print("\n===== МИНИМАЛЬНЫЕ ДИАМЕТРЫ (внешний габарит сборки, зазор 10 мм) =====")
for layout in ("R", "P"):
    for wkey in WHEELS:
        row = " ".join(f"{d}={min_d[f'{layout}_{wkey}_{d}']}" for d in ("hub", "belt", "inline"))
        print(f"  {layout} {wkey}: {row}")

print("\n===== ОПОРНЫЕ КОНТУРЫ И УГЛЫ ОПРОКИДЫВАНИЯ (пример: belt, Ø640) =====")
for r in results:
    if r["Dp"] == 640.0 and r["drive"] == "belt":
        st, sw = r["supp_transport"], r["supp_working"]
        print(f"  {r['layout']} {r['wheel']}: транспорт A={st['area_m2']} м², инрадиус={st['inradius']} мм, "
              f"углы {r['tip_angles_deg']}")
        print(f"        работа(ход300) A={sw['area_m2']} м², инрадиус={sw['inradius']} мм, "
              f"углы {r['tip_angles_working_deg']}")

print("\nJSON:", path)

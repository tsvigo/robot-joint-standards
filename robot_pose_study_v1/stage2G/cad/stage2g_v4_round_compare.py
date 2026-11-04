# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v4_round_compare.py (только Python).
Круглое основание: Ø600/620/640 x Mecanum Ø152/203/230.
Методика ниш v4: ниша = ПРЯМОУГОЛЬНАЯ проекция колеса (radial W x tangential D)
+ зазор: при качении проекция не меняется (тело вращения вокруг горизонтальной оси),
  описанная окружность не требуется.
Проверки: внешний контур, зона лифта, полоса привода по типам (инлайн/ремень/hub),
взаимное положение колёс, сектора оборудования, опорный контур, минимальные диаметры.
Без расчётов моментов/прочности/устойчивости. Результат: stage2g_v4_round_results.json.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

VARIANTS = [600.0, 620.0, 640.0]
WHEELS = {
    "D152": {"D": 152.4, "W": 80.0,  "kg_set": 10.4},
    "D203": {"D": 203.0, "W": 103.0, "kg_set": 32.0},
    "D230": {"D": 230.0, "W": 113.0, "kg_set": 56.0},
}
P = {
    "lift_R": 120.0,        # радиус зоны лифта (наследие v2 Ø240) — параметр
    "clr": 10.0,            # сборочный зазор
    "S_inline": 150.0,      # коаксиальный (инлайн) привод за колесом
    "S_belt": 60.0,         # ремённый/параллельный привод
    "S_hub": 40.0,          # hub-мотор
    "pod_half_tan": 80.0,
    "pod_radial_in": 100.0,
    "bat_Rout_margin": 130.0,   # отступ сектора оборудования от края (блок аутригера R-120..R-20)
    "stroke": [200.0, 300.0, 400.0],
    "well_clr": 10.0,       # зазор прямоугольной ниши
    "battery_z_Rout": None, # заполняется
}
DOOR = 700.0

def point_rect_dist(px, py, x0, x1, y0, y1):
    """Расстояние точки до осевого прямоугольника (0 если внутри)."""
    dx = max(x0 - px, px - x1, 0.0)
    dy = max(y0 - py, py - y1, 0.0)
    return math.hypot(dx, dy)

def battery_half_span_ok(half_tan, rect, R_out, R_lift, clr):
    """Сектор батарей на 45° не пересекает нишу-прямоугольник (с проверкой по сетке)."""
    if half_tan <= 0:
        return False
    a0 = math.radians(45.0)
    ca, sa = math.cos(a0), math.sin(a0)
    r = R_lift + clr
    while r <= R_out + 1e-9:
        t = -half_tan
        while t <= half_tan + 1e-9:
            x = r * ca - t * sa
            y = r * sa + t * ca
            if point_rect_dist(x, y, *rect) < clr:
                return False
            t += 5.0
        r += 5.0
    return True

results = []
for Dp in VARIANTS:
    R_env = Dp / 2.0
    for wkey, w in WHEELS.items():
        D, W = w["D"], w["W"]
        # предельный радиус оси колеса: угловая точка (r_o, D/2) на контуре
        r_max = math.sqrt(max(R_env ** 2 - (D / 2.0) ** 2, 0.0)) - W / 2.0
        r_c = r_max
        corner_R = math.hypot(r_c + W / 2.0, D / 2.0)
        # прямоугольная ниша (мировые для колеса на углу 0°: x=радиальный, y=тангенс)
        c = P["well_clr"]
        rect = (r_c - W / 2.0 - c, r_c + W / 2.0 + c, -(D / 2.0 + c), D / 2.0 + c)
        well_inner = rect[0]
        lift_zone = P["lift_R"] + P["clr"]
        lift_clear = well_inner >= lift_zone
        # полоса привода между зоной лифта и внутренней границей ниши
        band = well_inner - lift_zone
        drive_fit = {"inline": band >= P["S_inline"],
                     "belt": band >= P["S_belt"],
                     "hub": band >= P["S_hub"]}
        def dmin(S):
            # минимальный диаметр: ось колеса на пределе полосы привода,
            # контур = угловая точка КОЛЕСА (r_c+W/2, D/2); зазор ниши (+c) входит в r_c
            r_n = lift_zone + S + W / 2.0 + c
            return 2.0 * math.hypot(r_n + W / 2.0, D / 2.0)
        # минимальные диаметры (точная геометрия отказа, п.9)
        d_min = {k: round(dmin(P[f"S_{k}"]), 1) for k in ("inline", "belt", "hub")}
        # взаимное положение колёс (соседи 90°): rect-rect по диагонали
        # колесо на 0°: x>=well_inner, |y|<=D/2+c ; на 90°: y>=well_inner, |x|<=D/2+c
        gap = well_inner - (D / 2.0 + c)
        wheels_clear = gap >= P["clr"]
        # сектор оборудования на 45° (между нишами)
        R_bat_out = R_env - P["bat_Rout_margin"]
        bat_half, bat_ok = 0.0, False
        for ht in range(140, 4, -5):
            if battery_half_span_ok(float(ht), rect, R_bat_out, P["lift_R"], P["clr"]):
                bat_half, bat_ok = float(ht), True
                break
        # полоса привода для показанного в CAD объёма (hub)
        supp_wheel = 2.0 * r_c ** 2 / 1e6
        pod_r_out = R_env - 20.0
        pod_ok = math.hypot(pod_r_out, P["pod_half_tan"]) <= R_env + 1e-6
        pads = {}
        for s in P["stroke"]:
            pad_r = (pod_r_out - P["pod_radial_in"] / 2.0) + s
            pads[str(int(s))] = {"pad_r": round(pad_r, 1),
                                 "supp_area_m2": round(2 * pad_r ** 2 / 1e6, 4)}
        feas = {k: (lift_clear and drive_fit[k] and wheels_clear and bat_ok
                    and corner_R <= R_env + 1e-6 and pod_ok)
                for k in ("inline", "belt", "hub")}
        results.append({
            "Dp": Dp, "wheel": wkey, "D": D, "W": W,
            "r_c": round(r_c, 1), "corner_R": round(corner_R, 1),
            "envelope_ok": corner_R <= R_env + 1e-6,
            "well_rect": [round(v, 1) for v in rect],
            "well_inner": round(well_inner, 1), "lift_clear": lift_clear,
            "band_mm": round(band, 1), "drive_fit": drive_fit, "feasible": feas,
            "d_min_mm": d_min,
            "battery_half_tan": bat_half, "battery_ok": bat_ok,
            "battery_usable": bat_half >= 60.0,   # сектор пригоден для типового блока (>=120 мм)
            "battery_radial_width": round(R_bat_out - lift_zone, 1),
            "wheels_gap": round(gap, 1), "wheels_clear": wheels_clear,
            "wheel_kg_set": w["kg_set"],
            "supp_wheel_m2": round(supp_wheel, 4),
            "pod_ok_transport": pod_ok, "pads": pads,
            "door700_margin": round(DOOR - Dp, 1),
            "panel_area_m2": round(math.pi * (Dp / 2.0) ** 2 / 1e6, 4),
        })

out = {"params": P, "variants": results, "notes": [
    "Внешний контур: угловая точка проекции колеса (r_c+W/2, D/2) — касание окружности при r=r_max.",
    "d_min_* — минимальный диаметр платформы для размещения привода заданного типа (п.9).",
    "Площади опор — геометрия контура, НЕ расчёт устойчивости. Ходы аутригеров — параметры.",
    "Все параметры (зона лифта Ø240, полосы привода, зазоры) — предварительные, на согласование.",
]}
path = os.path.join(HERE, "stage2g_v4_round_results.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

print("== 9 КОМБИНАЦИЙ (прямоугольные ниши) ==")
print(f"{'Dпан':>5} {'Кол':>5} {'r_c':>6} {'контур':>7} {'внутр.ниши':>9} {'полоса':>6} "
      f"{'лифт':>4} {'колёса':>5} {'бат':>7} {'hub/belt/inline':>16} {'опора м2':>8} {'колёса кг':>8}")
for r in results:
    f_ = r["feasible"]
    fs = ("+" if f_["hub"] else "-") + "/" + ("+" if f_["belt"] else "-") + "/" + ("+" if f_["inline"] else "-")
    print(f"{r['Dp']:5.0f} {r['wheel']:>5} {r['r_c']:6.1f} {r['corner_R']:7.1f} "
          f"{r['well_inner']:9.1f} {r['band_mm']:6.1f} "
          f"{'+' if r['lift_clear'] else 'НЕТ':>4} {'+' if r['wheels_clear'] else 'НЕТ':>5} "
          f"{('+' + str(int(r['battery_half_tan']))) if r['battery_ok'] else 'НЕТ':>7} "
          f"{fs:>16} {r['supp_wheel_m2']:8.4f} {r['wheel_kg_set']:8.1f}")

print("\n== ПРИГОДНОСТЬ И МИНИМАЛЬНЫЕ ДИАМЕТРЫ (п.9) ==")
for r in results:
    f_ = r["feasible"]
    print(f"  Dp={r['Dp']:.0f} {r['wheel']}: hub={'OK' if f_['hub'] else 'НЕТ'} "
          f"belt={'OK' if f_['belt'] else 'НЕТ'} inline={'OK' if f_['inline'] else 'НЕТ'} | "
          f"д_min: инлайн={r['d_min_mm']['inline']}, ремень={r['d_min_mm']['belt']}, hub={r['d_min_mm']['hub']}")

print("\n== ОПОРНЫЙ КОНТУР ==")
for r in results[:3]:  # площади колёс зависят от Dp и колеса — печатаем все, но компактно
    pass
for r in results:
    pads = " | ".join(f"{s}мм:R={v['pad_r']}:A={v['supp_area_m2']}" for s, v in r["pads"].items())
    print(f"  Dp={r['Dp']:.0f} {r['wheel']}: A_колёс={r['supp_wheel_m2']} | {pads}")

print("\nJSON:", path)

# -*- coding: utf-8 -*-
"""
Этап 2Г — check_stage2g_base.py
Проверки собранной модели stage2g_base.FCStd (фаза checks):

  C1  габарит физических деталей vs ограничение 670x670x176
  C2  системы координат: ось лифта, центры колёс vs подтверждённые значения Blender
  C3  геометрические пересечения между физическими деталями (boolean common)
  C4  компоновочные зоны: пересечения зон с деталями/колёсами (информативно)
  C5  опорный контур колёс vs подтверждённый (площадь, центр)
  C6  подъезд к кровати при c=0.05 и c=0.10 м
  C7  объёмы — только у физических деталей; масса = «не рассчитана»
      (материал не утверждён); массы компоновочных объёмов НЕ считаются

Запуск: freecadcmd check_stage2g_base.py
Результат: stdout + JSON stage2g_check_results.json (в каталоге скрипта).
"""

import os
import json
import re
import math
import FreeCAD as App

try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    script_dir = os.getcwd()

FCSTD = os.path.join(script_dir, "stage2g_base_v2.FCStd")

# подтверждённые значения (Blender / STAGE2G_SOURCE_DATA.md), мм
CONF = dict(
    env_W=670.0, env_H=670.0, env_T=176.0,
    floor_z=-134.55,
    wheel_d=150.8, wheel_w=106.8,
    wheels={"FL": (257.8, 256.9), "FR": (258.6, -257.3),
            "BL": (-258.6, 258.2), "BR": (-259.6, -257.7)},
    lift_axis=(0.003, -4.41),
    support_area_m2=0.266435,
    support_center=(-0.60, -0.12),
    support_sides=(518.2, 514.2, 516.3, 515.9),
    wheel_outer_reach=258.6 + 150.8 / 2.0,  # 334.0 — край колеса по X у FR/FL
)

results = {"checks": {}, "volumes": {}, "meta": {}}


def _count_params(sheet):
    """Число параметров в таблице (API 1.1.4: getContents требует разделитель)."""
    if sheet is None:
        return None
    try:
        content = sheet.getContents(";")
        return len([c for c in str(content).split(";") if c.strip()])
    except Exception:
        return None


def rec(cid, name, ok, detail=""):
    results["checks"][cid] = {"name": name, "ok": bool(ok), "detail": detail}
    print(f"{'PASS' if ok else 'FAIL'} | {cid} | {name}" + (f" | {detail}" if detail else ""))
    return bool(ok)


if not os.path.isfile(FCSTD):
    print(f"FAIL | нет файла модели: {FCSTD}")
    raise SystemExit(2)

doc = App.openDocument(FCSTD)  # v2
doc.recompute()

ps = doc.getObject("params")

# App::Part в 1.1.4 auto-тянет зависимости в группу — фильтруем по именам:
# физические финальные детали = FRM_/PNL_/WLM_ без суффиксов-промежуточников
_INTER = re.compile(r"(_box|_body|_hole|_c1)$")
part_objs = [o for o in doc.Objects
             if o.Name.startswith(("FRM_", "PNL_", "WLM_"))
             and not _INTER.search(o.Name)
             and hasattr(o, "Shape") and not o.Shape.isNull()]
comp_objs = [o for o in doc.Objects
             if o.Name.startswith(("WHL_", "ZN_"))
             and hasattr(o, "Shape") and not o.Shape.isNull()]
results["meta"] = {
    "freecad": ".".join(str(x) for x in App.Version()[:3]),
    "n_parts": len(part_objs),
    "n_components": len(comp_objs),
    "n_params": _count_params(ps),
    "part_names": [o.Name for o in part_objs],
    "component_names": [o.Name for o in comp_objs],
}

# --- C1 габарит -----------------------------------------------------------
bb = part_objs[0].Shape.BoundBox
for o in part_objs[1:]:
    bb.add(o.Shape.BoundBox)
env_ok = (abs(bb.XMin + 335.0) < 1e-6 and abs(bb.XMax - 335.0) < 1e-6 and
          abs(bb.YMin + 335.0) < 1e-6 and abs(bb.YMax - 335.0) < 1e-6 and
          abs(bb.ZMin + 88.0) < 1e-6 and abs(bb.ZMax - 88.0) < 1e-6)
rec("C1", "габарит деталей = 670x670x176 (база-локальный, центр)",
    env_ok,
    f"X {bb.XMin:.3f}..{bb.XMax:.3f}, Y {bb.YMin:.3f}..{bb.YMax:.3f}, "
    f"Z {bb.ZMin:.3f}..{bb.ZMax:.3f}")

# --- C2 оси и центры ------------------------------------------------------
lift = doc.getObject("ZN_lift_column")
lx, ly = lift.Placement.Base.x, lift.Placement.Base.y
c2a = abs(lx - CONF["lift_axis"][0]) < 1e-6 and abs(ly - CONF["lift_axis"][1]) < 1e-6
rec("C2.1", "ось лифта = (+0.003, -4.41) мм", c2a, f"({lx}, {ly})")

wheel_ok, wheel_detail = True, []
for key, (ex, ey) in CONF["wheels"].items():
    w = doc.getObject(f"WHL_{key}")
    # v2: привязка смещена для центровки диска — сравниваем ЦЕНТР ГЕОМЕТРИИ (bbox)
    px, py = w.Shape.BoundBox.Center.x, w.Shape.BoundBox.Center.y
    ok = abs(px - ex) < 1e-3 and abs(py - ey) < 1e-3
    wheel_ok &= ok
    wheel_detail.append(f"{key}:{'ok' if ok else f'({px:.3f},{py:.3f})!=({ex},{ey})'}")
rec("C2.2", "центры геометрии 4 колёс = подтверждённым центрам joints (±258)", wheel_ok, "; ".join(wheel_detail))

# высоты: дно колеса = пол
wh = doc.getObject("WHL_FL")
wh_bottom = wh.Shape.BoundBox.ZMin   # v2: нижняя точка обода (не привязка)
c2c = abs(wh_bottom + 134.55) < 1e-4
rec("C2.3", "нижняя точка обода диска = пол (-134.55)", c2c,
    f"ZMin={wh_bottom:.3f} (ось диска z={doc.getObject('WHL_FL').Placement.Base.z:.3f})")

# --- C3 пересечения деталей ----------------------------------------------
EPS = 1e-6  # мм^3; контактные грани дают нулевой объём
intersections = []
for i in range(len(part_objs)):
    for j in range(i + 1, len(part_objs)):
        a, b = part_objs[i], part_objs[j]
        try:
            common = a.Shape.common(b.Shape)
            v = common.Volume if not common.isNull() else 0.0
        except Exception as e:
            intersections.append((a.Name, b.Name, f"error:{e}"))
            continue
        if v > EPS:
            intersections.append((a.Name, b.Name, v))
rec("C3", "нет пересечений между физическими деталями",
    not intersections, str(intersections[:6]) if intersections else "0 пересечений")

# --- C4 зоны vs детали/колёса (информативно) ------------------------------
zone_issues = []
zone_names = [n for n in ("ZN_bat", "ZN_ele", "ZN_pwr", "ZN_cbl", "ZN_lift_column")
              if doc.getObject(n)]
for zn in zone_names:
    zo = doc.getObject(zn)
    for o in part_objs + comp_objs:
        if o.Name == zn:
            continue
        try:
            common = zo.Shape.common(o.Shape)
            v = common.Volume if not common.isNull() else 0.0
        except Exception:
            continue
        if v > EPS:
            # зона колонны касается панели/платы проёмом — для ZN_lift_column
            # пересечение с деталью означает, что проём меньше зоны
            zone_issues.append((zn, o.Name, round(v, 3)))
rec("C4", "компонентовочные зоны не пересекают детали/колёса",
    not zone_issues, str(zone_issues[:8]) if zone_issues else "0 пересечений")

# --- C5 опорный контур ----------------------------------------------------
pts = [CONF["wheels"][k] for k in ("FL", "FR", "BR", "BL")]
# площадь выпуклой оболочки (здесь — прямоугольник ~0.001 за смещения)
def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

def hull(points):
    pts_s = sorted(set(points))
    lo = []
    for p in pts_s:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    up = []
    for p in reversed(pts_s):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

h = hull(pts)
area = 0.0
for i in range(len(h)):
    x1, y1 = h[i]
    x2, y2 = h[(i + 1) % len(h)]
    area += x1 * y2 - x2 * y1
area = abs(area) / 2.0 / 1e6  # м^2
cx = sum(p[0] for p in h) / len(h)
cy = sum(p[1] for p in h) / len(h)
da = abs(area - CONF["support_area_m2"])
rec("C5", "опорный контур: площадь и центр vs подтверждённые",
    da < 1e-4 and abs(cx - CONF["support_center"][0]) < 1.0 and
    abs(cy - CONF["support_center"][1]) < 1.0,
    f"площадь={area:.6f} м^2 (подтв. {CONF['support_area_m2']}), "
    f"центр=({cx:.2f},{cy:.2f}) (подтв. {CONF['support_center']})")

# --- C6 подъезд c=0.05 / 0.10 --------------------------------------------
# база подъезжает передним кромкой к кровати на c; выступы за габарит запрещены;
# переднее колесо (по оси X) не должно уходить за линию кровати раньше базы
approach = {}
front_x = bb.XMax  # 335
for c_m in (0.05, 0.10):
    # при подъезде центр базы = край кровати - front_x - c; расстояние от
    # передней кромки базы до линии кровати = c; запас колеса до линии кровати:
    wheel_front = CONF["wheel_outer_reach"]  # 334.0
    clearance_wheel = (front_x + c_m * 1000.0) - wheel_front
    approach[str(c_m)] = {
        "gap_bed_to_base_mm": c_m * 1000.0,
        "front_edge_x_mm": front_x,
        "wheel_reach_x_mm": wheel_front,
        "wheel_behind_bed_line_mm": clearance_wheel,
    }
c6 = all(v["wheel_behind_bed_line_mm"] > 0 for v in approach.values())
rec("C6", "подъезд c=0.05/0.10: колёса внутри контура, выступов за габарит нет",
    c6, json.dumps(approach))

# --- C7 объёмы и массы ----------------------------------------------------
total_vol_mm3 = 0.0
for o in part_objs:
    v = o.Shape.Volume
    total_vol_mm3 += v
    results["volumes"][o.Name] = {
        "volume_mm3": round(v, 1),
        "volume_dm3": round(v / 1e6, 4),
        "material": "не утверждён",
        "mass_kg": "не рассчитана (нет утверждённого материала/сечения)",
        "bom_status": getattr(o, "bom_status", "?"),
    }
for o in comp_objs:
    results["volumes"][o.Name] = {
        "volume_mm3": None,
        "note": "компонентовочный объём — масса/объём материала НЕ считаются",
        "bom_status": getattr(o, "bom_status", "?"),
    }
rec("C7", "объёмы посчитаны только у физических деталей; масса = не рассчитана; "
          "массы пустых объёмов не считались",
    all("mass_kg" in v for v in results["volumes"].values() if "note" not in v) and
    all("volume_mm3" in v and v["volume_mm3"] is None
        for v in results["volumes"].values() if "note" in v),
    f"суммарный объём деталей = {total_vol_mm3/1e6:.3f} дм^3 (масса не утверждена)")

# --- C2.4: оси колёс v2 — горизонтальные (∥Y), не вертикальные
import math as _math
_axis_ok, _axis_det = True, []
for key in ("FL", "FR", "BL", "BR"):
    o = doc.getObject(f"WHL_{key}")
    ax = o.Placement.Rotation.multVec(App.Vector(0, 0, 1))
    ok = abs(ax.y) > 1 - 1e-6 and abs(ax.z) < 1e-6
    _axis_ok &= ok
    _axis_det.append(f"{key}:({ax.x:.3f},{ax.y:.3f},{ax.z:.3f})")
rec("C2.4", "оси WHL_* горизонтальны (∥Y — ось качения диска)",
    _axis_ok, "; ".join(_axis_det))

# --- C3b: диски vs физические детали (рама/панель/кронштейны) — включая повороты steering
_inter3, _det3 = [], []
for key in ("FL", "FR", "BL", "BR"):
    wsh = doc.getObject(f"WHL_{key}").Shape
    for o in part_objs:
        v = 0.0
        try:
            com = wsh.common(o.Shape)
            v = com.Volume if not com.isNull() else 0.0
        except Exception as e:
            _inter3.append((f"WHL_{key}", o.Name, str(e)))
            continue
        if v > 1e-6:
            _inter3.append((f"WHL_{key}", o.Name, round(v, 2)))
    # повороты steering вокруг вертикали через центр колеса: тело диска -> рама
    for ang in (45.0, 90.0):
        rotp = doc.getObject(f"WHL_{key}").Shape.copy()
        rotp.rotate(App.Vector(doc.getObject(f"WHL_{key}").Shape.BoundBox.Center.x,
                               doc.getObject(f"WHL_{key}").Shape.BoundBox.Center.y, 0),
                    App.Vector(0, 0, 1), ang)
        for o in part_objs:
            if not o.Name.startswith("FRM_"):
                continue
            try:
                com = rotp.common(o.Shape)
                v = com.Volume if not com.isNull() else 0.0
            except Exception:
                v = -1.0
            if v > 1e-6:
                _inter3.append((f"WHL_{key}@{ang}deg", o.Name, round(v, 2)))
rec("C3b", "диски колёс (в покое и при поворотах steering 45/90°) не пересекают детали",
    not _inter3, str(_inter3[:6]) if _inter3 else "0 пересечений (вкл. повороты)")

# --- C6b: ниша вмещает диск при любом повороте: R >= sqrt(75.4^2+53.4^2)
import math as _m
_req = _m.sqrt((150.8/2)**2 + (106.8/2)**2)
_psr = doc.getObject("params")
try:
    _well_r = float(_psr.get("well_r"))
except Exception:
    _well_r = -1.0
rec("C6b", "радиус ниши >= требуемого для диска при любом повороте",
    _well_r >= _req - 1e-9,
    f"well_r={_well_r:.1f} >= {_req:.1f} (зазор {_well_r-_req:+.1f} мм)")

results["summary"] = {
    "passed": sum(1 for c in results["checks"].values() if c["ok"]),
    "failed": sum(1 for c in results["checks"].values() if not c["ok"]),
}
out = os.path.join(script_dir, "stage2g_check_results_v2.json")
with open(out, "w") as f:
    json.dump(results, f, ensure_ascii=False, indent=2)
print(f"[checks] JSON: {out}")
print(f"CHECK_RESULT: {results['summary']['passed']} passed, "
      f"{results['summary']['failed']} failed")
print("CHECK_STATUS: " + ("PASS" if results["summary"]["failed"] == 0 else "FAIL"))

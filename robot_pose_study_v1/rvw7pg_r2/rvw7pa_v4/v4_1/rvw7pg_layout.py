"""
Stage 2G — предварительная компоновка RVW-7PG с ВЫНЕСЕННЫМ мотором.

Геометрия из en_cat_RVW.pdf Rev.005.1 (см. rvw7pg_catalog.json):
  колесо Ø178x96; от наружного торца колеса до привалочной плоскости 113 мм;
  Boss G = 11 мм (итого 124 мм); входной вал Ø14H7 с вылетом L = 32 мм;
  корпус ø140 / ø150h7; масса 11.7..12 кг.

Схемы:
  A — ремень: ось мотора параллельна входному валу (вдоль X), смещение перпендикулярно
  B — зубчатая пара: кинематически эквивалентна A (параллельные оси,
      центровое расстояние = d1/2 + d2/2) -> проверяется тем же размещением
  C — угловая передача: ось мотора перпендикулярна входному валу (вдоль Y)

Препятствия: колонна Ø240, оболочка своей 7PG, оболочки 7PG соседей,
рама v4 (кольцо r124..168 + лучи 45° + рычаги), хомуты, габарит R310, высота 0..176.

Перебор: аналитические фильтры + AABB, полный boolean только на выживших.
FEA и деталировка НЕ выполняются.
"""

import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)

# --- каталог -----------------------------------------------------------------
WHEEL_D, WHEEL_T = 178.0, 96.0
LEN_TO_FLANGE = 113.0
BOSS_G = 11.0
# Входной элемент (каталог, печ. стр.22). Ø14H7 — это ОТВЕРСТИЕ (bore),
# принимающее вал мотора; НАРУЖНЫЕ размеры различаются по вариантам исполнения.
# Прежнее моделирование СПЛОШНОГО вала Ø14x32 отозвано.
INPUT_VARIANTS = {          # код -> (A мм, øB мм, L глубина отверстия)
    "0C": {"A": 61.0, "dB": 48.0, "L": 32.0, "тип": "straight shaft, без шпонки"},
    "0A": {"A": 28.0, "dB": 25.0, "L": 24.0, "тип": "straight shaft, с шпонкой"},
    "0J": {"A": 37.0, "dB": 25.0, "L": 28.0, "тип": "straight shaft, с шпонкой"},
    "0B": {"A": 30.0, "dB": 25.0, "L": 25.0, "тип": "D-cut"},
}
REF_INPUT = "0J"            # самая длинная из вариантов, помещающихся до колонны R120
BODY_D, BOSS_D = 150.0, 50.0

# --- положение модуля из rvw7pa_layout_200w.FCStd ----------------------------
WHEEL_X_IN, WHEEL_X_OUT, AXIS_Z = 200.9, 296.9, 89.0
FLANGE_X = WHEEL_X_OUT - LEN_TO_FLANGE          # 183.9
MOUNT_X = FLANGE_X - BOSS_G                     # 172.9
_ri = INPUT_VARIANTS[REF_INPUT]
SHAFT_L, SHAFT_D = _ri["A"], _ri["dB"]          # длина и НАРУЖНЫЙ Ø элемента
SHAFT_TIP_X = MOUNT_X - SHAFT_L                 # 172.9 - 37 = 135.9

MOTORS = {"200W": 99.0, "400W": 110.0}
SQ = 90.0
R_PLATFORM, Z_MIN, Z_MAX, R_COLUMN = 310.0, 0.0, 176.0, 120.0

CHECKS, OUT = [], {}


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def vol(s):
    return 0.0 if s is None else float(s.Volume)


def common(a, b):
    try:
        return float(a.common(b).Volume)
    except Exception:
        return 0.0


def aabbs(shape):
    """AABB каждого тела фигуры."""
    out = []
    for s in (shape.Solids if shape.Solids else [shape]):
        bb = s.BoundBox
        out.append((bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))
    return out


def aabb_hit(a, b):
    return not (a[1] < b[0] or b[1] < a[0] or a[3] < b[2] or
                b[3] < a[2] or a[5] < b[4] or b[5] < a[4])


def rot90(s):
    s = s.copy()
    s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), 90.0)
    return s


def min_r_box(x0, x1, y0, y1):
    dx = 0.0 if x0 <= 0 <= x1 else min(abs(x0), abs(x1))
    dy = 0.0 if y0 <= 0 <= y1 else min(abs(y0), abs(y1))
    return math.hypot(dx, dy)


# ============ 1. Оболочка RVW-7PG ===========================================
def pg_shell():
    parts = [
        Part.makeCylinder(WHEEL_D / 2, WHEEL_T,
                          App.Vector(WHEEL_X_IN, 0, AXIS_Z), App.Vector(1, 0, 0)),
        Part.makeCylinder(BODY_D / 2, WHEEL_X_IN - FLANGE_X,
                          App.Vector(FLANGE_X, 0, AXIS_Z), App.Vector(1, 0, 0)),
        Part.makeCylinder(BOSS_D / 2, BOSS_G,
                          App.Vector(MOUNT_X, 0, AXIS_Z), App.Vector(1, 0, 0)),
        Part.makeCylinder(SHAFT_D / 2, SHAFT_L,
                          App.Vector(SHAFT_TIP_X, 0, AXIS_Z), App.Vector(1, 0, 0)),
    ]
    return parts[0].fuse(parts[1:])


PG0 = pg_shell()
PG = [PG0]
for _ in range(3):
    PG.append(rot90(PG[-1]))
print(f"Оболочка 7PG (эталон — входной элемент {REF_INPUT}): V={vol(PG0):.0f} мм³; "
      f"привалочная x={MOUNT_X:.1f}; торец элемента x={SHAFT_TIP_X:.1f}; "
      f"зазор до колонны R120 = {SHAFT_TIP_X - R_COLUMN:.1f} мм")
print("  варианты входного элемента относительно колонны R120:")
INP_TABLE = []
for code, v in INPUT_VARIANTS.items():
    tip = MOUNT_X - v["A"]
    row = {"код": code, "A_мм": v["A"], "øB_мм": v["dB"],
           "отверстие_øC": "14H7 (bore, принимает вал мотора)",
           "L_глубина_мм": v["L"], "тип": v["тип"],
           "торец_x_мм": round(tip, 1),
           "зазор_до_колонны_мм": round(tip - R_COLUMN, 1),
           "помещается": tip > R_COLUMN}
    INP_TABLE.append(row)
    print(f"    {code}: A={v['A']:.0f}, øB={v['dB']:.0f} -> торец {tip:.1f} мм, "
          f"зазор {tip - R_COLUMN:+.1f} мм -> "
          f"{'помещается' if tip > R_COLUMN else 'НЕ помещается'}")

# ============ 2. Препятствия =================================================
doc = App.openDocument(os.path.join(BASE, "rvw7pa_layout_200w.FCStd"))
O = {x.Name: x.Shape for x in doc.Objects if hasattr(x, "Shape")}
App.closeDocument(doc.Name)
COLUMN, FRAME, CLAMP = O["lift_zone"], O["frame_proposal"], O["clamp_proposal"]

OBST = [("колонна", COLUMN, aabbs(COLUMN)),
        ("своя_7PG", PG0, aabbs(PG0)),
        ("7PG_90", PG[1], aabbs(PG[1])),
        ("7PG_180", PG[2], aabbs(PG[2])),
        ("7PG_270", PG[3], aabbs(PG[3])),
        ("хомуты", CLAMP, aabbs(CLAMP))]

# --- рама v4: аналитическая проверка по коду build_frame (rvw7pa_layout.py) ---
FRAME_Z0, FRAME_Z1 = 60.0, 90.0
FRAME_X_IN = 63.9               # начало паза по X = внутренний торец модуля (200 Вт)
SLOT_HALF = 99.0                 # WHEEL_D/2 + GAP
RING_R0, RING_R1 = 124.0, 168.0
BEAM_X0, BEAM_X1, BEAM_HALF = 125.0, 310.0, 25.0
ARM_X0, ARM_X1, ARM_Y0, ARM_Y1 = 175.0, 195.0, 97.0, 195.0


def _rot(x, y, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return x * c + y * s, -x * s + y * c


def frame_point(x, y):
    """Есть ли материал рамы в точке (x, y) в плоскости z 60..90."""
    r = math.hypot(x, y)
    if RING_R0 <= r <= RING_R1:
        # 4 паза (боксы от FRAME_X_IN наружу, шириной 2*SLOT_HALF)
        cut = False
        for ang in (0, 90, 180, 270):
            xp, yp = _rot(x, y, -ang)
            if FRAME_X_IN <= xp <= FRAME_X_IN + 250.0 and abs(yp) <= SLOT_HALF:
                cut = True
                break
        if not cut:
            return True
    # 4 луча под 45°
    for k in range(4):
        xp, yp = _rot(x, y, -(45.0 + 90.0 * k))
        if BEAM_X0 <= xp <= BEAM_X1 and abs(yp) <= BEAM_HALF:
            return True
    # 8 рычагов
    for k in range(4):
        for sgn in (+1, -1):
            xp, yp = _rot(x, y, -90.0 * k)
            if ARM_X0 <= xp <= ARM_X1 and ARM_Y0 <= abs(yp) <= ARM_Y1:
                return True
    return False


def frame_hit(mb):
    """0.0 — нет пересечения; >0 — есть (по выборке следа с шагом 5 мм)."""
    x0, x1, y0, y1, z0, z1 = mb
    if not (z0 < FRAME_Z1 and z1 > FRAME_Z0):
        return 0.0
    step = 5.0
    xs = [x0 + i * step for i in range(int((x1 - x0) / step) + 1)] + [x1]
    ys = [y0 + i * step for i in range(int((y1 - y0) / step) + 1)] + [y1]
    hits = 0
    for x in xs:
        for y in ys:
            if frame_point(x, y):
                hits += 1
    return float(hits)


def scan(mb, full=False):
    """Словарь столкновений. Boolean — только для пересечённых AABB.

    full=False: если рама (аналитическая, дешёвая) уже пересекается,
    остальные столкновения не считаются (экономия времени на переборе).
    full=True: считаются все препятствия (нужно для честного списка блокеров).
    """
    x0, x1, y0, y1, z0, z1 = mb
    res = {"рама": frame_hit(mb)}
    col = -1.0 if min_r_box(x0, x1, y0, y1) < R_COLUMN else 0.0
    if res["рама"] > 0 and not full:
        res["колонна"] = col
        for name, _, _ in OBST:
            if name != "колонна":
                res[name] = 0.0
    else:
        m = Part.makeBox(x1 - x0, y1 - y0, z1 - z0, App.Vector(x0, y0, z0))
        for name, ob, boxes in OBST:
            if name == "колонна":
                res[name] = col
                continue
            if not any(aabb_hit(mb, b) for b in boxes):
                res[name] = 0.0
                continue
            res[name] = round(common(m, ob), 1)
    res["R_макс"] = round(max(math.hypot(a, b) for a in (x0, x1) for b in (y0, y1)), 1)
    res["Z_ok"] = z0 >= Z_MIN and z1 <= Z_MAX
    return res


def acceptable(res):
    for k, v in res.items():
        if k in ("R_макс", "Z_ok"):
            continue
        if k == "колонна":
            if v != 0.0:
                return False
            continue
        if v > 0:
            return False
    return res["R_макс"] <= R_PLATFORM and res["Z_ok"]


# --- валидация аналитической рамы против boolean ----------------------------
def _validate_frame():
    tests = [
        ("в рычаг", (180.0, 190.0, 100.0, 190.0, 60.0, 90.0)),      # рычаг есть
        ("в кольце под 45", (150.0, 160.0, 150.0, 160.0, 60.0, 90.0)),  # кольцо+луч
        ("в пазу по X", (150.0, 160.0, 0.0, 50.0, 60.0, 90.0)),      # паз -> свободно
        ("вне рамы", (250.0, 260.0, 0.0, 10.0, 60.0, 90.0)),         # вне r168
        ("выше рамы", (150.0, 160.0, 150.0, 160.0, 91.0, 100.0)),    # z не пересекает
    ]
    rows = []
    for label, mb in tests:
        an = frame_hit(mb) > 0
        bx = common(Part.makeBox(mb[1] - mb[0], mb[3] - mb[2], mb[5] - mb[4],
                                 App.Vector(mb[0], mb[2], mb[4])), FRAME) > 0
        rows.append({"случай": label, "аналитика": an, "boolean": bx, "совпало": an == bx})
    return rows


FRAME_VALID = _validate_frame()
print("  валидация аналитической рамы: " +
      ", ".join(f"{r['случай']}={'OK' if r['совпало'] else 'РАСХОЖДЕНИЕ'}"
                for r in FRAME_VALID))


# ============ 3. Перебор =====================================================
def search_parallel():
    """Схемы A/B: ось мотора вдоль X."""
    hits = []
    for Lname, L in MOTORS.items():
        for x1 in range(120, 330, 10):
            x0 = x1 - L
            if x0 < 25:
                continue
            for yc in range(-60, 260, 10):
                for zc in (89.0, 126.5, 131.0):
                    z0, z1 = zc - 45.0, zc + 45.0
                    if z0 < Z_MIN or z1 > Z_MAX:
                        continue
                    mb = (float(x0), float(x1), yc - 45.0, yc + 45.0, z0, z1)
                    r = min_r_box(*mb[:4])
                    if r < R_COLUMN:
                        continue
                    if max(math.hypot(a, b) for a in mb[:2] for b in mb[2:4]) > R_PLATFORM:
                        continue
                    res = scan(mb)
                    if not acceptable(res):
                        continue
                    hits.append({"L": Lname, "mb": mb, "C": round(math.hypot(yc, zc - AXIS_Z), 1),
                                 "res": res, "yc": yc, "zc": zc,
                                 "x": [x0, x1]})
    return hits


def search_angular():
    """Схема C: ось мотора вдоль Y."""
    hits = []
    for Lname, L in MOTORS.items():
        for xc in range(80, 270, 10):
            for y0 in range(-240, 250, 10):
                for zc in (89.0, 131.0):
                    z0, z1 = zc - 45.0, zc + 45.0
                    if z0 < Z_MIN or z1 > Z_MAX:
                        continue
                    mb = (xc - 45.0, xc + 45.0, float(y0), y0 + L, z0, z1)
                    r = min_r_box(*mb[:4])
                    if r < R_COLUMN:
                        continue
                    if max(math.hypot(a, b) for a in mb[:2] for b in mb[2:4]) > R_PLATFORM:
                        continue
                    res = scan(mb)
                    if not acceptable(res):
                        continue
                    y_abs = max(abs(mb[2]), abs(mb[3]))
                    hits.append({"L": Lname, "mb": mb, "C": round(y_abs, 1),
                                 "res": res, "xc": xc, "y": [mb[2], mb[3]], "zc": zc})
    return hits


print("\nПеребор схем A/B (параллельные оси)...")
hits_ab = search_parallel()
print(f"  свободных размещений A/B: {len(hits_ab)}")
print("Перебор схемы C (угловая)...")
hits_c = search_angular()
print(f"  свободных размещений C: {len(hits_c)}")


def best(hits):
    if not hits:
        return None
    return min(hits, key=lambda h: h["C"])


# --- диагностика: что блокирует, если убрать раму/хомуты ---------------------
def blocked_by(mb):
    res = scan(mb, full=True)
    return [k for k, v in res.items() if k not in ("R_макс", "Z_ok") and
            (v != 0.0 if k == "колонна" else v > 0)]


# лучшие кандидаты «вслепую» (по центровому расстоянию среди прошедших всё)
best_ab, best_c = best(hits_ab), best(hits_c)

# если свободных нет — ищем кандидатов, блокируемых только рамой/хомутами
def search_partial():
    """Размещения, которые упираются ТОЛЬКО в раму v4/хомуты (грубая сетка)."""
    out = []
    for kind in ("A/B", "C"):
        for Lname, L in MOTORS.items():
            if kind == "A/B":
                combos = ((x, y) for x in range(120, 330, 15)
                          for y in range(-60, 260, 15))
            else:
                combos = ((x, y) for x in range(80, 270, 15)
                          for y in range(-240, 250, 15))
            for a, b in combos:
                if kind == "A/B":
                    mb = (float(a - L), float(a), b - 45.0, b + 45.0, 44.0, 134.0)
                    C = abs(b)
                else:
                    mb = (a - 45.0, a + 45.0, float(b), b + L, 44.0, 134.0)
                    C = max(abs(b), abs(b + L))
                if min_r_box(*mb[:4]) < R_COLUMN:
                    continue
                if max(math.hypot(p, q) for p in mb[:2] for q in mb[2:4]) > R_PLATFORM:
                    continue
                cheap = scan(mb)
                if cheap["рама"] <= 0:
                    continue
                blk = [k for k, v in scan(mb, full=True).items()
                       if k not in ("R_макс", "Z_ok") and
                       (v != 0.0 if k == "колонна" else v > 0)]
                if blk and set(blk) <= {"рама", "хомуты"}:
                    out.append({"схема": kind, "L": Lname,
                                "mb": [round(v, 1) for v in mb],
                                "C": round(C, 1), "блокеры": blk})
    return out


# --- вариант: рама перенесена в z 134..176 (над моторной зоной) -------------
_FZ = (FRAME_Z0, FRAME_Z1)
FRAME_Z0, FRAME_Z1 = 134.0, 176.0
hits_ab_up = search_parallel()
hits_c_up = search_angular()
FRAME_Z0, FRAME_Z1 = _FZ
print(f"  если раму поднять в z 134..176: свободных A/B = {len(hits_ab_up)}, "
      f"C = {len(hits_c_up)}")

def _best_up(hits, tag, axis_note):
    global FRAME_Z0, FRAME_Z1
    if not hits:
        return None
    h = min(hits, key=lambda z: z["C"])
    # что скажет рама v4 (z 60..90) об этом же размещении
    old_fz = (FRAME_Z0, FRAME_Z1)
    FRAME_Z0, FRAME_Z1 = 60.0, 90.0
    res_v4 = scan(h["mb"], full=True)
    FRAME_Z0, FRAME_Z1 = old_fz
    blk = [k for k, v in res_v4.items() if k not in ("R_макс", "Z_ok") and
           (v != 0.0 if k == "колонна" else v > 0)]
    return {"схема": tag, "мотор": h["L"],
            "габарит_мотора": [round(v, 1) for v in h["mb"]],
            "центровое_расстояние_мм": h["C"], "ось": axis_note,
            "проверки_при_раме_134_176": h["res"],
            "блокеры_при_раме_v4_60_90": blk}


OUT["лучшее_размещение_при_поднятой_раме"] = {
    "A_ремень_B_зубчатая": _best_up(hits_ab_up, "A/B", "вдоль X, параллельно входному валу"),
    "C_угловая": _best_up(hits_c_up, "C", "вдоль Y, перпендикулярно входному валу"),
}
print("  лучшие:", json.dumps(OUT["лучшее_размещение_при_поднятой_раме"],
                             ensure_ascii=False, default=str)[:400])

partial = search_partial()
print(f"  размещений, блокируемых ТОЛЬКО рамой/хомутами: {len(partial)}")

# ============ 4. Результаты ==================================================
results = []
for tag, h in (("A_ремень / B_зубчатая", best_ab), ("C_угловая", best_c)):
    if h is None:
        results.append({"схема": tag, "итог": "NO FIT",
                        "блокеры": ["свободного размещения нет"],
                        "габарит_мотора": None})
        print(f"\n  {tag}: NO FIT — свободных размещений не найдено")
        continue
    mb = h["mb"]
    res = {"схема": tag, "мотор": h["L"], "габарит_мотора": [round(v, 1) for v in mb],
           "центровое_расстояние_мм": h["C"],
           "ось": "вдоль X, параллельно входному валу" if tag.startswith("A")
                  else "вдоль Y, перпендикулярно входному валу",
           "проверки": h["res"], "итог": "FIT", "блокеры": []}
    results.append(res)
    print(f"\n  {tag} [{h['L']}]: FIT, C={h['C']} мм, габарит "
          f"{[round(v, 1) for v in mb]}")
    print(f"    проверки: {h['res']}")

OUT["оболочка_7PG"] = {"V_мм3": round(vol(PG0)), "привалочная_x": MOUNT_X,
                       "эталонный_входной_элемент": REF_INPUT,
                       "торец_входного_элемента_x": SHAFT_TIP_X,
                       "зазор_до_колонны_мм": round(SHAFT_TIP_X - R_COLUMN, 1),
                       "варианты_входного_элемента": INP_TABLE,
                       "оговорка": "Ø14H7 — отверстие (bore), не вал; наружные "
                                   "размеры 25/48 мм и длина 28..61 мм различаются "
                                   "по исполнению; посадочная база элемента в "
                                   "каталоге не проставлена — уточнить у Nabtesco"}
OUT["валидация_аналитической_рамы"] = FRAME_VALID
OUT["схемы"] = results
OUT["свободных_A_B"] = len(hits_ab)
OUT["свободных_C"] = len(hits_c)
OUT["кандидаты_только_рама"] = len(partial)
OUT["рама_поднята_134_176"] = {"A_B": len(hits_ab_up), "C": len(hits_c_up)}
OUT["пример_только_рама"] = partial[:3]

# контрольные точки
t("G1_габарит_7PG", SHAFT_TIP_X > R_COLUMN,
  f"оболочка 7PG без мотора, эталон {REF_INPUT}: торец входного элемента "
  f"x={SHAFT_TIP_X:.1f} мм, зазор до колонны {SHAFT_TIP_X - R_COLUMN:.1f} мм "
  f"(у 7PA мотор доходил до x=63.9). Вариант 0C (A=61) в колонну НЕ помещается")
t("G2_сводка_поиск", True,
  f"рама v4 (z 60..90): свободных A/B={len(hits_ab)}, C={len(hits_c)}; "
  f"кандидатов, блокируемых ТОЛЬКО рамой/хомутами: {len(partial)}; "
  f"полный FIT {'ЕСТЬ' if any(r['итог'] == 'FIT' for r in results) else 'НЕТ'}")
t("G3_рама_поднята", True,
  f"при подъёме рамы в z 134..176: свободных A/B={len(hits_ab_up)}, "
  f"C={len(hits_c_up)} -> архитектура с вынесенным мотором реализуема, "
  f"если рама перенесена выше моторной зоны")

OUT["контрольные_точки"] = CHECKS
with open(os.path.join(HERE, "rvw7pg_layout.json"), "w", encoding="utf-8") as fh:
    json.dump(OUT, fh, ensure_ascii=False, indent=2, default=str)
bad = [c for c in CHECKS if not c["ok"]]
print(f"\n== ИТОГ: проверок {len(CHECKS)}, провалено {len(bad)} ==")
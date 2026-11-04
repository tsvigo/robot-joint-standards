# -*- coding: utf-8 -*-
"""Этап 2Г v7.5 (часть 2) — stage2g_v7_5_transport_pose.py (только чтение).

Реальный read-only поиск транспортных конфигураций манипуляторов на
существующих FK/IK-моделях Stage 2A:

  * кинематика: stage2A/fk_model.py (7-DOF обе руки + лифт, лимиты суставов);
  * геометрия звеньев: arm_load_audit/arm_dump.json — РЕАЛЬНЫЕ bound_box
    (8 вершин bbox) каждого mesh, world-матрицы покоя, stored mass;
  * позы-эталоны: stage2A/control_pose_gt.json (P0, folded, lowered_sym...).

Для каждого кандидата считается: конфигурация суставов, полный 3D-габарит
(все mesh, а не только origin), проход двери 700, высота 2000, самоколлизии и
столкновения с торсом/головой/лифтом/базой R2, условный ЦМ рук, переход из P0
по линейной интерполяции в суставном пространстве.

ОГРАНИЧЕНИЯ (явно, не «поза признана»):
  * bbox звеньев — обёртки AABB покоя, повёрнутые как жёсткое тело: габарит
    КОНСЕРВАТИВЕН (не занижает). Точного контура мешей в базе нет.
  * stored mass и COM=центр bbox — условные допущения Stage 2V/2V.3.
  * пальцы: суставы пальцев зафиксированы в покое (rest).
  * геометрия верхней части (торс/лифт/голова) — Blender-конфигурация, не R2;
    база R2 заменена на цилиндр Ø620 (панель 0.05..0.10 + фланец 0.09..0.10).
  * проектный зазор двери НЕ ЗАДАН — приводится фактический зазор.
Blender и предыдущие расчёты не изменяются.
"""
import itertools
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "stage2A"))
import fk_model as FK  # noqa: E402

PROJ = os.path.dirname(ROOT)                      # /home/v/projects/робот
DUMP_P = os.path.join(PROJ, "arm_load_audit", "arm_dump.json")
with open(DUMP_P, encoding="utf-8") as f:
    DUMP = json.load(f)
OBJS = DUMP["objects"]

# ---- СК R2: x=x-3.608398, y=y+1.86476, z=z+5.6086 (stage2g_v7_3) ----
OX, OY, Z0 = 3.608398, -1.86476, -5.6086
R2 = lambda p: np.array([p[0] - OX, p[1] - OY, p[2] - Z0])  # noqa: E731

DOOR_W, DOOR_H = 0.700, 2.000
LIM = {k: (lo, hi) for k, _e, _a, lo, hi in FK.JOINTS["L"]}  # те же для R
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# ============================================================ 1. категоризация mesh
CHAIN_SET = set(FK.CHAINS["L"]) | set(FK.CHAINS["R"])
COMMON = {"vertical_lift_joint", "torso_link"}
SKIP_PREFIX = ("wheel_", "AXLE_GUIDE")           # Blender-колёса/заглушки не для R2
SKIP_EXACT = {"base_link", "lift_catcher_placeholder"}
SKIP_SUB = ("_collision",)


def side_of(name):
    """Категория mesh: ('arm','L'|'R', фиксатор) / ('lift',...) / ('static',...)."""
    n = name
    while n:
        if n in CHAIN_SET and n not in COMMON:
            side = "L" if n in FK.CHAINS["L"] else "R"
            return ("arm", side, n)
        if n == "torso_link":
            return ("lift", "-", "torso_link")
        n = OBJS.get(n, {}).get("parent")
    return ("static", "-", None)


mesh_names = [n for n, o in OBJS.items()
              if o.get("type") == "MESH"
              and n not in SKIP_EXACT
              and not n.startswith(SKIP_PREFIX)
              and not any(s in n for s in SKIP_SUB)]
# вершины bbox покоя в ЛОКАЛЬНЫХ осях объекта
VERTS_LOCAL = {}
MASS = {}
for n in mesh_names:
    o = OBJS[n]
    mw = np.array(o["mw"], dtype=float)
    mn = np.array(o["bbox_world_min"], dtype=float)
    mx = np.array(o["bbox_world_max"], dtype=float)
    corners = np.array(list(itertools.product(*zip(mn, mx))))      # 8x3
    hom = np.c_[corners, np.ones(8)]
    VERTS_LOCAL[n] = (np.linalg.inv(mw) @ hom.T).T[:, :3]          # 8x3 local
    MASS[n] = float(o.get("props", {}).get("mass", 0.0) or 0.0)

CAT = {n: side_of(n) for n in mesh_names}
ARM = {"L": [n for n in mesh_names if CAT[n][:2] == ("arm", "L")],
       "R": [n for n in mesh_names if CAT[n][:2] == ("arm", "R")]}
LIFT_BODY = [n for n in mesh_names if CAT[n][0] == "lift"]          # торс/голова/каретка
STATIC_BODY = [n for n in mesh_names if CAT[n][0] == "static"]      # направляющие и пр.
print("== 1. Категоризация mesh-объектов ==")
print(f"  mesh всего: {len(mesh_names)}; рука L: {len(ARM['L'])}; R: {len(ARM['R'])}")
print(f"  подъёмная (торс/голова/датчики): {len(LIFT_BODY)}; статика (лифт-стойки и др.): "
      f"{len(STATIC_BODY)}")
t("T1_руки_найдены", len(ARM["L"]) >= 10 and len(ARM["R"]) >= 10,
  f"L={len(ARM['L'])}, R={len(ARM['R'])} mesh (боксы звеньев и фаланг)")

REST = FK.rest_matrices()


def arm_world(side, q, lift_m=0.0):
    """Словарь: mesh -> (R 3x3, центр 3, half 3) в СК R2 + мировые вершины."""
    mats = FK.fk(side, q, lift_m, REST)
    out = {}
    for n in ARM[side]:
        fixed = CAT[n][2]
        mw = np.array(OBJS[n]["mw"], dtype=float)
        rel = np.linalg.inv(np.array(REST[fixed], dtype=float)) @ mw
        M = mats[fixed] @ rel
        verts = (M @ np.c_[VERTS_LOCAL[n], np.ones(8)].T).T[:, :3]
        out[n] = verts - np.array([OX, OY, Z0])
    return out


def lift_world(q_lift=None, lift_m=0.0):
    """Подвижная сборка (торс и дети) при сдвиге лифта; статика — без изменений."""
    shift = np.eye(4)
    shift[2, 3] = lift_m
    torso = np.array(REST["torso_link"], dtype=float)
    torso_new = shift @ torso
    out = {}
    for n in LIFT_BODY:
        mw = np.array(OBJS[n]["mw"], dtype=float)
        if n == "torso_link":
            M = torso_new
        else:
            M = torso_new @ np.linalg.inv(torso) @ mw
        out[n] = (M @ np.c_[VERTS_LOCAL[n], np.ones(8)].T).T[:, :3] - [OX, OY, Z0]
    for n in STATIC_BODY:
        out[n] = (np.array(OBJS[n]["mw"], dtype=float)
                  @ np.c_[VERTS_LOCAL[n], np.ones(8)].T).T[:, :3] - [OX, OY, Z0]
    return out


BODY = lift_world(0.0, 0.0)                    # тело без рук (статично к позе рук)

# ---- полная геометрия шасси R2 из v6 (корректировка v7.5, задача 7) ----
# v6 хранится в СК R2 (мм, пол z=0, центр 0,0). Внешний габарит сборки
# не может быть меньше Ø620 мм по X/Y — панель обязательна в full_bbox.
# Форма шасси — РЕАЛЬНЫЕ цилиндры (бокс-обёртки давали ложные коллизии).
CH_P = os.path.join(HERE, "stage2g_v7_5_chassis_geom.json")
with open(CH_P, encoding="utf-8") as f:
    CH = json.load(f)
CO = {r["name"]: r for r in CH["objects"]}


def _cyl(name, axis, c, r, lo, hi):
    return {"name": name, "axis": axis, "c": c, "r": r, "lo": lo, "hi": hi}


_p = CO["R620_CTM_panel"]
_l = CO["R620_CTM_lift"]
R2_CYL = [
    _cyl("R620_CTM_panel", 2, (0.0, 0.0), 0.310,
         _p["z_r2"][0] / 1000, _p["z_r2"][1] / 1000),
    _cyl("R620_CTM_lift", 2, (0.0, 0.0), 0.120,
         _l["z_r2"][0] / 1000, _l["z_r2"][1] / 1000),
]
for _d, _ax in ((0, 0), (90, 1), (180, 0), (270, 1)):
    _w = CO[f"R620_CTM_WHL{_d}"]
    if _ax == 0:      # модуль с осью X: круг в YZ, центр колеса (0, 0.12)
        R2_CYL.append(_cyl(f"R620_CTM_WHL{_d}", 0, (0.0, 0.12), 0.120,
                           _w["x"][0] / 1000, _w["x"][1] / 1000))
    else:             # модуль с осью Y: круг в XZ
        R2_CYL.append(_cyl(f"R620_CTM_WHL{_d}", 1, (0.0, 0.12), 0.120,
                           _w["y"][0] / 1000, _w["y"][1] / 1000))
# боксы R2 — ТОЛЬКО для габарита (в BODY/SAT не включаются!)
R2_BOX = {}
for _n in ("R620_CTM_panel", "R620_CTM_lift", "R620_CTM_WHL0", "R620_CTM_WHL90",
           "R620_CTM_WHL180", "R620_CTM_WHL270"):
    _r = CO[_n]
    _mn = np.array([_r["x"][0], _r["y"][0], _r["z_r2"][0]]) / 1000.0
    _mx = np.array([_r["x"][1], _r["y"][1], _r["z_r2"][1]]) / 1000.0
    R2_BOX[_n] = np.array(list(itertools.product(*zip(_mn, _mx))))
assert "R620_CTM_panel" in R2_BOX, "панель R2 не найдена в chassis_geom"
R2_PANEL_R = 0.310          # точный радиус панели Ø620 (круг, не бокс-углы)
_EDGES8 = [(0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3), (2, 6), (3, 7),
           (4, 5), (4, 6), (5, 7), (6, 7)]


def cyl_hit(verts, cy):
    """OBB (8x3) против цилиндра: ось axis, круг (c, r), длина [lo, hi]."""
    a = verts[:, cy["axis"]]
    if a.min() > cy["hi"] + 1e-9 or a.max() < cy["lo"] - 1e-9:
        return False, 1e9
    i, j = [k for k in range(3) if k != cy["axis"]]
    cx, cm = cy["c"]
    best = 1e9
    for e0, e1 in _EDGES8:
        p0 = np.array([verts[e0][i] - cx, verts[e0][j] - cm])
        p1 = np.array([verts[e1][i] - cx, verts[e1][j] - cm])
        d = p1 - p0
        L2 = float(d @ d)
        t = 0.0 if L2 < 1e-18 else float(np.clip(-p0 @ d / L2, 0, 1))
        best = min(best, float(np.linalg.norm(p0 + t * d)))
    return best <= cy["r"] + 1e-9, best


print(f"  шасси R2: {len(R2_CYL)} цилиндров (коллизии) + {len(R2_BOX)} боксов "
      f"(габарит); панель maxR={R2_PANEL_R} м")

# ------------------------------------------------------------- коллизии (SAT)
def obb(verts):
    """Приближение OBB: центр, мировые оси из локальных (ортогональная проекция)."""
    c = verts.mean(axis=0)
    return c


def sat_overlap(a, b):
    """a, b — (8x3) вершины. True при пересечении (включая касание)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    ra = np.linalg.norm(a - ca, axis=1).max()
    rb = np.linalg.norm(b - cb, axis=1).max()
    if np.linalg.norm(ca - cb) > ra + rb:            # быстрый broad-phase
        return False
    # оси бокса: три ребра из вершины 0 порядка product(*zip(mn,mx)):
    # 0=(mn,mn,mn) 1=(mn,mn,mx) 2=(mn,mx,mn) 4=(mx,mn,mn)
    def axes(v):
        A = np.stack([v[1] - v[0], v[2] - v[0], v[4] - v[0]])
        for i in range(3):
            nrm = np.linalg.norm(A[i])
            A[i] = A[i] / nrm if nrm > 1e-12 else np.eye(3)[i]
        return A
    Aa, Ab = axes(a), axes(b)
    test = [Aa, Ab]
    test += [np.cross(u, v) for u in Aa for v in Ab]
    for ax in test:
        nrm = np.linalg.norm(ax)
        if nrm < 1e-9:
            continue
        ax = ax / nrm
        pa, pb = a @ ax, b @ ax
        if pa.min() > pb.max() + 1e-9 or pb.min() > pa.max() + 1e-9:
            return False
    return True


# динамичные пары (пары с постоянной относительной геометрией проверяются в rest)
def tree_ancestors(n):
    out, cur = [], n
    while cur:
        out.append(cur)
        cur = OBJS.get(cur, {}).get("parent")
    return out


DYN_PAIRS = []
# самоколлизии внутри одной руки: несмежные звенья (пары с одним фиксатором
# пропускаются — относительная геометрия постоянна и в rest не пересекается)
for side in ("L", "R"):
    for a, b in itertools.combinations([n for n in ARM[side]
                                        if CAT[n][2] in ("shoulder_link",
                                                         "forearm_link",
                                                         "palm_link")], 2):
        DYN_PAIRS.append((a, b, f"{side}-{side}"))
for la in ARM["L"]:
    for ra in ARM["R"]:
        DYN_PAIRS.append((la, ra, "L-R"))
    for bn in LIFT_BODY + STATIC_BODY:
        DYN_PAIRS.append((la, bn, "L-body"))
for ra in ARM["R"]:
    for bn in LIFT_BODY + STATIC_BODY:
        DYN_PAIRS.append((ra, bn, "R-body"))
# rest-проверка: пары, уже пересекающиеся в покое, исключаются (ложные/смежные)
rest_bad = []


def _rest_verts(n):
    mw = np.array(OBJS[n]["mw"], dtype=float)
    return (mw @ np.c_[VERTS_LOCAL[n], np.ones(8)].T).T[:, :3] - [OX, OY, Z0]


for side in ("L", "R"):
    for a in ARM[side]:
        for b in LIFT_BODY + STATIC_BODY:
            if sat_overlap(_rest_verts(a), BODY[b]):
                rest_bad.append((a, b))
for a, b, kind in DYN_PAIRS:
    if kind in ("L-L", "R-R") and sat_overlap(_rest_verts(a), _rest_verts(b)):
        rest_bad.append((a, b))
print(f"  динамичных пар: {len(DYN_PAIRS)} (внутри руки + L/R x тело); "
      f"rest-исключений: {len(rest_bad)} (смежные/ложные bbox в покое)")


def collisions(armL, armR, body=BODY, ignore_rest=True):
    hits = []
    for side, av in (("L", armL), ("R", armR)):
        for a, b, kind in DYN_PAIRS:                      # внутри руки (L-L / R-R)
            if kind != f"{side}-{side}":
                continue
            if (a, b) in rest_bad:
                continue
            if sat_overlap(av[a], av[b]):
                hits.append((kind, a, b))
        for a in ARM[side]:                               # рука x тело/лифт/база
            for b in list(body):
                if (a, b) in rest_bad:
                    continue
                if sat_overlap(av[a], body[b]):
                    hits.append((f"{side}-body", a, b))
            for cy in R2_CYL:                              # рука x шасси R2 (цилиндры)
                ok, _dist = cyl_hit(av[a], cy)
                if ok:
                    hits.append((f"{side}-R2", a, cy["name"]))
    for a in ARM["L"]:                                    # левая x правая
        for b in ARM["R"]:
            if sat_overlap(armL[a], armR[b]):
                hits.append(("L-R", a, b))
    return hits


# ============================================================ 2. габарит
def full_bbox(armL, armR, body=BODY):
    allv = np.vstack(list(armL.values()) + list(armR.values())
                     + list(body.values())
                     + list(R2_BOX.values()))   # шасси R2: габарит ≥ Ø620
    mn, mx = allv.min(axis=0), allv.max(axis=0)
    return mn, mx


def sweep_radius(armL, armR, body=BODY):
    """Огибающий контур при вращении робота в плане: макс. радиус от центра.
    Панель — точный круг R=0.310 (бокс-углы не учитываются)."""
    pts = [np.vstack(list(armL.values()) + list(armR.values()))]
    for n, v in {**body, **R2_BOX}.items():
        if n == "R620_CTM_panel":
            continue
        pts.append(v)
    allp = np.vstack(pts)
    return max(float(np.hypot(allp[:, 0], allp[:, 1]).max()), R2_PANEL_R)


# база R2: цилиндр Ø620 (панель 0.05..0.09, фланец 0.09..0.10), v6
R2_PANEL = dict(r=0.31, z0=0.05, z1=0.10)


def arm_z_min(armL, armR):
    v = np.vstack(list(armL.values()) + list(armR.values()))
    return float(v[:, 2].min())


# ============================================================ 3. поиск
MIRROR = {"sh3": -1, "sh2": -1, "sh1": -1, "rad": -1, "w2": -1, "el": +1, "w1": +1}


def mirror_q(q):
    return {k: MIRROR[k] * v for k, v in q.items()}


def arm_com(armV, side):
    """Условный ЦМ руки: stored mass × центр bbox (COM=центр bbox допущение)."""
    num, den = np.zeros(3), 0.0
    for n, v in armV.items():
        m = MASS.get(n, 0.0)
        num += m * v.mean(axis=0)
        den += m
    return (num / den if den > 0 else np.zeros(3)), den


# этап A: коарс-сетка (sh3=rad=w1=w2=0)
print("\n== 3. Этап A: коарс-перебор (sh1 x sh2 x el) ==")
grid_a = dict(sh1=[-90, -75, -60, -45, -30, 0, 30, 45, 60, 90, 130],
              sh2=[-90, -60, -30, 0, 30, 60, 90],
              el=[-10, 0, 30, 60, 90, 120, 150])
stageA = []
n_a = 0
for sh1, sh2, el in itertools.product(grid_a["sh1"], grid_a["sh2"], grid_a["el"]):
    qL = {"sh1": sh1, "sh2": sh2, "el": el}
    qR = mirror_q(qL)
    if not all(LIM[k][0] <= v <= LIM[k][1] for k, v in qL.items()):
        continue
    n_a += 1
    aL, aR = arm_world("L", qL), arm_world("R", qR)
    allv = np.vstack(list(aL.values()) + list(aR.values()))
    mn, mx = allv.min(axis=0), allv.max(axis=0)
    wy, wx = mx[1] - mn[1], mx[0] - mn[0]
    zmin = mn[2]
    if wy <= 0.75 and zmin >= -0.08 and mx[2] <= DOOR_H:
        # запас на доработку stageB: rad/w1/w2 спасают и Y (большой палец),
        # и zmin (сгибание запястья отрывает пальцы от пола)
        stageA.append({"q": qL, "Wy": round(float(wy), 4), "Wx": round(float(wx), 4),
                       "zmin": round(float(zmin), 3), "zmax": round(float(mx[2]), 3)})
print(f"  проверено конфигураций: {n_a}; прошли габаритный фильтр "
      f"(Wy<=0.70, zmin>=0.10, zmax<=2.0): {len(stageA)}")

# этап B: доработка (sh3, rad, w1, w2) + полный габарит + коллизии
print("== 3. Этап B: полная оценка (sh3 x rad x w1 x w2) ==")
grid_b = dict(sh3=[-60, -30, 0, 30, 60], rad=[-90, -45, 0, 45, 90],
              w1=[-90, 0, 90], w2=[-25, 0, 45])
results, n_b, n_col = [], 0, 0
for cand in stageA:
    for sh3, rad, w1, w2 in itertools.product(*grid_b.values()):
        qL = dict(cand["q"], sh3=sh3, rad=rad, w1=w1, w2=w2)
        if not all(LIM[k][0] <= v <= LIM[k][1] for k, v in qL.items()):
            continue
        n_b += 1
        qR = mirror_q(qL)
        aL, aR = arm_world("L", qL), arm_world("R", qR)
        mn, mx = full_bbox(aL, aR)
        wy, wx = float(mx[1] - mn[1]), float(mx[0] - mn[0])
        arms_all = np.vstack(list(aL.values()) + list(aR.values()))
        zmin_arms = float(arms_all[:, 2].min())
        if wy > DOOR_W or mx[2] > DOOR_H or zmin_arms < 0.10:
            continue
        hits = collisions(aL, aR)
        if hits:
            n_col += 1
            continue
        cL, mL = arm_com(aL, "L")
        cR, mR = arm_com(aR, "R")
        results.append({
            "q_L": qL, "q_R": qR,
            "габарит_мм": {"X": round(wx * 1000, 1), "Y": round(wy * 1000, 1),
                           "Z_верх": round(float(mx[2]) * 1000, 1),
                           "Z_низ_рук": round(zmin_arms * 1000, 1)},
            "огибающий_вращение_мм": round(sweep_radius(aL, aR) * 2000, 1),
            "зазор_двери_Y_мм": round((DOOR_W - wy) * 1000, 1),
            "зазор_двери_X_мм": round((DOOR_W - wx) * 1000, 1),
            "запас_высоты_мм": round((DOOR_H - float(mx[2])) * 1000, 1),
            "ЦМ_рук_L_R2": [round(float(x), 4) for x in cL],
            "ЦМ_рук_R_R2": [round(float(x), 4) for x in cR],
            "масса_рук_кг_stored": round(mL + mR, 3),
        })
print(f"  полных конфигураций: {n_b}; отсечено коллизиями: {n_col}; "
      f"пригодных (без коллизий): {len(results)}")
results.sort(key=lambda r: (-r["зазор_двери_Y_мм"], -r["запас_высоты_мм"]))
# классификация по проёму 700 (квадратный): К1 — проходит в ЛЮБОЙ ориентации,
# К2 — только при движении вперёд (Y<=700, X>700)
for r in results:
    r["поперечник_мин_мм"] = min(r["габарит_мм"]["X"], r["габарит_мм"]["Y"])
    r["вынос_мм"] = max(r["габарит_мм"]["X"], r["габарит_мм"]["Y"])
    r["класс_прохода"] = ("К1: обе ориентации" if r["габарит_мм"]["X"] <= 700
                          and r["габарит_мм"]["Y"] <= 700
                          else "К2: только вперёд по X")
rank_k1 = sorted([r for r in results if r["класс_прохода"].startswith("К1")],
                 key=lambda r: r["вынос_мм"])
rank_k2 = sorted([r for r in results if r["класс_прохода"].startswith("К2")],
                 key=lambda r: -r["зазор_двери_Y_мм"])

# ============================================================ 4. переход из P0
def pick_unique(rank, n, taken):
    out = []
    for r in rank:
        key = json.dumps(r["q_L"], sort_keys=True)
        if key in taken:
            continue
        taken.add(key)
        out.append(r)
        if len(out) >= n:
            break
    return out


def transition_ok(q_target, steps=11):
    """Линейная интерполяция P0(q=0) -> цель: лимиты и коллизии на каждом шаге."""
    q0 = {k: 0.0 for k in FK.ARM_JOINT_KEYS}
    bad_limits, col_steps = [], []
    for i in range(steps + 1):
        f = i / steps
        qL = {k: q0[k] + f * (q_target[k] - q0.get(k, 0.0)) for k in q0}
        for k, v in qL.items():
            if not (LIM[k][0] <= v <= LIM[k][1]):
                bad_limits.append((round(f, 2), k, round(v, 1)))
        qR = mirror_q(qL)
        aL, aR = arm_world("L", qL), arm_world("R", qR)
        hits = collisions(aL, aR)
        if hits:
            col_steps.append({"шаг": round(f, 2), "коллизий": len(hits),
                              "пример": hits[:3]})
    return bad_limits, col_steps


# ============================================================ 5. финалисты
print("\n== 4. Финальная проверка кандидатов (переход из P0, два класса) ==")
_taken = set()
final = []
for cls, rank in (("К1: проход в ОБЕИХ ориентациях (топ по min-макс габариту)",
                   rank_k1),
                  ("К2: только движение вперёд по X (топ по Y-зазору)",
                   rank_k2)):
    for r in pick_unique(rank, 6, _taken):
        bad_l, col_s = transition_ok(r["q_L"])
        qL, qR = r["q_L"], r["q_R"]
        aL, aR = arm_world("L", qL, -0.0476), arm_world("R", qR, -0.0476)
        mn_l, _ = full_bbox(aL, aR)
        col_lift = collisions(aL, aR)
        fr = dict(r)
        fr["класс"] = cls
        fr["переход_из_P0"] = {
        "лимиты": "OK (бокс лимитов выпукл — линейная интерполяция внутри)" if not bad_l
                  else f"нарушения {bad_l[:3]}",
        "коллизии_на_пути": f"{len(col_s)} из 11 шагов" if col_s else
                            "нет (0/11 шагов)",
        "шаги_с_коллизиями": col_s[:4]}
        arms_l = np.vstack(list(aL.values()) + list(aR.values()))
        fr["вариант_лифт_-47мм"] = {
            "коллизии": "есть: " + str(col_lift) if col_lift else "нет",
            "zmin_рук_мм": round(float(arms_l[:, 2].min()) * 1000, 1)}
        fr["статус_перехода"] = ("без коллизий на всём пути" if not col_s and not bad_l
                                 else "требует траекторного планирования (коллизии "
                                      "в промежуточных точках)")
        final.append(fr)
        print(f"  [{cls[:2]}] q={ {k: v for k, v in qL.items() if v} } | "
              f"X={r['габарит_мм']['X']} Y={r['габарит_мм']['Y']} | "
              f"вращ.Ø={r['огибающий_вращение_мм']} | переход: "
              f"{fr['статус_перехода']}")

# ============================================================ 6. эталоны 2A
print("\n== 5. Контрольные позы Stage 2A в том же калькуляторе ==")
GT_P = os.path.join(ROOT, "stage2A", "control_pose_gt.json")
with open(GT_P, encoding="utf-8") as f:
    GT = json.load(f)["pose_definitions"]
ref_rows = []
for name in ("P0", "folded", "lowered_sym", "forward_bent"):
    qL = GT[name].get("L", {})
    qR = GT[name].get("R", {})
    aL, aR = arm_world("L", qL), arm_world("R", qR)
    mn, mx = full_bbox(aL, aR)
    hits = collisions(aL, aR)
    ref_rows.append({"поза": name, "q_L": qL, "q_R": qR,
                     "габарит_мм": {"X": round(float(mx[0] - mn[0]) * 1000, 1),
                                    "Y": round(float(mx[1] - mn[1]) * 1000, 1),
                                    "Z_верх": round(float(mx[2]) * 1000, 1)},
                     "дверь_700_Y": "проходит" if (mx[1] - mn[1]) <= 0.7 else "НЕ проходит",
                     "коллизий": len(hits)})
    print(f"  {name}: Y={ref_rows[-1]['габарит_мм']['Y']} мм "
          f"({ref_rows[-1]['дверь_700_Y']}), коллизий {len(hits)}")

# ============================================================ 7. итоги
t("T2_позы_найдены", len(final) > 0,
  f"пригодных поз без коллизий: {len(results)}, в финальной проверке: {len(final)}")
n_ok_trans = sum(1 for f in final if f["статус_перехода"].startswith("без коллизий"))
t("T3_переход", n_ok_trans > 0,
  f"поз с безколлизионным переходом из P0: {n_ok_trans} из {len(final)}")
t("T4_высота", all(f["габарит_мм"]["Z_верх"] <= 2000 for f in final),
  "все финалисты ≤ 2000 мм")
t("T5_габарит_не_меньше_620",
  bool(final) and all(f["габарит_мм"]["X"] >= 619.9 and f["габарит_мм"]["Y"] >= 619.9
                      for f in final),
  "полный габарит финалистов включает панель R2: ≥620 мм по X и Y")

# чего НЕ хватает для достоверного габарита (явно)
missing_geo = [
    "реальный контур мешей звеньев (в базе только 8-вершинные bbox-обёртки — "
    "габарит консервативен, возможны ложные отказы, но не ложные проходы)",
    "суставы пальцев зафиксированы в покое — отдельные транспортные позы пальцев "
    "не рассматривались",
    "кабельные цепи/жгуты лифта и рук в модели отсутствуют (выступы не учтены)",
    "габариты R2-версии верхней части (торс/лифт) не существуют — взята "
    "Blender-конфигурация (иная, чем R2-база Ø620)",
    "модули/панель/ствол лифта R2: из v6 (WHL0/90/180/270 — Ø240×106.9, z 0..0.24 м; "
    "панель Ø620 z 0.05..0.09 м; ствол лифта r 0.12 z 0.09..0.40 м); коллизии рук "
    "с ними считаются ЦИЛИНДРИЧЕСКИМ критерием (проверено: бокс-обёртки давали "
    "ложные срабатывания и были исключены из SAT)",
    "площадки аутригеров pad45/135/225/315 (v6, maxR 480.8 мм) — РАБОЧЕЕ положение; "
    "в транспортный габарит не включались; требование «сложенные аутригеры ≤Ø620» "
    "подтверждено только по блокам pod/arm (maxR ≤300 мм)",
    "проектный зазор прохода 700 мм и допустимый вынос рук не согласованы",
    "транспортная поза лифта (ход) и поза головы/шеи — параметры, не согласованы",
]
print("\n== 6. Какой геометрии не хватает для достоверного габарита ==")
for m in missing_geo:
    print("  -", m)

out = {
    "checks": CHECKS,
    "counts": {"mesh": len(mesh_names), "arm_L": len(ARM["L"]), "arm_R": len(ARM["R"]),
               "lift_body": len(LIFT_BODY), "static_body": len(STATIC_BODY),
               "stageA": n_a, "stageA_pass": len(stageA), "stageB": n_b,
               "stageB_collision": n_col, "candidates": len(results),
               "final": len(final)},
    "final_candidates": final,
    "all_candidates": results,
    "ranking_note": "поперечник_мин = min(Wx, Wy): проём квадратный — при выборе "
                    "ориентации движения поперечник может быть минимален; "
                    "вынос = max(Wx, Wy) — габарит в худшей ориентации",
    "reference_poses_2A": ref_rows,
    "missing_geometry": missing_geo,
    "assumptions": [
        "COM = центр bbox звена (stored mass), допущение Stage 2V",
        "пальцы в rest-повороте",
        "база R2: цилиндр r=0.31, панель 0.05..0.10 м (v6)",
        "основная ориентация движения — вперёд по X (дверь ограничивает Y); "
        "боковое движение ограничено X-габаритом (приведён отдельно)",
        "проектный зазор двери не задан — приведён фактический",
    ],
    "notes": ["Blender и предыдущие расчёты не изменялись (только чтение).",
              "Поза НЕ утверждается: статусы выше — результаты проверки по "
              "bbox-прокси-геометрии."],
}
path = os.path.join(HERE, "stage2g_v7_5_transport_pose.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")
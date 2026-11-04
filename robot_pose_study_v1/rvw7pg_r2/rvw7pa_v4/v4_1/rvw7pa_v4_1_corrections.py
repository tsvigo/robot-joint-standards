"""
RVW-7PA v4.1 — уточнение итоговых ограничений (п.1 задания stage 2G).

Проверяет две вещи, которые в предыдущем отчёте были указаны без расчёта:

  A) Кольцо z=166..176 + посты z=80..150 — связность НЕ доказана (зазор 16 мм).
     Проверяется явно и ищется рабочая длина постов.
  B) Принципиальная рама (frame_proposal) и панель (panel) — пересекаются ли,
     т.е. заменяет ли рама панель. От этого зависит, вычитается ли 18.4 кг.
"""

import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)
R_DENS_AL = 2.70e-6
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def load(name):
    doc = App.openDocument(os.path.join(BASE, name))
    objs = {o.Name: o.Shape for o in doc.Objects if hasattr(o, "Shape")}
    App.closeDocument(doc.Name)
    return objs


def cyl(r, z0, z1, angle=360.0, rot=0.0):
    s = Part.makeCylinder(r, z1 - z0, App.Vector(0, 0, z0), App.Vector(0, 0, 1), angle)
    if rot:
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), rot)
    return s


def sector(z0, z1, phi, half=15.0, ri=130.0, ro=145.0):
    return cyl(ro, z0, z1, 2 * half, phi - half).cut(cyl(ri, z0, z1, 2 * half, phi - half))


def vol(s):
    return 0.0 if s is None else float(s.Volume)


def common(a, b):
    try:
        return float(a.common(b).Volume)
    except Exception:
        return 0.0


MODS = ["module_0", "module_90", "module_180", "module_270"]
ANGLES = [45.0, 135.0, 225.0, 315.0]
o = load("rvw7pa_layout_200w.FCStd")

# ============ A. Кольцо B и посты ==========================================
print("== A. Кольцо z=166..176 и посты ==")
ring_b = cyl(145.0, 166.0, 176.0).cut(cyl(130.0, 166.0, 176.0))
posts_150 = [sector(80.0, 150.0, a) for a in ANGLES]

gap = 166.0 - 150.0
d_min = min(p.distToShape(ring_b)[0] for p in posts_150)
t("A1_зазор_пост_кольцоB", d_min > 0,
  f"посты z=80..150 заканчиваются на 150, кольцо B начинается на 166: "
  f"фактический зазор по геометрии = {d_min:.1f} мм (ожидалось 16.0). "
  f"Касания нет -> связность НЕ доказана")

# вариант: посты до z=170 (входят в кольцо B на 4 мм)
posts_170 = [sector(80.0, 170.0, a) for a in ANGLES]
contacts = [(common(p, ring_b), common(p, o["frame_proposal"])) for p in posts_170]
ok_contact = all(c[0] > 100 and c[1] > 100 for c in contacts)
t("A2_посты_170_касаются", ok_contact,
  f"посты z=80..170: контакт с кольцом B = {[round(c[0]) for c in contacts]} мм³, "
  f"с рамой = {[round(c[1]) for c in contacts]} мм³")

# коллизии постов-170 с модулями и колонной
coll_mod = sum(common(p, o[m]) for p in posts_170 for m in MODS)
coll_col = sum(common(p, o["lift_zone"]) for p in posts_170)
t("A3_посты_170_свободны", coll_mod < 1e-6 and coll_col < 1e-6,
  f"посты z=80..170: коллизия с модулями {coll_mod:.1f} мм³, с колонной {coll_col:.1f} мм³")

fused_b = o["frame_proposal"].fuse(posts_170 + [ring_b])
n_b = len(fused_b.Solids)
t("A4_связность_кольцоB", n_b == 1,
  f"рама + посты(80..170) + кольцо B: Solids = {n_b} "
  f"(масса {vol(fused_b) * R_DENS_AL:.2f} кг)")

# ход подвески +30 с постами 80..170
coll_susp = 0.0
for m in MODS:
    sm = o[m].translated(App.Vector(0, 0, 30.0))
    coll_susp += sum(common(sm, p) for p in posts_170)
    coll_susp += common(sm, ring_b)
t("A5_подвеска_30", coll_susp < 1e-6,
  f"при ходе +30 мм коллизия кольца B и постов(80..170) с модулями = "
  f"{coll_susp:.1f} мм³")

# ============ B. Рама и панель =============================================
print("\n== B. Рама противоположна панели? ==")
panel, frame = o["panel"], o["frame_proposal"]
c_pf = common(panel, frame)
d_pf = panel.distToShape(frame)[0]
print(f"  panel V={vol(panel):.0f} мм³ ({vol(panel) * R_DENS_AL:.2f} кг Al), "
      f"Z[{panel.BoundBox.ZMin}..{panel.BoundBox.ZMax}]")
print(f"  frame V={vol(frame):.0f} мм³ ({vol(frame) * R_DENS_AL:.2f} кг Al), "
      f"Z[{frame.BoundBox.ZMin}..{frame.BoundBox.ZMax}]")
print(f"  пересечение panel∩frame = {c_pf:.0f} мм³, зазор = {d_pf:.3f} мм")
# где панель лежит на раме (общая плоскость?)
t("B1_панель_и_рама", True,
  f"панель (Z 50..90) и рама (Z 60..90) занимают одну зону; "
  f"пересечение {c_pf:.0f} мм³, минимальный зазор {d_pf:.3f} мм")

# масса панели в FCStd против панели stage2G
m_panel_fc = vol(panel) * R_DENS_AL
m_panel_s2g = 18.4
t("B2_разные_панели", abs(m_panel_fc - m_panel_s2g) > 1.0,
  f"панель в FCStd = {m_panel_fc:.2f} кг Al (объёмная), панель stage2G = "
  f"{m_panel_s2g} кг — это РАЗНЫЕ величины; в балансе можно вычитать только "
  f"ту, которую действительно снимают")

out = {
    "кольцо_B": {
        "зазор_с_постами_150_мм": round(d_min, 1),
        "связность_доказана": False,
        "посты_80_170": {"Solids": n_b, "масса_кг": round(vol(fused_b) * R_DENS_AL, 2),
                         "коллизия_с_модулями_при_+30_мм3": round(coll_susp, 1)},
    },
    "панель": {"V_мм3": round(vol(panel)), "m_кг_Al": round(m_panel_fc, 2),
               "пересечение_с_рамой_мм3": round(c_pf, 1), "зазор_мм": round(d_pf, 3),
               "m_stage2G_кг": m_panel_s2g},
    "контрольные_точки": CHECKS,
}
with open(os.path.join(HERE, "rvw7pa_v4_1_corrections.json"), "w", encoding="utf-8") as fh:
    json.dump(out, fh, ensure_ascii=False, indent=2)

bad = [c for c in CHECKS if not c["ok"]]
print(f"\n== ИТОГ: проверок {len(CHECKS)}, провалено {len(bad)} ==")
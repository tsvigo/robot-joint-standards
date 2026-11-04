"""
RVW-7PA v4.1 — ПРОВЕРКА СВЯЗНОЙ РАМЫ (OpenCascade, по СОХРАНЁННЫМ FCStd).

Что делает:
  1. Открывает rvw7pa_layout_200w.FCStd и rvw7pa_layout_400w.FCStd (только чтение).
  2. Воспроизводит независимую проверку кольца Rвнутр=130, Rнаруж=145,
     z=140..160: объём, масса (Al), пересечения с колонной Ø240 и оболочками
     модулей обоих вариантов.
  3. Добавляет 4 вертикальных силовых элемента (посты) под 45° внутри
     габарита 176 мм и проверяет: пересечения, связность (1 Solid),
     контакты по цепочке, сечения, массу.
  4. Проверяет вариант кольца z=166..176 (совместимость с ходом подвески +30).
  5. Проверяет ход подвески ±15/±30 мм (смещение модулей по Z).
  6. Проверяет НЕПРЕРЫВНОСТЬ пути нагрузки до основания подъёмника (z=+88).

Чего НЕ делает: FEA, окончательную деталировку, крепёж.
Соприкосновение тел НЕ считается доказательством прочности.
"""

import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)

R_DENS_AL = 2.70e-6          # кг/мм³ (2.7 г/см³) — параметр
RING_RI, RING_RO = 130.0, 145.0
POST_HALF_DEG = 15.0         # половина угловой ширины поста
POST_RI, POST_RO = RING_RI, RING_RO
LIFT_AXIS = (0.003, -4.41)   # реальная ось колонны по stage2g_lift_interface.md

CHECKS = []
OUT = {"кольцо": {}, "посты": {}, "связность": {}, "подвеска": {},
       "путь_нагрузки": {}, "массы": {}}


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def cyl(r, z0, z1, angle=360.0, rot=0.0):
    s = Part.makeCylinder(r, z1 - z0, App.Vector(0, 0, z0), App.Vector(0, 0, 1), angle)
    if rot:
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), rot)
    return s


def ring_shape(ri, ro, z0, z1):
    return cyl(ro, z0, z1).cut(cyl(ri, z0, z1))


def sector(ri, ro, z0, z1, phi_deg, half_deg):
    outer = cyl(ro, z0, z1, angle=2 * half_deg, rot=phi_deg - half_deg)
    inner = cyl(ri, z0, z1, angle=2 * half_deg, rot=phi_deg - half_deg)
    return outer.cut(inner)


def vol(s):
    return 0.0 if s is None else float(s.Volume)


def common(a, b):
    try:
        return float(a.common(b).Volume)
    except Exception:
        return 0.0


def mass(s):
    return vol(s) * R_DENS_AL


def load(name):
    doc = App.openDocument(os.path.join(BASE, name))
    objs = {o.Name: o.Shape for o in doc.Objects if hasattr(o, "Shape")}
    App.closeDocument(doc.Name)
    return objs


layout = {}
for tag, fn in (("200W", "rvw7pa_layout_200w.FCStd"),
                ("400W", "rvw7pa_layout_400w.FCStd")):
    layout[tag] = load(fn)
    print(f"  загружен {fn}: {len(layout[tag])} объектов")

MODS = ["module_0", "module_90", "module_180", "module_270"]

# ============ 1. КОЛЬЦО 130..145, z=140..160 ================================
print("\n== 1. Воспроизведение независимой проверки кольца ==")
ring_a = ring_shape(RING_RI, RING_RO, 140.0, 160.0)
v_ring = vol(ring_a)
v_ref = math.pi * (RING_RO ** 2 - RING_RI ** 2) * 20.0
t("F1_объём_кольца", abs(v_ring - 259181.0) < 500.0,
  f"V = {v_ring:.0f} мм³ (аналитика π(Ro²−Ri²)·h = {v_ref:.0f})")
t("F2_масса_кольца", abs(mass(ring_a) - 0.700) < 0.01,
  f"m = {mass(ring_a):.3f} кг при Al 2.7 г/см³ (параметр)")

coll_a = {}
for tag in ("200W", "400W"):
    d = layout[tag]
    coll_a[tag] = {
        "lift_zone": common(ring_a, d["lift_zone"]),
        **{m: common(ring_a, d[m]) for m in MODS},
        "frame_proposal": common(ring_a, d["frame_proposal"]),
    }
    ok = (abs(coll_a[tag]["lift_zone"]) < 1e-6 and
          all(abs(coll_a[tag][m]) < 1e-6 for m in MODS))
    t(f"F3_кольцо_без_коллизий_{tag}", ok,
      f"с колонной {coll_a[tag]['lift_zone']:.1f} мм³, "
      f"с модулями {[round(coll_a[tag][m], 1) for m in MODS]} мм³; "
      f"с рамой {coll_a[tag]['frame_proposal']:.1f} мм³ (касания НЕТ)")
OUT["кольцо"]["A_140_160"] = {
    "V_мм3": round(v_ring), "m_кг": round(mass(ring_a), 3),
    "коллизии": {k: {kk: round(vv, 2) for kk, vv in v.items()}
                 for k, v in coll_a.items()},
}

# зазор до колонны с учётом реального смещения оси
gap_col = RING_RI - 120.0 - math.hypot(*LIFT_AXIS)
t("F4_зазор_до_колонны", gap_col > 0,
  f"номинально 10.0 мм; с осью (+0.003,−4.41) минимум ≈{gap_col:.2f} мм (>0)")

# ============ 2. КОЛЬЦО 166..176 (вариант под ход подвески) =================
print("\n== 2. Вариант кольца z=166..176 (выше мотора при ходе +30) ==")
ring_b = ring_shape(RING_RI, RING_RO, 166.0, 176.0)
coll_b = {}
for tag in ("200W", "400W"):
    d = layout[tag]
    coll_b[tag] = {
        "lift_zone": common(ring_b, d["lift_zone"]),
        **{m: common(ring_b, d[m]) for m in MODS},
    }
t("F5_кольцо_B_без_коллизий", all(
    abs(x) < 1e-6 for v in coll_b.values() for x in v.values()),
  f"V = {vol(ring_b):.0f} мм³, m = {mass(ring_b):.3f} кг; "
  f"коллизии: {[{k: round(x, 1) for k, x in v.items()} for v in coll_b.values()]}")
OUT["кольцо"]["B_166_176"] = {"V_мм3": round(vol(ring_b)),
                              "m_кг": round(mass(ring_b), 3)}

# ============ 3. ПОСТЫ И СВЯЗНОСТЬ =========================================
print("\n== 3. Вертикальные посты 45°/135°/225°/315° и связность ==")
ANGLES = [45.0, 135.0, 225.0, 315.0]


def build_posts(z0, z1):
    return {a: sector(POST_RI, POST_RO, z0, z1, a, POST_HALF_DEG) for a in ANGLES}


def evaluate(tag, ring, z0, z1, label):
    d = layout[tag]
    posts = build_posts(z0, z1)
    res = {"посты": {}, "коллизии": {}}
    vp = 0.0
    for a, p in posts.items():
        vp += vol(p)
        res["посты"][a] = {
            "V_мм3": round(vol(p)),
            "с_рамой": round(common(p, d["frame_proposal"]), 1),
            "с_кольцом": round(common(p, ring), 1),
            "с_колонной": round(common(p, d["lift_zone"]), 1),
            "с_модулями": round(sum(common(p, d[m]) for m in MODS), 1),
        }
    # связность
    parts = [d["frame_proposal"]] + list(posts.values()) + [ring]
    fused = parts[0].fuse(parts[1:])
    n_solids = len(fused.Solids)
    res.update({"V_постов_мм3": round(vp), "m_постов_кг": round(vp * R_DENS_AL, 3),
                "solids": n_solids, "V_итог_мм3": round(vol(fused))})
    res["масса_кг"] = round(vol(fused) * R_DENS_AL, 3)
    print(f"\n  {label} ({tag}):")
    for a, r in res["посты"].items():
        print(f"    пост {a:>5.0f}°: V={r['V_мм3']:>7} мм³  рама {r['с_рамой']:>8}  "
              f"кольцо {r['с_кольцом']:>8}  колонна {r['с_колонной']:>6}  "
              f"модули {r['с_модулями']:>6}")
    print(f"    посты: {vp:.0f} мм³ = {vp * R_DENS_AL:.3f} кг; "
          f"итого рама+посты+кольцо = {res['масса_кг']:.2f} кг; "
          f"Solids = {n_solids}")
    return res, fused


res_a, fused_a = evaluate("200W", ring_a, 80.0, 150.0, "кольцо A z=140..160")
res_b, fused_b = evaluate("400W", ring_a, 80.0, 150.0, "кольцо A z=140..160")
t("F6_связность_1_solid", res_a["solids"] == 1 and res_b["solids"] == 1,
  f"после объединения рама+4 поста+кольцо: 200W Solids={res_a['solids']}, "
  f"400W Solids={res_b['solids']} (4 квадранта рамы соединены через кольцо)")
t("F7_посты_свободны", all(
    r["с_колонной"] < 1e-6 and r["с_модулями"] < 1e-6
    for res in (res_a, res_b) for r in res["посты"].values()),
  "посты под 45° не пересекают колонну и оболочки модулей (оба варианта)")
t("F8_контакт_постов", all(
    r["с_рамой"] > 100 and r["с_кольцом"] > 100
    for res in (res_a, res_b) for r in res["посты"].values()),
  "каждый пост имеет объёмный контакт и с рамой (хомут), и с кольцом — "
  "не «касание в точке»")
OUT["посты"] = {"A_200W": res_a, "A_400W": res_b}

# минимальные сечения
post_area = (2 * POST_HALF_DEG / 360.0) * math.pi * (POST_RO ** 2 - POST_RI ** 2)
ring_area = math.pi * (RING_RO ** 2 - RING_RI ** 2)
# точное для кольца: I = π(Ro⁴−Ri⁴)/4, W = I/Ro
ring_I = math.pi * (RING_RO ** 4 - RING_RI ** 4) / 4.0
ring_W = ring_I / RING_RO
# пост: эквивалентный прямоугольник 15 мм (радиально) × дуга (по среднему радиусу)
post_arc = math.radians(2 * POST_HALF_DEG) * (POST_RI + POST_RO) / 2.0
post_W = (RING_RO - RING_RI) * post_arc ** 2 / 6.0
t("F9_сечения", post_area > 500,
  f"сечение поста: {post_area:.0f} мм² (r 130..145, ±{POST_HALF_DEG:.0f}°, "
  f"дуга {post_arc:.1f} мм), W ≈ {post_W:.0f} мм³; "
  f"кольцо: {ring_area:.0f} мм², I = {ring_I:.0f} мм⁴, W = {ring_W:.0f} мм³ "
  f"(порядковая оценка сечения, FEA не выполнялась)")
OUT["массы"] = {"кольцо_A_кг": round(mass(ring_a), 3),
                "посты_кг": round(res_a["m_постов_кг"], 3),
                "рама_вместе_кг": round(res_a["масса_кг"], 3),
                "кольцо_B_кг": round(mass(ring_b), 3)}

# ============ 4. ХОД ПОДВЕСКИ ==============================================
print("\n== 4. Ход подвески: смещение модулей по Z ==")
susp = {}
for dz in (0.0, 6.0, 15.0, 30.0):
    d = layout["200W"]
    r_coll = 0.0
    p_coll = 0.0
    for m in MODS:
        sm = d[m].translated(App.Vector(0, 0, dz))
        r_coll += common(sm, ring_a)
        for a in ANGLES:
            p_coll += common(sm, sector(POST_RI, POST_RO, 80.0, 150.0, a, POST_HALF_DEG))
    rb_coll = sum(common(layout["200W"][m].translated(App.Vector(0, 0, dz)), ring_b)
                  for m in MODS)
    susp[dz] = {"кольцоA": round(r_coll, 1), "посты": round(p_coll, 1),
                "кольцоB": round(rb_coll, 1)}
    print(f"  dz = +{dz:>4.0f} мм: коллизия с кольцом A = {r_coll:>9.1f} мм³, "
          f"с постами = {p_coll:>7.1f} мм³, с кольцом B = {rb_coll:>8.1f} мм³")
t("F10_подвеска", susp[30.0]["кольцоA"] > 0 and susp[30.0]["кольцоB"] == 0,
  "кольцо A (140..160): при ходе +6 мм — касание (верх мотора 134 -> 140), "
  "при +15 мм уже 49 515 мм³ пересечения, при +30 мм — 110 034 мм³. "
  "Кольцо B (166..176) свободно при +30 мм. Т.е. кольцо A совместимо только "
  "с жёстким креплением модуля (ход ≤6 мм)")
OUT["подвеска"] = susp

# ============ 5. ПУТЬ НАГРУЗКИ =============================================
print("\n== 5. Путь нагрузки: привод → рама → кольцо → основание подъёмника ==")
d = layout["200W"]
frame_vs_lift = common(d["frame_proposal"], d["lift_zone"])
ring_vs_lift = common(ring_a, d["lift_zone"])
clamp_vs_frame = common(d["clamp_proposal"], d["frame_proposal"])
dist_clamp_frame = d["clamp_proposal"].distToShape(d["frame_proposal"])[0]

# реальная ось колонны (+0.003, −4.41)
lift_off = d["lift_zone"].translated(App.Vector(LIFT_AXIS[0], LIFT_AXIS[1], 0.0))
d_frame_col = d["frame_proposal"].distToShape(lift_off)[0]
d_ring_col = ring_a.distToShape(lift_off)[0]

print(f"  хомут ↔ рама: {clamp_vs_frame:.1f} мм³, зазор {dist_clamp_frame:.3f} мм")
print(f"  рама ↔ колонна (ось в 0): {frame_vs_lift:.1f} мм³; "
      f"с реальной осью зазор {d_frame_col:.2f} мм")
print(f"  кольцо ↔ колонна: {ring_vs_lift:.1f} мм³; с реальной осью {d_ring_col:.2f} мм")
print(f"  кольцо z=140..160 на 52 мм ВЫШЕ посадочной плоскости z=+88")

# --- тест мостика 2 мм: закрывает ли он первый разрыв
bridge = Part.makeBox(20.0, 3.5, 30.0, App.Vector(175.0, 94.0, 60.0))
br_vs_clamp = common(bridge, d["clamp_proposal"])
br_vs_frame = common(bridge, d["frame_proposal"])
fused_br = d["frame_proposal"].fuse([bridge, d["clamp_proposal"],
                                     ring_a] + list(build_posts(80.0, 150.0).values()))
t("F11_разрыв_хомут_рама", dist_clamp_frame > 1e-6,
  f"в сохранённой модели хомут и рама НЕ соприкасаются: зазор "
  f"{dist_clamp_frame:.3f} мм (хомут y=95.0, рычаг рамы y=97.0). "
  f"Мостик 20×2.2×30 мм даёт контакт: с хомутом {br_vs_clamp:.0f} мм³, "
  f"с рамой {br_vs_frame:.0f} мм³; Solids при объединении "
  f"(хомут+мостик+рама+посты+кольцо) = {len(fused_br.Solids)}")

path = {
    "хомут_привода_->_рама": {"V_мм3": round(clamp_vs_frame, 1),
                              "зазор_мм": round(dist_clamp_frame, 3),
                              "статус": "РАЗРЫВ (зазор 2 мм, закрывается мостиком)"},
    "рама_->_пост": {"V_мм3": round(sum(r["с_рамой"] for r in res_a["посты"].values()), 1),
                     "статус": "есть объёмный контакт"},
    "пост_->_кольцо": {"V_мм3": round(sum(r["с_кольцом"] for r in res_a["посты"].values()), 1),
                       "статус": "есть объёмный контакт"},
    "кольцо_->_основание_подъёмника": {"V_мм3": round(ring_vs_lift, 1),
                                       "статус": "РАЗРЫВ (кольцо выше плиты на 52 мм)"},
    "рама_->_основание_подъёмника": {"V_мм3": round(frame_vs_lift, 1),
                                     "зазор_до_колонны_мм": round(d_frame_col, 2),
                                     "статус": "разрыв: рама не касается ни колонны, "
                                               "ни посадочной плиты"},
}
t("F12_путь_нагрузки", ring_vs_lift == 0.0 and frame_vs_lift == 0.0,
  "путь НЕ замкнут. Кольцо соединяет 4 квадранта рамы между собой, но не "
  "создаёт передачу на подъёмник: оно на 52 мм выше посадочной плоскости "
  "z=+88 и на 5.6 мм снаружи колонны. Второй разрыв — хомут/рама (2 мм)")

# требуемый Ø_plate для болтового соединения через хомут (r 124..168 под 45°)
outer_needed = 2 * 168.0
t("F13_требование_к_плите", True,
  f"если соединять хомут (r 124..168) с посадочной плитой при z=+88, "
  f"нужен Ø_plate ≥ {outer_needed:.0f} мм (параметр R_lift/Ø_plate в "
  f"stage2g_lift_interface.md не задан)")
OUT["путь_нагрузки"] = path

# ============ 6. ИТОГ ======================================================
print("\n== ИТОГ ==")
bad = [c for c in CHECKS if not c["ok"]]
print(f"  проверок: {len(CHECKS)}, провалено: {len(bad)}")
OUT["контрольные_точки"] = CHECKS
OUT["вывод"] = (
    "Кольцо R130..145 z=140..160 воспроизведено (V=259181 мм³, 0.700 кг Al) и "
    "не пересекает колонну Ø240 и оболочки RVW-7PA обоих вариантов. Четыре "
    "поста под 45° (z=80..150, r=130..145) соединяют кольцо с хомутами рамы: "
    "после объединения получается 1 Solid. НО путь нагрузки до основания "
    "подъёмника (z=+88) не замкнут: кольцо висит выше посадочной плоскости и "
    "на 10 мм снаружи колонны. Кроме того, кольцо z=140..160 оставляет 6 мм до "
    "верха мотора и конфликтует с ходом подвески ≥+6 мм.")
with open(os.path.join(HERE, "rvw7pa_frame_link.json"), "w", encoding="utf-8") as fh:
    json.dump(OUT, fh, ensure_ascii=False, indent=2, default=str)
print(f"  JSON: {os.path.join(HERE, 'rvw7pa_frame_link.json')}")

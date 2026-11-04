# -*- coding: utf-8 -*-
"""Виды stage2g_v5 (задача 10 — состояния НЕ совмещаются):
 v5_R_transport.png / v5_R_working.png — схема R (радиальные оси), Ø600/620/640 x 3 вида;
 v5_P_transport.png / v5_P_working.png — схема P (параллельные оси, диагонали);
 v5_top_grid.png — 18 комбинаций сверху (транспортное состояние) с маркерами запасов.
Транспорт: колёса/ступицы/зоны/блоки аутригеров, БЕЗ площадок.
Рабочее: + телескопические рычаги, площадки, точки опоры, контур опоры."""
import json
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

TRI = json.load(open("stage2g_v5_tris.json"))["groups"]
GEO = json.load(open("stage2g_v5_geometry.json"))
CASES = {(c["layout"], int(c["Dp"]), c["wheel"], c["drive"]): c for c in GEO["cases"]}
S2 = math.sqrt(2.0)

COL = {"body": "#9aa5b1", "wheels": "#2f6fdb", "zones": "#7fd3e8",
       "pods": "#c9a227", "arms": "#e8944a", "pads": "#b3591d"}
SOLID_T = ("body", "wheels", "zones", "pods")
SOLID_W = ("body", "wheels", "zones", "pods", "arms", "pads")


def proj_funcs(view):
    if view == "top":
        return (lambda x, y, z: (x, y)), (lambda x, y, z: z)
    if view == "side":
        return (lambda x, y, z: (x, z)), (lambda x, y, z: y)
    s30 = math.sin(math.radians(35.264))
    c30 = math.cos(math.radians(35.264))
    return ((lambda x, y, z: ((x - y) / S2, (x + y) / S2 * s30 + z * c30)),
            (lambda x, y, z: (x + y) / S2 * c30 - z * s30))


def draw(ax, gname, view, state, show_env=False):
    gd = TRI[gname]
    proj, depth = proj_funcs(view)
    deep = "suppext" if state == "working" else "supp"
    polys = [[proj(*p) for p in t] for t in gd.get(deep, [])]
    if polys:
        ax.add_collection(PolyCollection(polys, facecolor="#44aa66",
                                         edgecolor="#2b7348", linewidth=0.3, alpha=0.45))
    solid = SOLID_W if state == "working" else SOLID_T
    items = []
    for cls in solid:
        for t in gd.get(cls, []):
            items.append((sum(depth(*p) for p in t) / 3.0, [proj(*p) for p in t], cls))
    items.sort(key=lambda r: r[0])
    if items:
        ax.add_collection(PolyCollection([uv for _, uv, _ in items],
                                         facecolors=[COL[c] for _, _, c in items],
                                         edgecolor="#3c444b", linewidth=0.06, alpha=0.93))
    if show_env:
        env = [[proj(*p) for p in t] for t in gd.get("env", [])]
        if env:
            ax.add_collection(PolyCollection(env, facecolor="#333333",
                                             edgecolor="none", alpha=0.05))
    us, vs = [], []
    for _, uv, _ in items:
        for u, v in uv:
            us.append(u); vs.append(v)
    if show_env:
        for t in gd.get("env", []):
            for p in t:
                u, v = proj(*p)
                us.append(u); vs.append(v)
    if state == "working":
        for t in gd.get("suppext", []):
            for p in t:
                u, v = proj(*p)
                us.append(u); vs.append(v)
    if us:
        m = 25
        ax.set_xlim(min(us) - m, max(us) + m)
        ax.set_ylim(min(vs) - m, max(vs) + m)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, linewidth=0.5)
    ax.tick_params(labelsize=7)


def facts(layout, Dp, wkey):
    c = CASES[(layout, Dp, wkey, "belt")]
    ch = CASES[(layout, Dp, wkey, "hub")]
    pos = c.get("r_c", c.get("u"))
    return (f"{layout} Ø{Dp} {wkey}: ось/центр={pos:.0f} мм; зазор контура={c['gap_to_contour']:.0f} мм\n"
            f"зона лифта ±{ch['lift_margin']:+.1f} (hub) / {c['lift_margin']:+.1f} мм (belt); "
            f"полоса(belt)={c['band_mm']:.0f} (нужно 60); сектор АКБ ±{c['battery_half_tan']:.0f}\n"
            f"опора колёс A={c['supp_transport']['area_m2']} м²; "
            f"с выдвинутыми аутригерами A={c['supp_working']['area_m2']} м² (ход 300)")


# ================= 3-в-1 виды: 4 листа (схема x состояние) =================
for layout in ("R", "P"):
    for state in ("transport", "working"):
        fig, axes = plt.subplots(3, 3, figsize=(17, 15), dpi=105)
        for i, Dp in enumerate((600, 620, 640)):
            g = f"{layout}{Dp}_D152"
            draw(axes[i][0], g, "top", state)
            draw(axes[i][1], g, "side", state, show_env=True)
            draw(axes[i][2], g, "iso", state, show_env=True)
            axes[i][0].set_title(f"Ø{Dp} сверху", fontsize=11)
            axes[i][1].set_title(f"Ø{Dp} сбоку (конверт 1900)", fontsize=11)
            axes[i][2].set_title(f"Ø{Dp} изометрия", fontsize=11)
            axes[i][0].set_xlabel("X"); axes[i][0].set_ylabel("Y")
            axes[i][1].set_xlabel("X"); axes[i][1].set_ylabel("Z")
            axes[i][2].set_xlabel("u"); axes[i][2].set_ylabel("v")
            axes[i][0].text(0.0, -0.30, facts(layout, Dp, "D152"),
                            transform=axes[i][0].transAxes, fontsize=8, ha="left",
                            family="monospace",
                            bbox=dict(boxstyle="round", fc="#eef3f8", ec="#4a5259", lw=0.7))
        title_layout = ("схема R: оси радиальные, колёса 0/90/180/270°, руки одной руки (LLLL)"
                        if layout == "R" else
                        "схема P: оси параллельные || Y, колёса FL/FR/RL/RR на диагоналях, X-раскладка")
        title_state = ("ТРАНСПОРТНОЕ состояние: аутригеры убраны, площадок нет"
                       if state == "transport" else
                       "РАБОЧЕЕ состояние: телескопические рычаги выдвинуты (ход 300), точки опоры на полу")
        fig.suptitle(f"v5 {title_layout}\n{title_state}", fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        name = f"v5_{layout}_{state}.png"
        fig.savefig(name)
        plt.close(fig)
        print("PNG:", name)

# ================= сетка 6x3 сверху (транспорт) =================
fig, axes = plt.subplots(6, 3, figsize=(15, 26), dpi=100)
for i, layout in enumerate(("R", "P")):
    for j0, Dp in enumerate((600, 620, 640)):
        for k, wkey in enumerate(("D152", "D203", "D230")):
            g = f"{layout}{Dp}_{wkey}"
            ax = axes[i * 3 + j0][k]
            draw(ax, g, "top", "transport")
            ch = CASES[(layout, Dp, wkey, "hub")]
            cb = CASES[(layout, Dp, wkey, "belt")]
            ok = ch["pass"] or cb["pass"]
            badge = (f"hub: лифт{ch['lift_margin']:+.0f} | "
                     f"belt: лифт{cb['lift_margin']:+.0f}, полоса{cb['band_margin']:+.0f}")
            ax.set_title(f"{layout} Ø{Dp} / {wkey}  [{badge}]", fontsize=9,
                         color=("#1a6b2e" if ok else "#a11"))
            ax.set_xlabel("X", fontsize=8); ax.set_ylabel("Y", fontsize=8)
fig.suptitle("v5: все 18 комбинаций, вид сверху, ТРАНСПОРТНОЕ состояние "
             "(площадки аутригеров не показаны)\nзапасы в мм: лифт = до зоны Ø240, "
             "полоса = до требуемых 60 мм (belt)", fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig("v5_top_grid.png")
print("PNG: v5_top_grid.png")

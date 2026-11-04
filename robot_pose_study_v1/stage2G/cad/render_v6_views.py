# -*- coding: utf-8 -*-
"""Виды v6 (задача 9): сверху / сбоку / В РАЗРЕЗЕ с обозначением реальных пересечений.
 v6_variant_P1.png — вариант P1 (схемема P, D152, Ø640, лифт Z-разнесён), разрез x=y;
 v6_variant_R2.png — вариант R2 (схема R, реальный модуль CTM1000-10R, Ø620), разрез y=0;
 v6_baseline_section.png — базис v5 P640/D203 (лифт во всю высоту): пересечения красным.
"""
import json
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection

DATA = json.load(open("stage2g_v6_views_data.json"))
TRI, SECT, RED = DATA["tris"], DATA["sections"], DATA["intersections"]
S2 = math.sqrt(2.0)
COL = {"body": "#9aa5b1", "wheels": "#2f6fdb", "lift": "#7fd3e8", "zones": "#7fd3e8",
       "pods": "#c9a227", "arms": "#e8944a", "pads": "#b3591d"}
SOLID = ("body", "wheels", "lift", "zones", "pods", "arms", "pads")
SOLID_T = ("body", "wheels", "lift", "zones", "pods")   # транспорт: рычаги/площадки не показаны


def draw_view(ax, gname, view, show_env=True):
    gd = TRI[gname]
    if view == "top":
        proj, depth = lambda x, y, z: (x, y), lambda x, y, z: z
    else:
        proj, depth = lambda x, y, z: (x, z), lambda x, y, z: y
    items = []
    for cls in SOLID_T:
        for t in gd.get(cls, []):
            items.append((sum(depth(*p) for p in t) / 3.0, [proj(*p) for p in t], cls))
    items.sort(key=lambda r: r[0])
    if items:
        ax.add_collection(PolyCollection([uv for _, uv, _ in items],
                                         facecolors=[COL[c] for _, _, c in items],
                                         edgecolor="#3c444b", linewidth=0.06, alpha=0.93))
    if show_env and "env" in gd:
        env = [[proj(*p) for p in t] for t in gd["env"]]
        ax.add_collection(PolyCollection(env, facecolor="#333333", edgecolor="none", alpha=0.05))
    us, vs = [], []
    for _, uv, _ in items:
        for u, v in uv:
            us.append(u); vs.append(v)
    if show_env:
        for t in gd.get("env", []):
            for p in t:
                u, v = proj(*p)
                us.append(u); vs.append(v)
    if us:
        m = 30
        ax.set_xlim(min(us) - m, max(us) + m)
        ax.set_ylim(min(vs) - m, max(vs) + m)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, linewidth=0.5)
    ax.tick_params(labelsize=7)


def draw_section(ax, key, title, subtitle):
    s = SECT[key]
    kind = s["plane"]
    proj = (lambda x, y, z: ((x + y) / S2, z)) if kind == "diag" else (lambda x, y, z: (x, z))
    segs, cols = [], []
    for cls, lines in s["lines"].items():
        for ln in lines:
            pts = [proj(*p) for p in ln]
            for a, b in zip(pts, pts[1:]):
                segs.append([a, b])
                cols.append(COL.get(cls, "#666"))
    ax.add_collection(LineCollection(segs, colors=cols, linewidths=1.3))
    for ln in RED.get(key, []):
        pts = [proj(*p) for p in ln]
        for a, b in zip(pts, pts[1:]):
            ax.add_collection(LineCollection([[a, b]], colors="#d00000", linewidths=3.0))
    nred = len(RED.get(key, []))
    us = [u for seg in segs for u, _ in seg]
    vs = [v for seg in segs for _, v in seg]
    if us:
        m = 40
        ax.set_xlim(min(us) - m, max(us) + m)
        ax.set_ylim(min(vs) - m, max(vs) + m)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, linewidth=0.5)
    ax.tick_params(labelsize=7)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("u" + (" = (x+y)/√2" if kind == "diag" else " = x"))
    ax.set_ylabel("Z")
    ax.text(0.02, 0.02, subtitle + ("\nКРАСНЫМ: реальные пересечения" if nred else
                                    "\nреальных пересечений нет"),
            transform=ax.transAxes, fontsize=8, va="bottom",
            bbox=dict(boxstyle="round", fc="#fff7e6", ec="#c90", lw=0.6))


# ================= P1 =================
fig, axes = plt.subplots(1, 3, figsize=(19, 8), dpi=105)
draw_view(axes[0], "P640_D152_Z", "top")
axes[0].set_title("P1 сверху (транспорт: площадки не показаны)", fontsize=11)
axes[0].set_xlabel("X"); axes[0].set_ylabel("Y")
draw_view(axes[1], "P640_D152_Z", "side")
axes[1].set_title("P1 сбоку (конверт 1900)", fontsize=11)
axes[1].set_xlabel("X"); axes[1].set_ylabel("Z")
draw_section(axes[2], "P1", "P1 разрез x=y (через колёса FL/RR)",
             "лифт: стойка Ø120 (z90-162) -> конус -> механизм Ø240 (z182+)\n"
             "зазор колесо-стойка 73.9 мм; путь сил: стойка->панель->рёбра->узлы колёс")
fig.suptitle("Вариант P1: схема P (параллельные оси, X-раскладка), Mecanum Ø152, Ø640, "
             "лифт Z-разнесён (транспортный габарит Ø640 x 1900)", fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.92))
fig.savefig("v6_variant_P1.png")
print("PNG: v6_variant_P1.png")

# ================= R2 =================
fig, axes = plt.subplots(1, 3, figsize=(19, 8), dpi=105)
draw_view(axes[0], "R620_CTM", "top")
axes[0].set_title("R2 сверху (транспорт: площадки не показаны)", fontsize=11)
axes[0].set_xlabel("X"); axes[0].set_ylabel("Y")
draw_view(axes[1], "R620_CTM", "side")
axes[1].set_title("R2 сбоку (конверт 1900)", fontsize=11)
axes[1].set_xlabel("X"); axes[1].set_ylabel("Z")
draw_section(axes[2], "R2", "R2 разрез y=0 (через модули 0°/180°)",
             "реальный модуль CTM1000-10R: Ø240 x 106.9, 25 кг, 74 Н·м\n"
             "лифт Ø240 во всю высоту (запас 48 мм); путь сил: лифт->фланец->панель->узлы")
fig.suptitle("Вариант R2: схема R (радиальные оси, 4 одинаковых модуля), "
             "RoboCT CTM1000-10R, Ø620 (транспортный габарит Ø620 x 1900)", fontsize=13)
fig.tight_layout(rect=(0, 0, 1, 0.92))
fig.savefig("v6_variant_R2.png")
print("PNG: v6_variant_R2.png")

# ================= базис =================
fig, ax = plt.subplots(figsize=(10, 9), dpi=110)
draw_section(ax, "BASE", "Базис v5: схема P, D203, Ø640, лифт во всю высоту z0-400",
             "пересечение колесо-лифт 147 см3 и ступица-лифт 5 см3")
fig.suptitle("Разрез базиса (без Z-разнесения): реальные пересечения колёс с зоной лифта",
             fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig("v6_baseline_section.png")
print("PNG: v6_baseline_section.png")

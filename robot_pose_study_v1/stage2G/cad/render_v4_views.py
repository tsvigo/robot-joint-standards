# -*- coding: utf-8 -*-
"""Виды круглой платформы v4:
 (1) stage2g_v4_top_grid.png  — 3x3: Ø600/620/640 x D152/D203/D230, вид сверху;
 (2) stage2g_v4_views.png     — Ø600/620/640 x (сверху/сбоку/изо), колёса D152.
Провайдер: stage2g_v4_round_tris.json (gname -> class -> tris). Painter's algorithm.
"""
import json, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

TRI = json.load(open("stage2g_v4_round_tris.json"))["groups"]
CMP = json.load(open("stage2g_v4_round_results.json"))
FEAS = {(int(v["Dp"]), v["wheel"]): v for v in CMP["variants"]}
S2 = math.sqrt(2)
COL = {"frame": "#9aa5b1", "zones": "#7fd3e8", "wheels": "#2f6fdb", "pads": "#e8944a"}
SOLID = ("pads", "frame", "zones", "wheels")

def cam_dir(elev, azim):
    e, a = math.radians(elev), math.radians(azim)
    return (math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e))

def draw(ax, gname, proj, depth, show_env=False, elev=0, azim=0):
    gd = TRI[gname]
    # 1) опорные контуры (нижний слой, полупрозрачные)
    supp = [ [proj(*p) for p in t] for cls in ("supp",) for t in gd.get(cls, []) ]
    if supp:
        ax.add_collection(PolyCollection(supp, facecolor="#44aa66",
                                         edgecolor="#2b7348", linewidth=0.3, alpha=0.45))
    # 2) основные тела: глобальная сортировка по глубине
    items = []
    for cls in SOLID:
        for t in gd.get(cls, []):
            items.append((sum(depth(*p) for p in t) / 3.0,
                          [proj(*p) for p in t], cls))
    items.sort(key=lambda r: r[0])
    polys = [uv for _, uv, _ in items]
    fcs = [COL[c] for _, _, c in items]
    if polys:
        ax.add_collection(PolyCollection(polys, facecolors=fcs,
                                         edgecolor="#3c444b", linewidth=0.06, alpha=0.93))
    # 3) транспортный конверт (только бок/изо, сверху последним слоем-контуром)
    if show_env:
        env = [[proj(*p) for p in t] for t in gd.get("env", [])]
        if env:
            ax.add_collection(PolyCollection(env, facecolor="#333333",
                                             edgecolor="none", alpha=0.05))
    us, vs = [], []
    for _, uv, _ in items:
        for u, vv in uv:
            us.append(u); vs.append(vv)
    if show_env:
        for t in gd.get("env", []):
            for p in t:
                u, vv = proj(*p)
                us.append(u); vs.append(vv)
    if us:
        m = 25
        ax.set_xlim(min(us) - m, max(us) + m)
        ax.set_ylim(min(vs) - m, max(vs) + m)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, linewidth=0.5)
    ax.tick_params(labelsize=7)

# ================= ЛИСТ 1: 3x3 сверху =================
fig, axes = plt.subplots(3, 3, figsize=(16, 16), dpi=105)
for i, Dp in enumerate((600, 620, 640)):
    for j, wk in enumerate(("D152", "D203", "D230")):
        g = f"R{Dp}_{wk}"
        v = FEAS[(Dp, wk)]
        ax = axes[i][j]
        draw(ax, g, lambda x, y, z: (x, y), lambda x, y, z: z)
        f = v["feasible"]
        badge = f"hub={'OK' if f['hub'] else 'НЕТ'}  belt={'OK' if f['belt'] else 'НЕТ'}  inline={'OK' if f['inline'] else 'НЕТ'}"
        ax.set_title(f"Ø{Dp} / {wk}  [{badge}]", fontsize=11,
                     color=("#1a6b2e" if f["hub"] else "#a11"))
        ax.set_xlabel("X, мм"); ax.set_ylabel("Y, мм")
        if i == 0 and j == 0:
            ax.text(0.02, 0.02, "оранжевые — площадки аутригеров (ход 300, параметр)\n"
                                "зелёный ромб — опорный контур с выдвинутыми аутригерами\n"
                                "(контур колёс — вершины на r_c, скрыт под панелью)",
                    transform=ax.transAxes, fontsize=8, va="bottom",
                    bbox=dict(boxstyle="round", fc="#fff7e6", ec="#c90", lw=0.6))
fig.suptitle("v4 круглая платформа: вид сверху, все 9 комбинаций (ниши = проекции колёс)", fontsize=14)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig("stage2g_v4_top_grid.png")
print("PNG1 saved")

# ================= ЛИСТ 2: 3 компоновки x 3 вида (D152) =================
fig, axes = plt.subplots(3, 3, figsize=(17, 15), dpi=105)
for i, Dp in enumerate((600, 620, 640)):
    g = f"R{Dp}_D152"
    v = FEAS[(Dp, "D152")]
    # сверху
    draw(axes[i][0], g, lambda x, y, z: (x, y), lambda x, y, z: z)
    axes[i][0].set_title(f"Ø{Dp} сверху", fontsize=11)
    axes[i][0].set_xlabel("X"); axes[i][0].set_ylabel("Y")
    # сбоку (взгляд по Y)
    draw(axes[i][1], g, lambda x, y, z: (x, z), lambda x, y, z: y, show_env=True)
    axes[i][1].set_title(f"Ø{Dp} сбоку (конверт 1900)", fontsize=11)
    axes[i][1].set_xlabel("X"); axes[i][1].set_ylabel("Z")
    # изо
    s30 = math.sin(math.radians(35.264)); c30 = math.cos(math.radians(35.264))
    draw(axes[i][2], g,
         lambda x, y, z: ((x - y) / S2, (x + y) / S2 * s30 + z * c30),
         lambda x, y, z: (x + y) / S2 * c30 - z * s30, show_env=True,
         elev=35.264, azim=45)
    axes[i][2].set_title(f"Ø{Dp} изометрия", fontsize=11)
    axes[i][2].set_xlabel("u"); axes[i][2].set_ylabel("v")
    # строка фактов
    facts = (f"колёса D152: A_опор={v['supp_wheel_m2']} м²; полоса привода={v['band_mm']} мм\n"
             f"батарейный сектор ±{int(v['battery_half_tan'])} мм; аутригер: ход 300 (параметр)\n"
             f"дверь 700: запас {v['door700_margin']:.0f} мм; круг — без требований к курсу")
    axes[i][0].text(0.0, -0.28, facts, transform=axes[i][0].transAxes,
                    fontsize=8, ha="left", family="monospace",
                    bbox=dict(boxstyle="round", fc="#eef3f8", ec="#4a5259", lw=0.7))
fig.suptitle("v4 круглая платформа Ø600/620/640 — сравнительные компоновки (Mecanum Ø152, аутригеры показаны с ходом 300)",
             fontsize=14)
fig.tight_layout(rect=(0, 0, 1, 0.965))
fig.savefig("stage2g_v4_views.png")
print("PNG2 saved")

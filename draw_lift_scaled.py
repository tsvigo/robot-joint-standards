"""Масштабная схема компоновки; размеры заданы в миллиметрах."""
from pathlib import Path
import os

ROOT = Path(__file__).resolve().parent
os.environ['MPLCONFIGDIR'] = str(ROOT / '.work' / 'matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

fig, ax = plt.subplots(figsize=(7, 13))
blue = '#1677c8'
def box(x, z, w, h, color):
    ax.add_patch(Rectangle((x, z), w, h, facecolor=color,
                           edgecolor='#39444d', linewidth=0.9))

span = 549.9090833947009
spacing = 129.8
foot = 310  # Условно: пластина 262 + балка 40 + подошва 8.
height = 1600
for cx in (-221.50454169735046, 221.50454169735046):
    box(cx-53.45, 0, 106.9, 240, '#343c43')
    for z in range(10, 220, 30):
        ax.plot([cx-48, cx+48], [z, z+20], color='#69747e', lw=2)
box(-120, 0, 240, 240, '#293137')
for z in range(10, 220, 30):
    ax.plot([-110, 110], [z, z+20], color='#69747e', lw=2)
box(-163, 16, 326, 8, '#b5bec5')
for x in (-208, 138):
    box(x, 252, 70, 10, '#cbd1d5')
box(-138, 252, 276, 10, '#cbd1d5')
box(-173, 262, 346, 40, blue)
for x in (-spacing/2, spacing/2):
    box(x-40, 302, 80, 8, blue)
    box(x-25, foot, 50, height, '#d7dde2')
    sign = -1 if x < 0 else 1
    ax.add_patch(Polygon([(x+sign*25, foot), (x+sign*65, foot),
                          (x+sign*25, foot+80)], facecolor=blue,
                         edgecolor='#39444d', lw=0.9))
    ax.plot([x, x], [302, foot+height+115], '--', color='#65747e', lw=.7)
box(-89.9, foot+height, 179.8, 30, blue)

def hdim(a, b, z, label):
    ax.annotate('', (a, z), (b, z), arrowprops=dict(arrowstyle='<->', lw=.9))
    ax.plot([a, a], [z-15, z+15], color='#444', lw=.8)
    ax.plot([b, b], [z-15, z+15], color='#444', lw=.8)
    ax.text((a+b)/2, z+19, label, ha='center', va='bottom', fontsize=11)

hdim(-spacing/2, spacing/2, 2010, '129,8 мм')
hdim(-span/2, span/2, -75, '549,9 мм')
ax.annotate('', (-340, foot), (-340, foot+height),
            arrowprops=dict(arrowstyle='<->', lw=.9))
for z in (foot, foot+height):
    ax.plot([-355, -90], [z, z], color='#999', lw=.6)
ax.text(-360, foot+height/2, '1600 мм — высота направляющих',
        rotation=90, ha='center', va='center', fontsize=11)
ax.annotate('Подошвы направляющих', (90, foot), (190, 500),
            fontsize=9, arrowprops=dict(arrowstyle='-', lw=.7))
ax.plot([-310, 310], [0, 0], color='#888', lw=.7)
ax.set(xlim=(-440, 440), ylim=(-140, 2080))
ax.set_aspect('equal', adjustable='box')
ax.axis('off')
fig.suptitle('Направляющие: межосевое расстояние 129,8 мм', fontsize=13, y=.985)
fig.text(.5, .955, 'Вид в плоскости направляющих · единый масштаб X/Z',
         ha='center', fontsize=10)
fig.text(.5, .045, '549,9 / 1600 = 0,344 · Ширина стоек в этой проекции — 50 мм',
         ha='center', fontsize=9)
fig.text(.5, .027, 'Детали основания условные. Балка 40 мм и подошва 8 мм — допущения.',
         ha='center', fontsize=8)
fig.text(.5, .012, 'Это схема компоновки, не расчёт прочности.', ha='center', fontsize=8)
fig.subplots_adjust(top=.93, bottom=.075, left=.05, right=.95)
fig.savefig(ROOT / 'lift_129_8_scaled.png', dpi=160, facecolor='white')
fig.savefig(ROOT / 'lift_129_8_scaled.svg', facecolor='white')
print('Сохранены PNG и SVG. Масштаб X/Z:', ax.get_aspect())

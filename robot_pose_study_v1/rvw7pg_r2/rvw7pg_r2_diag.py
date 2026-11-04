# -*- coding: utf-8 -*-
"""Диагностика: какая часть модуля RVW-7PG пересекает панель (read-only)."""
import FreeCAD as App
import Part

WHEEL_D, WHEEL_B = 178.0, 96.0
RW = 262.0
MAIN_L, VALVE_L = 113.0, 11.0
D_MOTOR, L_MOTOR = 80.0, 100.0
Z_AX = WHEEL_D / 2
x_wheel0 = RW - WHEEL_B / 2
x_body0 = x_wheel0 - (MAIN_L - WHEEL_B)
x_valve0 = x_body0 - VALVE_L

panel = Part.makeCylinder(310.0, 40.0, App.Vector(0, 0, 50))
half_w = WHEEL_B / 2 + 15.0
x_notch0 = x_valve0 - L_MOTOR - 10.0
panel2 = panel
# пазы по 4 осям напрямую (без rotate)
x1 = 361.0
boxes = [
    ("+X", App.Vector(x_notch0, -half_w, 40),
     Part.makeBox(x1 - x_notch0, 2 * half_w, 60.0,
                  App.Vector(x_notch0, -half_w, 40))),
    ("+Y", App.Vector(-half_w, x_notch0, 40),
     Part.makeBox(2 * half_w, x1 - x_notch0, 60.0,
                  App.Vector(-half_w, x_notch0, 40))),
    ("-X", App.Vector(-x1, -half_w, 40),
     Part.makeBox(x1 - x_notch0, 2 * half_w, 60.0,
                  App.Vector(-x1, -half_w, 40))),
    ("-Y", App.Vector(-half_w, -x1, 40),
     Part.makeBox(2 * half_w, x1 - x_notch0, 60.0,
                  App.Vector(-half_w, -x1, 40))),
]
for name, pnt, box in boxes:
    bb = box.BoundBox
    before = panel2.Volume
    panel2 = panel2.cut(box)
    print(f"  паз {name}: бокс x[{bb.XMin:.0f},{bb.XMax:.0f}] "
          f"y[{bb.YMin:.0f},{bb.YMax:.0f}] -> убрано "
          f"{(before - panel2.Volume)/1e6:.2f} млн мм3")
print("вырезано 4 паза: панель до/после, мм3:", round(panel.Volume, 0),
      round(panel2.Volume, 0))
print("панель до/после выреза, мм3:", round(panel.Volume, 0),
      round(panel2.Volume, 0))
parts = {
    "wheel": Part.makeCylinder(WHEEL_D / 2, WHEEL_B,
                               App.Vector(x_wheel0, 0, Z_AX), App.Vector(1, 0, 0)),
    "body": Part.makeCylinder(FLANGE_D / 2 if False else 70, MAIN_L - WHEEL_B,
                              App.Vector(x_body0, 0, Z_AX), App.Vector(1, 0, 0)),
    "valve": Part.makeCylinder(20, VALVE_L,
                               App.Vector(x_valve0, 0, Z_AX), App.Vector(1, 0, 0)),
    "motor": Part.makeCylinder(D_MOTOR / 2, L_MOTOR,
                               App.Vector(x_valve0 - L_MOTOR, 0, Z_AX),
                               App.Vector(1, 0, 0)),
}
for n, p in parts.items():
    c = panel2.common(p)
    v = c.Volume if c else 0.0
    d = p.distToShape(panel2)[0]
    extra = ""
    if c and v > 0:
        b = c.BoundBox
        extra = (f" | зона пересечения x[{b.XMin:.0f},{b.XMax:.0f}] "
                 f"y[{b.YMin:.0f},{b.YMax:.0f}] z[{b.ZMin:.0f},{b.ZMax:.0f}]")
    print(f"{n}: общий объём с панелью {v:.0f} мм3, зазор {d:.1f} мм, "
          f"bbox x[{p.BoundBox.XMin:.0f},{p.BoundBox.XMax:.0f}] "
          f"z[{p.BoundBox.ZMin:.0f},{p.BoundBox.ZMax:.0f}]{extra}")
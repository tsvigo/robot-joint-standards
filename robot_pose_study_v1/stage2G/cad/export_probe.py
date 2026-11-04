# -*- coding: utf-8 -*-
"""Локализация сегфола STEP-экспорта в freecadcmd 1.1.4.
Каждый шаг логируется с flush — невыполненный маркер = виновник."""
import os
import FreeCAD as App
import Part

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

LOG = open(os.path.join(d, "export_probe_log.txt"), "a", buffering=1)
LOG.write("--- probe start ---\n")

doc = App.newDocument("probe")
a = doc.addObject("Part::Box", "A")
a.Length, a.Width, a.Height = 40.0, 30.0, 20.0
b = doc.addObject("Part::Box", "B")
b.Length, b.Width, b.Height = 10.0, 10.0, 10.0
b.Placement = App.Placement(App.Vector(100, 0, 0), App.Rotation())
doc.recompute()

shapes = [a.Shape, b.Shape]

LOG.write("A: makeCompound+Shape.exportStep ...\n")
Part.makeCompound(shapes).exportStep(os.path.join(d, "probe_shape.step"))
LOG.write("A ok (Shape.exportStep)\n")

LOG.write("B: Part.export ...\n")
Part.export([a, b], os.path.join(d, "probe_part.step"))
LOG.write("B ok (Part.export)\n")

LOG.write("C: Import.export ...\n")
import Import
Import.export([a, b], os.path.join(d, "probe_import.step"))
LOG.write("C ok (Import.export)\n")

LOG.write("PROBE_RESULT: ALL_PASS\n")
LOG.close()

import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(d, "stage2g_v4_round.FCStd"))
names = [o.Name for o in doc.Objects]
print("total:", len(names))
print("body-like:", [n for n in names if "body" in n.lower()][:12])
print("panelC-like:", [n for n in names if "panelC" in n][:8])
print("R600_D152*:", [n for n in names if n.startswith("R600_D152")][:30])

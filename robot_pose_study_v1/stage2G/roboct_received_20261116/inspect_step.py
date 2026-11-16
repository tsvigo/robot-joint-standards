"""Чтение STEP изготовителя без изменения рабочего CAD-файла."""
import FreeCAD
import Part
import json
from pathlib import Path

path=str(Path(__file__).resolve().parent/'CTM1000-10R V1.2  3D.STEP')
shape=Part.read(path)
b=shape.BoundBox
info={'valid':shape.isValid(),'solid_count':len(shape.Solids),
      'bbox_mm':[b.XMin,b.YMin,b.ZMin,b.XMax,b.YMax,b.ZMax],
      'dimensions_mm':[b.XLength,b.YLength,b.ZLength],
      'volume_mm3':shape.Volume,
      'note':'Объём и геометрический центроид STEP не определяют реальную массу и центр масс многоматериальной сборки.'}
print('STEP_GEOMETRY',json.dumps(info,ensure_ascii=False),flush=True)
with open(Path(__file__).resolve().parent/'step_geometry.json','w') as f:
    json.dump(info,f,ensure_ascii=False,indent=2)

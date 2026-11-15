"""Линейное упругое сравнение плиты и крестовины; единицы Н, мм, МПа.

Структурированная сетка C3D20R, CalculiX. Не расчёт безопасности робота.
Отверстия и сварные швы в сетке не моделируются; массы отверстий учтены отдельно.
"""
from pathlib import Path
import argparse
import json
import math
import os
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parent
RHO = 7.85e-6  # кг/мм³, принятая сталь
CAD_FRAME_VOLUME = 461080.4959209482
PAD_VOLUME = 2 * 158 * 114 * 8

def axis(bounds, spacing):
    values = sorted(set(bounds))
    out = []
    for a, b in zip(values, values[1:]):
        n = math.ceil((b-a)/spacing)
        out.extend(a+(b-a)*i/n for i in range(n))
    return np.array(out+[values[-1]])

def make_mesh(kind, thickness, spacing, wide_support=False):
    xs = axis([-173,-138,-79,-30,-26,0,26,30,79,138,173], spacing)
    ys = axis([-173,-138.9,-138,-89.9,-39.9,-30,-26,-24.9,
               0,24.9,26,30,39.9,89.9,138,138.9,173], spacing)
    t = 40 if kind == 'cross' else thickness
    zb = [0,2,4,12,20,28,36,38,40,44,48] if kind == 'cross' else [0,t/2,t,t+4,t+8]
    zs = np.array(zb)
    nodes, ids, elements, volumes, loads = [], {}, [], [], []
    def node(p):
        key = tuple(round(float(v),8) for v in p)
        if key not in ids:
            ids[key] = len(nodes)+1
            nodes.append(key)
        return ids[key]
    for i in range(len(xs)-1):
      x0,x1 = xs[i:i+2]; x=(x0+x1)/2
      for j in range(len(ys)-1):
        y0,y1=ys[j:j+2]; y=(y0+y1)/2
        pad = abs(x)<79 and (24.9<abs(y)<138.9)
        for k in range(len(zs)-1):
          z0,z1=zs[k:k+2]; z=(z0+z1)/2
          if z<t:
            if kind=='cross':
              main=abs(x)<30 and (abs(x)>26 or z<4 or z>36)
              arm=abs(x)>30 and abs(y)<30 and (abs(y)>26 or z<4 or z>36)
              solid=main or arm
            else: solid=True
          else: solid=pad
          if not solid: continue
          corners=np.array([(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
                            (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)])
          edges=[(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                 (0,4),(1,5),(2,6),(3,7)]
          conn=[node(p) for p in corners]+[node((corners[a]+corners[b])/2) for a,b in edges]
          elements.append(conn);volumes.append((x1-x0)*(y1-y0)*(z1-z0))
          if abs(z1-(t+8))<1e-6 and abs(x)<30 and 39.9<abs(y)<89.9:
              loads.append((len(elements),(x1-x0)*(y1-y0),x,y))
    nodes=np.array(nodes)
    halfwidth=138 if wide_support else 30
    fixed=[n+1 for n,(x,y,z) in enumerate(nodes) if abs(z)<1e-7 and
           ((abs(x)>=138-1e-6 and abs(y)<=halfwidth+1e-6) or
            (abs(y)>=138-1e-6 and abs(x)<=halfwidth+1e-6))]
    assert fixed and loads
    a=np.array([row[1:] for row in loads])
    assert abs(a[:,0].sum()-6000)<1e-4
    pressure=[1000/a[:,0].sum()*np.ones(len(a)),
              100000*a[:,1]/np.sum(a[:,0]*a[:,1]**2),
              100000*a[:,2]/np.sum(a[:,0]*a[:,2]**2)]
    totals=[]
    for p in pressure:
        totals.append([float(np.sum(p*a[:,0])),float(np.sum(p*a[:,0]*a[:,1])),
                       float(np.sum(p*a[:,0]*a[:,2]))])
    return nodes,elements,np.array(volumes),fixed,loads,pressure,totals

def write_inp(path, mesh):
    nodes,elements,volumes,fixed,loads,pressure,totals=mesh
    with path.open('w') as f:
        f.write('*HEADING\nPlate versus cross, linear elasticity N mm MPa\n*NODE,NSET=NALL\n')
        for n,p in enumerate(nodes,1): f.write(f'{n},'+','.join(f'{v:.9g}' for v in p)+'\n')
        f.write('*ELEMENT,TYPE=C3D20R,ELSET=EALL\n')
        for e,conn in enumerate(elements,1):
            f.write(str(e)+','+','.join(map(str,conn[:15]))+',\n')
            f.write(','.join(map(str,conn[15:]))+'\n')
        f.write('*MATERIAL,NAME=STEEL\n*ELASTIC\n210000,0.3\n*SOLID SECTION,ELSET=EALL,MATERIAL=STEEL\n')
        f.write('*NSET,NSET=FIXED\n')
        for i in range(0,len(fixed),15):f.write(','.join(map(str,fixed[i:i+15]))+'\n')
        f.write('*BOUNDARY\nFIXED,1,3\n')
        for p in pressure:
            f.write('*STEP\n*STATIC\n*DLOAD,OP=NEW\n')
            for row,value in zip(loads,p):f.write(f'{row[0]},P2,{value:.10g}\n')
            f.write('*NODE PRINT,NSET=NALL\nU\n*NODE PRINT,NSET=FIXED\nRF\n*EL PRINT,ELSET=EALL\nS\n*END STEP\n')

def parse_dat(path):
    # Последовательное чтение структурированного вывода, без вывода всего журнала.
    out={};mode=None;current=None
    with path.open() as f:
      for line in f:
        text=line.strip()
        if text.startswith(('displacements (','stresses (','forces (')):
            mode='u' if text.startswith('displacements') else ('s' if text.startswith('stresses') else 'rf')
            step=int(round(float(text.split()[-1])))
            current=out.setdefault(step,{'u':[],'s':[],'rf':[]})
            continue
        if not text or current is None:continue
        try: vals=[float(v.replace('D','E')) for v in text.split()]
        except ValueError:mode=None;continue
        if mode=='u' and len(vals)==4:current['u'].append(vals)
        elif mode=='rf' and len(vals)==4:current['rf'].append(vals)
        elif mode=='s' and len(vals)==8:current['s'].append(vals)
    assert set(out)=={1,2,3},out.keys()
    return out

def vm(s):
    x,y,z,xy,xz,yz=s.T
    return np.sqrt(.5*((x-y)**2+(y-z)**2+(z-x)**2)+3*(xy**2+xz**2+yz**2))

def metrics(u,s,volumes,nodes,t):
    stress=vm(s[:,2:]); w=volumes[s[:,0].astype(int)-1]
    order=np.argsort(stress);cum=np.cumsum(w[order])/w.sum()
    top=abs(nodes[:,2]-(t+8))<1e-6
    loaded=top & (abs(nodes[:,0])<=30+1e-6) & (abs(nodes[:,1])>=39.9-1e-6) & (abs(nodes[:,1])<=89.9+1e-6)
    assert np.array_equal(u[:,0],np.arange(1,len(nodes)+1))
    return {'pad_displacement_mm':float(np.linalg.norm(u[loaded,1:],axis=1).max()),
            'global_displacement_mm':float(np.linalg.norm(u[:,1:],axis=1).max()),
            'stress_vm_p99_mpa':float(stress[order[np.searchsorted(cum,.99)]]),
            'stress_vm_max_mpa':float(stress.max())}

def run(kind,thickness,spacing,wide_support=False):
    name=f'{kind}_{thickness:g}_h{spacing:g}'+('_wide' if wide_support else '')
    folder=ROOT/'runs'/name;folder.mkdir(parents=True,exist_ok=True)
    mesh=make_mesh(kind,thickness,spacing,wide_support)
    nodes,elements,volumes,fixed,loads,pressure,totals=mesh
    print(name,'nodes',len(nodes),'elements',len(elements),flush=True)
    write_inp(folder/'job.inp',mesh)
    log=folder/'solver.log'
    with log.open('w') as f:
        result=subprocess.run(['ccx','-i','job'],cwd=folder,stdout=f,stderr=subprocess.STDOUT,
                              env={**os.environ,'OMP_NUM_THREADS':'4'},timeout=240)
    if result.returncode:
        print('Размер журнала:',log.stat().st_size,'байт',flush=True)
        from collections import deque
        print(''.join(deque(log.open(),maxlen=35)))
        raise RuntimeError(f'CalculiX failed: {result.returncode}')
    raw=parse_dat(folder/'job.dat')
    output={}
    balance={}
    t=40 if kind=='cross' else thickness
    for k,label in [(1,'Fz_1000N'),(2,'My_100Nm'),(3,'Mx_100Nm')]:
        output[label]=metrics(np.array(raw[k]['u']),np.array(raw[k]['s']),volumes,nodes,t)
        rf=np.array(raw[k]['rf'])
        assert len(rf)==len(fixed)
        xyz=nodes[rf[:,0].astype(int)-1]
        force=rf[:,1:].sum(axis=0);moment=np.cross(xyz,rf[:,1:]).sum(axis=0)
        balance[label]={'force_N':force.tolist(),'moment_Nmm':moment.tolist()}
        expected=[np.array([0,0,1000]),np.zeros(3),np.zeros(3)][k-1]
        expected_m=[np.zeros(3),np.array([0,-100000,0]),np.array([100000,0,0])][k-1]
        assert np.linalg.norm(force-expected)<0.2,(label,force)
        assert np.linalg.norm(moment-expected_m)<20,(label,moment)
    # Суперпозиция тензоров и перемещений, затем вычисление Мизеса.
    u=np.array(raw[1]['u']);s=np.array(raw[1]['s'])
    u[:,1:]+=np.array(raw[2]['u'])[:,1:];s[:,2:]+=np.array(raw[2]['s'])[:,2:]
    output['Fz_plus_My']=metrics(u,s,volumes,nodes,t)
    if kind=='cross': volume=CAD_FRAME_VOLUME
    else:
        # В плите: 8 отверстий Ø9 для крепежа и 8 Ø22 под гайки направляющих подвесок.
        volume=(346**2-8*math.pi*(4.5**2+11**2))*thickness
    report={'name':name,'kind':kind,'thickness_mm':t,'mesh_spacing_mm':spacing,
            'support':'wide_ideal_clamp' if wide_support else 'same_contact_patches',
            'nodes':len(nodes),'elements':len(elements),'frame_mass_kg':volume*RHO,
            'frame_and_feet_mass_kg':(volume+PAD_VOLUME)*RHO,
            'mesh_volume_mm3':float(volumes.sum()),'load_resultants_N_Nmm':totals,
            'support_reaction_balance':balance,'results':output}
    (folder/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return report

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--kind',choices=['cross','plate'],default='cross')
    ap.add_argument('--thickness',type=float,default=8);ap.add_argument('--spacing',type=float,default=12)
    ap.add_argument('--wide-support',action='store_true')
    args=ap.parse_args();run(args.kind,args.thickness,args.spacing,args.wide_support)

"""Сводка воспроизводимого сравнения; исходная модель FreeCAD не изменяется."""
from pathlib import Path
import csv
import json
import os
import numpy as np
from compare import ROOT,make_mesh,parse_dat

os.environ['MPLCONFIGDIR']=str(ROOT/'.matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

selected=['cross_8_h8','plate_4_h12','plate_6_h12','plate_8_h8',
          'plate_10_h12','plate_12_h12','plate_16_h8','plate_8_h8_wide']
data=[]
for name in selected:
    r=json.loads((ROOT/'runs'/name/'result.json').read_text())
    r['label']='Крестовина 60×40×4' if r['kind']=='cross' else f"Плита {r['thickness_mm']:g} мм"
    if name.endswith('wide'):r['label']+=' — широкое закрепление'
    data.append(r)

convergence=[]
for name in ['cross_8','plate_8','plate_16']:
    a=json.loads((ROOT/'runs'/(name+'_h12')/'result.json').read_text())
    b=json.loads((ROOT/'runs'/(name+'_h8')/'result.json').read_text())
    for case in a['results']:
        convergence.append({'model':name,'case':case,
           'displacement_change_percent':100*abs(b['results'][case]['pad_displacement_mm']/a['results'][case]['pad_displacement_mm']-1),
           'p99_change_percent':100*abs(b['results'][case]['stress_vm_p99_mpa']/a['results'][case]['stress_vm_p99_mpa']-1),
           'peak_change_percent':100*abs(b['results'][case]['stress_vm_max_mpa']/a['results'][case]['stress_vm_max_mpa']-1)})
doc={'date':'2026-11-15','units':{'force':'N','moment':'N m','length':'mm','stress':'MPa','mass':'kg'},
     'source_cad':'/home/v/projects/робот/robot_pose_study_v1/stage2G/cad/robot_r2_roboct.FCStd',
     'material_assumed':{'E_MPa':210000,'nu':.3,'rho_kg_m3':7850},
     'models':data,'mesh_convergence':convergence}
(ROOT/'results.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2))
with (ROOT/'comparison.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['Вариант','Масса детали, кг','Масса с двумя подошвами, кг','Случай','Перемещение зоны нагрузки, мм','Мизес p99, МПа','Мизес максимум, МПа'])
    for r in data:
        for case,m in r['results'].items():
            w.writerow([r['label'],r['frame_mass_kg'],r['frame_and_feet_mass_kg'],case,m['pad_displacement_mm'],m['stress_vm_p99_mpa'],m['stress_vm_max_mpa']])

labels=['Крестовина','Плита 4','Плита 6','Плита 8','Плита 10','Плита 12','Плита 16']
fig,axes=plt.subplots(1,3,figsize=(14,5),layout='constrained')
case='Fz_plus_My'
for ax,key,title,unit in [(axes[0],'mass','Масса несущей детали','кг'),
                          (axes[1],'pad_displacement_mm','Перемещение опор направляющих','мм'),
                          (axes[2],'stress_vm_p99_mpa','Напряжения Мизеса — объёмный p99','МПа')]:
    vals=[r['frame_mass_kg'] if key=='mass' else r['results'][case][key] for r in data[:7]]
    ax.bar(range(7),vals,color=['#1765a5']+['#77838c']*6)
    ax.set_xticks(range(7),labels,rotation=45,ha='right');ax.set_ylabel(unit);ax.set_xlabel('Вариант несущей детали')
    ax.set_title(title,fontsize=10);ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2)
fig.suptitle('Крестовина и плоская плита: 1000 Н вниз + 100 Н·м вокруг Y',fontsize=14)
fig.text(.5,-.03,'Источник: расчёт CalculiX 2.21, C3D20R · 15.11.2026 · сталь принята · одинаковые идеальные зажимы',ha='center',fontsize=9)
fig.savefig(ROOT/'comparison.png',dpi=170,bbox_inches='tight');plt.close(fig)

# Поля перемещений в плане — видимая верхняя поверхность.
fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained')
for ax,name,title,kind,t in [(axes[0],'cross_8_h8','Крестовина 60×40×4','cross',40),
                            (axes[1],'plate_8_h8','Плита 8 мм','plate',8)]:
    nodes,*_=make_mesh(kind,t,8)
    raw=parse_dat(ROOT/'runs'/name/'job.dat')
    u=np.array(raw[1]['u'])[:,1:]+np.array(raw[2]['u'])[:,1:]
    visible={}
    for i,(x,y,z) in enumerate(nodes):
        key=(round(x,6),round(y,6))
        if key not in visible or z>nodes[visible[key],2]:visible[key]=i
    idx=np.array(list(visible.values()))
    chart=ax.scatter(nodes[idx,0],nodes[idx,1],c=np.linalg.norm(u[idx],axis=1),s=4,
                     vmin=0,vmax=.0133,cmap='viridis',rasterized=True)
    ax.set_aspect('equal');ax.set(xlabel='X, мм',ylabel='Y, мм',title=title,xlim=(-178,178),ylim=(-178,178))
fig.colorbar(chart,ax=axes,label='Полное перемещение |u|, мм',shrink=.8)
fig.suptitle('1000 Н + 100 Н·м вокруг Y · одинаковая цветовая шкала')
fig.savefig(ROOT/'displacement_fields.png',dpi=170);plt.close(fig)

maxdelta=max(c['displacement_change_percent'] for c in convergence)
maxp99=max(c['p99_change_percent'] for c in convergence)
maxpeak=max(c['peak_change_percent'] for c in convergence)
lines=['# Сравнение верхней плиты и крестовины','',
'Дата: 15.11.2026. Предварительное линейное упругое сравнение, не проверка безопасности робота.','',
'## Исходная геометрия','',
'Источник: `/home/v/projects/робот/robot_pose_study_v1/stage2G/cad/robot_r2_roboct.FCStd`.',
'Крестовина: профиль 60×40×4 мм, внешний габарит 346×346 мм; объём из CAD 461080,50 мм³.',
'Плита: плоский квадрат 346×346 мм; толщины 4, 6, 8, 10, 12, 16 мм. Рёбер и отбортовок нет.',
'В обе расчётные модели включены две общие подошвы 158×114×8 мм; их суммарная масса 2,262 кг.',
'Направляющие, их косынки, верхняя перемычка, колёса, болты и сварные швы не включены в массу сравниваемой детали.','',
'## Материал и нагрузки','',
'Принята одинаковая сталь: E=210000 МПа, ν=0,3, ρ=7850 кг/м³. Марка реального материала не задана.',
'Fz: суммарно 1000 Н вниз. My: суммарно 100 Н·м, создающие градиент вертикальных сил вдоль X.',
'Mx: суммарно 100 Н·м, создающие градиент сил вдоль Y. Комбинация Fz+My получена суперпозицией перемещений и тензоров напряжений.',
'Нагрузки распределены по двум прямоугольникам 60×50 мм на верхних поверхностях подошв, центры Y=±64,9 мм.',
'Это нормировочные пробы, а не утверждённые нагрузки от пациента. Собственный вес не приложен.',
'Горизонтальная сила и момент от неё на высоте 1600 мм отдельно не рассчитаны.','',
'## Закрепление и допущения','',
'Основное сравнение: одинаковые четыре зажатые площадки 35×60 мм на нижней стороне несущей детали. Они совпадают с пересечением крестовины и верхних монтажных пластин колёсных модулей.',
'Дополнительный благоприятный вариант плиты 8 мм: четыре идеально зажатых участка 35×276 мм, то есть вся доступная ширина существующих монтажных пластин.',
'Полное закрепление — идеализация; болты и реальный односторонний контакт не эквивалентны такому зажиму. Подвески и колесная рама считаются неподвижными.',
'Пересечения профилей и подошвы считаются идеально связанными. Жёсткость косынок и направляющих отдельно не моделируется: нагрузки вводятся непосредственно в подошвы.',
'В конечно-элементной сетке нет отверстий. Для массы крестовины использован реальный объём CAD; для массы плиты вычтены 8 отверстий Ø9 и 8 предполагаемых отверстий Ø22 под гайки подвески.',
'Поэтому жёсткость рассчитана для неперфорированных деталей, а масса — с указанными отверстиями. Это оптимистическое приближение жёсткости; локальные напряжения вокруг отверстий не оценены.',
'Плита и крестовина опираются нижней плоскостью на одну отметку; при замене высота подъёмника изменится на 40−t мм. Если нужен прежний уровень направляющих, потребуется отдельное решение по высоте.','',
'## Численные результаты','']
for r in data:
    lines.append('### '+r['label']);lines.append('')
    lines.append(f"Масса несущей детали: {r['frame_mass_kg']:.2f} кг; с двумя подошвами: {r['frame_and_feet_mass_kg']:.2f} кг.")
    for case,m in r['results'].items():
        lines.append(f"- {case}: |u| в зоне приложения нагрузки ≈ {m['pad_displacement_mm']:.4f} мм; Мизес p99 ≈ {m['stress_vm_p99_mpa']:.2g} МПа; абсолютный пик ≈ {m['stress_vm_max_mpa']:.2g} МПа.")
    lines.append('')
lines+=['## Проверка численного решения','',
'CalculiX 2.21, квадратичные объёмные элементы C3D20R. Сетки с максимальным шагом 12 и 8 мм проверены для крестовины и плит 8 и 16 мм.',
f'Максимальное изменение перемещений опор при сгущении: {maxdelta:.2f}%; объёмного p99 напряжений: {maxp99:.2f}%; абсолютного пика: {maxpeak:.1f}%.',
'Баланс реакций проверен по каждой отдельной нагрузке; невязка существенно меньше 0,1 Н по силе и 0,1 Н·м по моменту.',
'p99 — уровень, ниже которого находятся напряжения в 99% объёма расчётной модели, включая подошвы. Это не максимум и не критерий прочности.',
'Абсолютные пики чувствительны к сетке у границ зажимов и связей. По ним нельзя назначать допустимую нагрузку без уточнения соединений и дальнейшей сходимости.','',
'## Вывод','',
'При этих размерах и идеальных зажимах крестовина выгоднее плоской плиты по вертикальной жёсткости на единицу массы.',
'При почти равной массе плита 4 мм значительно податливее; плита 8 мм примерно вдвое тяжелее и всё ещё податливее при вертикальной и комбинированной пробе.',
'Плита 16 мм даёт сопоставимое вертикальное перемещение и меньшие расчётные напряжения, но весит примерно вчетверо больше самой крестовины.',
'По моменту My плита может иметь меньшее перемещение и напряжения: форма и направление нагрузки важны, общего превосходства по всем метрикам нет.',
'Следующий содержательный кандидат — тонкая плита с рёбрами либо отбортовками; для него требуется отдельная геометрия и расчёт.',
'Эти результаты не задают несущую способность, коэффициент запаса, усталостный ресурс или безопасность контакта с пациентом.','']
(ROOT/'report.md').write_text('\n'.join(lines))
print(json.dumps({'models':len(data),'mesh_max_displacement_change_percent':maxdelta,'mesh_max_p99_change_percent':maxp99,'mesh_max_peak_change_percent':maxpeak},ensure_ascii=False))

# -*- coding: utf-8 -*-
"""Этап 2Г v7.4, задачи 3–4 — stage2g_v7_4_budget_cm.py (только Python).

Задача 3: бюджет с лимитом больничного лифта (требование среды, v7.2):
    M_робота + M_перевозимой_нагрузки + M_людей_в_лифте <= 500 кг.
Раздельный учёт: каталог/проверенная геометрия | условные stored Blender |
параметрические сценарии Stage 2V | неизвестные. Остаток для НЕОПРЕДЕЛЁННЫХ
подсистем по сценариям — без назначения им вымышленных масс.
Параметрика приводов рук 105–433 кг: НЕ утверждена; используется только для
оценки риска превышения, с указанием происхождения/состава/рискa двойного учёта.

Задача 4: проверка преобразования координат Blender -> R2; оговорка
origin != геометрический ЦМ; параметрическая модель общего ЦМ (неизвестные —
переменные); чувствительность ЦМ; приоритет определения величин.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
with open(os.path.join(ROOT, "stage2V", "stage2v_blender_mass_dump.json"),
          encoding="utf-8") as f:
    D = json.load(f)
OBJS = D["objects"]
FLOOR, OX, OY = -5.6086, 3.608398, -1.86476
r2 = lambda p: (p[0] - OX, p[1] - OY, p[2] - FLOOR)
G = 9.81
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


# ============ 4a. Проверка преобразования координат Blender -> R2 ============
print("== 4a. Проверка преобразования координат Blender -> R2 ==")
p_base = r2(OBJS["base_link"]["matrix_world_translation"])
p_lw = r2(OBJS["wheel_FL_joint"]["matrix_world_translation"]) if \
    OBJS.get("wheel_FL_joint", {}).get("matrix_world_translation") else None
# колёса: joint-empties из ключ-фактов
kf_w = None
import contextlib
with open(os.path.join(ROOT, "model_key_facts.json"), encoding="utf-8") as f:
    kf = json.load(f)
wc = kf["wheels"]["centers"]
w_r2 = {k: [round(v[0] - OX, 4), round(v[1] - OY, 4), round(v[2] - FLOOR, 4)]
        for k, v in wc.items()}
t("T1_центр_базы", abs(p_base[0]) < 0.01 and abs(p_base[1]) < 0.01,
  f"origin базы в СК R2 = ({p_base[0]:.4f}, {p_base[1]:.4f}) — ≈центр (допуск 10 мм)")
xs = [v[0] for v in w_r2.values()]
ys = [v[1] for v in w_r2.values()]
sym_ok = (abs(min(xs) + max(xs)) < 0.01) and (abs(min(ys) + max(ys)) < 0.01)
t("T2_симметрия_колёс", sym_ok,
  f"колёса R2: X [{min(xs):.3f}..{max(xs):.3f}], Y [{min(ys):.3f}..{max(ys):.3f}] — "
  f"центр симметрии ≈0")
# ось лифта: world лифта -> (0,0,*)
lw = kf["lift"]["world"]
lift_xy = (lw[0] - OX, lw[1] - OY)
t("T3_ось_лифта", abs(lift_xy[0]) < 0.01 and abs(lift_xy[1]) < 0.01,
  f"ось лифта Blender -> ({lift_xy[0]:.4f}, {lift_xy[1]:.4f}) в СК R2 — совпадает с центром")
t("T4_пол_и_единицы", abs(kf["geometry"]["visual_bbox"]["min"][2] - FLOOR) < 0.01 and
  kf["scene"]["unit_system"] == "METRIC" and kf["scene"]["scale_length"] == 1.0,
  "пол Blender (−5.6086) -> z_R2=0 совпадает с нижней гранью визуала; "
  "сцена METRIC scale 1.0 — без пересчёта единиц")

# ============ 4b. Параметрическая модель ЦМ (неизвестные — переменные) ============
print("\n== 4b. Параметрическая модель ЦМ (НЕ подтверждённый ЦМ) ==")
# Известные/условные слагаемые (со статусами)
KNOWN = [
    {"m": 100.0, "z": 0.12, "x": 0.0, "y": 0.0,
     "назв": "RoboCT 4×25 (каталог, предв.)", "cls": "каталог"},
    {"m": 18.4, "z": 0.07, "x": 0.0, "y": 0.0,
     "назв": "панель Al с вырезами (CAD-объём)", "cls": "проверенная геометрия (материал не выбран)"},
    {"m": 21.711, "z": 1.06, "x": 0.18, "y": 0.0,
     "назв": "верх Blender stored (голова+руки+датчики)", "cls": "условная stored②③"},
]
# Неизвестные — ТОЛЬКО переменные
UNK = ["m_АКБ (и z_АКБ, x_АКБ, y_АКБ)", "m_панель_факт (материал/рёбра/крепления)",
       "m_подъёмник (после B) (z от хода лифта)", "m_аутригеры",
       "m_приводы_рук (верх, параметрика 105–433 НЕ принята)",
       "m_торс/каретка", "m_электроника/проводка/крепёж",
       "M_перевозимой_нагрузки", "M_людей_в_лифте"]
print("  слагаемые со статусами:", ", ".join(f"{k['назв']}={k['m']} кг" for k in KNOWN))
print("  переменные (без значений):", "; ".join(UNK))

def cm(parts):
    M = sum(p["m"] for p in parts)
    z = sum(p["m"] * p["z"] for p in parts) / M
    x = sum(p["m"] * p["x"] for p in parts) / M
    y = sum(p["m"] * p["y"] for p in parts) / M
    return round(M, 1), (round(x, 3), round(y, 3), round(z, 3))

M1, CM1 = cm(KNOWN)
M2, CM2 = cm(KNOWN[:2])             # без stored-верха
print(f"  частичный ЦМ (все 3 строки, {M1} кг): {CM1} м")
print(f"  частичный ЦМ (только R2-база, {M2} кг): {CM2} м")
# символьная запись для отчёта
param_cm = {
    "формула": "z_ЦМ = Σ(m_i·z_i) / Σ(m_i); x/y — аналогично; все m_i неизвестных — переменные",
    "в_частном_виде": "z_ЦМ = (100·0.12 + m_пан·0.07 + 21.711·1.06 + m_АКБ·z_АКБ + "
                      "m_подъём·z_подъём(t) + m_привод·z_привод + …) / "
                      "(140.1 + m_АКБ + m_подъём + m_привод + …)",
    "статус": "параметрическая модель; результат НЕ является подтверждённым ЦМ",
}
print(" ", param_cm["статус"])

# ============ 4c. Чувствительность ============
print("\n== 4c. Чувствительность ЦМ к неизвестным массам ==")
M_base = 140.111                       # 100 + 18.4 + 21.711
sens = []
for name, z_i in (("АКБ (внизу, z≈0.10)", 0.10),
                  ("панель (факт, z≈0.07)", 0.07),
                  ("подъёмник средний (z≈0.6)", 0.60),
                  ("приводы рук (верх, z≈1.06)", 1.06),
                  ("нагрузка/люди (на панели z≈0.15)", 0.15)):
    # ∂z_ЦМ/∂m_i при z_ЦМ частичном
    z0 = CM1[2]
    dzdm = (z_i - z0) / (M_base + 0.0)      # м на каждый кг добавленной массы
    sens.append({"подсистема": name, "dz_ЦМ_м_на_1кг": round(dzdm, 5),
                 "Δz_ЦМ_при_10кг_м": round(dzdm * 10, 4)})
    print(f"  {name}: ∂z/∂m = {dzdm*1000:+.2f} мм/кг; +10 кг -> Δz_ЦМ "
          f"{dzdm*10*1000:+.1f} мм")
prior = sorted(sens, key=lambda s: -abs(s["dz_ЦМ_м_на_1кг"]))
print("  приоритет определения (по |вклад| на кг):",
      " -> ".join(s["подсистема"].split(" (")[0] for s in prior))
sens_note = (f"больше всего на z_ЦМ влияют ПРОВОДЫ РУК (верх, z≈1.06) и ПОДЪЁМНИК; "
             f"АКБ/панель внизу снижают ЦМ (относительно частичного {CM1[2]:.3f}). "
             f"На x/y: АКБ и транспортная поза рук (вынос ±Y) — при несимметрии")
print(" ", sens_note)

# ============ 3. Бюджет 500 кг ============
print("\n== 3. Бюджет массы с лимитом 500 кг (больничный лифт, требование среды) ==")
LIMIT = 500.0
budget_scen = []


def scen(name, m_robot_parts, m_load, m_people, note):
    m_known = sum(p["m"] for p in m_robot_parts)
    rem = LIMIT - m_known - m_load - m_people
    row = {"сценарий": name,
           "части_робота": [p["назв"] for p in m_robot_parts],
           "M_робота_с_известным": round(m_known, 1),
           "M_нагрузки": m_load, "M_людей": m_people,
           "остаток_на_неопределённые_подсистемы_кг": round(rem, 1),
           "примечание": note}
    budget_scen.append(row)
    print(f"  {name}: робот {m_known} + нагр {m_load} + люд {m_people} "
          f"-> остаток НЕОПРЕДЕЛЁННЫМ: {rem:.1f} кг  [{note}]")


PAN_LOW = KNOWN[1]
PAN_HIGH = dict(PAN_LOW); PAN_HIGH["m"] = 53.4     # сталь-вариант
scen("S1 минимум: RoboCT+панель Al", [KNOWN[0], PAN_LOW], 0, 0,
     "только подтверждённое+CAD-объём; верх Blender НЕ включён")
scen("S1b: + панель сталь", [KNOWN[0], PAN_HIGH], 0, 0, "панель стали вместо Al")
scen("S2: + stored-верх Blender", [KNOWN[0], PAN_HIGH, KNOWN[2]], 0, 0,
     "верх условный (stored)")
scen("S3: S2 + нагрузка 100 кг (параметр)", [KNOWN[0], PAN_HIGH, KNOWN[2]], 100, 0,
     "нагрузка — ПАРАМЕТР, не факт")
scen("S4: S3 + 1 человек 80 кг (параметр)", [KNOWN[0], PAN_HIGH, KNOWN[2]], 100, 80,
     "люди — ПАРАМЕТР")
P_DRIVE_LIGHT = {"m": 105.0, "z": 1.06, "x": 0.18, "y": 0.0,
                 "назв": "приводы рук, нижн. сценарий параметрики (2V ②)",
                 "cls": "параметрическая оценка, НЕ факт"}
P_DRIVE_HEAVY = dict(P_DRIVE_LIGHT); P_DRIVE_HEAVY["m"] = 433.0
P_DRIVE_HEAVY["назв"] = "приводы рук, верхний сценарий параметрики (2V ②)"
scen("S5 риск: + приводы рук 105 кг (параметр)",
     [KNOWN[0], PAN_HIGH, KNOWN[2], P_DRIVE_LIGHT], 100, 80,
     "параметрика 2V НЕ утверждена; показана как риск")
scen("S6 риск: + приводы рук 433 кг (параметр)",
     [KNOWN[0], PAN_HIGH, KNOWN[2], P_DRIVE_HEAVY], 100, 80,
     "ПРЕВЫШЕНИЕ ЛИМИТА — критический риск при тяжёлой трактовке приводов")
s5 = budget_scen[-2]["остаток_на_неопределённые_подсистемы_кг"]
s6 = budget_scen[-1]["остаток_на_неопределённые_подсистемы_кг"]

# анализ риска параметрики приводов
arms_param = {
    "происхождение": "Stage 2V arm_mass_budget_v1 (оболочки Al/сталь 2-5 мм + "
                     "параметрические приводы k·T* 0.01–0.06 кг/(Н·м) + кабели)",
    "состав": "оболочки звеньев + узлы приводов (моторы+редукторы+подшипники) + кабели; "
              "доля приводов 93–96% итога",
    "статус": "НЕ приняты как утверждённые (2V: категория ② — сценарии light/nominal/heavy)",
    "двойной_учёт": "возможен: stored-массы рук (5.999+7.531 кг) уже могут содержать "
                    "часть узлов (происхождение stored неизвестно) — сложение "
                    "stored-рук и параметрики приводов дало бы двойной учёт; "
                    "допустима только замена, не суммирование, до выяснения состава",
    "риск_500": f"при включении параметрики: S5 (105 кг): остаток {s5} кг; "
                f"S6 (433 кг): остаток {s6} кг — "
                f"{'ПРЕВЫШЕНИЕ лимита' if s6 < 0 else 'без превышения'}",
}
print("\n== Анализ риска параметрики приводов рук (105–433 кг) ==")
for k, v in arms_param.items():
    print(f"  {k}: {v}")

out = {"coord_checks": CHECKS, "param_cm": param_cm,
       "partial_cm": {"все_строки": {"M_кг": M1, "ЦМ_м": CM1},
                      "только_R2": {"M_кг": M2, "ЦМ_м": CM2}},
       "sensitivity": sens, "sensitivity_note": sens_note,
       "budget_limit_500": budget_scen, "arms_parametric_risk": arms_param,
       "unknown_variables": UNK,
       "notes": ["Лимит 500 кг = больничный грузовой лифт (требование среды).",
                 "Параметры нагрузки/людей — не факты.",
                 "ЦМ параметрический; подтверждённым не является."]}
path = os.path.join(HERE, "stage2g_v7_4_budget_cm.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")

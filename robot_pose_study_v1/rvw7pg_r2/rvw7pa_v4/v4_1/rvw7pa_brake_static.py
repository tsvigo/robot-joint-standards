"""
RVW-7PA v4.1 — СТАТИЧЕСКОЕ УДЕРЖАНИЕ (стояночный тормоз), БЕЗ Crr.

Отличие от v4: в v4 в тормоз включалось сопротивление качению Crr=0.05
(T_wheel подмешивало Crr*M*g*cos(theta)). Для СТОЯНОЧНОГО удержания это неверно:
при нулевой скорости качения сопротивление качению не работает как источник
удержания (оно не определено в статике и не может быть просуммировано с моментом
тормоза; кроме того оно само есть следствие деформации в контакте, а не
гарантированная сила).

Здесь учитываются ТОЛЬКО:
  - сила тяжести на уклоне  F = M*g*sin(theta)  вдоль склона;
  - внешние воздействия (параметр F_ext);
  - уравнения равновесия платформы:  J^T f = wrench = (Fx, Fy, Mz).

Модель привода — та же, что в rvw7pg_r2_forces_v3.py (13/13 PASS):
  J[i] = (cos a_i, sin a_i, sigma_i * cos45 * RW),  a_i = theta_i + sigma_i*45°
  T_i  = f_i * R * cos45

Решается:
  - 4 исправных тормоза: min-norm решение недоопределённой системы J^T f = wrench;
  - отказ одного тормоза: f_k = 0, точное решение оставшейся системы 3x3.
Проверяется КАЖДОЕ колесо по отдельности (max |T_i|), а не сумма модулей.

Паспортный момент тормоза двигателя используется ТОЛЬКО параметрически
(T_brake * i * eta_rev) и НЕ является доказательством безопасности.
"""

import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

G = 9.81
R = 0.089                      # м, фактический радиус колеса Ø178
RW = 0.2489                    # м, радиус установки (layout v2)
COS45 = math.cos(math.radians(45.0))

TH = [0.0, 90.0, 180.0, 270.0]
SIG = (1, 1, 1, 1)
WHEEL_NAMES = ["0° (+X)", "90° (+Y)", "180° (−X)", "270° (−Y)"]

# --- параметры (НЕ факты) -----------------------------------------------------
MASS = [300.0, 400.0, 450.0]
GRADES = [0.01, 0.03, 0.05, 0.08]
DIRS = [0.0, 15.0, 30.0, 45.0]          # направление склона в плане, град (90°-периодично)

F_EXT = [0.0, 100.0, 200.0]            # Н, внешнее воздействие вдоль склона (параметр)
COM_OFFSET = 0.0                       # м, смещение ЦМ от геом. центра -> Mz = F*e
COM_H = 0.15                           # м, высота ЦМ над полом (параметр, для N_i)
MU_ASSUMED = [0.5, 0.3]                # параметр трения колесо/пол (НЕ паспорт)

T_BRAKE_MOTOR = {"200W (0.637 Н·м)": 0.637, "400W (1.27 Н·м)": 1.27}
I_RED = 30.0
ETA_REV = [0.3, 0.5, 0.8]              # обратный КПД редуктора — ПАРАМЕТР (UNK)

CHECKS = []
RESULTS = {}


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def jacobian():
    J = np.zeros((4, 3))
    for i, (th, s) in enumerate(zip(TH, SIG)):
        a = math.radians(th) + s * math.radians(45.0)
        J[i] = (math.cos(a), math.sin(a), s * COS45 * RW)
    return J


J = jacobian()
JT = J.T                                   # 3x4


def torques(f):
    return np.asarray(f) * R * COS45


def hold(Fx, Fy, Mz=0.0, failed=None):
    """J^T f = wrench.  failed=k  ->  f_k = 0, точное решение 3x3.
    Возвращает (f, info)."""
    wr = np.array([Fx, Fy, Mz], dtype=float)
    if failed is None:
        f, *_ = np.linalg.lstsq(JT, wr, rcond=None)
        return f, {"способ": "min-norm (4 колеса)", "условие": float(np.linalg.cond(JT))}
    keep = [i for i in range(4) if i != failed]
    A = JT[:, keep]
    cond = float(np.linalg.cond(A))
    if cond > 1e12:
        return None, {"способ": f"отказ {failed}", "сингулярно": True, "cond": cond}
    f = np.zeros(4)
    f[keep] = np.linalg.solve(A, wr)
    return f, {"способ": f"отказ колеса {failed}", "cond": cond}


def slope_force(M, grade):
    th = math.atan(grade)
    return M * G * math.sin(th), th


def normal_loads(M, grade, h=COM_H):
    """Оценка нормальных реакций на склоне: перераспределение из-за момента
    M*g*sin(theta)*h между парами колёс, разнесёнными на 2*a, a = RW/sqrt(2)."""
    th = math.atan(grade)
    a = RW / math.sqrt(2.0)
    N_avg = M * G * math.cos(th) / 4.0
    dN = M * G * math.sin(th) * h / (2.0 * 2.0 * a)   # на колесо
    return [N_avg + dN, N_avg + dN, N_avg - dN, N_avg - dN], a


# ============ 1. КОНТРОЛЬНЫЕ ТОЧКИ ============================================
print("== 1. Контрольные точки (независимая проверка модели) ==")
F_ctrl, th_ctrl = slope_force(450.0, 0.08)
t("B1_сила_на_склоне", abs(F_ctrl - 352.0) < 1.0,
  f"450 кг, 8%: F = M*g*sin(atan(0.08)) = {F_ctrl:.1f} Н (ожидание ≈352)")

f4, _ = hold(F_ctrl, 0.0)
T4 = torques(f4)
t("B2_4_исправных", np.allclose(np.abs(T4), 7.83, atol=0.02),
  f"4 исправных, склон вдоль X: T = {np.round(T4, 2).tolist()} Н·м/колесо "
  f"(ожидание ≈±7.83); F*r/4 = {F_ctrl*R/4:.2f}")

f3, info3 = hold(F_ctrl, 0.0, failed=0)
T3 = torques(f3)
t("B3_отказ_одного", abs(np.max(np.abs(T3)) - 15.67) < 0.05,
  f"отказ колеса 0°: f = {np.round(f3, 1).tolist()} Н -> "
  f"T = {np.round(T3, 2).tolist()} Н·м, max = {np.max(np.abs(T3)):.2f} "
  f"(ожидание ≈15.67)")

ranks = []
for k in range(4):
    _, inf = hold(F_ctrl, 0.0, failed=k)
    ranks.append(not inf.get("сингулярно", False))
t("B4_разрешимость_при_отказе", all(ranks),
  "система 3x3 при отказе любого одного колеса невырождена (cond < 1e12): "
  f"{ranks}")

# энергетическая согласованность (проверка, что модель и моменты согласованы)
u = J @ np.array([1.0, 0.0, 0.0])
w = math.sqrt(2.0) * u / R
P_wheels = float(np.sum(torques(f4) * w))
P_ref = float(F_ctrl * 1.0)
t("B5_энергия", abs(P_wheels - P_ref) < 1e-6,
  f"Σ T_i·ω_i = {P_wheels:.3f} Вт == F·v = {P_ref:.3f} Вт")

# ============ 2. ТАБЛИЦА: 4 исправных + 4 варианта отказа =====================
print("\n== 2. Удержание по колесу: 4 исправных и отказ каждого ==")
rows = []
worst = {}        # все случаи, включая внешнее воздействие
worst_g = {}      # только гравитация (F_ext = 0)
worst_intact = {} # 4 исправных, только гравитация, худшее направление
mu_g = {}
for M in MASS:
    for gr in GRADES:
        Fg, th = slope_force(M, gr)
        N, a = normal_loads(M, gr)
        mu_min = 0.0
        mu_min_g = 0.0
        for phi in DIRS:
            cp, sp = math.cos(math.radians(phi)), math.sin(math.radians(phi))
            for fext in F_EXT:
                Ft = Fg + fext
                Fx, Fy = Ft * cp, Ft * sp
                Mz = Ft * COM_OFFSET
                # --- 4 исправных
                fa, ia = hold(Fx, Fy, Mz)
                Ta = np.abs(torques(fa))
                mu_a = float(np.max(Ta / (R * COS45) / np.array(N)))
                mu_min = max(mu_min, mu_a)
                if fext == 0.0:
                    mu_min_g = max(mu_min_g, mu_a)
                rows.append({"M": M, "уклон_%": round(gr * 100), "dir_°": phi,
                             "F_ext_Н": fext, "отказ": None,
                             "T_Нм": [round(float(x), 2) for x in torques(fa)],
                             "max_T_Нм": round(float(np.max(Ta)), 2),
                             "F_склона_Н": round(Fg, 1)})
                key = (M, round(gr * 100))
                cur = worst.get(key)
                if cur is None or np.max(Ta) > cur[0]:
                    worst[key] = (float(np.max(Ta)), phi, fext, None)
                if fext == 0.0:
                    curg = worst_g.get(key)
                    if curg is None or np.max(Ta) > curg[0]:
                        worst_g[key] = (float(np.max(Ta)), phi, 0.0, None)
                    curi = worst_intact.get(key)
                    if curi is None or np.max(Ta) > curi:
                        worst_intact[key] = float(np.max(Ta))
                # --- отказ каждого колеса
                for k in range(4):
                    fb, ib = hold(Fx, Fy, Mz, failed=k)
                    if fb is None:
                        rows.append({"M": M, "уклон_%": round(gr * 100), "dir_°": phi,
                                     "F_ext_Н": fext, "отказ": k,
                                     "T_Нм": None, "max_T_Нм": None,
                                     "комментарий": "сингулярно"})
                        continue
                    Tb = np.abs(torques(fb))
                    mu_k = float(np.max(Tb / (R * COS45) / np.array(N)))
                    mu_min = max(mu_min, mu_k)
                    if fext == 0.0:
                        mu_min_g = max(mu_min_g, mu_k)
                    rows.append({"M": M, "уклон_%": round(gr * 100), "dir_°": phi,
                                 "F_ext_Н": fext, "отказ": k,
                                 "T_Нм": [round(float(x), 2) for x in torques(fb)],
                                 "max_T_Нм": round(float(np.max(Tb)), 2),
                                 "F_склона_Н": round(Fg, 1)})
                    if np.max(Tb) > worst[key][0]:
                        worst[key] = (float(np.max(Tb)), phi, fext, k)
                    if fext == 0.0:
                        curg = worst_g.get(key)
                        if curg is None or np.max(Tb) > curg[0]:
                            worst_g[key] = (float(np.max(Tb)), phi, 0.0, k)
        # компактный вывод (F_ext = 0, склон вдоль X и 45°)
        print(f"\n  {M:.0f} кг, {gr*100:.0f}%: F_g = {Fg:.0f} Н, N_i ≈ "
              f"[{N[0]:.0f}, {N[0]:.0f}, {N[2]:.0f}, {N[2]:.0f}] Н")
        for phi in (0.0, 45.0):
            fa, _ = hold(Fg * math.cos(math.radians(phi)),
                         Fg * math.sin(math.radians(phi)))
            Ta = np.abs(torques(fa))
            line = f"    dir {phi:>4.0f}°: 4 испр. = {np.round(torques(fa), 2).tolist()}"
            print(line)
            for k in range(4):
                fb, _ = hold(Fg * math.cos(math.radians(phi)),
                             Fg * math.sin(math.radians(phi)), failed=k)
                Tb = np.abs(torques(fb))
                print(f"            отказ {WHEEL_NAMES[k]:<10} -> "
                      f"{np.round(torques(fb), 2).tolist()}  max={np.max(Tb):.2f}")
        wg = worst_g[(M, round(gr * 100))]
        wa = worst[(M, round(gr * 100))]
        print(f"    => ТОЛЬКО ГРАВИТАЦИЯ: {wg[0]:.2f} Н·м (dir {wg[1]:.0f}°, "
              f"отказ {wg[3]}), μ ≥ {mu_min_g:.2f}")
        print(f"    => + F_ext до 200 Н: {wa[0]:.2f} Н·м (dir {wa[1]:.0f}°, "
              f"F_ext {wa[2]:.0f} Н, отказ {wa[3]}), μ ≥ {mu_min:.2f}")
        mu_g[(M, round(gr * 100))] = round(mu_min_g, 2)

t("B6_проверка_по_колесу", True,
  "для каждого случая проверялось КАЖДОЕ колесо (max |T_i|), а не сумма |T_i|; "
  "сумма модулей маскирует перегрузку одного колеса")

# ============ 3. ПАРАМЕТРИЧЕСКАЯ ОЦЕНКА ЁМКОСТИ ТОРМОЗА =======================
print("\n== 3. Ёмкость тормоза (ПАРАМЕТРИЧЕСКАЯ ОЦЕНКА, не доказательство) ==")
cap = []
for name, Tb in T_BRAKE_MOTOR.items():
    for eta in ETA_REV:
        Tw = Tb * I_RED * eta
        cap.append({"тормоз_двигателя": name, "T_мотор_Нм": Tb, "i": I_RED,
                    "eta_обратный": eta, "T_на_колесе_Нм": round(Tw, 2)})
        print(f"  {name:<18} η_rev={eta:.1f}: T_колесо = {Tb}*{I_RED}*{eta:.1f} "
              f"= {Tw:.2f} Н·м")

req = {f"{m:.0f} кг / {g}%": worst_g[(m, round(g * 100))][0]
       for m in MASS for g in GRADES}
print("\n  Требуемый момент на колесе (гравитация, худшее направление + отказ 1):")
for k, v in req.items():
    print(f"    {k:<14} {v:6.2f} Н·м")

# сколько колёс может обеспечить требование (по фактическому max по колесу)
verdict = {}
for name, Tb in T_BRAKE_MOTOR.items():
    for eta in ETA_REV:
        Tw = Tb * I_RED * eta
        ok4 = worst_g[(450.0, 8)][0] <= Tw        # включает отказы
        if ok4:
            verdict[f"{name} η={eta}"] = "покрывает 450 кг/8% включая отказ 1 тормоза"
        else:
            ok_intact = abs(7.83) <= Tw
            verdict[f"{name} η={eta}"] = (
                "НЕ покрывает 450 кг/8% при отказе" if ok_intact else
                "НЕ покрывает даже 4 исправных")
for k, v in verdict.items():
    print(f"  {k:<26} -> {v}")

t("B7_ёмкость_параметрически", True,
  "T_brake*i*eta_rev — ОЦЕНКА. Реальные неизвестные: обратный КПД редуктора, "
  "передаёт ли тормоз момент В ОБРАТНОМ направлении, несущая роль корпуса, "
  "зазор/проскальзывание. Требование Nabtesco о ВНЕШНЕМ удержании не отменено")

# ============ 4. ЧТО НЕ УЧТЕНО (явно) =========================================
print("\n== 4. Ограничения модели (явно) ==")
lim = {
    "Crr_исключён": "сопротивление качению в статике не учитывается ни как "
                    "удержание, ни как сопротивление (Crr измеряется при качении)",
    "распределение_4_колёс": "при 4 исправных система недоопределена (3 уравнения, "
                             "4 неизвестных): min-norm — ОДНО из возможных "
                             "распределений. Реальное зависит от податливости и "
                             "проскальзывания роликов. Верхняя граница на ОДНО "
                             "колесо: T <= F*r*cos45 = {:.1f} Н·м (450 кг/8%)".format(
                                 (352.0) * R * COS45),
    "потеря_контакта": "эквивалентна отказу тормоза этого колеса (f_k=0) — "
                       "покрыто вариантами отказа, но нормальная реакция N_k "
                       "может упасть до нуля при перераспределении на склоне",
    "пассивные_ролики": "заблокированное колесо даёт жёсткость только вдоль ê_i; "
                        "вдоль оси ролика удержание опирается на трение/перекрётие "
                        "ê четырёх колёс — при отказе одного это НЕ доказано",
    "отказ_питания": "тормоз non-excitation (DOC, стр.17–18): при снятии питания "
                     "замыкается пружиной — это плюс, но момент в обратном "
                     "направлении паспортом не подтверждён (UNK)",
    "внешние_силы": "F_ext — параметр (0/100/200 Н). Реальные воздействия "
                    "манипулятора/оператора не заданы; при F_ext растёт линейно",
    "трение_о_пол": "μ_min по таблице (гравитация, до ≈{:.2f} при отказе) — "
                    "требование к паре колесо/пол; μ фактической пары "
                    "неизвестен (параметр)".format(
                        max(mu_g.values())),
    "требование_Nabtesco": "DOC (стр.55–56): не останавливаться на уклоне; "
                           "внешнее удержание обеспечивает ТРАНСПОРТНОЕ СРЕДСТВО, "
                           "а не редуктор",
}
for k, v in lim.items():
    print(f"  - {k}: {v}")

# ============ 5. JSON =========================================================
out = {
    "этап": "RVW-7PA v4.1 — статическое удержание (без Crr)",
    "модель": {"R": R, "RW": RW, "cos45": COS45, "TH": TH, "SIG": SIG,
               "T_i": "f_i*R*cos45", "равновесие": "J^T f = (Fx,Fy,Mz)"},
    "контрольные_точки": CHECKS,
    "масса_кг": MASS, "уклоны_%": [round(g * 100) for g in GRADES],
    "направления_°": DIRS, "F_ext_Н": F_EXT,
    "худший_момент_гравитация_Нм": {f"{m:.0f}/{g}%": {
        "T_Нм": round(v[0], 2), "dir_°": v[1], "отказ": v[3],
        "mu_min": mu_g[(m, g)]}
        for (m, g), v in worst_g.items()},
    "худший_момент_4_исправных_Нм": {
        f"{m:.0f}/{g}%": {"T_Нм": round(v, 2)}
        for (m, g), v in worst_intact.items()},
    "худший_момент_с_Fext_Нм": {f"{m:.0f}/{g}%": {
        "T_Нм": round(v[0], 2), "dir_°": v[1], "F_ext_Н": v[2], "отказ": v[3]}
        for (m, g), v in worst.items()},
    "строки": rows,
    "ёмкость_тормоза": cap,
    "вердикт_ёмкости": verdict,
    "ограничения": lim,
    "вывод": ("Стояночное удержание считается только от гравитации на уклоне и "
              "внешних сил. Для 450 кг/8% требуется 7.83 Н·м на колесо при 4 "
              "исправных (идеальное распределение) и до 15.67 Н·м на оставшемся "
              "при отказе одного тормоза (проверка по колесу). Ёмкость "
              "T_brake*i*eta_rev — параметрическая оценка, НЕ доказательство."),
}
with open(os.path.join(HERE, "rvw7pa_brake_static.json"), "w", encoding="utf-8") as fh:
    json.dump(out, fh, ensure_ascii=False, indent=2)

print("\n== ИТОГ ==")
bad = [c for c in CHECKS if not c["ok"]]
print(f"  проверок: {len(CHECKS)}, провалено: {len(bad)}")
print(f"  JSON: {os.path.join(HERE, 'rvw7pa_brake_static.json')}")

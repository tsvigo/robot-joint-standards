# ЧЕРНОВИК технического запроса RoboCT — редакция v7.1

**Статус переписки: ОТПРАВЛЕНО 26.10.2026 заказчиком (e-mail RoboCT).**
Повторно НЕ отправлять. Ответ производителя на дату последней сверки (26.10.2026)
**в имеющихся данных отсутствует** — ожидается; при получении ответа — занести в
документацию этапа и пересчитать зависимые результаты (динамика, удержание, схема колёс).
Данный файл хранит текст отправленного письма для истории переписки.

**Статус черновика v7.1 (текст ниже — что было отправлено):** черновик, редакция v7.1.
**Адресат:** RoboCT (roboct-global.com), контакт с сайта: taku@roboct.com, +86-18767267131.
**Изделие:** Mecanum wheel module **CTM1000-10R**.

**Источники (все три официальные страницы):**
- P1: `/mobile-robot-components/mecanum-wheel/heavy-duty-mecanum-wheel.html`
- P2: `/mobile-robot-components/mecanum-wheel/industrial-mecanum-wheel.html`
- P3: `/mobile-robot-components/mecanum-wheel/heavy-duty-mecanum-wheel-module.html` ← новая

---

**Subject: Technical clarification request — CTM1000-10R (4 units, medical indoor robot)**

We evaluate four CTM1000-10R modules for a wheeled base of a medical non-standard
robot (indoor, linoleum floor). **The total robot mass is NOT yet confirmed**;
our current calculations use a scenario class of about 400 kg (provisional
calculation scenario only — not a confirmed specification). One confirmed input
from your side matters early: the per-wheel load rating under item 2. Your three product
pages give different parameters for the same model. Before engineering approval we
need one confirmed specification. Please answer items 1–7 and attach drawings.

**1. Confirmed specification — three sources disagree (please confirm each row):**

| Parameter | P1 Heavy Duty | P2 Industrial | P3 Heavy Duty Module | Your confirmation |
|---|---|---|---|---|
| Motor/rated power | 220–440 W | 180 W | 220/330/440 W @ 24/36/48 V | ? |
| Efficiency | — | — | 70/75/76 % | ? |
| Rated output speed | 56 rpm | (motor 3000 rpm → 80.9 rpm out.) | 28/43/56 rpm @ 24/36/48 V (no-load 36/54/72) | ? |
| Rated / peak torque | 74 N·m | motor 2/4 N·m → 74/148 N·m out. | 74 / **140** N·m out. | ? |
| IP rating | **IP44** | **IP54** | — | ? |
| Input voltage | 24–48 V | 48 V | 24/36/48 V | ? |
| Encoder | incremental magnetic **4096** | magnetic **4096** | **4384 line** | ? |
| Payload (set of 4) | 1000 kg | 1000 kg | **rated 1000 / max 1200 kg** | ? |
| Weight | 25 kg/wheel | 10 kg w/o shock / ~25 kg with | — | ? |
| Rated current | — | — | 12.8 / 12.9 / 13 A @ 24/36/48 V | ? |

- Notes: (a) 74 N·m × 56 rpm ≈ 434 W shaft power ≈ rated "440 W" — we read P3 power
  as MECHANICAL OUTPUT; please confirm and give the ELECTRICAL input per point
  (our estimate 571 W @ 48 V by η=76 % vs 624 W by U×I = 48×13 A — which is right?).
- (b) peak: 4 N·m × 37.1 = 148.4 N·m vs P3 140 N·m (ratio 0.943 ≈ gearbox η?) —
  confirm peak value and ALLOWED DURATION/duty cycle.
- (c) is "4384 line" a typo for 4096? Per motor or per wheel revolution?
- (d) P2 (180 W, IP54) seems to contradict P1/P3 — is it a different revision?

**2. Load rating:**
- Rated load per wheel (continuous) and MAX load per wheel (short-term, dynamic)?
  Is 1200 kg/set static only? What dynamic factor (floor joints, 10–15 mm
  thresholds at 0.5 m/s, transport of a patient with shifted CG)?
- Admissible floor pressure / suitability of cast polyurethane rollers for
  linoleum (marking, hardening)? Roller material/durometer and replacement interval.

**3. Full mounting envelope and drawing (CRITICAL):**
- Is "106.9 mm outer space between two sides" the overall width INCLUDING shock
  absorber, mounting bracket and cable outlet, or wheel body only?
  Our layout tolerates total module width up to 145 mm at Ø620 platform.
- Please attach **general assembly drawing** and **mounting/interface drawing**
  (PDF/DWG): bolt pattern, datum faces, allowable mounting orientations
  (our axes are RADIAL to the vehicle center), maximum local loads.

**4. Suspension:**
- Shock absorber: travel, spring rate, damping, axis/geometry of travel?
- Overall dimensions with absorber fully compressed / fully extended?
  (We checked ±30 mm module-to-body travel — please confirm the real range.)
- Position of cable outlet, cable length, min bending radius, connector type —
  cables must survive full suspension travel in a wheel well.

**5. Configuration question (CRITICAL for us):**
Our layout: four wheels with **RADIAL axes** (wheel axes pass through vehicle
center), **all four modules the SAME roller hand** (roller-axle lines in ground
view form a square through the contact points). Questions:
- Is CTM1000-10R left- or right-hand roller arrangement? Is a mirror version available?
- Do you APPROVE four same-hand modules in a radial arrangement for 1000 kg-class
  AGVs, or do you only recommend the standard X arrangement (2L+2R, parallel axes)?
- Any application references / test data for radial or non-standard arrangements?

**6. Drive, brake, holding (parking brake):**
- Gearbox type (planetary / worm / other): is the output back-drivable?
- **Holding at power OFF — we need your official figures, not our estimate:**
  - nominal holding (brake) torque at the output shaft and MAXIMUM holding torque;
  - conditions and time to engage/hold on power failure (fail-safe behaviour);
  - can the module hold external forces with power off (e.g. loads applied by a
    robot-mounted manipulator, not only the vehicle weight)?
  - parking/stationary brake: type, torque rating, engagement cycles, life,
    control signal, and whether it is included or optional.
  - Note: our ideal kinematic estimate for a 5 % grade at a ~400 kg scenario class
    is ≈5.9 N·m per wheel — this is a CALCULATED IDEAL COMPONENT ONLY, without any
    engineering margin (no efficiency/back-drive, dynamic, wear, or grade-tolerance
    allowance). Please do not treat it as a requirement value; we will set the real
    requirement from your brake specification.
- Continuous torque vs duty cycle at 40 °C ambient; regenerative braking /
  behaviour on emergency stop; recommended derating curves.

**7. Control interface:**
- Encoder wiring/resolution as confirmed above; hall sensor use; recommended
  controller/SDK for 4-wheel omni control (do you provide the kinematic
  controller for non-standard arrangements?).

**Attachments appreciated:** general assembly drawing, mounting drawing,
performance curves (torque–speed–power per voltage), efficiency map,
load rating documentation, brake spec.

---

### Внутренние примечания (НЕ ВКЛЮЧАТЬ В ПИСЬМО)
- П3 (24/36/48 В, 28/43/56 об/мин) — самая согласованная точка: 24 В
  (P_out 217 Вт ≈ 220, η 0.706 ≈ 0.70, ток 12.9 ≈ 12.8 А). Номинальная скорость
  платформы R2: 0.35 / 0.54 / 0.70 м/с по напряжениям; холостая 48 В — 0.905 м/с
  (P2-80.9 об/мин даёт 1.016 м/с — расхождение 12% тоже спросить).
- Если ответ подтвердит 440 Вт (выход), η 76%, пик 140 Н·м, IP54, тормоз —
  динамика v7.1 становится подтверждённой; если 180 Вт/IP44 без тормоза —
  пересчитать тёпловой режим и удержание.
- Если производитель НЕ разрешает 4 одинаковых руки на радиальных осях —
  R2 блокируется: на радиальных осях смешение рук даёт cond 2.44 и
  асимметричные силы (проверено v7.1), а 2L+2R в радиальной раскладке сингулярен
  (LRLR/RLRL — ранг 2). Варианты: заказать 4 модуля одной руки (если разрешат),
  либо вернуться к P1 (параллельные оси, X).
- MIT/Morin 2024 (GMWC): математика покрывает произвольные схемы, наши оси
  роликов проходят оба теста Force Graph — но это НЕ замена одобрению
  производителя для 1000-кг класса.

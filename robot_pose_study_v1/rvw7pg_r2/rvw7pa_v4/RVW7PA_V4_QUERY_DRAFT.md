# DRAFT technical query — RVW-7PA in Ø620 radial Mecanum base (NOT SENT)

**Status: DRAFT ONLY — do not send without explicit approval.**
Catalogue used: **Rev.005.1, CAT.210125_B, issued 03.09.2026.**
Product: **RVW-7PALA-30-0G01 / RVW-7PARA-30-0G01** (200 W) and
**-0G02** (400 W).
Context: circular indoor AGV platform Ø620 mm, 4 modules, radial wheel axes
(R2 scheme, wheel axes point to the platform centre), central lift column
Ø240 mm through the whole base height, hospital environment, mass scenarios
300 / 400 / 450 kg, slopes up to 8 %, door width 700 mm.

The previous query `/home/v/projects/робот/robot_pose_study_v1/rvw7pg_r2/NAVTESCO_QUERY_DRAFT.md`
concerned RVW-7PG and is superseded for the actuator questions below.

---

**Subject: Technical data required for RVW-7PA actuator integration
(Ø148 flange, radial axes, Ø620 platform)**

Dear Nabtesco Precision Europe team,

we are evaluating four RVW-7PA units for a circular indoor AGV and need the
following data to complete the integration study. Our own layout, based on
the catalogue dimensions (a = 178, b = 96, c = 233 / 244, d = 99 / 110,
□90 motor, Ø148 flange, 6×M6-16 on P.C.D. 136), shows that the units must be
enveloped conservatively and that 56–67 mm of axial clearance is missing at
the platform centre. We need the exact data to confirm or reduce this.

**A. Geometry and mounting interface**

1. STEP data (or a dimensioned drawing) for **RVW-7PALA-30-0G01 and
   RVW-7PARA-30-0G02**, including the motor, brake and connector.
   The download portal requires member login.
2. Which surface is the **machine mounting interface**: the Ø148 flange face
   at 129 mm from the wheel outer face, or the housing outer diameter?
   Please provide the bolt pattern, the number and size of the fasteners, the
   recommended tightening torque and the permissible axial/radial load on the
   interface.
3. Is the **gearhead housing** intended to carry structural loads (i.e. may it
   be used as a load path element of the vehicle frame, clamped on the Ø148
   housing)? If yes, what bending/torsion loads are permissible?
4. Exact axial position of the **cable connector** (Ø14 on the drawing) and
   the minimum bending radius / service clearance of the motor cable
   (catalogue shows a 500 mm lead).
5. Is a **hollow-shaft or offset motor** variant available, and is the motor
   orientation fixed relative to the mounting flange? Our layout is limited by
   the 233 / 244 mm total length.

**B. Torque and duty**

6. Please confirm the **duty/time limits** for the rated torque 7 N·m and the
   allowable acceleration/deceleration torque 27 N·m of the RVW-7PA:
   permitted duration, number of cycles, and whether 27 N·m may be applied
   repeatedly.
7. What is the **gearhead efficiency** at rated torque and rated speed?
   The catalogue does not state it; our motor-side torque calculations depend
   on it (we use 0.70–0.90 as a parameter range).
8. **Thermal data**: permissible continuous output torque at low wheel speed
   (0.1–0.5 m/s), thermal time constant, and the relation between surface
   temperature (catalogue limit 60 °C) and continuous torque. We need to know
   whether continuous operation at 5–7 N·m wheel torque, 0.5 m/s, is allowed.
9. Continuous operation at **3219 rpm motor speed** (1.0 m/s wheel speed,
   catalogue maximum) — is this inside the continuous duty range, given the
   rated speed of 3000 rpm?
10. Is **diagonal motion** (both platform axes simultaneously, 4552 rpm motor
    at 1.0 m/s) explicitly excluded? The catalogue states a speed range of
    1–4000 rpm and warns against diagonal travel on slopes.

**C. Brake and holding**

11. Brake holding torque is stated as 0.637 N·m (200 W) / 1.27 N·m (400 W)
    and the type is non-excitation. Please confirm:
    - whether this is a **static holding** brake or a dynamic service brake;
    - the permissible **number of emergency stops** with the brake;
    - the holding torque after **wear limit** and the recommended inspection
      interval;
    - the brake response time.
12. Is the gearbox **back-drivable** at i = 30? If partially, what is the
    reverse efficiency, i.e. how much of the wheel torque reaches the brake?
13. The catalogue requires an external position-retention function for
    stopping on slopes. For a 450 kg vehicle on an 8 % slope
    (≈51 N·m total at the wheel, ≈13 N·m per wheel), what **external brake
    capacity** do you recommend, and is a brake on the wheel output or on the
    motor shaft preferable?
14. With one of the four brakes failed, is holding on a slope permitted at all?

**D. Environment**

15. What is the **IP rating** of RVW-7PA (motor, brake, connector)? The
    catalogue only states «do not use in locations with water, oil, sand»
    and gives an ambient range of 0–40 °C for actuators. Is operation in a
    hospital environment with **wet floor cleaning** excluded entirely, and
    which protection class would be required for that use?
16. Permissible **axial load** on the wheel and on the output bearing for
    sideways Mecanum motion, and the resulting life reduction.

**E. Configuration**

17. Can we order **two L and two R** units (2× PALA + 2× PARA) as a set, and
    is there any restriction on the radial arrangement (four axes pointing to
    one centre)?
18. Which torsion code (L/R) is recommended for each of the four radial
    positions for a radial Mecanum layout, and does the standard 45° roller
    geometry require alternating hands?
19. Delivery time and minimum order quantity for a set of four units with the
    above configuration.

Thank you for your support.

---

**Примечание для внутреннего использования.**
Запрос **не отправлен**. Прежний черновик
`/home/v/projects/робот/robot_pose_study_v1/rvw7pg_r2/NAVTESCO_QUERY_DRAFT.md`
не изменялся.

Обоснование вопросов — разделы 3, 5, 6, 9 отчёта
`/home/v/projects/робот/robot_pose_study_v1/rvw7pg_r2/rvw7pa_v4/RVW7PA_V4_REPORT.md`.
Ключевые числа, на которые опираются вопросы:

| Вопрос | Наше число | Источник |
|---|---|---|
| A2–A5 | монтажная плоскость 129 мм от торца колеса, Ø148, 6×M6 PCD136 | `rvw7pa_layout.py` |
| B6–B10 | 46 из 60 режимов > 7 Н·м; диагональ 4552 об/мин | `rvw7pa_drive_check.json` |
| C11–C14 | 450 кг, 8 %: 50.9 Н·м суммарно, запас 1.20 (200 Вт), 0.90 при отказе | `rvw7pa_drive_check.json` |
| D15–D16 | IP не указан; 112.5 кгс/колесо при 450 кг | `rvw7pa_impact.json` |
| E17–E18 | 2×PALA + 2×PARA, радиальная схема | `rvw7pa_layout.json` |

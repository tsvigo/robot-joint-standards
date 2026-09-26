# Перечень суставов робота

Порядок: снизу вверх (стопа → шея), правая рука, левая рука

---

## Нижняя конечность (снизу вверх)

### ankle_joint_empty — Голеностопный сустав

В начальной позе голень перпендикулярна земле — вдоль оси Z, выпрямлена (0°). Диапазон сгибания: +30°–55°, ось вращения — Y.

[01] ankle_joint_empty — голеностопный сустав должен вращать всё кроме base_link с колёсами на +30° градусов (носок вверх) вокруг оси Y… и на -55° (носок вниз) вокруг оси Y

---

### knee_joint_empty — Коленный сустав

В начальной позе бедро перпендикулярна земле — вдоль оси Z, выпрямлено (0°). Диапазон сгибания: +10°–150°, ось вращения — Y.

[02] knee_joint_empty — коленный сустав должен вращать всё выше колена на +10° градусов вокруг оси Y… и на -150° вокруг оси Y

---

### hip_joint_empty — Тазобедренный сустав

В начальной позе туловище перпендикулярно земле — вдоль оси Z, выпрямлено (0°). Диапазон сгибания: +20°–130°, ось вращения — Y.

[03] hip_joint_empty — тазобедренный сустав должен вращать всё выше пояса на +20° градусов вокруг оси Y… и на -130° вокруг оси Y

---

## Шея (снизу вверх)

### atlanto_occipital_joint_empty — Атланто-затылочный сустав (C0–C1)

В начальной позе голова перпендикулярна земле — вдоль оси Z, выпрямлена (0°). Диапазон сгибания: –50°+85°, ось вращения — Y.

[04] atlanto_occipital_joint_empty — Атланто-затылочный сустав (C0–C1) должен вращать всё выше себя по геометрии на –50° градусов вокруг оси Y… и на +85° вокруг оси Y

---

### atlanto_axial_joint_empty — Атланто-осевой сустав (C1–C2)

В начальной позе голова смотрит вперёд — вдоль оси +X, выпрямлена (0°). Диапазон наклона: –90°+90°, ось вращения — Z.

[05] atlanto_axial_joint_empty — Атланто-осевой сустав (C1–C2) должен поворачивать всё выше себя по геометрии на –90° градусов вокруг оси Z… и на +90° вокруг оси Z

---

### intervertebral_joints_C2_C7_empty — Межпозвоночные суставы C2–C7

В начальной позе голова смотрит вперёд — вдоль оси +X, выпрямлена (0°). Диапазон наклона: –45°+45°, ось вращения — X.

[06] intervertebral_joints_C2_C7_empty — Межпозвоночные суставы C2–C7 должны наклонять всё выше себя по геометрии на –45° градусов вокруг оси X… и на +45° вокруг оси X

---

## Правая рука (суставы)

### right_shoulder_joint_empty_1 — Правый плечевой сустав 1

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон отведения-приведения: -90°+130°, ось вращения — X.

[07] right_shoulder_joint_empty_1 — Правый плечевой сустав 1 должен вращать всю руку на –90° градусов вокруг оси X… и на +130° вокруг оси X

```yaml
joint:
  name: right_shoulder_joint_1
  blender_empty: right_shoulder_joint_empty_1

  parent_link: right_shoulder_2_link
  child_link:   right_shoulder_link

  anatomical:
    side: right
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -90
      upper: 130

  description: |
    Отведение и приведение правой руки вокруг оси X -90°+130°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### right_shoulder_joint_empty_2 — Правый плечевой сустав 2

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон сгибания-разгибания: –90°+90°, ось вращения — Z.

[08] right_shoulder_joint_empty_2 — Правый плечевой сустав 2 должен вращать всю руку на –90° градусов вокруг оси Z и на +90° вокруг оси Z

```yaml
joint:
  name: right_shoulder_joint_2
  blender_empty: right_shoulder_joint_empty_2

  parent_link: right_shoulder_3_link
  child_link:   right_shoulder_2_link

  anatomical:
    side: right
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -90
      upper: 90

  description: |
    Сгибание и разгибание правой руки вокруг оси Z -90°+90°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### right_shoulder_joint_empty_3 — Правый плечевой сустав 3

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон внутренней ротации-внешней ротации: –180°+60°, ось вращения — Y.

[09] right_shoulder_joint_empty_3 — Правый плечевой сустав 3 должен вращать всю руку на –180° градусов вокруг оси Y и на +60° вокруг оси Y

```yaml
joint:
  name: right_shoulder_joint_3
  blender_empty: right_shoulder_joint_empty_3

  parent_link: torso_link
  child_link:   right_shoulder_3_link

  anatomical:
    side: right
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -180
      upper: 60

  description: |
    Внутренняя и наружная ротация правой руки вокруг оси Y -180°+60°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### right_elbow_joint_empty — Правый локтевой сустав

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон сгибания-разгибания: –10°+150°, ось вращения — Z.

[10] right_elbow_joint_empty — Правый локтевой сустав должен вращать предплечье и кисть на –10° градусов вокруг оси Z и на +150° вокруг оси Z

```yaml
joint:
  name: right_elbow_joint
  blender_empty: right_elbow_joint_empty

  parent_link: right_shoulder_link
  child_link:   right_forearm_link

  anatomical:
    side: right
    region: elbow
    type: humeroulnar

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -10
      upper: 150

  description: |
    Сгибание и разгибание предплечья и кисти правой руки вокруг оси Z -10°+150°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

#### [10b] Проксимальный лучелоктевой сустав (пронация/супинация) — right_proximal_radioulnar_joint_empty

```yaml
joint:
  name: right_proximal_radioulnar_joint
  blender_empty: right_proximal_radioulnar_joint_empty

  parent_link: right_forearm_link
  child_link:   right_radioulnar_link

  anatomical:
    side: right
    region: forearm
    type: proximal_radioulnar

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -90
      upper: 90

  description: |
    Пронация и супинация предплечья правой руки вокруг оси Y -90°+90°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
    Линейная цепочка (Фаза 1C, без upper_arm_link): forearm_link → radioulnar → radioulnar_link → wrist_1 → wrist_pitch_link → wrist_2 → palm_link.
```

---

### Правая рука: лучезапястные суставы

#### right_wrist_joint_1_empty — Первый лучезапястный сустав

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон сгибания-разгибания: –90°+90°, ось вращения — Z.

[11] right_wrist_joint_1_empty — Первый лучезапястный сустав должен вращать кисть на –90° градусов вокруг оси Z и на +90° вокруг оси Z

```yaml
joint:
  name: right_wrist_joint_1
  blender_empty: right_wrist_joint_1_empty

  parent_link: right_radioulnar_link
  child_link:   right_wrist_pitch_link

  anatomical:
    side: right
    region: wrist
    type: radiocarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -90
      upper: 90

  description: |
    Сгибание и разгибание правой кисти вокруг оси Z -90°+90°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

#### right_wrist_joint_2_empty — Второй лучезапястный сустав

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон отведения-приведения: –25°+45°, ось вращения — X.

[12] right_wrist_joint_2_empty — Второй лучезапястный сустав должен вращать кисть на –25° градусов вокруг оси X и на +45° вокруг оси X

```yaml
joint:
  name: right_wrist_joint_2
  blender_empty: right_wrist_joint_2_empty

  parent_link: right_wrist_pitch_link
  child_link:   right_palm_link

  anatomical:
    side: right
    region: wrist
    type: midcarpal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 45

  description: |
    Отведение и приведение правой кисти вокруг оси X -25°+45°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### Суставы большого пальца правой руки

#### [13] Запястно-пястный сустав (CMC) — right_thumb_cmc_joint_empty_1

```yaml
joint:
  name: right_thumb_cmc_joint_1
  blender_empty: right_thumb_cmc_joint_empty_1

  parent_link: right_palm_link
  child_link:   third_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Сгибание и разгибание большого пальца правой руки вокруг оси Y -25°+25°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [14] Запястно-пястный сустав (CMC) — right_thumb_cmc_joint_empty_2

```yaml
joint:
  name: right_thumb_cmc_joint_2
  blender_empty: right_thumb_cmc_joint_empty_2

  parent_link: right_palm_link
  child_link:   third_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -40
      upper: 40

  description: |
    Отведение и приведение большого пальца правой руки вокруг оси Z -40°+40°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [15] Запястно-пястный сустав (CMC) — right_thumb_cmc_joint_empty_3

```yaml
joint:
  name: right_thumb_cmc_joint_3
  blender_empty: right_thumb_cmc_joint_empty_3

  parent_link: right_palm_link
  child_link:   third_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -60
      upper: 60

  description: |
    Противопоставление (касание подушечек остальных пальцев) большим пальцем правой руки вокруг оси X -60°+60°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [16] Пястно-фаланговый сустав (MCP) — right_thumb_mcp_joint_empty_1

```yaml
joint:
  name: right_thumb_mcp_joint_1
  blender_empty: right_thumb_mcp_joint_empty_1

  parent_link: third_phalanx_right_thumb_link
  child_link:   second_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -70
      upper: 70

  description: |
    Сгибание и разгибание большого пальца правой руки вокруг оси Z -70°+70°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [17] Пястно-фаланговый сустав (MCP) — right_thumb_mcp_joint_empty_2

```yaml
joint:
  name: right_thumb_mcp_joint_2
  blender_empty: right_thumb_mcp_joint_empty_2

  parent_link: third_phalanx_right_thumb_link
  child_link:   second_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение большого пальца правой руки вокруг оси X -25°+25°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [18] Межфаланговый сустав (IP) — right_thumb_ip_joint_empty

```yaml
joint:
  name: right_thumb_ip_joint
  blender_empty: right_thumb_ip_joint_empty

  parent_link: second_phalanx_right_thumb_link
  child_link:   first_phalanx_right_thumb_link

  anatomical:
    side: right
    digit: 1
    region: hand
    type: interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание крайней фаланги большого пальца правой руки вокруг оси Z 0°+90°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

---

### Суставы указательного пальца правой руки

#### [19] Первый Пястно-фаланговый сустав (mcp) указательного пальца — right_digit2_mcp_joint

```yaml
joint:
  name: right_digit2_mcp_joint
  blender_empty: right_digit2_mcp_joint_empty

  parent_link: right_digit2_metacarpal
  child_link:   right_digit2_proximal_phalanx

  anatomical:
    side: right
    digit: 2
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание указательного пальца правой руки вокруг оси Z 0°+100°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [20] Второй Пястно-фаланговый сустав (mcp2) указательного пальца — right_digit2_mcp2_joint

```yaml
joint:
  name: right_digit2_mcp2_joint
  blender_empty: right_digit2_mcp2_joint_empty

  parent_link: right_digit2_metacarpal
  child_link:   right_digit2_proximal_phalanx

  anatomical:
    side: right
    digit: 2
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение указательного пальца правой руки вокруг оси X -25°+25°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [21] Проксимальный межфаланговый сустав (pip2) указательного пальца — right_digit2_pip_joint

```yaml
joint:
  name: right_digit2_pip_joint
  blender_empty: right_digit2_pip_joint_empty

  parent_link: right_digit2_proximal_phalanx
  child_link:   right_digit2_middle_phalanx

  anatomical:
    side: right
    digit: 2
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг указательного пальца правой руки вокруг оси Z 0°+110°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [22] Дистальный межфаланговый сустав (dip2) указательного пальца — right_digit2_dip_joint

```yaml
joint:
  name: right_digit2_dip_joint
  blender_empty: right_digit2_dip_joint_empty

  parent_link: right_digit2_middle_phalanx
  child_link:   right_digit2_distal_phalanx

  anatomical:
    side: right
    digit: 2
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги указательного пальца правой руки вокруг оси Z 0°+90°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

суставы среднего пальца правой руки:

#### [23] Первый Пястно-фаланговый сустав (mcp3) среднего пальца — right_digit3_mcp_joint

```yaml
joint:
  name: right_digit3_mcp_joint
  blender_empty: right_digit3_mcp_joint_empty

  parent_link: right_digit3_metacarpal
  child_link:   right_digit3_proximal_phalanx

  anatomical:
    side: right
    digit: 3
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание среднего пальца правой руки вокруг оси Z 0°+100°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [24] Второй Пястно-фаланговый сустав (mcp3-2) среднего пальца — right_digit3_mcp2_joint

```yaml
joint:
  name: right_digit3_mcp2_joint
  blender_empty: right_digit3_mcp2_joint_empty

  parent_link: right_digit3_metacarpal
  child_link:   right_digit3_proximal_phalanx

  anatomical:
    side: right
    digit: 3
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение среднего пальца правой руки вокруг оси X -25°+25°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [25] Проксимальный межфаланговый сустав (pip3) среднего пальца — right_digit3_pip_joint

```yaml
joint:
  name: right_digit3_pip_joint
  blender_empty: right_digit3_pip_joint_empty

  parent_link: right_digit3_proximal_phalanx
  child_link:   right_digit3_middle_phalanx

  anatomical:
    side: right
    digit: 3
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание среднего пальца правой руки вокруг оси Z 0°+110°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [26] Дистальный межфаланговый сустав (dip3) среднего пальца — right_digit3_dip_joint

```yaml
joint:
  name: right_digit3_dip_joint
  blender_empty: right_digit3_dip_joint_empty

  parent_link: right_digit3_middle_phalanx
  child_link:   right_digit3_distal_phalanx

  anatomical:
    side: right
    digit: 3
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги среднего пальца правой руки вокруг оси Z 0°+90°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [27] Первый Пястно-фаланговый сустав (mcp4) безымянного пальца — right_digit4_mcp_joint

```yaml
joint:
  name: right_digit4_mcp_joint
  blender_empty: right_digit4_mcp_joint_empty

  parent_link: right_digit4_metacarpal
  child_link:   right_digit4_proximal_phalanx

  anatomical:
    side: right
    digit: 4
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание безымянного пальца правой руки вокруг оси Z 0°+100°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [28] Второй Пястно-фаланговый сустав (mcp2) безымянного пальца — right_digit4_mcp2_joint

```yaml
joint:
  name: right_digit4_mcp2_joint
  blender_empty: right_digit4_mcp2_joint_empty

  parent_link: right_digit4_metacarpal
  child_link:   right_digit4_proximal_phalanx

  anatomical:
    side: right
    digit: 4
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение безымянного пальца правой руки вокруг оси X -25°+25°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [29] Проксимальный межфаланговый сустав (pip4) безымянного пальца — right_digit4_pip_joint

```yaml
joint:
  name: right_digit4_pip_joint
  blender_empty: right_digit4_pip_joint_empty

  parent_link: right_digit4_proximal_phalanx
  child_link:   right_digit4_middle_phalanx

  anatomical:
    side: right
    digit: 4
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг безымянного пальца правой руки вокруг оси Z 0°+110°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [30] Дистальный межфаланговый сустав (dip4) безымянного пальца — right_digit4_dip_joint

```yaml
joint:
  name: right_digit4_dip_joint
  blender_empty: right_digit4_dip_joint_empty

  parent_link: right_digit4_middle_phalanx
  child_link:   right_digit4_distal_phalanx

  anatomical:
    side: right
    digit: 4
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги безымянного пальца правой руки вокруг оси Z 0°+90°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [31] Первый Пястно-фаланговый сустав (mcp5) мизинца — right_digit5_mcp_joint

```yaml
joint:
  name: right_digit5_mcp_joint
  blender_empty: right_digit5_mcp_joint_empty

  parent_link: right_digit5_metacarpal
  child_link:   right_digit5_proximal_phalanx

  anatomical:
    side: right
    digit: 5
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание мизинца правой руки вокруг оси Z 0°+100°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [32] Второй Пястно-фаланговый сустав (mcp5-2) мизинца — right_digit5_mcp2_joint

```yaml
joint:
  name: right_digit5_mcp2_joint
  blender_empty: right_digit5_mcp2_joint_empty

  parent_link: right_digit5_metacarpal
  child_link:   right_digit5_proximal_phalanx

  anatomical:
    side: right
    digit: 5
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение мизинца правой руки вокруг оси X -25°+25°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [33] Проксимальный межфаланговый сустав (pip5) мизинца — right_digit5_pip_joint

```yaml
joint:
  name: right_digit5_pip_joint
  blender_empty: right_digit5_pip_joint_empty

  parent_link: right_digit5_proximal_phalanx
  child_link:   right_digit5_middle_phalanx

  anatomical:
    side: right
    digit: 5
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг мизинца правой руки вокруг оси Z 0°+110°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [34] Дистальный межфаланговый сустав (dip5) мизинца — right_digit5_dip_joint

```yaml
joint:
  name: right_digit5_dip_joint
  blender_empty: right_digit5_dip_joint_empty

  parent_link: right_digit5_middle_phalanx
  child_link:   right_digit5_distal_phalanx

  anatomical:
    side: right
    digit: 5
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги мизинца правой руки вокруг оси Z 0°+90°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

---

## Пястные звенья (metacarpal) — обе руки

`{right,left}_digit{2,3,4,5}_metacarpal` — фиксированные звенья без сустава
(Empty, PLAIN_AXES 0.01; созданы в Blender, Фаза 3; 8 шт):
`parent: {side}_palm_link`, дальше `mcp/mcp2 → proximal_phalanx`.
В yaml ниже фигурируют как `parent_link` всех `mcp/mcp2`-блоков (32 шт);
отдельных yaml-блоков для самих metacarpal не требуется.

---

## Левая рука (суставы)

### left_shoulder_joint_empty_1 — Левый плечевой сустав 1

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон отведения-приведения: -90°+130°, ось вращения — X.

[07] left_shoulder_joint_empty_1 — Левый плечевой сустав 1 должен вращать всю руку на –90° градусов вокруг оси X… и на +130° вокруг оси X

```yaml
joint:
  name: left_shoulder_joint_1
  blender_empty: left_shoulder_joint_empty_1

  parent_link: left_shoulder_2_link
  child_link:   left_shoulder_link

  anatomical:
    side: left
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -90
      upper: 130

  description: |
    Отведение и приведение левой руки вокруг оси X -90°+130°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### left_shoulder_joint_empty_2 — Левый плечевой сустав 2

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон сгибания-разгибания: –90°+90°, ось вращения — Z.

[08] left_shoulder_joint_empty_2 — Левый плечевой сустав 2 должен вращать всю руку на –90° градусов вокруг оси Z и на +90° вокруг оси Z

```yaml
joint:
  name: left_shoulder_joint_2
  blender_empty: left_shoulder_joint_empty_2

  parent_link: left_shoulder_3_link
  child_link:   left_shoulder_2_link

  anatomical:
    side: left
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -90
      upper: 90

  description: |
    Сгибание и разгибание левой руки вокруг оси Z -90°+90°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### left_shoulder_joint_empty_3 — Левый плечевой сустав 3

В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°). Диапазон внутренней ротации-внешней ротации: –180°+60°, ось вращения — Y.

[09] left_shoulder_joint_empty_3 — Левый плечевой сустав 3 должен вращать всю руку на –180° градусов вокруг оси Y и на +60° вокруг оси Y

```yaml
joint:
  name: left_shoulder_joint_3
  blender_empty: left_shoulder_joint_empty_3

  parent_link: torso_link
  child_link:   left_shoulder_3_link

  anatomical:
    side: left
    region: shoulder
    type: glenohumeral

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -180
      upper: 60

  description: |
    Внутренняя и наружная ротация левой руки вокруг оси Y -180°+60°.
    Линейная цепочка (Фаза 1B, dummy-линки): torso_link → shoulder_3 → shoulder_3_link → shoulder_2 → shoulder_2_link → shoulder_1 → shoulder_link.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### left_proximal_radioulnar_joint_empty — Левый проксимальный лучелоктевой сустав

#### [10b] Проксимальный лучелоктевой сустав (пронация/супинация) — left_proximal_radioulnar_joint_empty

```yaml
joint:
  name: left_proximal_radioulnar_joint
  blender_empty: left_proximal_radioulnar_joint_empty

  parent_link: left_forearm_link
  child_link:   left_radioulnar_link

  anatomical:
    side: left
    region: forearm
    type: proximal_radioulnar

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -90
      upper: 90

  description: |
    Пронация и супинация предплечья левой руки вокруг оси Y -90°+90°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
    Линейная цепочка (Фаза 1C, без upper_arm_link): forearm_link → radioulnar → radioulnar_link → wrist_1 → wrist_pitch_link → wrist_2 → palm_link.
```

---

### left_elbow_joint_empty — Левый локтевой сустав

[10] left_elbow_joint_empty — Левый локтевой сустав должен вращать предплечье и кисть на –10° градусов вокруг оси Z и на +150° вокруг оси Z

```yaml
joint:
  name: left_elbow_joint
  blender_empty: left_elbow_joint_empty

  parent_link: left_shoulder_link
  child_link:   left_forearm_link

  anatomical:
    side: left
    region: elbow
    type: humeroulnar

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -10
      upper: 150

  description: |
    Сгибание и разгибание предплечья и кисти левой руки вокруг оси Z -10°+150°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### Левая рука: лучезапястные суставы

#### [11] left_wrist_joint_1_empty — Первый лучезапястный сустав

```yaml
joint:
  name: left_wrist_joint_1
  blender_empty: left_wrist_joint_1_empty

  parent_link: left_radioulnar_link
  child_link:   left_wrist_pitch_link

  anatomical:
    side: left
    region: wrist
    type: radiocarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -90
      upper: 90

  description: |
    Сгибание и разгибание левой кисти вокруг оси Z -90°+90°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

#### [12] left_wrist_joint_2_empty — Второй лучезапястный сустав

```yaml
joint:
  name: left_wrist_joint_2
  blender_empty: left_wrist_joint_2_empty

  parent_link: left_wrist_pitch_link
  child_link:   left_palm_link

  anatomical:
    side: left
    region: wrist
    type: midcarpal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 45

  description: |
    Отведение и приведение левой кисти вокруг оси X -25°+45°.
    В начальной позе рука отведена в сторону — вдоль оси Y, выпрямлена (0°).
```

---

### Суставы большого пальца левой руки

#### [13] Запястно-пястный сустав (CMC) — left_thumb_cmc_joint_empty_1

```yaml
joint:
  name: left_thumb_cmc_joint_1
  blender_empty: left_thumb_cmc_joint_empty_1

  parent_link: left_palm_link
  child_link:   third_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 1
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Сгибание и разгибание большого пальца левой руки вокруг оси Y -25°+25°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [14] Запястно-пястный сустав (CMC) — left_thumb_cmc_joint_empty_2

```yaml
joint:
  name: left_thumb_cmc_joint_2
  blender_empty: left_thumb_cmc_joint_empty_2

  parent_link: left_palm_link
  child_link:   third_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -40
      upper: 40

  description: |
    Отведение и приведение большого пальца левой руки вокруг оси Z -40°+40°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [15] Запястно-пястный сустав (CMC) — left_thumb_cmc_joint_empty_3

```yaml
joint:
  name: left_thumb_cmc_joint_3
  blender_empty: left_thumb_cmc_joint_empty_3

  parent_link: left_palm_link
  child_link:   third_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: carpometacarpal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -60
      upper: 60

  description: |
    Противопоставление (касание подушечек остальных пальцев) большим пальцем левой руки вокруг оси X -60°+60°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [16] Пястно-фаланговый сустав (MCP) — left_thumb_mcp_joint_empty_1

```yaml
joint:
  name: left_thumb_mcp_joint_1
  blender_empty: left_thumb_mcp_joint_empty_1

  parent_link: third_phalanx_left_thumb_link
  child_link:   second_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: -70
      upper: 70

  description: |
    Сгибание и разгибание большого пальца левой руки вокруг оси Z -70°+70°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [17] Пястно-фаланговый сустав (MCP) — left_thumb_mcp_joint_empty_2

```yaml
joint:
  name: left_thumb_mcp_joint_2
  blender_empty: left_thumb_mcp_joint_empty_2

  parent_link: third_phalanx_left_thumb_link
  child_link:   second_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение большого пальца левой руки вокруг оси X -25°+25°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

#### [18] Межфаланговый сустав (IP) — left_thumb_ip_joint_empty

```yaml
joint:
  name: left_thumb_ip_joint
  blender_empty: left_thumb_ip_joint_empty

  parent_link: second_phalanx_left_thumb_link
  child_link:   first_phalanx_left_thumb_link

  anatomical:
    side: left
    digit: 1
    region: hand
    type: interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание крайней фаланги большого пальца левой руки вокруг оси Z 0°+90°.
    В начальной позе большой палец идёт вдоль оси Y, выпрямлен (0°).
```

---

### Суставы указательного пальца левой руки

#### [19] Первый Пястно-фаланговый сустав (mcp) — left_digit2_mcp_joint

```yaml
joint:
  name: left_digit2_mcp_joint
  blender_empty: left_digit2_mcp_joint_empty

  parent_link: left_digit2_metacarpal
  child_link:   left_digit2_proximal_phalanx

  anatomical:
    side: left
    digit: 2
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание указательного пальца левой руки вокруг оси Z 0°+100°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [20] Второй Пястно-фаланговый сустав (mcp2) — left_digit2_mcp2_joint

```yaml
joint:
  name: left_digit2_mcp2_joint
  blender_empty: left_digit2_mcp2_joint_empty

  parent_link: left_digit2_metacarpal
  child_link:   left_digit2_proximal_phalanx

  anatomical:
    side: left
    digit: 2
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение указательного пальца левой руки вокруг оси X -25°+25°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [21] Проксимальный межфаланговый сустав (pip) — left_digit2_pip_joint

```yaml
joint:
  name: left_digit2_pip_joint
  blender_empty: left_digit2_pip_joint_empty

  parent_link: left_digit2_proximal_phalanx
  child_link:   left_digit2_middle_phalanx

  anatomical:
    side: left
    digit: 2
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг указательного пальца левой руки вокруг оси Z 0°+110°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [22] Дистальный межфаланговый сустав (dip) — left_digit2_dip_joint

```yaml
joint:
  name: left_digit2_dip_joint
  blender_empty: left_digit2_dip_joint_empty

  parent_link: left_digit2_middle_phalanx
  child_link:   left_digit2_distal_phalanx

  anatomical:
    side: left
    digit: 2
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги указательного пальца левой руки вокруг оси Z 0°+90°.
    В начальной позе указательный палец идёт вдоль оси -Y, выпрямлен (0°).
```

---

### Суставы среднего пальца левой руки

#### [23] Первый Пястно-фаланговый сустав (mcp) — left_digit3_mcp_joint

```yaml
joint:
  name: left_digit3_mcp_joint
  blender_empty: left_digit3_mcp_joint_empty

  parent_link: left_digit3_metacarpal
  child_link:   left_digit3_proximal_phalanx

  anatomical:
    side: left
    digit: 3
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание среднего пальца левой руки вокруг оси Z 0°+100°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [24] Второй Пястно-фаланговый сустав (mcp2) — left_digit3_mcp2_joint

```yaml
joint:
  name: left_digit3_mcp2_joint
  blender_empty: left_digit3_mcp2_joint_empty

  parent_link: left_digit3_metacarpal
  child_link:   left_digit3_proximal_phalanx

  anatomical:
    side: left
    digit: 3
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение среднего пальца левой руки вокруг оси X -25°+25°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [25] Проксимальный межфаланговый сустав (pip) — left_digit3_pip_joint

```yaml
joint:
  name: left_digit3_pip_joint
  blender_empty: left_digit3_pip_joint_empty

  parent_link: left_digit3_proximal_phalanx
  child_link:   left_digit3_middle_phalanx

  anatomical:
    side: left
    digit: 3
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг среднего пальца левой руки вокруг оси Z 0°+110°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [26] Дистальный межфаланговый сустав (dip) — left_digit3_dip_joint

```yaml
joint:
  name: left_digit3_dip_joint
  blender_empty: left_digit3_dip_joint_empty

  parent_link: left_digit3_middle_phalanx
  child_link:   left_digit3_distal_phalanx

  anatomical:
    side: left
    digit: 3
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги среднего пальца левой руки вокруг оси Z 0°+90°.
    В начальной позе средний палец идёт вдоль оси -Y, выпрямлен (0°).
```

---

### Суставы безымянного пальца левой руки

#### [27] Первый Пястно-фаланговый сустав (mcp) — left_digit4_mcp_joint

```yaml
joint:
  name: left_digit4_mcp_joint
  blender_empty: left_digit4_mcp_joint_empty

  parent_link: left_digit4_metacarpal
  child_link:   left_digit4_proximal_phalanx

  anatomical:
    side: left
    digit: 4
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание безымянного пальца левой руки вокруг оси Z 0°+100°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [28] Второй Пястно-фаланговый сустав (mcp2) — left_digit4_mcp2_joint

```yaml
joint:
  name: left_digit4_mcp2_joint
  blender_empty: left_digit4_mcp2_joint_empty

  parent_link: left_digit4_metacarpal
  child_link:   left_digit4_proximal_phalanx

  anatomical:
    side: left
    digit: 4
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение безымянного пальца левой руки вокруг оси X -25°+25°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [29] Проксимальный межфаланговый сустав (pip) — left_digit4_pip_joint

```yaml
joint:
  name: left_digit4_pip_joint
  blender_empty: left_digit4_pip_joint_empty

  parent_link: left_digit4_proximal_phalanx
  child_link:   left_digit4_middle_phalanx

  anatomical:
    side: left
    digit: 4
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг безымянного пальца левой руки вокруг оси Z 0°+110°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [30] Дистальный межфаланговый сустав (dip) — left_digit4_dip_joint

```yaml
joint:
  name: left_digit4_dip_joint
  blender_empty: left_digit4_dip_joint_empty

  parent_link: left_digit4_middle_phalanx
  child_link:   left_digit4_distal_phalanx

  anatomical:
    side: left
    digit: 4
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги безымянного пальца левой руки вокруг оси Z 0°+90°.
    В начальной позе безымянный палец идёт вдоль оси -Y, выпрямлен (0°).
```

---

### Суставы мизинца левой руки

#### [31] Первый Пястно-фаланговый сустав (mcp) — left_digit5_mcp_joint

```yaml
joint:
  name: left_digit5_mcp_joint
  blender_empty: left_digit5_mcp_joint_empty

  parent_link: left_digit5_metacarpal
  child_link:   left_digit5_proximal_phalanx

  anatomical:
    side: left
    digit: 5
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 100

  description: |
    Сгибание и разгибание мизинца левой руки вокруг оси Z 0°+100°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [32] Второй Пястно-фаланговый сустав (mcp2) — left_digit5_mcp2_joint

```yaml
joint:
  name: left_digit5_mcp2_joint
  blender_empty: left_digit5_mcp2_joint_empty

  parent_link: left_digit5_metacarpal
  child_link:   left_digit5_proximal_phalanx

  anatomical:
    side: left
    digit: 5
    region: hand
    type: metacarpophalangeal

  simulation:
    type: revolute
    axis:
      x: 1
      y: 0
      z: 0
    limits:
      lower: -25
      upper: 25

  description: |
    Отведение и приведение мизинца левой руки вокруг оси X -25°+25°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [33] Проксимальный межфаланговый сустав (pip) — left_digit5_pip_joint

```yaml
joint:
  name: left_digit5_pip_joint
  blender_empty: left_digit5_pip_joint_empty

  parent_link: left_digit5_proximal_phalanx
  child_link:   left_digit5_middle_phalanx

  anatomical:
    side: left
    digit: 5
    region: hand
    type: proximal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 110

  description: |
    Сгибание и разгибание второй и третьей фаланг мизинца левой руки вокруг оси Z 0°+110°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

#### [34] Дистальный межфаланговый сустав (dip) — left_digit5_dip_joint

```yaml
joint:
  name: left_digit5_dip_joint
  blender_empty: left_digit5_dip_joint_empty

  parent_link: left_digit5_middle_phalanx
  child_link:   left_digit5_distal_phalanx

  anatomical:
    side: left
    digit: 5
    region: hand
    type: distal_interphalangeal

  simulation:
    type: revolute
    axis:
      x: 0
      y: 0
      z: 1
    limits:
      lower: 0
      upper: 90

  description: |
    Сгибание и разгибание концевой фаланги мизинца левой руки вокруг оси Z 0°+90°.
    В начальной позе мизинец идёт вдоль оси -Y, выпрямлен (0°).
```

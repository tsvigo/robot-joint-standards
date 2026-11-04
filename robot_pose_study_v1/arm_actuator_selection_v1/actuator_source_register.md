# Реестр источников — Stage 2X, подбор промышленных приводов

Каталог: `/home/v/projects/робот/robot_pose_study_v1/arm_actuator_selection_v1/`

Все данные фиксировались **4 ноября 2026 г.** Страница засчитывалась как
источник характеристики только если значение приведено на самой странице или
в скачанном официальном каталоге. Расчётные значения помечены отдельно.

Легенда: `[Ф]` факт источника · `[Р]` результат расчёта · `[О]` инженерная
оценка · `[П]` предположение · `[Н]` данных нет (UNKNOWN).

---

## 1. Исходные требования проекта (только чтение)

| Источник | Файл | Что взято |
|---|---|---|
| Stage 2B.2, позы P6/P7 | `/home/v/projects/робот/robot_pose_study_v1/arm_architecture_comparison_v1/arm_architecture_comparison.py` | 24 эксплуатационные позы, лимиты, цепочки, `tg_jacobian`, `external_wrench` |
| Stage 2W.2 | `/home/v/projects/робот/robot_pose_study_v1/arm_architecture_comparison_v1/audit_v2w_2/audit_v2w2.py` | массовые модели A/B1/B2, `build_points_v2`, `mass_matrix_audit` |
| Stage 2W.2 sign_correction | `/home/v/projects/робот/robot_pose_study_v1/arm_architecture_comparison_v1/audit_v2w_2/sign_correction/audit_v2w2_signs.py` | исправленные знаки, `transition`, `legacy_elements` |
| Лимиты осей | `/home/v/projects/робот/joint_limits_audit.txt` | 7 осей руки, `high` confidence (yaml+prose) |
| Прежний аудит приводов | `/home/v/projects/робот/arm_load_audit/arm_actuator_feasibility_v1/arm_actuator_feasibility.md`, `arm_actuator_requirements.csv` | контрольные 1093.6 / 1203.5 Н·м, хваты K1–K3t |
| Массовый бюджет | `/home/v/projects/робот/robot_pose_study_v1/stage2G/STAGE2G_V7_5_REPORT.md`, `cad/stage2g_v7_4_budget_cm.json` | лимит 500 кг, варианты В1–В4, неопределённые подсистемы |
| Stage 2V | `/home/v/projects/робот/robot_pose_study_v1/stage2V/STAGE2V_MASS_INERTIA_AUDIT.md` | реестр 182 объектов Blender; **42 объекта с массой, 133 без**; `base_link` 213.134 кг одним блоком; торс и лифт — без массы; stored-руки 5.950 + 7.451 = 13.4 кг (признаки placeholder: forearm = shoulder, L ≠ R) |
| Параметрический бюджет рук | `/home/v/projects/робот/arm_load_audit/arm_mass_budget_v1/arm_mass_budget.md` | `m = k·T*`, k = 0.015 / 0.030 / 0.060 кг/(Н·м) — **этот** источник параметрики, а не Stage 2V |
| COM звеньев | `/home/v/projects/робот/robot_pose_study_v1/stage2G/STAGE2G_V7_5_REPORT.md` §2 | **0 из 182 объектов имеют COM** → все «положения ЦМ» = origin (допущение) |

## 2. Промышленное оборудование (веб, 2026-11-04)

### 2.1 Nabtesco

| URL | Что взято | Статус |
|---|---|---|
| `https://precision.nabtesco.com/en/` | состав линейки: RV-N, RV-C, RV-E, AF-N, RDS-E/RDS-C, RDR, RDP, GH, RS | [Ф] |
| `https://precision.nabtesco.com/en/products/` | полный Product Lineup, обозначения серий | [Ф] |
| `https://precision.nabtesco.com/en/products/detail/RV-N` | таблица спецификаций RV-25N…RV-700N: Rated Torque, Allowable Acceleration/Deceleration Torque, Ratio, Rated Output Speed, Rated life, Backlash, Lost Motion, Torsional/Moment Rigidity, Start-up Efficiency, Moment of Inertia | [Ф] |
| `https://precision.nabtesco.com/en/download/` | список каталогов; скачивание каталога RV-N (`/en/member/download/f/catalog/52532`) — **302 → login** | [Н] каталог под логином |

**Ограничение:** поле `Mass (kg)` в таблице RV-N **пустое во всех 60 строках**
(проверено парсером `_parse_rvn.py`). Допустимые радиальные/осевые/опрокидывающие
нагрузки (`Allowable Moment 1/2`, `Allowance Thrust 1/2`) и момент тормоза
(`Brake Holding Torque`) на странице тоже пусты. Сохранено в
`sources/rvn_specs.json`.

**Сохранено:** `sources/rvn_page.html` (300 078 байт).

### 2.2 Harmonic Drive SE (Германия)

| URL | Что взято | Статус |
|---|---|---|
| `https://harmonicdrive.de/en/` | состав группы, ссылки на каталоги | [Ф] |
| `https://harmonicdrive.de/fileadmin/Downloads/Produkte/Kataloge/Harmonic_Drive_Gears_EN_1053522.pdf` | каталог **Harmonic Drive® Gears**, секция 2.1 `CSG-2UH/HFUC-2UH`, Table 2.1.4: TR/TA/TN/TM, входные скорости, момент инерции, масса 2UH и 2UH-LW | [Ф] |
| `https://harmonicdrive.de/fileadmin/Downloads/Produkte/Kataloge/Harmonic_Drive_Mechatronics_EN_1053524.pdf` | каталог **Harmonic Drive® Mechatronics**: CanisDrive-14B/17B/20B/25B/32B/40B (Table 1.3.7/1.3.8), CanisDrive-50A/58A (Table 1.3.9), IHD-17A/20A/25A: TMAX, **T0 (continuous stall torque)**, nMAX, тормоз (TBr, время срабатывания), масса с/без тормоза | [Ф] |

**Сохранено:** `sources/harmonic_gears.pdf` (49 989 368 байт),
`sources/harmonic_gears.txt` (18 473 строки, `pdftotext -layout`),
`sources/harmonic_mech.pdf` (47 055 564 байта), `sources/harmonic_mech.txt`
(21 693 строки).

**Важно о массах (контроль):** массы из каталога SE совпали с прежним аудитом
`arm_actuator_candidates.csv` для аналогичных размеров: size 40 → 5.0 кг,
size 50 → 8.9 кг, size 65 → 20.9 кг. **Но это РАЗНЫЕ серии и РАЗНЫЕ
производители:** прежний аудит брал `CSF-2UH` с `harmonicdrive.net`
(Harmonic Drive LLC, Япония/США), настоящий каталог — `CSG-2UH/HFUC-2UH` с
`harmonicdrive.de` (Harmonic Drive SE, Германия). Числа не отождествляются.

**Не использовано:** `FHA-C` (серия FHA-C Mini в каталоге мехатроники — только
малый класс до 28 Н·м; полная FHA-C не попала в скачанные каталоги) — [Н].

### 2.3 SPINEA (Словакия)

| URL | Что взято | Статус |
|---|---|---|
| `https://www.spinea.com/en/` | состав линейки TwinSpin T/E/H/M/G/W, DriveSpin Standard/Hollowshaft/Modular/Flat | [Ф] |
| `https://www.spinea.com/en/products/drivespin/standard/ds155` | DS 155: i=63/109/133, Tr=460 Н·м, Tmax=1150 Н·м, nr=2000 об/мин | [Ф] |
| `https://www.spinea.com/en/products/drivespin/standard/ds140` | DS 140: i=69/115, Tr=268 Н·м, Tmax=670 Н·м, nr=2000 об/мин | [Ф] |
| `https://www.spinea.com/en/products/drivespin/hollowshaft/dsh170` | DSH 170: i=69/125, полый вал Ø40 мм, Tr=420 Н·м, Tmax=1050 Н·м | [Ф] |
| `https://www.spinea.com/en/downloads` | каталоги доступны **только после формы** (email + страна + согласие на обработку ПДн) | [Н] каталог не скачан |

**Не приписано:** масса, момент инерции, тормоз, допустимые нагрузки — на
страницах изделий SPINEA этих полей нет. В `actuator_candidates.csv` отмечено
`UNKNOWN`.

**Не отправлялось:** ни один запрос производителю не оформлялся (ограничение
задания).

### 2.4 Прежние данные (Stage 2X использует как перекрёстную проверку)

Из `/home/v/projects/робот/arm_load_audit/arm_actuator_feasibility_v1/`
(дата фиксации 2026-10-18, источники — `harmonicdrive.net`, `zeroerr.com`):

| Изделие | Что взято | Примечание |
|---|---|---|
| CSF-65-120-2UH | rated 951, repeated 2510, momentary 4750 Н·м, масса 20.9 кг | другая серия/производитель |
| CSF-50-120-2UH | rated 529, repeated 1080, momentary 2060 Н·м, масса 8.9 кг | — |
| CSF-40-120-2UH | rated 294, avg 451, repeated 617, momentary 1180 Н·м, 5.0 кг | — |
| FHA-40C-120 | continuous 225, max 756 Н·м, 12 кг, опция тормоза «B» | полный сервоактуатор |
| eRob170I | Max.Torque 1180 Н·м, 9.5 кг, фрикционный тормоз | параметр-лист PDF — image-only |

Эти данные в `actuator_candidates.csv` **не переносились** (чтобы не смешивать
серии); они используются только для сопоставления порядка величин.

## 3. Что НЕ найдено (открытые источники)

| Параметр | Источник | Статус |
|---|---|---|
| Масса редукторов Nabtesco RV-N | страница изделия — поле пустое; каталог под логином | [Н] |
| Допустимые радиальные/осевые/моментные нагрузки RV-N | то же | [Н] |
| Момент удерживающего тормоза RV-N | то же | [Н] |
| Масса/тормоз/энкодер SPINEA DriveSpin | страницы изделий; каталог за формой | [Н] |
| Цена и сроки поставки у всех производителей | не публикуются на страницах | [Н] |
| Момент инерции ротора для подбора (кроме RV-N и CanisDrive) | частично в каталогах | [Ч] частично |
| КПД редукторов | Harmonic Drive SE — в разделе Engineering data; для CSG-2UH не извлечён | [Н] для выбранных |
| Момент инерции ротора/двигателя для теплового расчёта удержания | — | [Н] |
| Допустимое время удержания нагрузки | — | [Н], не заявлено производителем в открытых источниках |

## 4. Воспроизводимость

| Файл | Назначение |
|---|---|
| `joint_requirements.py` | восстановление таблицы требований (сетка поз + уточнение, нагрузки на опоры) |
| `mass_budget.py` | проверка лимита 500 кг и двойного учёта |
| `selection_calculations.py` | реестр кандидатов, A vs B1a/B1b, подбор по T0, пересчёт масс, инерция, бюджет |
| `_parse_rvn.py` | парсинг таблицы RV-N из сохранённого HTML по классам `<td>` |
| `_parse_harmonic.py` | парсинг Table 2.1.4 каталога Gears из текста `pdftotext` |
| `_verify_req.py` | независимый пересчёт моментов из сохранённых поз JSON |
| `_probe_torque.py`, `_timing.py`, `_check_w2.py` | разовые проверки (сетка, замеры, аномалия w2) |

Запуск: `python3 joint_requirements.py && python3 mass_budget.py && python3 selection_calculations.py`
(из каталога Stage 2X). Зависимости не устанавливались.

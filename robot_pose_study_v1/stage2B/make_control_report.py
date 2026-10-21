#!/usr/bin/env python3
"""Generate the stage 2B P6/P7 control report from motion_study.json."""

from __future__ import annotations

import collections
import json
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
PAYLOAD = json.loads((HERE / "motion_study.json").read_text(encoding="utf-8"))


def fmt(value: Any, digits: int = 4) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def table(header: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join("" if value is None else str(value) for value in row) + " |")
    return "\n".join(lines)


def q_text(q: dict[str, float] | None) -> str:
    if not q:
        return "—"
    return json.dumps(q, ensure_ascii=False)


def global_critical(operation: dict[str, Any]) -> dict[str, dict[str, Any]]:
    best: dict[str, dict[str, Any]] = {
        "min_clearance": {"value": None},
        "min_margin": {"value": None},
        "max_condition": {"value": None},
    }
    for transition in operation["transitions"]:
        selected = transition.get("selected")
        if not selected:
            continue
        for index, sample in enumerate(selected["samples"]):
            for key, field, mode in (
                ("min_clearance", "clearance_min_m", "min"),
                ("min_margin", "min_margin_deg", "min"),
                ("max_condition", "condition", "max"),
            ):
                value = sample.get(field)
                if value is None:
                    continue
                current = best[key]["value"]
                if current is None or (mode == "min" and value < current) or (mode == "max" and value > current):
                    best[key] = {
                        "value": value,
                        "transition": f"{transition['from']} → {transition['to']}",
                        "sample_index": index,
                        "t": sample.get("t"),
                        "q_deg": sample.get("q_deg"),
                    }
    return best


def main() -> None:
    meta = PAYLOAD["metadata"]
    operations = PAYLOAD["operations"]
    bimanual = PAYLOAD["bimanual"]
    sensitivity = PAYLOAD.get("sensitivity", {})

    lines: list[str] = []
    lines.append("# Stage 2B — контрольный отчёт P6/P7")
    lines.append("")
    lines.append("**Статус:** кинематическая проверка последовательностей движений. Это не оценка безопасности, сил, моментов или контакта с пациентом.")
    lines.append("")
    lines.append(f"**Дата расчёта:** {meta['generated_at']}  ")
    lines.append(f"**Код:** `motion_study.py`, Git HEAD на момент расчёта: `{meta['git_head']}`  ")
    lines.append(f"**Исходная модель:** `{meta.get('blend_sha256')}`  ")
    lines.append(f"**Параметры:** боковой подход, H={meta['baseline']['height_m']} м, W={meta['baseline']['width_m']} м, зазор базы={meta['baseline']['clearance_m']} м.")
    lines.append("")

    lines.append("## 0. Краткий вывод")
    lines.append("")
    lines.append("- 🟢 Для P6 и P7 при боковом подходе, H=0.7 м, W=1.0 м, c=0.10 м найдены непрерывные 5D-траектории одной руки для L/R и синхронные траектории двух рук.")
    lines.append("- 🟢 Все 20 переходов P6/P7 (4 операции × 5 переходов) имеют статус `valid`: лимиты соблюдены, столкновений в упрощённой модели нет.")
    lines.append("- 🟡 Минимальный зазор вдоль траекторий — 0.0477 м (P6 R и синхронная P6), это зазор рука–кровать; минимум запаса в базовом случае — 5.46° (P7 R).")
    lines.append("- 🟡 P7 L при c=0.15 остаётся кинематически возможной, но имеет запас 2.41° и `cond` 44.7 — пограничный случай, не рабочий запас.")
    lines.append("- 🔴 6D-траектории не планировались; в 5D ориентация пальцев вокруг нормали не гарантируется. Для P6/P7 в сетке этапа 2А полная 6D достижима только при `+X`, а не при `+Y`.")
    lines.append("- ⬛ Силы, моменты, устойчивость, пациент, простыня/подгузник и реальные collision-меши не моделировались.")
    lines.append("")

    lines.append("## 1. Исходные данные и воспроизводимость")
    lines.append("")
    lines.append(f"- 🟢 `workspace_search.json`: {meta['source_counts']['json_rows']} строк; `workspace_search.csv`: {meta['source_counts']['csv_rows']} строк.")
    lines.append("- 🟢 На старте этапа Git HEAD был `d6f064d`, рабочее дерево чистое; `.blend` SHA256 `1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb6440708289829cd280` не изменялся.")
    lines.append("- 🟢 Воспроизведены 8 решений P6/P7 (5D/6D, L/R): position, normal, orientation, `cond`, margin и clearance совпали с `workspace_search.json`.")
    lines.append("- 🟢 Для выбранных переходов повторная FK-проверка концов дала максимальную ошибку положения ≤5·10⁻⁵ мм и нормали ≤1.3·10⁻⁶°.")
    lines.append("- 🟡 Проверка траекторий дискретная: шаг 1 см по дескриптору и 0.5° по суставам; столкновения проверялись в каждом сэмпле, но между сэмплами формально не исключены.")
    lines.append("")

    lines.append("## 2. Последовательности опорных положений")
    lines.append("")
    lines.append("P6: `ready → above → grasp → tension → lift → ready`. P7: `ready → above → contact → outward manipulation → lift → ready`.")
    lines.append("")
    sequence_rows = []
    for operation, sides in operations.items():
        for side, result in sides.items():
            for reference in result["references"]:
                sequence_rows.append([
                    operation,
                    side,
                    reference["label"],
                    fmt(reference["position_floor"][0], 3),
                    fmt(reference["position_floor"][1], 3),
                    fmt(reference["position_floor"][2], 3),
                    "✅" if reference["reachable"] else "❌",
                    fmt(reference["min_margin_deg"], 2),
                    fmt(reference["condition"], 2),
                ])
    lines.append(table(
        ["Операция", "Рука", "Точка", "x, м", "y, м", "z, м", "Достижима", "Запас, °", "cond"],
        sequence_rows,
    ))
    lines.append("")
    lines.append("- 5D-точки задают позицию контактной грани и нормаль; поворот вокруг нормали свободен.")
    lines.append("- 6D-статус в этапе 2А: для P6/P7 при H=0.7, W=1.0, c=0.10 достижим только `+X` (по одной строке на руку); `+Y` для P7 в 6D не достижим. Поэтому эти траектории нельзя называть 6D-траекториями.")
    lines.append("")

    lines.append("## 3. Достижимость переходов")
    lines.append("")
    transition_rows = []
    method_counts: collections.Counter[str] = collections.Counter()
    candidate_total = 0
    candidate_valid = 0
    for operation, sides in operations.items():
        for side, result in sides.items():
            for transition in result["transitions"]:
                selected = transition.get("selected")
                if selected:
                    method_counts[selected["method"]] += 1
                for candidate in transition["candidates"]:
                    candidate_total += 1
                    candidate_valid += candidate["status"] == "valid"
                transition_rows.append([
                    operation,
                    side,
                    f"{transition['from']} → {transition['to']}",
                    transition["status"],
                    selected["method"] if selected else "—",
                    selected["sample_count"] if selected else 0,
                    fmt(selected["min_clearance_m"] if selected else None, 4),
                    fmt(selected["min_margin_deg"] if selected else None, 2),
                    fmt(selected["max_condition"] if selected else None, 2),
                    fmt(selected["max_joint_step_deg"] if selected else None, 2),
                ])
    lines.append(table(
        ["Операция", "Рука", "Переход", "Статус", "Метод", "Сэмплов", "Мин. зазор, м", "Мин. запас, °", "Макс. cond", "Макс. шаг, °"],
        transition_rows,
    ))
    lines.append("")
    lines.append(f"- Всего кандидатов: {candidate_total}; валидных: {candidate_valid}; невалидных: {candidate_total - candidate_valid}.")
    lines.append(f"- Выбранные методы: {dict(method_counts)}.")
    lines.append("- Политика выбора: сначала отбрасываются невалидные кандидаты; затем предпочитаются пути с максимальным шагом ≤6°; среди них — максимальный запас до лимитов, затем максимальный зазор, затем минимальный `cond`, затем короткий путь.")
    lines.append("- Линейная интерполяция суставов не принималась автоматически: она проверялась как `joint_lerp` наравне с `task_space`, `via_lift` и альтернативными ветвями.")
    lines.append("- «Макс. шаг» — изменение суставов между соседними сэмплами, а не скорость; для задачи планирования это мера гладкости дискретной траектории.")
    lines.append("")

    lines.append("## 4. Минимальные зазоры и критические конфигурации")
    lines.append("")
    clearance_rows = []
    critical_rows = []
    for operation, sides in operations.items():
        for side, result in sides.items():
            critical = global_critical(result)
            summary = result.get("summary")
            clearance_rows.append([
                operation,
                side,
                fmt(summary["min_clearance_m"] if summary else None, 4),
                fmt(summary["min_margin_deg"] if summary else None, 2),
                fmt(summary["max_condition"] if summary else None, 2),
                fmt(summary["max_joint_step_deg"] if summary else None, 2),
            ])
            for key, label, digits in (
                ("min_clearance", "мин. зазор", 4),
                ("min_margin", "мин. запас", 2),
                ("max_condition", "макс. cond", 2),
            ):
                item = critical[key]
                critical_rows.append([
                    operation,
                    side,
                    label,
                    fmt(item.get("value"), digits),
                    item.get("transition", "—"),
                    item.get("sample_index", "—"),
                    q_text(item.get("q_deg")),
                ])
    lines.append(table(
        ["Операция", "Рука", "Мин. зазор, м", "Мин. запас, °", "Макс. cond", "Макс. шаг, °"],
        clearance_rows,
    ))
    lines.append("")
    lines.append(table(
        ["Операция", "Рука", "Критерий", "Значение", "Переход", "Индекс", "q, °"],
        critical_rows,
    ))
    lines.append("")
    lines.append("- Минимальный зазор во всех случаях — рука–кровать в упрощённой модели, не реальная кисть/объект.")
    lines.append("- Полный список критических сэмплов и все q находятся в `motion_study.json` (`critical`) и `motion_samples.csv`.")
    lines.append("")

    lines.append("## 5. Синхронная работа двух рук")
    lines.append("")
    bimanual_rows = []
    for operation, result in bimanual.items():
        selected = result.get("selected")
        bimanual_rows.append([
            operation,
            result["status"],
            selected["mode"] if selected else "—",
            fmt(selected["min_clearance_m"] if selected else None, 4),
            fmt(selected["min_arm_arm_m"] if selected else None, 4),
            fmt(selected["min_margin_deg"] if selected else None, 2),
            fmt(selected["max_condition"] if selected else None, 2),
        ])
    lines.append(table(
        ["Операция", "Статус", "Выбранный режим", "Мин. зазор, м", "Мин. рука–рука, м", "Мин. запас, °", "Макс. cond"],
        bimanual_rows,
    ))
    lines.append("")
    lines.append("Все три режима (`synchronized`, `sequential_L_then_R`, `phase_shift_0.2`) для P6 и P7 валидны; выбран синхронный режим, так как он соответствует требованию синхронной работы.")
    lines.append("")
    lines.append("- `min_arm_arm_m` — общий минимум рука–рука в упрощённой модели, не distal-метрика; для P7 ср. distal 0.1801 м из этапа 2А.")
    lines.append("- Зазор рука–рука 0.1839 м не означает безопасность: реальные кисти, объект и пациент не моделировались.")
    lines.append("")

    lines.append("## 6. Чувствительность к расположению кровати")
    lines.append("")
    sensitivity_rows = []
    for case, result in sensitivity.items():
        for side, operation_result in result.items():
            summary = operation_result.get("summary")
            sensitivity_rows.append([
                case,
                side,
                operation_result["status"],
                fmt(summary["min_clearance_m"] if summary else None, 4),
                fmt(summary["min_margin_deg"] if summary else None, 2),
                fmt(summary["max_condition"] if summary else None, 2),
                fmt(summary["max_joint_step_deg"] if summary else None, 2),
            ])
    lines.append(table(
        ["Случай", "Рука", "Статус", "Мин. зазор, м", "Мин. запас, °", "Макс. cond", "Макс. шаг, °"],
        sensitivity_rows,
    ))
    lines.append("")
    lines.append("- Динамически варьировался только зазор базы c=0.05/0.10/0.15 м для P6 L и P7 L; H=0.7 м, W=1.0 м.")
    lines.append("- P7 L при c=0.15: 4 старта статического IK не нашли решение, 12 стартов нашли; это чувствительность численного поиска, а не доказательство невозможности. Запас 2.41°, `cond` 44.7 — пограничная конфигурация.")
    lines.append("- Статическая чувствительность к H/W из этапа 2А: P6 5D 184/192; P7 5D 55/192 (только боковой), фронтальный P7 0/96. Для P7 L боковой подход: W=1.2 — 0/4 уже при c=0.10; W=1.0, c=0.20 — 0/4.")
    lines.append("- Динамические траектории при H≠0.7 или W≠1.0 в этом контрольном прогоне не проверялись.")
    lines.append("")

    lines.append("## 7. Предварительные требования к рабочему пространству")
    lines.append("")
    lines.append("- 🟢 P6/P7 требуют бокового подхода; для P7 фронтальный подход в этапе 2А не достижим.")
    lines.append("- 🟢 Базовый случай H=0.7, W=1.0, c=0.10: P6 L/R и P7 L/R проходят; синхронные P6/P7 проходят.")
    lines.append("- 🟡 P6: c=0.05–0.15 проходят; минимальный зазор 47.7–54.9 мм (рука–кровать), запас 9.7–17.6° в проверенных случаях.")
    lines.append("- 🟡 P7: при c=0.10 запас L 12.16°, R 5.46°; при c=0.15 L — 2.41° и `cond` 44.7. Для P7 L рекомендуется c≤0.10 или отдельная перепроверка/перемещение базы.")
    lines.append("- 🟡 Дополнительные степени свободы для 5D-траекторий не требуются. Для полной 6D-ориентации P6/P7 в текущей сетке доступен только `+X`; если нужен `+Y`, требуется другая ориентация/база/DOF.")
    lines.append("- 🟡 Перемещение базы для P6/P7 в базовом случае не требуется. Для P7 L при c=0.15 уменьшение c (приближение базы) выглядит предпочтительнее удлинения руки, но это не проверено как отдельная операция.")
    lines.append("")

    lines.append("### 7.1. Операции, для которых нужны изменения конструкции")
    lines.append("")
    lines.append(table(
        ["Операция", "Статус этапа 2B", "Вывод"],
        [
            ["P6", "5D L/R и синхронная — valid", "Обязательных изменений не доказано; следить за зазором 47.7 мм у P6 R."],
            ["P7", "5D L/R и синхронная — valid", "Обязательных изменений не доказано; P7 R имеет запас 5.46°, P7 L c=0.15 — 2.41°; 6D `+Y` не достижим."],
            ["P5", "В 2B не пересчитывалась", "По этапу 2А: 0/192; нужен анализ перемещения базы/длины руки/противоположной стороны, а не вывод о невозможности."],
            ["P8/P9", "В 2B не исследовались", "После согласования P6/P7."],
        ],
    ))
    lines.append("")

    lines.append("### 7.2. Исходные данные для последующего расчёта сил, моментов и устойчивости")
    lines.append("")
    lines.append("- Массы, тензоры инерции и положения центров масс звеньев обеих рук, кистей и полезной нагрузки.")
    lines.append("- Кинематические пределы, максимальные скорости/ускорения и допустимые моменты приводов.")
    lines.append("- Модель контакта: жёсткость/демпфирование, трение, допустимое давление на пациента, площадь контакта.")
    lines.append("- Свойства простыни/подгузника: масса, трение, деформация, сила натяжения; масса и геометрия предмета для P9.")
    lines.append("- Антропометрия пациента, положение тела, допустимые зоны контакта и усилия; поручни/ограждения кровати.")
    lines.append("- Реальная collision-геометрия кистей, объекта, кровати и пациента; погрешности датчиков и оценка положения.")
    lines.append("- Требования по устойчивости базы, распределению нагрузки и аварийному освобождению.")
    lines.append("- ⬛ Силовые расчёты в этом этапе не выполнялись.")
    lines.append("")

    lines.append("## 8. Что нельзя достоверно смоделировать")
    lines.append("")
    lines.append("- 🟡 Деформацию и захват простыни/подгузника, трение и силу натяжения.")
    lines.append("- 🟡 Контакт с пациентом, давление, поворот/подъём пациента, работу с поручнями.")
    lines.append("- 🟡 Реальную геометрию кистей, объекта и постели; упрощённая collision-модель может пропускать контакты.")
    lines.append("- ⬛ Силы, моменты, устойчивость, приводы, датчики, управление и динамику.")
    lines.append("- 🟡 Дискретная проверка не является формальным доказательством отсутствия столкновений между сэмплами.")
    lines.append("")

    lines.append("## 9. «Траектория не найдена» и «математически невозможна»")
    lines.append("")
    lines.append("- В базовом случае P6/P7 все 20 переходов найдены; статуса `not_found` нет.")
    lines.append("- P7 L при c=0.15 сначала не находилась при 4 стартах, но найдена при 12 стартах. Это «не найдена при данном бюджете поиска», а не «математически невозможна».")
    lines.append("- Для P5 в этапе 2А 0/192 — это геометрический предел текущей базы/руки, а не доказательство невозможности при другой конструкции.")
    lines.append("- Ни один результат не следует трактовать как доказательство невозможности без анализа альтернативной базы, длины руки и ориентации.")
    lines.append("")

    lines.append("## 10. Артефакты и воспроизводимость")
    lines.append("")
    lines.append("- `motion_study.py` — планировщик и проверка; запуск: `python3 -B motion_study.py`.")
    lines.append("- `motion_study.json` — полные траектории, кандидаты, критические сэмплы, чувствительность.")
    lines.append("- `motion_sequences.csv`, `motion_transitions.csv`, `motion_samples.csv`, `motion_bimanual.csv`, `motion_sensitivity.csv`.")
    lines.append("- Исходный `.blend` не изменялся; проектная документация не изменялась; `/tmp` не использовался; зависимости не устанавливались.")
    lines.append("- Git: результаты сохраняются отдельным локальным commit; push не выполнялся.")
    lines.append("")

    lines.append("## 11. Следующий шаг")
    lines.append("")
    lines.append("Контрольное исследование P6/P7 завершено. По требованию этапа 2Б работа останавливается до согласования. После согласования: P8/P9, затем P5 с анализом перемещения базы/противоположной стороны; силовые расчёты — только после отдельного разрешения.")
    lines.append("")

    (HERE / "control_report_p6_p7.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"WROTE {HERE / 'control_report_p6_p7.md'}")


if __name__ == "__main__":
    main()

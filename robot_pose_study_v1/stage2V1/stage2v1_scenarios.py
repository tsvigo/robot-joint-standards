#!/usr/bin/env python3
"""Stage 2V.1 — three consistent mass-model scenarios (light / nominal / heavy).

Read-only. Reads:
- stage2v1_model_structure.json   (registry, recipes, subsystems)
- stage2v1_mesh_inertia.json      (per-mesh mass/COM/I at rho cases, world space)
Writes:
- stage2v1_scenarios.json         (machine-readable scenarios)
- stage2v1_scenarios.md           (human summary)

Scenario axes are component-derived:
- material of designed lift parts (Al 2700 vs steel 7800) — engineering assumption,
  stated per part group;
- arm-link/hand/finger mass basis: stored value vs solid-aluminum 2700*|V|
  (per component, min->light, max->heavy, stored->nominal);
- torso: 2700*|V_local| from mesh geometry (designed part, no stored mass);
- base/head/wheels/sensors: recipe values fixed across scenarios (geometric facts),
  with U2 flag on base composition.
Non-geometric content (cables, fasteners, finger drives) is NOT put into totals
as fabricated numbers: parametric addendum with formulas + intervals only.

No moment/stability/dynamics calculations. No Blender changes.
"""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
STRUCT = json.loads((HERE / "stage2v1_model_structure.json").read_text(encoding="utf-8"))
INERT = json.loads((HERE / "stage2v1_mesh_inertia.json").read_text(encoding="utf-8"))
OUT_JSON = HERE / "stage2v1_scenarios.json"
OUT_MD = HERE / "stage2v1_scenarios.md"

MESH = INERT["meshes"]

# ---------------------------------------------------------------------------
# Component classification for scenarios
# ---------------------------------------------------------------------------
# Fixed geometric recipes (identical in all scenarios)
FIXED_RECIPE = ["base_link", "head_link", "wheel_BL", "wheel_BR", "wheel_FL",
                "wheel_FR", "speaker", "stereo_microphone_1", "stereo_microphone_2",
                "odor_sensor", "ToF_Lens_001"]

# Lift parts by function (for nominal material split)
LIFT_TRANSMISSION = ["lift_screw_left", "lift_screw_right", "lift_guide_left",
                     "lift_guide_right", "lift_guide_block_left",
                     "lift_guide_block_right", "lift_sync_shaft",
                     "lift_nut_mount_left", "lift_nut_mount_right",
                     "lift_motor", "lift_brake"]           # steel in nominal
LIFT_BRACKETS = ["lift_stop_top", "lift_catcher_placeholder"]  # Al in nominal
LIFT_ALL = LIFT_TRANSMISSION + LIFT_BRACKETS

# Arm subsystems where stored vs solid-aluminum are the scenario bounds
STORED_BASIS = ["left_shoulder_link", "left_forearm_link", "right_shoulder_link",
                "right_forearm_link", "left_palm_link", "right_palm_link"]
FINGERS = sorted(n for n in MESH if (("digit" in n or "thumb" in n)
                 and n not in ("",)) and not n.endswith("_collision"))
# torso: designed part, no stored mass
TORSO = "torso_link"

RHO_AL = 2700.0
RHO_ST = 7800.0

# Parametric addendum (NOT in totals)
CABLE_LAMBDA = {"light": 0.08, "nominal": 0.15, "heavy": 0.30}  # kg/m
CABLE_TRACK_M = 2.5        # geometric estimate: 2 arms ~1.0 m each + torso/lift ~0.5
FASTENER_PCT = {"light": 0.02, "nominal": 0.035, "heavy": 0.05}
FINGER_DRIVES = "symbolic: delta_M >= 0 (no data, not estimated)"


def tensor(I):
    return [list(row) for row in I]


def parallel_axis(I_com, m, c, c_ref):
    """Shift inertia tensor from c to c_ref: I_ref = I_com + m(|d|^2 E - d d^T)."""
    d = [c_ref[0] - c[0], c_ref[1] - c[1], c_ref[2] - c[2]]
    d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            delta = (d2 if i == j else 0.0) - d[i] * d[j]
            out[i][j] = I_com[i][j] + m * delta
    return out


def add_tensors(A, B):
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]


def component_value(name, scenario):
    """-> (mass, com, I_com, basis_note)."""
    mesh = MESH[name]
    stored = mesh.get("stored_mass")

    # 1. fixed recipe components
    if name in FIXED_RECIPE:
        r = mesh["rhos"].get("stored_shape") or mesh["rhos"]["aluminum_2700"]
        return r["mass"], r["com_world"], r["I_com_world"], \
            "fixed geometric recipe (stored = rho*V_local)"

    # 2. torso (designed, no stored)
    if name == TORSO:
        r = mesh["rhos"]["aluminum_2700"]
        return r["mass"], r["com_world"], r["I_com_world"], \
            "designed part: m = 2700*|V| (mesh geometry, no stored mass)"

    # 3. lift parts
    if name in LIFT_ALL:
        label = "aluminum_2700" if scenario == "light" else (
            "steel_7800" if scenario == "heavy" else None)
        if label is None:  # nominal: per-function material
            label = "steel_7800" if name in LIFT_TRANSMISSION else "aluminum_2700"
        r = mesh["rhos"][label]
        return r["mass"], r["com_world"], r["I_com_world"], \
            f"designed part: m = {'2700' if 'aluminum' in label else '7800'}*|V| ({label})"

    # 4. stored-basis links (arms/hands): stored vs solid-aluminum bounds
    if name in STORED_BASIS:
        s = mesh["rhos"]["stored_shape"]
        al = mesh["rhos"]["aluminum_2700"]
        if scenario == "light":
            pick, note = min((s, "stored (unverified, category 3)"),
                             (al, "solid aluminum 2700*|V|"), key=lambda x: x[0]["mass"])
        elif scenario == "heavy":
            pick, note = max((s, "stored (unverified, category 3)"),
                             (al, "solid aluminum 2700*|V|"), key=lambda x: x[0]["mass"])
        else:
            pick, note = s, "stored (unverified, category 3)"
        return pick["mass"], pick["com_world"], pick["I_com_world"], note

    # 5. finger phalanges: stored vs solid-aluminum bounds (same rule)
    if "digit" in name or "thumb" in name:
        s = mesh["rhos"].get("stored_shape")
        al = mesh["rhos"]["aluminum_2700"]
        if s is None:
            r = al
            return r["mass"], r["com_world"], r["I_com_world"], \
                "no stored mass: solid aluminum 2700*|V|"
        if scenario == "light":
            pick, note = min((s, "stored placeholder (category 3)"),
                             (al, "solid aluminum 2700*|V|"), key=lambda x: x[0]["mass"])
        elif scenario == "heavy":
            pick, note = max((s, "stored placeholder (category 3)"),
                             (al, "solid aluminum 2700*|V|"), key=lambda x: x[0]["mass"])
        else:
            pick, note = s, "stored placeholder (category 3)"
        return pick["mass"], pick["com_world"], pick["I_com_world"], note

    raise KeyError(f"unclassified component: {name}")


SUBSYSTEM_OF = {}
SUBSYSTEM_OF["base_link"] = "mobile_base"
for w in ("wheel_BL", "wheel_BR", "wheel_FL", "wheel_FR"):
    SUBSYSTEM_OF[w] = "wheels"
for n in LIFT_ALL:
    SUBSYSTEM_OF[n] = "lift"
SUBSYSTEM_OF[TORSO] = "torso"
SUBSYSTEM_OF["head_link"] = "head"
SUBSYSTEM_OF["left_shoulder_link"] = "left_arm"
SUBSYSTEM_OF["left_forearm_link"] = "left_arm"
SUBSYSTEM_OF["right_shoulder_link"] = "right_arm"
SUBSYSTEM_OF["right_forearm_link"] = "right_arm"
SUBSYSTEM_OF["left_palm_link"] = "left_hand"
SUBSYSTEM_OF["right_palm_link"] = "right_hand"
for n in FINGERS:
    if "left" in n or (n.startswith(("first", "second", "third")) and "left" in n):
        SUBSYSTEM_OF[n] = "left_fingers" if "thumb" not in n else "left_hand"
    if "right" in n or (n.startswith(("first", "second", "third")) and "right" in n):
        SUBSYSTEM_OF[n] = "right_fingers" if "thumb" not in n else "right_hand"
# fix thumb classification explicitly
for n in FINGERS:
    if "thumb" in n:
        SUBSYSTEM_OF[n] = "left_hand" if "left" in n else "right_hand"
for n in ("speaker", "stereo_microphone_1", "stereo_microphone_2",
          "odor_sensor", "ToF_Lens_001"):
    SUBSYSTEM_OF[n] = "sensors"
# AXLE_GUIDE*: no mass, ignore (not in MESH-with-mass loop)

SCENARIOS = ("light", "nominal", "heavy")


def build_scenario(scenario):
    parts = {}   # subsystem -> list of dicts
    unclassified = []
    for name, mesh in MESH.items():
        stored = mesh.get("stored_mass")
        is_finger = ("digit" in name or "thumb" in name) and not name.endswith("_collision")
        if name == "wheel_FL_collision":
            continue
        # include: fixed recipe, designed lift parts, torso, fingers (even without
        # stored mass - geometry exists), stored-basis arm links; skip the rest
        eligible = (name in FIXED_RECIPE or name in LIFT_ALL or name == TORSO
                    or is_finger or name in STORED_BASIS)
        if not eligible:
            continue
        if stored is None and not (name == TORSO or name.startswith("lift_") or is_finger):
            continue
        try:
            m, c, I, note = component_value(name, scenario)
        except KeyError:
            unclassified.append(name)
            continue
        sub = SUBSYSTEM_OF.get(name)
        if sub is None:
            unclassified.append(name)
            continue
        parts.setdefault(sub, []).append(
            {"link_id": name, "mass": m, "com": c, "I": I, "basis": note})

    # aggregate per subsystem
    result = {}
    for sub, items in sorted(parts.items()):
        M = sum(i["mass"] for i in items)
        cx = sum(i["mass"] * i["com"][0] for i in items) / M
        cy = sum(i["mass"] * i["com"][1] for i in items) / M
        cz = sum(i["mass"] * i["com"][2] for i in items) / M
        com = [cx, cy, cz]
        I_tot = [[0.0] * 3 for _ in range(3)]
        for i in items:
            I_shift = parallel_axis(i["I"], i["mass"], i["com"], com)
            I_tot = add_tensors(I_tot, I_shift)
        result[sub] = {
            "mass_kg": round(M, 6),
            "com_world": [round(v, 6) for v in com],
            "I_com_world": [[round(v, 8) for v in row] for row in I_tot],
            "components": [
                {"link_id": i["link_id"], "mass_kg": round(i["mass"], 6),
                 "basis": i["basis"]} for i in sorted(items, key=lambda x: -x["mass"])],
            "component_count": len(items),
        }

    # total
    M_all = sum(v["mass_kg"] for v in result.values())
    cx = sum(v["mass_kg"] * v["com_world"][0] for v in result.values()) / M_all
    cy = sum(v["mass_kg"] * v["com_world"][1] for v in result.values()) / M_all
    cz = sum(v["mass_kg"] * v["com_world"][2] for v in result.values()) / M_all
    com = [cx, cy, cz]
    I_all = [[0.0] * 3 for _ in range(3)]
    for v in result.values():
        I_all = add_tensors(I_all, parallel_axis(
            v["I_com_world"], v["mass_kg"], v["com_world"], com))

    # parametric addendum (not in totals)
    addendum = {
        "cables": {
            "formula": f"m = lambda * L, L = {CABLE_TRACK_M} m (geometric estimate: "
                       f"2 arms ~1.0 m each + torso/lift routing ~0.5 m)",
            "lambda_kg_per_m": CABLE_LAMBDA[scenario],
            "mass_kg": round(CABLE_LAMBDA[scenario] * CABLE_TRACK_M, 4),
            "source": "parametric method from arm_mass_budget (density reused, not totals)",
        },
        "fasteners_connectors": {
            "formula": "m = p * (links + hands + lift mass)",
            "p_fraction": FASTENER_PCT[scenario],
            "mass_kg": round(FASTENER_PCT[scenario] * (
                result.get("left_arm", {}).get("mass_kg", 0) +
                result.get("right_arm", {}).get("mass_kg", 0) +
                result.get("left_hand", {}).get("mass_kg", 0) +
                result.get("right_hand", {}).get("mass_kg", 0) +
                result.get("lift", {}).get("mass_kg", 0)), 4),
            "source": "engineering assumption (stated, not measured)",
        },
        "finger_drives": {
            "formula": FINGER_DRIVES,
            "mass_kg": None,
            "source": "no data - parametric symbol only, excluded from totals",
        },
    }
    return {
        "scenario": scenario,
        "subsystems": result,
        "total_known_kg": round(M_all, 6),
        "total_known_com_world": [round(v, 6) for v in com],
        "total_known_I_com_world": [[round(v, 8) for v in row] for row in I_all],
        "parametric_addendum": addendum,
        "unclassified_skipped": unclassified,
    }


def main():
    payload = {
        "metadata": {
            "stage": "2V.1",
            "kind": "mass_model_scenarios",
            "blend_sha256": "1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb6440708289829cd280",
            "read_only": True,
            "configuration": "rest pose (q=0, lift=0); arm COM moves with q "
                             "(see stage2V/stage2v2_mass_distribution for configs)",
            "constraints": "no moments/dynamics/patient forces/stability; "
                           "no fabricated exact values for unknown data",
        },
        "scenario_axes": {
            "fixed_recipe": "base/head/wheels/sensors: stored = rho*V_local (unchanged)",
            "torso": "designed part m = 2700*|V| (no stored mass)",
            "lift_material": "light=all aluminum 2700; nominal=steel 7800 for "
                             "transmission+motor+brake, aluminum for brackets; "
                             "heavy=all steel 7800 (engineering assumption)",
            "arms_hands_fingers": "light=min(stored, 2700*|V|), nominal=stored, "
                                  "heavy=max(stored, 2700*|V|)",
            "non_geometric": "cables/fasteners/finger drives -> parametric addendum only, "
                             "excluded from totals",
        },
        "parametric_constants": {
            "cable_lambda_kg_per_m": CABLE_LAMBDA,
            "cable_track_m": CABLE_TRACK_M,
            "fastener_fraction": FASTENER_PCT,
            "finger_drives": FINGER_DRIVES,
        },
        "scenarios": {s: build_scenario(s) for s in SCENARIOS},
    }
    OUT_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---- markdown summary ----
    lines = []
    lines.append("# Этап 2В.1 — Три сценария массовой модели (до заполнения всех звеньев)\n")
    lines.append(f"Конфигурация: поза покоя (q=0, лифт=0). Моменты/динамика/устойчивость не считались.")
    lines.append(f".blend не изменён (sha 1ae88879…).\n")
    lines.append("## 1. Оси вариации сценариев\n")
    lines.append("| Ось | Облегчённый | Номинальный | Тяжёлый | Обоснование |")
    lines.append("|---|---|---|---|---|")
    lines.append("| Материал деталей лифта | Al 2700 | сталь 7800 (трансмиссия+привод), Al (кронштейны) | сталь 7800 | инженерное допущение, объёмы из мешей |")
    lines.append("| Звенья рук/кисти/пальцы | min(stored, 2700·\\|V\\|) | stored | max(stored, 2700·\\|V\\|) | геометрия vs stored (категории ②/③) |")
    lines.append("| База/голова/колёса/датчики | 2700·V (фикс.) | то же | то же | рецептура ② (U2 по базе) |")
    lines.append("| Торс | 2700·\\|V\\| | то же | то же | деталь из меша, stored отсутствует |")
    lines.append("| Кабели/крепёж/приводы пальцев | **вне итогов**, параметрич. | то же | то же | нет данных -> формулы, не числа |\n")
    lines.append("## 2. Итоги по сценариям (только геометрические/компонентные данные)\n")
    lines.append("| Сценарий | Итого, кг | ЦМ X | ЦМ Y | ЦМ Z |")
    lines.append("|---|---|---|---|---|")
    for s in SCENARIOS:
        sc = payload["scenarios"][s]
        c = sc["total_known_com_world"]
        lines.append(f"| {s} | {sc['total_known_kg']:.3f} | {c[0]:.4f} | {c[1]:.4f} | {c[2]:.4f} |")
    lines.append("")
    lines.append("## 3. Массы подсистем по сценариям, кг\n")
    subs = sorted({k for s in SCENARIOS for k in payload['scenarios'][s]['subsystems']})
    lines.append("| Подсистема | light | nominal | heavy |")
    lines.append("|---|---|---|---|")
    for sub in subs:
        row = [f"{payload['scenarios'][s]['subsystems'].get(sub, {}).get('mass_kg', 0):.3f}"
               for s in SCENARIOS]
        lines.append(f"| {sub} | {row[0]} | {row[1]} | {row[2]} |")
    lines.append("")
    lines.append("## 4. Параметрическое добавочное (вне итогов)\n")
    lines.append("| Параметр | Формула | light | nominal | heavy |")
    lines.append("|---|---|---|---|---|")
    ad = payload["scenarios"]["nominal"]["parametric_addendum"]
    lines.append(f"| Кабели | {ad['cables']['formula']} | 0.08×2.5=0.200 | 0.15×2.5=0.375 | 0.30×2.5=0.750 |")
    lines.append(f"| Крепёж/разъёмы | {ad['fasteners_connectors']['formula']} | 2.0% | 3.5% | 5.0% |")
    lines.append(f"| Приводы пальцев | {FINGER_DRIVES} | — | — | — |")
    lines.append("")
    lines.append("## 5. Тензоры инерции\n")
    lines.append("Диагонали I_com (кг·м², мировая СК) по подсистемам — в `stage2v1_scenarios.json`")
    lines.append("(`total_known_I_com_world` и `subsystems.*.I_com_world`).")
    lines.append("Метод: тетраэдральная декомпозиция + параллельная ось, валидирован")
    lines.append("(base Ixx 8.5305 vs аналитическое 8.5232). Знаки инвертированных мешей исправлены.")
    lines.append("")
    lines.append("## 6. Ограничения\n")
    lines.append("- Тензоры и ЦМ — при допущении ЦМ=геометрический ЦМ меша (stored-sided для ③).")
    lines.append("- Прокси-меши лифта могут превосходить реальные детали (указано в JSON basis).")
    lines.append("- U1/U2/U4 остаются открытыми: три несогласованные массовые базы, состав базы, происхождение stored-масс рук.")
    lines.append("- Итоги НЕ включают кабели/крепёж/приводы пальцев (см. §4).")
    OUT_MD.write_text("\n".join(lines), encoding="utf-8")
    print(f"[stage2v1] wrote {OUT_JSON.name} and {OUT_MD.name}")
    for s in SCENARIOS:
        sc = payload["scenarios"][s]
        print(f"  {s}: total={sc['total_known_kg']:.3f} kg, "
              f"unclassified={sc['unclassified_skipped']}")


if __name__ == "__main__":
    main()
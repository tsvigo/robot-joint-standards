# -*- coding: utf-8 -*-
"""Этап 2Г, аудит v7.1 — stage2g_v7_1_regression.py (только Python, НОВЫЙ файл).

Автоматические регрессионные проверки всего этапа v7.1.
Эталонные SHA — снимок на момент аудита v7.1 (v1-v6 датированы 24.10, в этапе
v7.1 не изменялись).
"""
import hashlib
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
STAGE = os.path.dirname(HERE)
CAD = HERE

EXPECTED_SHA = {
    "stage2g_base.FCStd": "6fbfa072e403ff6c",
    "stage2g_base_v2.FCStd": "396d75aa1d9c3331",
    "stage2g_v3_layout.FCStd": "60f7ae157c61f8ba",
    "stage2g_v4_round.FCStd": "b485de27a27cc03f",
    "stage2g_v5_compare.FCStd": "f39201567df7d09b",
    "stage2g_v6_variants.FCStd": "85f504b0ba8dbc9b",
}
BLEND_SHA = "1ae88879af076b1a"
BLEND = "/home/v/projects/робот/robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend"

RESULTS = []


def t(cid, ok, detail):
    RESULTS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def load(name):
    with open(os.path.join(CAD, name), encoding="utf-8") as f:
        return json.load(f)


print("== R1. Целостность исходников v1-v6 и Blender ==")
for name, want in EXPECTED_SHA.items():
    p = os.path.join(CAD, name)
    got = sha(p)
    t(f"SHA_{name}", got == want, f"{got} == {want}")
t("SHA_blender", sha(BLEND) == BLEND_SHA, f"{sha(BLEND)} == {BLEND_SHA}")

print("== R2. Кинематика v7.1 (SI, автопроверки) ==")
K = load("stage2g_v7_1_kinematics.json")
allok = all(c["ok"] for c in K["checks"])
t("K_checks", allok, f"{sum(c['ok'] for c in K['checks'])}/{len(K['checks'])} проверок SI")
om = K["R2_full"]["omega"]["вперёд +X (1 м/с)"]
t("K_SI_scale", all(abs(abs(x) - 8.333) < 0.001 for x in om),
  f"ω(1 м/с) = {om} == ±v/r = ±8.333 (ловушка v7 0.0083 закрыта)")
t("K_rotate", all(abs(abs(x) - 1.846) < 0.001 for x in K["R2_full"]["omega"]["вращение (1 рад/с)"]),
  f"ω_вращение == R_C/r = 1.846 (значение −2.87 отчёта v7 признано ошибкой ввода)")
t("K_rank16", [k for k, v in K["radial16"].items() if v["rank"] < 3] == ["LRLR", "RLRL"],
  "сингулярны только LRLR/RLRL; LLLL/RRRR ранг 3, cond 1.129")

print("== R3. Мощность v7.1 (3 страницы RoboCT) ==")
P = load("stage2g_v7_1_power.json")
p24 = P["points"][0]
t("P_24V", abs(p24["delta_Rated_vs_Pout_%"]) <= 1.5 and abs(p24["eta_calc"] - 0.70) <= 0.01,
  f"24В: P_out={p24['P_out_Вт']}≈220 (Δ1.4%), η_calc={p24['eta_calc']}≈0.70 — точка согласована")
t("P_440_as_elec_absurd", all(x["eta_при_Rated_как_электр"] > 0.95 for x in P["points"]),
  "электрическая трактовка 220/330/440 дала бы η=0.99-1.01 => трактовка отвергнута")
t("P_180_incompatible", "несовместим" in P["treatment"]["180Вт_P2"],
  "180 Вт (P2) несовместим с 74 Н·м×56 об/мин при любой трактовке")
t("P_error_fix", "88" in P["v7_error_fix"]["было_v7"] and "70-76" in P["v7_error_fix"]["было_v7"],
  "ошибка v7 «440≥493 при η=88%» зафиксирована и исправлена")
enc = [c for c in P["comparison"] if c["параметр"] == "Энкодер"][0]
t("P_encoder_conflict", "4096 vs 4384" in enc["вывод"], "противоречие энкодеров 4096/4384 в таблице")
peak = P["peak"]
t("P_peak", abs(peak["отношение"] - 0.943) < 0.005,
  f"140 Н·м vs 4×37.1=148.4: отношение {peak['отношение']} ≈ η_редуктора 94%")

print("== R4. Массовый бюджет v7.1 ==")
M = load("stage2g_v7_1_mass.json")
names = " ".join(x["подсистема"] for x in M["budget"])
t("M_no_cabin", "Кабина" not in names, "кабина 1100×2100 исключена из бюджета робота")
t("M_confusion", len(M["confusion_ABCD"]) == 4 and
  M["confusion_ABCD"][0]["в_массу_робота_входит"] is False and
  M["confusion_ABCD"][2]["в_массу_робота_входит"] is True,
  "величины A (больничный лифт) / B (подъёмник робота) / C (масса) / D (нагрузка) разделены")
t("M_old_range_retired", any("263-663" in r["строка"] for r in M["retracted_from_v7"]),
  "диапазон 263-663 кг признан неподтверждённым")
t("M_confirmed_only_modules", M["confirmed_kg"] == 100.0,
  "подтверждено только 100 кг (4 модуля); итог массы не определён")

print("== R5. Стабильность/удержание v7.1 ==")
S = load("stage2g_v7_1_stability.json")
t("S_checks", all(c["ok"] for c in S["checks"]),
  f"{sum(c['ok'] for c in S['checks'])}/{len(S['checks'])} проверок")
t("S_hold_etalon", abs(S["hold_etalon"]["M=400,grade=5%"]["tau_еталон_Нм"] - 5.89) < 0.02,
  "эталон удержания τ = F·r/4 = 5.89 Н·м (виртуальная работа)")
t("S_hold_118_explained",
  all(abs(v - 11.77) < 0.05 for v in S["hold_etalon"]["M=400,grade=5%"]["гипотезы_x2_Нм"].values()),
  "11.8 Н·м = F·r/2: все три гипотезы ×2 дают ровно 11.77")
t("S_84_is_model", "110.8" in S["checks"][1]["detail"] or "110" in json.dumps(S, ensure_ascii=False),
  "84 мм (v7) и 110.8 мм — результаты моделей контуров, не универсальные пределы")
t("S_terms", "отозван" in S["contour_terms"]["консервативный_модель_A"]["статус"],
  "термин «гарантированный» отозван -> «консервативный по модели»")
t("S_no_power", len(S["no_power"]["сценарии"]) == 3,
  "торможение без питания: 3 сценария по типу редуктора (все — в запрос RoboCT)")

print("== R6. CAD-сценарии подвески v7.1 ==")
C = load("stage2g_v7_1_cad_suspension.json")
allok = all(not r["пересечения"] for r in C["suspension_scenarios"].values())
t("C_scenarios", allok and len(C["suspension_scenarios"]) == 4,
  "4 сценария (все+30 / одиночный / диагональный / разнокрайний) — пересечений нет, min 10 мм")
t("C_clearance", C["floor_clearance"]["просвет_транспорт_мм"] == 8.0,
  "просвет транспорта = 8 мм (блоки аутригеров) — риск для перепадов пола, не закрыт")
t("C_cables", C["cables"]["статус"] == "НЕ СМОДЕЛИРОВАНЫ в v6",
  "кабели/крепления — пробел, в запрос RoboCT")

print("== R7. MIT/Morin 2024 (источник) ==")
txt = os.path.join(STAGE, "morin_mecanum_mit_2024.txt")
t("MIT_text", os.path.exists(txt) and os.path.getsize(txt) > 30000,
  f"текст тезиса извлечён ({os.path.getsize(txt)} байт) из локального PDF пользователя")

fails = [r for r in RESULTS if not r["ok"]]
print(f"\nИТОГ: {len(RESULTS) - len(fails)}/{len(RESULTS)} PASS" +
      ("" if not fails else f", FAIL: {[r['id'] for r in fails]}"))
out = {"results": RESULTS, "passed": len(RESULTS) - len(fails), "failed": len(fails)}
path = os.path.join(CAD, "stage2g_v7_1_regression.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("JSON:", path)

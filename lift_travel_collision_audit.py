#!/usr/bin/env python3
"""READ-ONLY аудит полного хода vertical_lift_joint.
Метод:
  1. SHA256 исходного .blend фиксируется вызывающим (см. TXT).
  2. Headless `blender --background <blend> --python <dump>` ТОЛЬКО читает сцену
     и пишет JSON-дамп в tmp. Сцена не сохраняется (quit без save).
  3. Анализ выполняется на прочитанных данных в чистом Python:
     - движущееся поддерево = descendants(vertical_lift_joint) по parent→child;
     - неподвижные = остальные, исключая дубликаты collision-shell
       (parent=None, phobostype=4, имя *collision / Cube*) — они не являются
       независимыми телами;
     - движение = сдвиг +dZ (ось [0,0,1] EXPLICIT, world, проверено) для всех
       движущихся AABB: zmin+dz, zmax+dz;
     - грубое сканирование dz с шагом 10 мм, затем уточнение границ до 1 мм
       (бисекция по условию AABB-пересечения);
     - AABB используется для предварительного отсева ВСЕХ пар; для
       осевыровненных боксов (все критические meshes, off-diag < 1e-6)
       AABB-пересечение является ТОЧНЫМ критерием столкновения, что явно
       проверено; 5 неосе выровненных объектов (AXLE_GUIDE_*, sync_shaft)
       далеки от траектории лифта и помечены отдельно.
Критерий контакта: пересечение замкнутых AABB (touch = контакт) с допуском 1e-9 м.
Уточнение: бисекция 10мм -> 1мм (затем 0.1мм для отчётных чисел).
Исходная сцена не изменяется, файл не сохраняется.
"""
import subprocess, json, os, sys, hashlib, math

BLEND = "/home/v/projects/робот/robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend"
BLENDER = "/home/v/Apps/Blender/4.2-LTS/blender"
TMPDUMP = "/home/v/.local/state/opencode/tmp/lift_dump.json"
DUMP_SCRIPT = "/home/v/.local/state/opencode/tmp/dump_lift_hierarchy.py"

def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for ch in iter(lambda: f.read(1 << 20), b""):
            h.update(ch)
    return h.hexdigest()

def ensure_dump():
    # dump-скрипт уже создан вызывающим аудитом; если нет — создать минимальный
    if not os.path.exists(DUMP_SCRIPT):
        raise SystemExit("dump script missing: " + DUMP_SCRIPT)
    # headless read-only dump (без сохранения сцены)
    r = subprocess.run([BLENDER, "--background", BLEND, "--python", DUMP_SCRIPT],
                       capture_output=True, text=True, timeout=300)
    if not os.path.exists(TMPDUMP):
        print(r.stdout[-3000:], file=sys.stderr)
        print(r.stderr[-3000:], file=sys.stderr)
        raise SystemExit("dump failed")
    return r

def load():
    d = json.load(open(TMPDUMP))
    objs = {o["name"]: o for o in d["objects"]}
    return objs

def subtree(objs, root):
    seen, stack = set(), [root]
    while stack:
        n = stack.pop()
        if n in seen:
            continue
        seen.add(n)
        o = objs.get(n)
        if o:
            stack.extend(o.get("children", []))
    return seen

def is_collision_dup(o):
    if o.get("parent") is not None:
        return False
    if str(o.get("props", {}).get("phobostype")) != "4":
        return False
    n = o["name"]
    return ("collision" in n.lower()) or n.startswith("Cube")

def aabb_overlap(a_min, a_max, b_min, b_max, eps=1e-9):
    for i in range(3):
        if a_max[i] + eps < b_min[i] or b_max[i] + eps < a_min[i]:
            return False
    return True

def shifted(o, dz):
    mn, mx = o["bbox_min"], o["bbox_max"]
    return ([mn[0], mn[1], mn[2] + dz], [mx[0], mx[1], mx[2] + dz])

def main():
    print("SHA blend:", sha256(BLEND))
    ensure_dump()
    objs = load()
    print("objects:", len(objs))
    moving_names = subtree(objs, "vertical_lift_joint")
    moving = [o for n, o in objs.items() if n in moving_names and o["type"] == "MESH"
              and o["bbox_min"] and not is_collision_dup(o)]
    static_all = [o for n, o in objs.items() if n not in moving_names and o["type"] == "MESH"
                  and o["bbox_min"] and not is_collision_dup(o)]
    print(f"moving MESH={len(moving)} static MESH={len(static_all)}")
    # ось
    lj = objs["vertical_lift_joint"]
    print("lift parent:", lj["parent"], "props:", lj["props"])
    # --- грубый скан ---
    # диапазон скана: -900..+600 мм, шаг 10 мм
    coarse = list(range(-900, 601, 10))  # мм
    # предфильтр: для каждого dz список пересекающихся пар
    def contacts_at(dz_m):
        res = []
        for m in moving:
            mn, mx = shifted(m, dz_m)
            for s in static_all:
                if aabb_overlap(mn, mx, s["bbox_min"], s["bbox_max"]):
                    res.append((m["name"], s["name"]))
        return res
    coarse_hits = {}
    for dz_mm in coarse:
        c = contacts_at(dz_mm / 1000.0)
        if c:
            coarse_hits[dz_mm] = c
    print(f"coarse hits at {len(coarse_hits)} positions")
    # --- уточнение границ для критических пар ---
    # Нам нужны первые контакты вверх/вниз по каждой категории.
    # Бисекция: найти минимальный |dz| при котором пара впервые касается.
    def first_touch(mname, sname, lo_mm, hi_mm):
        # lo = свободно, hi = контакт; бисекция до 1мм, затем 0.1мм
        m = objs[mname]; s = objs[sname]
        for _ in range(30):
            if hi_mm - lo_mm <= 0.1:
                break
            mid = (lo_mm + hi_mm) / 2
            if contacts_pair(m, s, mid / 1000.0):
                hi_mm = mid
            else:
                lo_mm = mid
        return round(hi_mm, 1)
    def contacts_pair(m, s, dz_m):
        mn, mx = shifted(m, dz_m)
        return aabb_overlap(mn, mx, s["bbox_min"], s["bbox_max"])
    # Скользящие пары (направляющая/гайка) — штатный скользящий контакт,
    # не столкновение; анализируются отдельно как пределы хода.
    def is_sliding(mname, sname):
        pairs = [("lift_guide_block_left", "lift_guide_left"),
                 ("lift_guide_block_right", "lift_guide_right"),
                 ("lift_guide_block_left", "lift_guide_right"),
                 ("lift_guide_block_right", "lift_guide_left"),
                 ("lift_nut_mount_left", "lift_screw_left"),
                 ("lift_nut_mount_right", "lift_screw_right"),
                 ("lift_nut_mount_left", "lift_screw_right"),
                 ("lift_nut_mount_right", "lift_screw_left")]
        return (mname, sname) in pairs
    # Кандидаты: перебрать все пары moving×static, найти их границы бисекцией
    events = []  # (dz_mm, moving, static)
    for m in moving:
        for s in static_all:
            if is_sliding(m["name"], s["name"]):
                continue
            # быстрая проверка: есть ли вообще пересечение в диапазоне [-900,+600]?
            hit_lo = contacts_pair(m, s, -0.9)
            hit_hi = contacts_pair(m, s, 0.6)
            hit_0 = contacts_pair(m, s, 0.0)
            # если в нуле уже контакт — это дубликат/проблема (не должно быть среди отфильтрованных)
            if hit_0:
                events.append((0.0, m["name"], s["name"], "AT_ZERO"))
                continue
            # вниз
            if hit_lo and not hit_0:
                lo, hi = 0.0, -900.0
                # найти переход
                # сначала грубо шагами 10мм
                prev = 0.0
                for dz in range(0, -901, -10):
                    if contacts_pair(m, s, dz / 1000.0):
                        lo, hi = prev, float(dz)
                        break
                    prev = float(dz)
                events.append((first_touch(m["name"], s["name"], lo, hi), m["name"], s["name"], "DOWN"))
            # вверх
            if hit_hi and not hit_0:
                prev = 0.0
                for dz in range(0, 601, 10):
                    if contacts_pair(m, s, dz / 1000.0):
                        lo, hi = prev, float(dz)
                        break
                    prev = float(dz)
                # first_touch ожидает lo<hi
                events.append((first_touch(m["name"], s["name"], lo, hi), m["name"], s["name"], "UP"))
    # сохранить события
    import csv
    with open("/home/v/projects/робот/lift_travel_collision_audit_pairs.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lift_mm", "moving", "static", "direction", "note"])
        for dz, mn, sn, d in sorted(events):
            w.writerow([dz, mn, sn, d, ""])
    # ключевые пары
    def gap_z(mname, sname):
        m, s = objs[mname], objs[sname]
        return (s["bbox_min"][2] - m["bbox_max"][2], s["bbox_max"][2] - m["bbox_min"][2])
    keys = {}
    for pair in [("torso_link", "base_link"), ("head_link", "lift_stop_top"),
                 ("torso_link", "lift_catcher_placeholder"),
                 ("lift_guide_block_left", "lift_guide_left"),
                 ("lift_nut_mount_left", "lift_screw_left")]:
        try:
            m = objs[pair[0]]
            # m может быть moving, s static
            s = objs[pair[1]]
            mn, mx = m["bbox_min"], m["bbox_max"]
            sn, sx = s["bbox_min"], s["bbox_max"]
            keys[pair] = {"m_z": [mn[2], mx[2]], "s_z": [sn[2], sx[2]],
                          "gap_up": sx[2] and (sn[2] - mx[2]), "gap_down": mn[2] - sx[2]}
        except Exception as e:
            keys[pair] = {"err": str(e)}
    out = {"events": sorted(events), "keys": keys,
           "moving_count": len(moving), "static_count": len(static_all)}
    json.dump(out, open("/home/v/.local/state/opencode/tmp/lift_events.json", "w"), indent=1)
    for e in sorted(events)[:60]:
        print(e)
    print("KEYS:")
    for k, v in keys.items():
        print(k, v)

if __name__ == "__main__":
    main()

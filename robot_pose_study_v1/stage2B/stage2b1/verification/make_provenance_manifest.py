#!/usr/bin/env python3
"""Build a provenance manifest for stage 2B.1 JSON artifacts that carry no
embedded ``script_sha256``.

Only claims that can be checked from the working tree are recorded:

``confirmed_reproduced_byte_identical``
    A rerun of the current generator produced a file with the same SHA256.
``confirmed_by_generator_hash``
    The artifact has no embedded hash, but it was written by the same run that
    wrote a sibling artifact which records the generator SHA256, and that
    recorded value equals the current bytes of the generator script.
``script_identified_not_confirmed``
    The generator is identified from code, but no hash evidence links the
    artifact to a specific script revision.

A presumed source is never recorded as confirmed.
"""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent  # stage2b1

OUT = ROOT / "stage2b1_provenance_manifest.json"

GENERATORS = {
    "stage2b1_unilateral.py": {
        "writes": ["stage2b1_unilateral_{tag}.json", "stage2b1_path_{tag}_{policy}.json"],
        "evidence": "stage2b1_unilateral.py, lines 409 and 415",
    },
    "stage2b1_force_candidates.py": {
        "writes": ["stage2b1_force_candidates.json"],
        "evidence": "stage2b1_force_candidates.py, line 21 (OUT) and line 172 (write_text)",
    },
}

REPRODUCED = {
    # artifact in this directory -> reproduced file in verification/ with equal SHA256
    "stage2b1_path_P6_L_c0.10_as_is.json": "stage2b1_reverify_path_P6_L_c0.10_as_is.json",
    "stage2b1_path_P6_L_c0.10_strict_6d.json": "stage2b1_reverify_path_P6_L_c0.10_strict_6d.json",
    "stage2b1_force_candidates.json": None,  # reproduced in an isolated copy, hash equal
}

EXPECTED_ABSENT = [
    {
        "artifact": "stage2b1_path_P7_L_c0.12_strict_6d.json",
        "reason": "strict_6d planning failed (0/5 transitions), no path file written",
    },
    {
        "artifact": "stage2b1_path_P7_L_c0.15_as_is.json",
        "reason": "as_is planning failed (0/5 transitions), no path file written",
    },
    {
        "artifact": "stage2b1_path_P7_L_c0.15_strict_6d.json",
        "reason": "strict_6d planning failed (0/5 transitions), no path file written",
    },
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_path_name(name: str) -> dict | None:
    # stage2b1_path_<op>_<side>_c<c>_<policy>.json
    if not name.startswith("stage2b1_path_"):
        return None
    body = name[len("stage2b1_path_") : -len(".json")]
    parts = body.split("_")
    if len(parts) < 4:
        return None
    return {
        "operation": parts[0],
        "side": parts[1],
        "clearance_m": float(parts[2][1:]),
        "policy": "_".join(parts[3:]),
        "tag": "_".join(parts[:3]),
    }


def main() -> int:
    generator_hashes = {
        name: sha256(ROOT / name) for name in GENERATORS if (ROOT / name).exists()
    }

    targets = sorted(
        [p for p in ROOT.glob("stage2b1_path_*.json")]
        + ([ROOT / "stage2b1_force_candidates.json"] if (ROOT / "stage2b1_force_candidates.json").exists() else [])
    )

    files = []
    for path in targets:
        name = path.name
        entry = {
            "path": name,
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
            "generator": None,
            "provenance_status": "script_identified_not_confirmed",
            "basis": None,
            "config": None,
        }

        config = parse_path_name(name)
        if config is not None:
            entry["config"] = {k: config[k] for k in ("operation", "side", "clearance_m", "policy")}
            entry["generator"] = "stage2b1_unilateral.py"
            sibling = ROOT / f"stage2b1_unilateral_{config['tag']}.json"
            recorded = None
            if sibling.exists():
                recorded = json.loads(sibling.read_text(encoding="utf-8")).get("metadata", {}).get("script_sha256")
            entry["sibling_artifact"] = sibling.name if sibling.exists() else None
            entry["sibling_script_sha256"] = recorded
        else:
            entry["generator"] = "stage2b1_force_candidates.py"

        if name in REPRODUCED:
            entry["provenance_status"] = "confirmed_reproduced_byte_identical"
            if REPRODUCED[name]:
                entry["basis"] = (
                    "rerun of the current generator produced an identical SHA256; "
                    f"reproduced copy kept in verification/{REPRODUCED[name]}"
                )
            else:
                entry["basis"] = "rerun of the current generator in an isolated directory produced an identical SHA256"
        elif config is not None and entry.get("sibling_script_sha256") == generator_hashes.get("stage2b1_unilateral.py"):
            entry["provenance_status"] = "confirmed_by_generator_hash"
            entry["basis"] = (
                "written by the same run that wrote the sibling artifact "
                f"{entry['sibling_artifact']}, whose metadata.script_sha256 equals the current "
                "SHA256 of stage2b1_unilateral.py"
            )
        elif config is not None:
            entry["basis"] = (
                "sibling artifact records a different script SHA256 "
                f"({entry.get('sibling_script_sha256')}); provenance of that run is analysed separately"
            )

        files.append(entry)

    summary = {"total": len(files)}
    for entry in files:
        summary[entry["provenance_status"]] = summary.get(entry["provenance_status"], 0) + 1

    payload = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "stage 2B.1 JSON artifacts without an embedded metadata.script_sha256",
        "status_levels": {
            "confirmed_reproduced_byte_identical": "rerun of the current generator gave the same SHA256",
            "confirmed_by_generator_hash": "same run wrote a sibling artifact whose recorded generator SHA256 matches current bytes",
            "script_identified_not_confirmed": "generator identified from code only; no hash evidence",
        },
        "generators": {
            name: {
                "sha256": generator_hashes.get(name),
                "writes": info["writes"],
                "evidence": info["evidence"],
            }
            for name, info in GENERATORS.items()
        },
        "summary": summary,
        "files": files,
        "expected_absent": EXPECTED_ABSENT,
        "limitations": [
            "No artifact in this manifest embeds its own generator hash.",
            "confirmed_by_generator_hash relies on two files being written by the same run; it is not a byte-level rerun of this artifact.",
            "Only P6 L c=0.10 path files and the force-candidate list were rerun; the remaining path files were not reproduced.",
        ],
    }

    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"written: {OUT}")
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

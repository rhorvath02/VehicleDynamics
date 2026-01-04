#!/usr/bin/env python3
"""
convert_suspension_json_to_record.py

Build-time tool for converting suspension JSON tuning files into
typed Modelica suspension records.

- No runtime JSON parsing
- FMU-safe
- Deterministic output
- Record hierarchy mirrors model topology
"""

from __future__ import annotations

from pathlib import Path
import json
from decimal import Decimal, getcontext

getcontext().prec = 28


# =============================================================================
# FORMATTER
# =============================================================================

def fmt(x) -> str:
    d = Decimal(str(x)).normalize()
    s = format(d, "f") if d.as_tuple().exponent > -12 else format(d, "g")
    return s.rstrip("0").rstrip(".") if "." in s else s


def fmt_vec(v):
    return "{" + ", ".join(fmt(x) for x in v) + "}"


# =============================================================================
# RECORD SCHEMAS (SINGLE SOURCE OF TRUTH)
# =============================================================================

RECORD_SCHEMAS = {
    "FrAxleBase": [
        # Upper
        "upper_fore_i",
        "upper_fore_i_c",
        "upper_fore_i_d",
        "upper_aft_i",
        "upper_aft_i_c",
        "upper_aft_i_d",
        "upper_outboard",

        # Lower
        "lower_fore_i",
        "lower_fore_i_c",
        "lower_fore_i_d",
        "lower_aft_i",
        "lower_aft_i_c",
        "lower_aft_i_d",
        "lower_outboard",

        # Tie
        "tie_inboard",
        "tie_inboard_c",
        "tie_inboard_d",
        "tie_outboard",

        # Tire
        "wheel_center",
        "static_gamma",
        "static_alpha",
    ],

    "FrAxleBellcrank": [
        # Bellcrank
        "bellcrank_pivot",
        "bellcrank_pivot_ref",
        "bellcrank_pickup_1",
        "bellcrank_pickup_2",
        "bellcrank_pickup_3",

        # Push / pull rod
        "LCA_mount",
        "shock_mount",
    ],
}


# =============================================================================
# JSON EXTRACTION
# =============================================================================

def extract_front_left(data: dict) -> dict:
    fl = data["Front"]["left"]

    return {
        # Upper
        "upper_fore_i": fl["upper"]["fore_i"],
        "upper_fore_i_c": fl["upper"]["fore_i_c"],
        "upper_fore_i_d": fl["upper"]["fore_i_d"],
        "upper_aft_i": fl["upper"]["aft_i"],
        "upper_aft_i_c": fl["upper"]["aft_i_c"],
        "upper_aft_i_d": fl["upper"]["aft_i_d"],
        "upper_outboard": fl["upper"]["outboard"],

        # Lower
        "lower_fore_i": fl["lower"]["fore_i"],
        "lower_fore_i_c": fl["lower"]["fore_i_c"],
        "lower_fore_i_d": fl["lower"]["fore_i_d"],
        "lower_aft_i": fl["lower"]["aft_i"],
        "lower_aft_i_c": fl["lower"]["aft_i_c"],
        "lower_aft_i_d": fl["lower"]["aft_i_d"],
        "lower_outboard": fl["lower"]["outboard"],

        # Tie
        "tie_inboard": fl["tie"]["inboard"],
        "tie_inboard_c": fl["tie"]["inboard_c"],
        "tie_inboard_d": fl["tie"]["inboard_d"],
        "tie_outboard": fl["tie"]["outboard"],

        # Bellcrank
        "bellcrank_pivot": fl["bellcrank"]["pivot"],
        "bellcrank_pivot_ref": fl["bellcrank"]["pivot_ref"],
        "bellcrank_pickup_1": fl["bellcrank"]["pickup_1"],
        "bellcrank_pickup_2": fl["bellcrank"]["pickup_2"],
        "bellcrank_pickup_3": fl["bellcrank"]["pickup_3"],

        # Push / pull rod
        "LCA_mount": fl["push/pull rod"]["LCA_mount"],
        "shock_mount": fl["push/pull rod"]["shock_mount"],

        # Tire
        "wheel_center": fl["tire"]["wheel_center"],
        "static_gamma": fl["tire"]["static_gamma"],
        "static_alpha": fl["tire"]["static_alpha"],
    }


# =============================================================================
# MODELICA EMISSION
# =============================================================================

def emit_record(
    name: str,
    params: dict,
    out_path: Path,
    src: Path,
    extends: str | None = None,
):
    lines = [
        "// ============================================================================",
        "// AUTO-GENERATED FILE — DO NOT EDIT",
        f"// Source: {src}",
        "// Tool: convert_suspension_json_to_record.py",
        "// ============================================================================",
        "",
        "within VehicleDynamics.Resources.Records.SUS;",
        "",
        f"record {name}",
        '  "Auto-generated suspension parameter record"',
        "",
        "  import Modelica.SIunits;",
        "",
    ]

    if extends:
        lines.append(f"  extends {extends};")
        lines.append("")

    for k, v in params.items():
        if isinstance(v, list):
            if len(v) != 3:
                raise ValueError(f"Expected 3-vector for {k}")
            lines.append(
                f"  parameter SIunits.Position {k}[3] = {fmt_vec(v)};"
            )
        else:
            lines.append(
                f"  parameter SIunits.Angle {k} = {fmt(v)};"
            )

    lines.extend([
        "",
        f"end {name};",
        "",
    ])

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines))

    # Maintain package.order
    pkg_order = out_path.parent / "package.order"
    if pkg_order.exists():
        entries = pkg_order.read_text().splitlines()
        if name not in entries:
            entries.append(name)
            pkg_order.write_text("\n".join(entries) + "\n")


# =============================================================================
# CLI
# =============================================================================

def main():
    import sys

    if len(sys.argv) != 2:
        print("Usage: convert_suspension_json_to_record.py <file.json>")
        sys.exit(1)

    src = Path(sys.argv[1]).resolve()
    data = json.loads(src.read_text())

    all_params = extract_front_left(data)

    out_dir = Path("VehicleDynamics/Resources/Records/SUS")

    # --- Base record ---
    base_params = {k: all_params[k] for k in RECORD_SCHEMAS["FrAxleBase"]}
    emit_record(
        name="FrAxleBase",
        params=base_params,
        out_path=out_dir / "FrAxleBase.mo",
        src=src,
    )

    # --- Bellcrank record ---
    bellcrank_params = {k: all_params[k] for k in RECORD_SCHEMAS["FrAxleBellcrank"]}
    emit_record(
        name="FrAxleBellcrank",
        params=bellcrank_params,
        out_path=out_dir / "FrAxleBellcrank.mo",
        src=src,
        extends="FrAxleBase",
    )

    print("Generated:")
    print(f"  - {out_dir / 'FrAxleBase.mo'}")
    print(f"  - {out_dir / 'FrAxleBellcrank.mo'}")


if __name__ == "__main__":
    main()

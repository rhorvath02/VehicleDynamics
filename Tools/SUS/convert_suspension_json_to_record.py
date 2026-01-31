#!/usr/bin/env python3
"""
convert_suspension_json_to_record.py

Build-time tool for converting suspension JSON tuning files into
typed Modelica suspension records.

- No runtime JSON parsing
- FMU-safe
- Deterministic output
- Supports Front + Rear axles
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
# RECORD SCHEMAS (AXLE-INDEPENDENT)
# =============================================================================

RECORD_SCHEMAS = {
    "Base": [
        # Upper
        "upper_fore_i",
        "upper_aft_i",
        "upper_outboard",

        # Lower
        "lower_fore_i",
        "lower_aft_i",
        "lower_outboard",

        # Tie
        "tie_inboard",
        "tie_outboard",

        # Tire
        "wheel_center",
        "static_gamma",
        "static_alpha",
    ],

    "Bellcrank": [
        # Bellcrank
        "bellcrank_pivot",
        "bellcrank_pivot_ref",
        "bellcrank_pickup_1",
        "bellcrank_pickup_2",
        "bellcrank_pickup_3",

        # Push / pull rod
        "rod_mount",     # normalized name
        "shock_mount",
    ],
}


# =============================================================================
# JSON EXTRACTION (AXLE-AGNOSTIC)
# =============================================================================

def extract_axle_left(data: dict, axle: str) -> dict:
    al = data[axle]["left"]

    # Normalize push/pull rod mount naming
    rod = al["push/pull rod"]
    rod_mount = rod.get("LCA_mount", rod.get("UCA_mount"))

    if rod_mount is None:
        raise KeyError(f"{axle}.left.push/pull rod missing LCA_mount/UCA_mount")

    return {
        # Upper
        "upper_fore_i": al["upper"]["fore_i"],
        "upper_aft_i": al["upper"]["aft_i"],
        "upper_outboard": al["upper"]["outboard"],

        # Lower
        "lower_fore_i": al["lower"]["fore_i"],
        "lower_aft_i": al["lower"]["aft_i"],
        "lower_outboard": al["lower"]["outboard"],

        # Tie
        "tie_inboard": al["tie"]["inboard"],
        "tie_outboard": al["tie"]["outboard"],

        # Bellcrank
        "bellcrank_pivot": al["bellcrank"]["pivot"],
        "bellcrank_pivot_ref": al["bellcrank"]["pivot_ref"],
        "bellcrank_pickup_1": al["bellcrank"]["pickup_1"],
        "bellcrank_pickup_2": al["bellcrank"]["pickup_2"],
        "bellcrank_pickup_3": al["bellcrank"]["pickup_3"],

        # Push / pull rod (normalized)
        "rod_mount": rod_mount,
        "shock_mount": rod["shock_mount"],

        # Tire
        "wheel_center": al["tire"]["wheel_center"],
        "static_gamma": al["tire"]["static_gamma"],
        "static_alpha": al["tire"]["static_alpha"],
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
        "within VehicleDynamics.Resources.Records.SUS;",
        "",
        "// ============================================================================",
        "// AUTO-GENERATED FILE — DO NOT EDIT",
        f"// Source: {src}",
        "// Tool: convert_suspension_json_to_record.py",
        "// ============================================================================",
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

    pkg_order = out_path.parent / "package.order"
    if pkg_order.exists():
        entries = pkg_order.read_text().splitlines()
        if name not in entries:
            entries.append(name)
            pkg_order.write_text("\n".join(entries) + "\n")


# =============================================================================
# DRIVER
# =============================================================================

def emit_axle(data: dict, axle: str, prefix: str, out_dir: Path, src: Path):
    all_params = extract_axle_left(data, axle)

    base_params = {k: all_params[k] for k in RECORD_SCHEMAS["Base"]}
    bellcrank_params = {k: all_params[k] for k in RECORD_SCHEMAS["Bellcrank"]}

    base_name = f"{prefix}AxleBase"
    bellcrank_name = f"{prefix}AxleBellcrank"

    emit_record(
        name=base_name,
        params=base_params,
        out_path=out_dir / f"{base_name}.mo",
        src=src,
    )

    emit_record(
        name=bellcrank_name,
        params=bellcrank_params,
        out_path=out_dir / f"{bellcrank_name}.mo",
        src=src,
        extends=base_name,
    )


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

    out_dir = Path("VehicleDynamics/Resources/Records/SUS")

    emit_axle(data, axle="Front", prefix="Fr", out_dir=out_dir, src=src)
    emit_axle(data, axle="Rear",  prefix="Rr", out_dir=out_dir, src=src)

    print("Generated:")
    print(f"  - {out_dir / 'FrAxleBase.mo'}")
    print(f"  - {out_dir / 'FrAxleBellcrank.mo'}")
    print(f"  - {out_dir / 'RrAxleBase.mo'}")
    print(f"  - {out_dir / 'RrAxleBellcrank.mo'}")


if __name__ == "__main__":
    main()

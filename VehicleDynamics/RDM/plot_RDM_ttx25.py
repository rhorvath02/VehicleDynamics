"""
================================================================================
plot_RDM_ttx25 — RDM v1 visualization & validation (MODEL + ERROR SURFACES)
================================================================================

Produces:
  1) Individual 2D curve plots (data vs model)
  2) Model-driven force surfaces (COMP / REB)
  3) Signed error surfaces (Model − Data) interpolated from dyno points

All surfaces:
  - STRICTLY data-supported
  - No v == 0 points
  - No silent extrapolation
================================================================================
"""

import numpy as np
import pathlib
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.interpolate import griddata
from matplotlib import cm

from VehicleDynamics.RDM.RDM_ttx25_fitted import FittedTTX25RDM

# =============================================================================
# CONFIG
# =============================================================================

HS_SURF_FIXED = 2.0

V_MAX = 0.30
N_V   = 180
N_LS  = 50

LS_MIN = 0.0
LS_MAX = 25.0

V_EPS = 1e-9

# =============================================================================
# PATHS
# =============================================================================

ROOT = pathlib.Path("./VehicleDynamics/RDM")
DATA_DIR = ROOT / "data/ttx25"
FIT_DIR  = ROOT / "fits"

PLOT_DIR = FIT_DIR / "plots"
SURF_DIR = PLOT_DIR / "surfaces"

PLOT_DIR.mkdir(parents=True, exist_ok=True)
SURF_DIR.mkdir(parents=True, exist_ok=True)

PARAMS_JSON = FIT_DIR / "ttx25_params.json"

# =============================================================================
# FILENAME FORMAT
# =============================================================================

FNAME_RE = re.compile(
    r"C(?P<C>\d+)_R(?P<R>\d+)"
    r"_LS_(?P<LSC>\d+p?\d*)_(?P<LSR>\d+p?\d*)"
    r"_HS_(?P<HSC>\d+p?\d*)_(?P<HSR>\d+p?\d*)"
)

def parse_float(s: str) -> float:
    return float(s.replace("p", "."))

# =============================================================================
# LOAD MODEL
# =============================================================================

rdm = FittedTTX25RDM(PARAMS_JSON)
print(">>> Loaded fitted RDM model")

# =============================================================================
# LOAD CSV DATA
# =============================================================================

csv_cache = []

for csv in sorted(DATA_DIR.glob("*.csv")):
    m = FNAME_RE.match(csv.stem)
    if not m:
        continue

    adj = dict(
        LS_comp=parse_float(m["LSC"]),
        LS_reb=parse_float(m["LSR"]),
        HS_comp=parse_float(m["HSC"]),
        HS_reb=parse_float(m["HSR"]),
    )

    data = np.loadtxt(csv, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[None, :]

    v = data[:, 0].astype(float)
    Fd = data[:, 1].astype(float)

    if np.max(np.abs(v)) > 5.0:
        v *= 1e-3

    msk = np.isfinite(v) & np.isfinite(Fd)
    v, Fd = v[msk], Fd[msk]

    idx = np.argsort(v)
    csv_cache.append((csv.stem, adj, v[idx], Fd[idx]))

print(f">>> Loaded {len(csv_cache)} curves")

# =============================================================================
# INDIVIDUAL CURVE PLOTS
# =============================================================================

for stem, adj, v, Fd in csv_cache:
    Fm = rdm.force(
        v,
        LS_comp=adj["LS_comp"],
        LS_reb=adj["LS_reb"],
        HS_comp=adj["HS_comp"],
        HS_reb=adj["HS_reb"],
    )

    plt.figure(figsize=(7, 5))
    plt.plot(v * 1e3, Fd, "o", ms=4, label="Data")
    plt.plot(v * 1e3, Fm, "-", lw=2, label="Model")
    plt.xlabel("Velocity (mm/s)")
    plt.ylabel("Force (N)")
    plt.title(stem)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(PLOT_DIR / f"{stem}.png", dpi=150)
    plt.close()

# =============================================================================
# MODEL FORCE SURFACES
# =============================================================================

def plot_force_surface(side: str):
    if side == "comp":
        v_line = np.linspace(V_EPS, V_MAX, N_V)
        title  = f"COMP force surface (HS={HS_SURF_FIXED})"
        fname  = f"surface_comp_HS_{HS_SURF_FIXED:.1f}.png"
    else:
        v_line = np.linspace(-V_MAX, -V_EPS, N_V)
        title  = f"REB force surface (HS={HS_SURF_FIXED})"
        fname  = f"surface_reb_HS_{HS_SURF_FIXED:.1f}.png"

    LS_line = np.linspace(LS_MIN, LS_MAX, N_LS)
    V, LSg = np.meshgrid(v_line, LS_line)

    V_flat  = V.ravel()
    LS_flat = LSg.ravel()

    if side == "comp":
        F_flat = rdm.force(
            V_flat,
            LS_comp=LS_flat,
            LS_reb=0.0,
            HS_comp=HS_SURF_FIXED,
            HS_reb=HS_SURF_FIXED,
        )
    else:
        F_flat = rdm.force(
            V_flat,
            LS_comp=0.0,
            LS_reb=LS_flat,
            HS_comp=HS_SURF_FIXED,
            HS_reb=HS_SURF_FIXED,
        )

    F = F_flat.reshape(V.shape)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot_surface(V * 1e3, LSg, F, alpha=0.85, linewidth=0)

    for _, adj, v_d, Fd in csv_cache:
        if side == "comp":
            if abs(adj["HS_comp"] - HS_SURF_FIXED) > 1e-6:
                continue
            mask = v_d > 0
            LS_val = adj["LS_comp"]
        else:
            if abs(adj["HS_reb"] - HS_SURF_FIXED) > 1e-6:
                continue
            mask = v_d < 0
            LS_val = adj["LS_reb"]

        if not np.any(mask):
            continue

        ax.scatter(
            v_d[mask] * 1e3,
            np.full(np.sum(mask), LS_val),
            Fd[mask],
            c="k",
            s=12,
        )

    ax.set_xlabel("Velocity (mm/s)")
    ax.set_ylabel("LS clicks")
    ax.set_zlabel("Force (N)")
    ax.set_title(title)

    plt.tight_layout()
    plt.savefig(SURF_DIR / fname, dpi=150)
    plt.show()

# =============================================================================
# ERROR SURFACES (MODEL − DATA)
# =============================================================================

def plot_error_surface(side: str):
    LS_vals = []
    v_lists = []
    err_lists = []

    HS_TOL = 1e-2  # REALISTIC tolerance for filename-derived floats

    # -------------------------------------------------------------------------
    # Collect per-LS error curves
    # -------------------------------------------------------------------------
    for _, adj, v_d, Fd in csv_cache:
        if side == "comp":
            if abs(adj["HS_comp"] - HS_SURF_FIXED) > HS_TOL:
                continue
            mask = v_d > 0
            LS_val = adj["LS_comp"]
        else:
            if abs(adj["HS_reb"] - HS_SURF_FIXED) > HS_TOL:
                continue
            mask = v_d < 0
            LS_val = adj["LS_reb"]

        if not np.any(mask):
            continue

        v = v_d[mask]
        F_data = Fd[mask]

        F_model = rdm.force(
            v,
            LS_comp=LS_val if side == "comp" else 0.0,
            LS_reb=LS_val if side == "reb" else 0.0,
            HS_comp=HS_SURF_FIXED,
            HS_reb=HS_SURF_FIXED,
        )

        err = F_model - F_data

        LS_vals.append(LS_val)
        v_lists.append(v)
        err_lists.append(err)

    if len(LS_vals) < 2:
        print(f"[WARN] Not enough {side.upper()} curves for error surface")
        return

    # -------------------------------------------------------------------------
    # Construct common velocity domain (intersection, not union)
    # -------------------------------------------------------------------------
    v_min = max(v.min() for v in v_lists)
    v_max = min(v.max() for v in v_lists)

    if v_max <= v_min:
        print(f"[WARN] No overlapping velocity domain for {side.upper()}")
        return

    v_common = np.linspace(v_min, v_max, 160)

    # -------------------------------------------------------------------------
    # Interpolate error curves onto common velocity grid
    # -------------------------------------------------------------------------
    E_rows = []
    for v, err in zip(v_lists, err_lists):
        E_rows.append(np.interp(v_common, v, err))

    LS_vals = np.array(LS_vals)
    E = np.vstack(E_rows)

    # sort LS axis
    idx = np.argsort(LS_vals)
    LS_vals = LS_vals[idx]
    E = E[idx, :]

    Vg, LSg = np.meshgrid(v_common * 1e3, LS_vals)

    # -------------------------------------------------------------------------
    # Plot
    # -------------------------------------------------------------------------
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    surf = ax.plot_surface(
        Vg,
        LSg,
        E,
        cmap=cm.coolwarm,
        linewidth=0,
        antialiased=True,
    )

    ax.set_xlabel("Velocity (mm/s)")
    ax.set_ylabel("LS clicks")
    ax.set_zlabel("Force error (N)")
    ax.set_title(f"{side.upper()} error surface (Model − Data)")

    fig.colorbar(surf, shrink=0.6, label="Force error (N)")
    plt.tight_layout()
    plt.savefig(
        SURF_DIR / f"error_surface_{side}_HS_{HS_SURF_FIXED:.1f}.png",
        dpi=150,
    )
    plt.show()


# =============================================================================
# RUN ALL
# =============================================================================

plot_force_surface("comp")
plot_force_surface("reb")

plot_error_surface("comp")
plot_error_surface("reb")

print("\n>>> DONE")

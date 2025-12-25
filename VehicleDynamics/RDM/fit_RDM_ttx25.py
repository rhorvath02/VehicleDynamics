# =============================================================================
# TTX25 RDM FITTER — GPU PRESCREEN + CONTINUATION + REAL-TIME PROGRESS (ROBUST)
#
# Output:
#   ./VehicleDynamics/RDM/fits/ttx25_params.json
# =============================================================================

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import pathlib
import re
import json
import time
import os

import numpy as np
from tqdm import tqdm
from scipy.optimize import least_squares
from joblib import Parallel, delayed

# CuPy optional
try:
    import cupy as cp
    _CUPY_OK = True
except Exception:
    cp = None
    _CUPY_OK = False


# =============================================================================
# CONFIG
# =============================================================================

DATA_DIR = pathlib.Path("./VehicleDynamics/RDM/data/ttx25")
OUTDIR   = pathlib.Path("./VehicleDynamics/RDM/fits")
OUTDIR.mkdir(parents=True, exist_ok=True)

N_JOBS   = int(os.environ.get("TTX25_N_JOBS", "8"))
RNG_SEED = 0

# continuation stages
CONTINUATION_VMAX  = [0.08, 0.25, None]
MAX_NFEV_PER_STAGE = [600, 1200, 2500]

# residual weighting
V_WEIGHT_THRESH = 0.05
V_WEIGHT_GAIN   = 5.0

# adjuster knots
LS_KNOTS = np.array([0, 2, 4, 6, 10, 15, 25], dtype=float)   # clicks
HS_KNOTS = np.array([0, 1, 2, 3, 4.3], dtype=float)          # turns

# robust loss
ROBUST_LOSS    = "linear"
ROBUST_F_SCALE = 1.0

# numerical safety + bounds eps
EPS_V0 = 1e-4
EPS_VK = 1e-4
P_MIN  = 0.25
P_MAX  = 6.0


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass(frozen=True)
class CurveMeta:
    valve_id: int
    LS_comp: float
    LS_reb: float
    HS_comp: float
    HS_reb: float
    file: str

@dataclass
class CurveData:
    v: np.ndarray
    F: np.ndarray
    abs_v: np.ndarray
    w: np.ndarray
    stage_masks: Dict[Optional[float], np.ndarray]
    meta: CurveMeta


# =============================================================================
# LOAD CSVs
# =============================================================================

FNAME_RE = re.compile(
    r"^C(?P<C>\d+)_R(?P<R>\d+)"
    r"_LS_(?P<LSC>\d+p?\d*)_(?P<LSR>\d+p?\d*)"
    r"_HS_(?P<HSC>\d+p?\d*)_(?P<HSR>\d+p?\d*)$"
)

def parse_float_token(s: str) -> float:
    return float(s.replace("p", "."))

valve_map: Dict[str, int] = {}

def get_valve_id(C: int, R: int) -> int:
    key = f"C{C}_R{R}"
    if key not in valve_map:
        valve_map[key] = len(valve_map)
    return valve_map[key]

def load_csv(path: pathlib.Path) -> Tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[None, :]
    v = data[:, 0].astype(float)
    F = data[:, 1].astype(float)

    # heuristic: if velocity looks like mm/s convert to m/s
    if np.nanmax(np.abs(v)) > 5.0:
        v *= 1e-3

    m = np.isfinite(v) & np.isfinite(F)
    v, F = v[m], F[m]

    idx = np.argsort(v)
    return v[idx], F[idx]

curve_data: List[CurveData] = []

for csv in sorted(DATA_DIR.glob("*.csv")):
    m = FNAME_RE.match(csv.stem)
    if not m:
        continue

    meta = CurveMeta(
        valve_id=get_valve_id(int(m["C"]), int(m["R"])),
        LS_comp=parse_float_token(m["LSC"]),
        LS_reb=parse_float_token(m["LSR"]),
        HS_comp=parse_float_token(m["HSC"]),
        HS_reb=parse_float_token(m["HSR"]),
        file=csv.name,
    )

    v, F = load_csv(csv)
    if v.size < 5:
        raise ValueError(f"Too few points in {csv.name}")

    abs_v = np.abs(v)
    w = 1.0 + V_WEIGHT_GAIN * (abs_v > V_WEIGHT_THRESH)

    stage_masks = {
        vm: (abs_v <= vm) if vm is not None else np.ones_like(v, dtype=bool)
        for vm in CONTINUATION_VMAX
    }

    curve_data.append(CurveData(v=v, F=F, abs_v=abs_v, w=w, stage_masks=stage_masks, meta=meta))

N_CURVES = len(curve_data)
N_VALVES = len(valve_map)
print(f"Loaded {N_CURVES} curves, {N_VALVES} valves")

if N_CURVES == 0:
    raise RuntimeError(f"No CSVs matched regex in: {DATA_DIR.resolve()}")


# =============================================================================
# PARAMETERIZATION
# =============================================================================
# Per valve baseline:
#   comp: F0, v0, p
#   reb : F0, v0, p
# => 6 per valve
#
# Shared maps:
#   LS_comp -> c_ls_comp(LS)
#   LS_reb  -> c_ls_reb(LS)
#   HS_comp -> c_hs_comp(HS), v_k_comp(HS)
#   HS_reb  -> c_hs_reb(HS),  v_k_reb(HS)

N_LS = len(LS_KNOTS)
N_HS = len(HS_KNOTS)

N_BASE_PER_VALVE = 6
N_MAPS = 2 * N_LS + 4 * N_HS
N_PARAMS = N_BASE_PER_VALVE * N_VALVES + N_MAPS


def unpack(theta: np.ndarray):
    off = 0
    base = theta[off:off + N_BASE_PER_VALVE * N_VALVES]
    off += N_BASE_PER_VALVE * N_VALVES

    LS_comp_c_ls = theta[off:off + N_LS]; off += N_LS
    LS_reb_c_ls  = theta[off:off + N_LS]; off += N_LS

    HS_comp_c_hs = theta[off:off + N_HS]; off += N_HS
    HS_comp_v_k  = theta[off:off + N_HS]; off += N_HS

    HS_reb_c_hs  = theta[off:off + N_HS]; off += N_HS
    HS_reb_v_k   = theta[off:off + N_HS]; off += N_HS

    maps = dict(
        LS_comp_c_ls=LS_comp_c_ls,
        LS_reb_c_ls=LS_reb_c_ls,
        HS_comp_c_hs=HS_comp_c_hs,
        HS_comp_v_k=HS_comp_v_k,
        HS_reb_c_hs=HS_reb_c_hs,
        HS_reb_v_k=HS_reb_v_k,
    )
    return base, maps


# =============================================================================
# BOUNDS + PROJECTION (CRITICAL FIX FOR `x0 is infeasible`)
# =============================================================================

def bounds() -> Tuple[np.ndarray, np.ndarray]:
    lo = np.full(N_PARAMS, -np.inf, dtype=float)
    hi = np.full(N_PARAMS,  np.inf, dtype=float)

    # per-valve
    for vid in range(N_VALVES):
        i = vid * N_BASE_PER_VALVE
        # F0 >= 0
        lo[i+0] = 0.0;    hi[i+0] = 8000.0
        lo[i+3] = 0.0;    hi[i+3] = 8000.0
        # v0
        lo[i+1] = EPS_V0; hi[i+1] = 0.5
        lo[i+4] = EPS_V0; hi[i+4] = 0.5
        # p
        lo[i+2] = P_MIN;  hi[i+2] = P_MAX
        lo[i+5] = P_MIN;  hi[i+5] = P_MAX

    off = N_BASE_PER_VALVE * N_VALVES

    # LS c_ls
    lo[off:off+N_LS] = 0.0;    hi[off:off+N_LS] = 2e6; off += N_LS
    lo[off:off+N_LS] = 0.0;    hi[off:off+N_LS] = 2e6; off += N_LS

    # HS c_hs and v_k (comp)
    lo[off:off+N_HS] = 0.0;    hi[off:off+N_HS] = 2e6; off += N_HS
    lo[off:off+N_HS] = EPS_VK; hi[off:off+N_HS] = 2.0; off += N_HS

    # HS c_hs and v_k (reb)
    lo[off:off+N_HS] = 0.0;    hi[off:off+N_HS] = 2e6; off += N_HS
    lo[off:off+N_HS] = EPS_VK; hi[off:off+N_HS] = 2.0; off += N_HS

    return lo, hi


def project_to_bounds(x: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """
    HARD projection + NaN/inf sanitation.
    This is the key to never seeing `x0 is infeasible`.
    """
    x = np.asarray(x, dtype=float).copy()
    bad = ~np.isfinite(x)
    if np.any(bad):
        # replace bad entries with midpoint of finite bounds if available, else 0
        mid = np.zeros_like(x)
        finite = np.isfinite(lo) & np.isfinite(hi)
        mid[finite] = 0.5 * (lo[finite] + hi[finite])
        x[bad] = mid[bad]
    # clip
    x = np.minimum(np.maximum(x, lo), hi)

    # extra safety: nudge strict-lower-bounded eps params away from boundary
    # (helps when float32->float64 round trips happen)
    x = np.where(x == lo, np.nextafter(x, np.inf), x)

    return x


# =============================================================================
# FORCE MODEL (CANONICAL RDM)
# =============================================================================

def force_mag(x: np.ndarray, F0: float, v0: float, p: float, c_ls: float, c_hs: float, v_k: float) -> np.ndarray:
    v0_eff = max(float(v0), EPS_V0)
    v_k_eff = max(float(v_k), EPS_VK)
    p_eff = float(np.clip(p, P_MIN, P_MAX))

    z = x / v_k_eff
    denom = 1.0 + np.exp(p_eff * np.log(np.maximum(z, 1e-12)))

    return (
        float(F0) * np.tanh(x / v0_eff)
        + float(c_hs) * x
        + (float(c_ls) * x) / denom
    )


# =============================================================================
# RESIDUAL
# =============================================================================

def residual(theta: np.ndarray, vmax: Optional[float]) -> np.ndarray:
    base, maps = unpack(theta)
    res_chunks: List[np.ndarray] = []

    for cd in curve_data:
        mask = cd.stage_masks[vmax]
        if not np.any(mask):
            continue

        v = cd.v[mask]
        x = np.abs(v)

        i = cd.meta.valve_id * N_BASE_PER_VALVE
        F0c, v0c, pc, F0r, v0r, pr = base[i:i+6]

        c_ls_c = np.interp(cd.meta.LS_comp, LS_KNOTS, maps["LS_comp_c_ls"])
        c_ls_r = np.interp(cd.meta.LS_reb,  LS_KNOTS, maps["LS_reb_c_ls"])

        c_hs_c = np.interp(cd.meta.HS_comp, HS_KNOTS, maps["HS_comp_c_hs"])
        v_k_c  = np.interp(cd.meta.HS_comp, HS_KNOTS, maps["HS_comp_v_k"])

        c_hs_r = np.interp(cd.meta.HS_reb,  HS_KNOTS, maps["HS_reb_c_hs"])
        v_k_r  = np.interp(cd.meta.HS_reb,  HS_KNOTS, maps["HS_reb_v_k"])

        Fp = np.zeros_like(v)

        comp = v >= 0
        if np.any(comp):
            Fp[comp] = force_mag(x[comp], F0c, v0c, pc, c_ls_c, c_hs_c, v_k_c)
        if np.any(~comp):
            Fp[~comp] = -force_mag(x[~comp], F0r, v0r, pr, c_ls_r, c_hs_r, v_k_r)

        res_chunks.append(cd.w[mask] * (Fp - cd.F[mask]))

    return np.concatenate(res_chunks) if res_chunks else np.zeros((0,), dtype=float)


# =============================================================================
# METRICS (POST-FIT, PHYSICAL, NORMALIZED)
# =============================================================================

def compute_metrics(theta: np.ndarray) -> Dict:
    base, maps = unpack(theta)

    curve_reports = []
    all_err = []
    all_err_w = []
    all_F = []

    for cd in curve_data:
        v = cd.v
        x = np.abs(v)

        i = cd.meta.valve_id * N_BASE_PER_VALVE
        F0c, v0c, pc, F0r, v0r, pr = base[i:i+6]

        c_ls_c = np.interp(cd.meta.LS_comp, LS_KNOTS, maps["LS_comp_c_ls"])
        c_ls_r = np.interp(cd.meta.LS_reb,  LS_KNOTS, maps["LS_reb_c_ls"])

        c_hs_c = np.interp(cd.meta.HS_comp, HS_KNOTS, maps["HS_comp_c_hs"])
        v_k_c  = np.interp(cd.meta.HS_comp, HS_KNOTS, maps["HS_comp_v_k"])

        c_hs_r = np.interp(cd.meta.HS_reb,  HS_KNOTS, maps["HS_reb_c_hs"])
        v_k_r  = np.interp(cd.meta.HS_reb,  HS_KNOTS, maps["HS_reb_v_k"])

        Fp = np.zeros_like(v)

        comp = v >= 0
        if np.any(comp):
            Fp[comp] = force_mag(x[comp], F0c, v0c, pc, c_ls_c, c_hs_c, v_k_c)
        if np.any(~comp):
            Fp[~comp] = -force_mag(x[~comp], F0r, v0r, pr, c_ls_r, c_hs_r, v_k_r)

        err = Fp - cd.F
        err_w = cd.w * err

        rmse = np.sqrt(np.mean(err**2))
        mae  = np.mean(np.abs(err))
        wrmse = np.sqrt(np.mean(err_w**2))

        F_scale = np.max(np.abs(cd.F))
        nrmse = rmse / F_scale if F_scale > 0 else np.nan
        nwrmse = wrmse / F_scale if F_scale > 0 else np.nan

        curve_reports.append({
            "file": cd.meta.file,
            "valve_id": cd.meta.valve_id,
            "LS_comp": cd.meta.LS_comp,
            "LS_reb": cd.meta.LS_reb,
            "HS_comp": cd.meta.HS_comp,
            "HS_reb": cd.meta.HS_reb,
            "rmse_N": float(rmse),
            "mae_N": float(mae),
            "weighted_rmse_N": float(wrmse),
            "nrmse": float(nrmse),
            "normalized_weighted_rmse": float(nwrmse),
            "force_scale_N": float(F_scale),
        })

        all_err.append(err)
        all_err_w.append(err_w)
        all_F.append(cd.F)

    all_err = np.concatenate(all_err)
    all_err_w = np.concatenate(all_err_w)
    all_F = np.concatenate(all_F)

    global_rmse = np.sqrt(np.mean(all_err**2))
    global_wrmse = np.sqrt(np.mean(all_err_w**2))
    global_mae = np.mean(np.abs(all_err))

    F_scale_global = np.max(np.abs(all_F))
    global_nrmse = global_rmse / F_scale_global if F_scale_global > 0 else np.nan
    global_nwrmse = global_wrmse / F_scale_global if F_scale_global > 0 else np.nan

    return {
        "global": {
            "rmse_N": float(global_rmse),
            "mae_N": float(global_mae),
            "weighted_rmse_N": float(global_wrmse),
            "nrmse": float(global_nrmse),
            "normalized_weighted_rmse": float(global_nwrmse),
            "force_scale_N": float(F_scale_global),
        },
        "per_curve": curve_reports,
    }


# =============================================================================
# INITIAL GUESS
# =============================================================================

def initial_guess() -> np.ndarray:
    theta = np.zeros(N_PARAMS, dtype=float)

    # per valve baseline
    for vid in range(N_VALVES):
        i = vid * N_BASE_PER_VALVE
        theta[i:i+6] = [200.0, 0.02, 1.5, 200.0, 0.02, 1.5]

    off = N_BASE_PER_VALVE * N_VALVES
    # LS maps
    theta[off:off+N_LS] = 3000.0; off += N_LS
    theta[off:off+N_LS] = 3000.0; off += N_LS
    # HS maps comp
    theta[off:off+N_HS] = 1500.0; off += N_HS
    theta[off:off+N_HS] = 0.08;   off += N_HS
    # HS maps reb
    theta[off:off+N_HS] = 1500.0; off += N_HS
    theta[off:off+N_HS] = 0.08;   off += N_HS

    return theta


# =============================================================================
# GPU PRESCREEN (returns FEASIBLE CPU candidates)  <<< FIXED
# =============================================================================

def prescreen(theta0: np.ndarray, n_samples: int = 200_000, keep: int = 64) -> np.ndarray:
    lo, hi = bounds()
    theta0 = project_to_bounds(theta0, lo, hi)

    # GPU path
    if _CUPY_OK:
        try:
            _ = cp.zeros(1)
            rng = cp.random.default_rng(RNG_SEED)

            t0 = cp.asarray(theta0, dtype=cp.float32)
            noise = rng.standard_normal((n_samples, t0.size), dtype=cp.float32)
            thetas = t0[None, :] + cp.float32(0.15) * noise

            # clamp on GPU
            thetas = cp.minimum(cp.maximum(thetas, cp.asarray(lo, dtype=cp.float32)),
                                cp.asarray(hi, dtype=cp.float32))

            # cheap score: L2 norm on baseline + map magnitudes (not perfect, but sane)
            # emphasize baseline feasibility:
            base_end = N_BASE_PER_VALVE * N_VALVES
            costs = (
                1e-6 * cp.sum(thetas[:, :base_end] ** 2, axis=1)
                + 1e-12 * cp.sum(thetas[:, base_end:] ** 2, axis=1)
            )

            best = cp.argsort(costs)[:keep]
            out = cp.asnumpy(thetas[best]).astype(float)

            # IMPORTANT: re-project on CPU to kill float32 boundary issues
            out = np.asarray([project_to_bounds(x, lo, hi) for x in out])
            return out

        except Exception:
            pass

    # CPU fallback
    rng = np.random.default_rng(RNG_SEED)
    noise = rng.standard_normal((n_samples, theta0.size)).astype(np.float32)
    thetas = theta0[None, :].astype(np.float32) + 0.15 * noise
    thetas = np.minimum(np.maximum(thetas, lo.astype(np.float32)), hi.astype(np.float32))

    base_end = N_BASE_PER_VALVE * N_VALVES
    costs = (1e-6 * np.sum(thetas[:, :base_end] ** 2, axis=1) +
             1e-12 * np.sum(thetas[:, base_end:] ** 2, axis=1))

    best = np.argsort(costs)[:keep]
    out = thetas[best].astype(float)
    out = np.asarray([project_to_bounds(x, lo, hi) for x in out])
    return out


# =============================================================================
# FIT ONE START (continuation)  <<< FIXED: always project before least_squares
# =============================================================================

def run_one_start(k: int, theta0: np.ndarray) -> Tuple[np.ndarray, float]:
    lo, hi = bounds()
    th = project_to_bounds(theta0, lo, hi)

    for stage_idx, vmax in enumerate(CONTINUATION_VMAX):
        th = project_to_bounds(th, lo, hi)  # enforce feasibility every stage

        res = least_squares(
            fun=lambda x: residual(x, vmax),
            x0=th,
            method="trf",
            bounds=(lo, hi),
            loss=ROBUST_LOSS,
            f_scale=ROBUST_F_SCALE,
            max_nfev=int(MAX_NFEV_PER_STAGE[stage_idx]),
            verbose=0,
        )
        th = res.x

    return th, float(res.cost)


# =============================================================================
# EXPORT (ttx25_params.json for plotting + Modelica ingestion)
# =============================================================================

def export_params(theta: np.ndarray, metrics: Dict) -> pathlib.Path:
    base, maps = unpack(theta)

    valves_out = []
    for vid in range(N_VALVES):
        i = vid * N_BASE_PER_VALVE
        F0c, v0c, pc, F0r, v0r, pr = base[i:i+6]

        valve_key = None
        for k, v in valve_map.items():
            if v == vid:
                valve_key = k
                break

        valves_out.append({
            "valve_key": valve_key,
            "compression": {"F0": float(F0c), "v0": float(v0c), "p": float(pc)},
            "rebound":     {"F0": float(F0r), "v0": float(v0r), "p": float(pr)},
        })

    out = {
        "schema": "TTX25_RDM_v1",
        "sign_convention": {
            "v_gt_0": "compression",
            "v_lt_0": "rebound",
            "force_sign": "aligned_with_velocity",
        },
        "knots": {
            "LS": LS_KNOTS.tolist(),
            "HS": HS_KNOTS.tolist(),
        },
        "valves": valves_out,
        "maps": {
            "LS_comp": {"c_ls": [float(x) for x in maps["LS_comp_c_ls"]]},
            "LS_reb":  {"c_ls": [float(x) for x in maps["LS_reb_c_ls"]]},
            "HS_comp": {
                "c_hs": [float(x) for x in maps["HS_comp_c_hs"]],
                "v_k":  [float(x) for x in maps["HS_comp_v_k"]],
            },
            "HS_reb": {
                "c_hs": [float(x) for x in maps["HS_reb_c_hs"]],
                "v_k":  [float(x) for x in maps["HS_reb_v_k"]],
            },
        },
        "fit": {
            "continuation_vmax": CONTINUATION_VMAX,
            "max_nfev_per_stage": MAX_NFEV_PER_STAGE,
            "robust_loss": ROBUST_LOSS,
            "robust_f_scale": ROBUST_F_SCALE,
            "v_weight_thresh": V_WEIGHT_THRESH,
            "v_weight_gain": V_WEIGHT_GAIN,
            "n_curves": N_CURVES,
            "n_valves": N_VALVES,
        },
        "metrics": metrics,
    }

    path = OUTDIR / "ttx25_params.json"
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    return path


# =============================================================================
# MAIN
# =============================================================================

def main():
    t0 = time.time()
    candidates = prescreen(initial_guess(), n_samples=200_000, keep=64)

    if _CUPY_OK:
        try:
            cp.cuda.Device().synchronize()
        except Exception:
            pass

    print(f"GPU prescreen done in {time.time() - t0:.2f}s")

    pbar = tqdm(total=len(candidates), desc="Continuation fits")

    def wrapped(k: int):
        th = candidates[k]
        sol = run_one_start(k, th)
        pbar.update(1)
        return sol

    # FORCE THREADING backend so tqdm updates reliably and no loky remote tracebacks
    results = Parallel(n_jobs=N_JOBS, backend="threading")(
        delayed(wrapped)(k) for k in range(len(candidates))
    )
    pbar.close()

    best_theta, best_cost = min(results, key=lambda r: r[1])
    print(f"Best cost: {best_cost:.3e}")

    metrics = compute_metrics(best_theta)

    print(
        f"Global NRMSE: {metrics['global']['nrmse']:.4f}, "
        f"Weighted NRMSE: {metrics['global']['normalized_weighted_rmse']:.4f}"
    )

    out_path = export_params(best_theta, metrics)
    print(f"Saved: {out_path.resolve()}")



if __name__ == "__main__":
    main()

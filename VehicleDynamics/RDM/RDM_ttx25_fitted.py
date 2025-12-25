"""
================================================================================
FittedTTX25RDM — Concrete Robert Damper Model for fitted TTX25 data
================================================================================

Implements the canonical RDM force law using parameters loaded from
a TTX25_RDM_v1 JSON file.

Force law (per side):
---------------------
    x = |v|

    F_mag(x) = F0 * tanh(x / v0)
             + c_hs * x
             + (c_ls * x) / (1 + (x / v_k)^p)

Returned force is SIGNED and aligned with velocity:
    v > 0 → compression → +F
    v < 0 → rebound     → -F

--------------------------------------------------------------------------------
PUBLIC API
--------------------------------------------------------------------------------

    rdm = FittedTTX25RDM("ttx25_params.json")

    F = rdm.force(
        v,
        LS_comp=..., LS_reb=...,
        HS_comp=..., HS_reb=...
    )

--------------------------------------------------------------------------------
"""

from __future__ import annotations

import json
import numpy as np
from typing import Union


# =============================================================================
# BASE RDM CLASS (unchanged contract)
# =============================================================================

class RDM:
    """
    Base class for Robert Damper Models.
    """

    def __init__(self, eps: float = 1e-9):
        self.eps = eps

    def force(self, v: Union[float, np.ndarray], **adjusters) -> np.ndarray:
        """
        Evaluate signed damper force.

        Parameters
        ----------
        v : float or ndarray
            Shaft velocity [m/s]
        adjusters : keyword arguments
            LS_comp, LS_reb, HS_comp, HS_reb

        Returns
        -------
        F : ndarray
            Signed force [N]
        """
        v = np.asarray(v, dtype=float)
        x = np.sqrt(v * v + self.eps * self.eps)

        F = np.zeros_like(v)

        comp = v >= 0
        reb  = v < 0

        if np.any(comp):
            F[comp] = self._force_side(x[comp], side="comp", **adjusters)

        if np.any(reb):
            F[reb] = -self._force_side(x[reb], side="reb", **adjusters)

        return F

    def _force_side(self, x: np.ndarray, side: str, **adjusters) -> np.ndarray:
        raise NotImplementedError


# =============================================================================
# FITTED TTX25 IMPLEMENTATION
# =============================================================================

class FittedTTX25RDM(RDM):
    """
    Concrete RDM implementation for fitted Öhlins TTX25 parameters.
    """

    # -------------------------------------------------------------------------
    # Construction
    # -------------------------------------------------------------------------

    def __init__(self, params_json_path: str, eps: float = 1e-9):
        super().__init__(eps=eps)

        with open(params_json_path, "r") as f:
            params = json.load(f)

        # ---- schema sanity ---------------------------------------------------
        if params.get("schema") != "TTX25_RDM_v1":
            raise ValueError(
                f"Unsupported schema: {params.get('schema')}"
            )

        # ---- knots -----------------------------------------------------------
        self.LS_knots = np.asarray(params["knots"]["LS"], dtype=float)
        self.HS_knots = np.asarray(params["knots"]["HS"], dtype=float)

        # ---- valve baseline --------------------------------------------------
        valve = params["valves"][0]   # single-valve model (by design)

        self.base = {
            "comp": {
                "F0": float(valve["compression"]["F0"]),
                "v0": float(valve["compression"]["v0"]),
                "p":  float(valve["compression"]["p"]),
            },
            "reb": {
                "F0": float(valve["rebound"]["F0"]),
                "v0": float(valve["rebound"]["v0"]),
                "p":  float(valve["rebound"]["p"]),
            },
        }

        # ---- maps ------------------------------------------------------------
        maps = params["maps"]

        self.maps = {
            # LS → c_ls
            "LS_comp_c_ls": np.asarray(maps["LS_comp"]["c_ls"], dtype=float),
            "LS_reb_c_ls":  np.asarray(maps["LS_reb"]["c_ls"], dtype=float),

            # HS → c_hs, v_k
            "HS_comp_c_hs": np.asarray(maps["HS_comp"]["c_hs"], dtype=float),
            "HS_comp_v_k":  np.asarray(maps["HS_comp"]["v_k"],  dtype=float),

            "HS_reb_c_hs":  np.asarray(maps["HS_reb"]["c_hs"], dtype=float),
            "HS_reb_v_k":   np.asarray(maps["HS_reb"]["v_k"],  dtype=float),
        }

    # -------------------------------------------------------------------------
    # Core constitutive law
    # -------------------------------------------------------------------------

    def _force_side(
        self,
        x: np.ndarray,
        side: str,
        LS_comp: float,
        LS_reb: float,
        HS_comp: float,
        HS_reb: float,
    ) -> np.ndarray:
        """
        Force magnitude for one side (compression or rebound).

        Parameters
        ----------
        x : ndarray
            Absolute velocity |v| >= 0
        side : "comp" or "reb"
        LS_comp, LS_reb : float
            Low-speed adjuster clicks
        HS_comp, HS_reb : float
            High-speed adjuster turns

        Returns
        -------
        F_mag : ndarray
            Positive force magnitude [N]
        """

        if side == "comp":
            LS = LS_comp
            HS = HS_comp
            base = self.base["comp"]
            c_ls = np.interp(LS, self.LS_knots, self.maps["LS_comp_c_ls"])
            c_hs = np.interp(HS, self.HS_knots, self.maps["HS_comp_c_hs"])
            v_k  = np.interp(HS, self.HS_knots, self.maps["HS_comp_v_k"])
        else:
            LS = LS_reb
            HS = HS_reb
            base = self.base["reb"]
            c_ls = np.interp(LS, self.LS_knots, self.maps["LS_reb_c_ls"])
            c_hs = np.interp(HS, self.HS_knots, self.maps["HS_reb_c_hs"])
            v_k  = np.interp(HS, self.HS_knots, self.maps["HS_reb_v_k"])

        # ---- numerical safety -----------------------------------------------
        v0 = max(base["v0"], 1e-6)
        v_k = max(v_k, 1e-6)
        p   = np.clip(base["p"], 0.25, 6.0)

        # ---- canonical force law --------------------------------------------
        return (
            base["F0"] * np.tanh(x / v0)
            + c_hs * x
            + (c_ls * x) / (1.0 + (x / v_k) ** p)
        )

    # -------------------------------------------------------------------------
    # Introspection helpers (optional, but useful)
    # -------------------------------------------------------------------------

    def parameter_schema(self):
        return {
            "baseline": {
                "F0_comp": "Near-zero force [N]",
                "v0_comp": "Zero-speed smoothing velocity [m/s]",
                "p_comp":  "Knee sharpness exponent",
                "F0_reb":  "Near-zero force [N]",
                "v0_reb":  "Zero-speed smoothing velocity [m/s]",
                "p_reb":   "Knee sharpness exponent",
            },
            "maps": {
                "LS_comp": "LS clicks → c_ls_comp",
                "LS_reb":  "LS clicks → c_ls_reb",
                "HS_comp": "HS turns → c_hs_comp, v_k_comp",
                "HS_reb":  "HS turns → c_hs_reb, v_k_reb",
            },
        }

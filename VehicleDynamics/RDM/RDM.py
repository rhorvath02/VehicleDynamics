"""
================================================================================
RDM - Robert Damper Model
================================================================================

Base class defining the interface and conventions for damper constitutive models.

IMPORTANT:
---------
RDM does NOT define a specific parameterization.
It defines the *contract* that subclasses must satisfy.

Subclasses (e.g. FittedTTX25RDM) define:
- which coefficients exist
- how adjusters map to coefficients
- the force law itself

Fitting scripts operate on those coefficients as pure numerical parameters.

-------------------------------------------------------------------------------
SIGN CONVENTION
-------------------------------------------------------------------------------
- v > 0 : compression
- v < 0 : rebound
- Returned force is SIGNED and aligned with velocity

-------------------------------------------------------------------------------
EXPECTED COEFFICIENT STRUCTURE (CANONICAL)
-------------------------------------------------------------------------------

Most physically-based RDM subclasses will follow this pattern:

Per-side baseline coefficients (compression & rebound):

    F0_side   : near-zero force magnitude          [N]
    v0_side   : zero-speed smoothing velocity      [m/s]
    p_side    : knee sharpness exponent             [-]

Adjuster-mapped coefficients:

    c_ls_side(LS_clicks) : low-speed damping       [N/(m/s)]
    c_hs_side(HS_turns)  : high-speed damping      [N/(m/s)]
    v_k_side(HS_turns)   : knee velocity            [m/s]

This yields the canonical force law:

    F(x) = F0 * tanh(x / v0)
         + c_hs * x
         + (c_ls * x) / (1 + (x / v_k)^p)

Where x = |v|.

RDM does NOT enforce this — it documents it.
================================================================================
"""

import numpy as np


class RDM:
    """
    Base class for Robert Damper Models.

    Subclasses should override:
        _force_side(x, side, **adjusters)

    The returned value must be a POSITIVE force magnitude.
    """

    def __init__(self, eps=1e-9):
        self.eps = eps

    # --------------------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------------------

    def force(self, v, **adjusters):
        """
        Evaluate signed damper force.

        Parameters
        ----------
        v : float or ndarray
            Damper shaft velocity [m/s]
        adjusters : keyword arguments
            Damper settings (LSC, LSR, HSC, HSR, etc.)

        Returns
        -------
        F : float or ndarray
            Signed damper force [N]
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

    # --------------------------------------------------------------------------
    # Constitutive law (intended override)
    # --------------------------------------------------------------------------

    def _force_side(self, x, side, **adjusters):
        """
        Force magnitude for one side of the damper.

        Parameters
        ----------
        x : ndarray
            Absolute shaft velocity |v| >= 0 [m/s]
        side : str
            "comp" or "reb"
        adjusters : keyword arguments

        Returns
        -------
        F_mag : ndarray
            Positive force magnitude [N]
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement _force_side()"
        )

    # --------------------------------------------------------------------------
    # Parameter documentation hooks
    # --------------------------------------------------------------------------

    def parameter_schema(self):
        """
        Return a dictionary describing model parameters.

        Intended to be overridden by subclasses.

        Example:
        --------
        {
            "baseline": {
                "F0_comp": "Near-zero force [N]",
                "v0_comp": "Zero-speed smoothing velocity [m/s]",
                "p_comp":  "Knee sharpness exponent",
                ...
            },
            "maps": {
                "LS_comp": "LS clicks -> c_ls_comp",
                "HS_comp": "HS turns -> c_hs_comp, v_k_comp",
                ...
            }
        }
        """
        return {}

    # --------------------------------------------------------------------------
    # Optional helpers
    # --------------------------------------------------------------------------

    def parameter_summary(self):
        """
        Human-readable parameter summary.
        """
        return str(self.parameter_schema())

    def sanity_check(self):
        """
        Optional consistency checks.
        """
        return True

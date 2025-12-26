within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.RDM;

function RDM_eval "Robert Damper Model: damper force evaluation, similar to MF"
  import Modelica.Math.{tanh, exp, log};

  // Inputs
  input Real v "Damper shaft velocity [m/s], v > 0 compression, v < 0 rebound";
  input Real LS_comp "Low-speed compression adjuster [clicks]";
  input Real LS_reb  "Low-speed rebound adjuster [clicks]";
  input Real HS_comp "High-speed compression adjuster [turns]";
  input Real HS_reb  "High-speed rebound adjuster [turns]";
  input Integer valve_id(min=1) "1-based valve index";

  // Parameter data (from ExternData JSON)
  input Real[:] LS_knots "LS knot vector [clicks]";
  input Real[:] HS_knots "HS knot vector [turns]";

  // Per-valve baseline: (F0c, v0c, pc, F0r, v0r, pr)
  input Real[:,6] valve_base;

  // Maps (knot-aligned)
  input Real[:] LS_comp_c_ls;
  input Real[:] LS_reb_c_ls;
  input Real[:] HS_comp_c_hs;
  input Real[:] HS_comp_v_k;
  input Real[:] HS_reb_c_hs;
  input Real[:] HS_reb_v_k;

  // Numerical safety
  input Real EPS_V0 = 1e-4;
  input Real EPS_VK = 1e-4;
  input Real P_MIN  = 0.25;
  input Real P_MAX  = 6.0;

  // Output
  output Real F "Damper force [N], aligned with velocity sign";

protected
  // Local variables (MF-style layout)
  Real x "Absolute shaft velocity";
  Real F0 "Baseline force scale";
  Real v0 "Tanh velocity scale";
  Real p  "LS-HS blending exponent";

  Real c_ls "Low-speed damping coefficient";
  Real c_hs "High-speed damping coefficient";
  Real v_k  "LS-HS transition velocity";

  Real z;
  Real denom;
  Real Fmag;

  // Local linear interpolation (clamped)
  function interp1
    input Real u;
    input Real[:] xp;
    input Real[:] fp;
    output Real y;
  protected
    Integer i;
    Real t;
  algorithm
    if u <= xp[1] then
      y := fp[1];
    elseif u >= xp[end] then
      y := fp[end];
    else
      i := 1;
      while not (xp[i] <= u and u <= xp[i+1]) loop
        i := i + 1;
      end while;
      t := (u - xp[i]) / (xp[i+1] - xp[i]);
      y := (1 - t)*fp[i] + t*fp[i+1];
    end if;
  end interp1;

algorithm
  // Velocity magnitude
  x := abs(v);

  // Compression / Rebound branch
  if v >= 0 then
    // Compression
    F0 := valve_base[valve_id, 1];
    v0 := valve_base[valve_id, 2];
    p  := valve_base[valve_id, 3];

    c_ls := interp1(LS_comp, LS_knots, LS_comp_c_ls);
    c_hs := interp1(HS_comp, HS_knots, HS_comp_c_hs);
    v_k  := interp1(HS_comp, HS_knots, HS_comp_v_k);

  else
    // Rebound
    F0 := valve_base[valve_id, 4];
    v0 := valve_base[valve_id, 5];
    p  := valve_base[valve_id, 6];

    c_ls := interp1(LS_reb, LS_knots, LS_reb_c_ls);
    c_hs := interp1(HS_reb, HS_knots, HS_reb_c_hs);
    v_k  := interp1(HS_reb, HS_knots, HS_reb_v_k);
  end if;

  // Numerical guards
  v0 := max(v0, EPS_V0);
  v_k := max(v_k, EPS_VK);
  p  := min(max(p, P_MIN), P_MAX);

  // Canonical RDM force law
  // F = F0*tanh(x/v0)
  //   + c_hs*x
  //   + (c_ls*x)/(1 + (x/v_k)^p)
  z := max(x / v_k, 1e-12);
  denom := 1 + exp(p * log(z));

  Fmag := F0*tanh(x / v0)
        + c_hs*x
        + (c_ls*x) / denom;

  // Sign convention: force aligned with velocity
  if v >= 0 then
    F :=  Fmag;
  else
    F := -Fmag;
  end if;

end RDM_eval;

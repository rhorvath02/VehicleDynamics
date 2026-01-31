within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrUCA
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 0.55776965;
  parameter SIunits.Position r_cm[3] = {-0.00187916, 0.45854113, 0.25343195};
  parameter SIunits.Inertia I[3,3] = {{0.16010718, -0.00106496, -0.00037293}, {-0.00106496, 0.03698976, 0.06598056}, {-0.00037293, 0.06598056, 0.12490358}};

end RrUCA;

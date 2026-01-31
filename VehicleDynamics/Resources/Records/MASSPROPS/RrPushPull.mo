within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrPushPull
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 0.12345678;
  parameter SIunits.Position r_cm[3] = {0.00384464, 0.38672888, 0.34796099};
  parameter SIunits.Inertia I[3,3] = {{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.01}};

end RrPushPull;

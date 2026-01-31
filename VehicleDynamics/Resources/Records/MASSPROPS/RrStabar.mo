within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrStabar
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 1;
  parameter SIunits.Position r_cm[3] = {-0.01, 0.4, 0.5};
  parameter SIunits.Inertia I[3,3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

end RrStabar;

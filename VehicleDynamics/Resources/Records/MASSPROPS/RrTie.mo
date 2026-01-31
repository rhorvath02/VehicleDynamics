within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrTie
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 0.13459415;
  parameter SIunits.Position r_cm[3] = {0.05709287, 0.34616483, 0.1281302};
  parameter SIunits.Inertia I[3,3] = {{0.0201276, 0.00265922, 0.0009845}, {0.00265922, 0.00267834, 0.0061809}, {0.0009845, 0.0061809, 0.01833132}};

end RrTie;

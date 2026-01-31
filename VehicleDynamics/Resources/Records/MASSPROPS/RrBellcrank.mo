within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrBellcrank
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 0.23456789;
  parameter SIunits.Position r_cm[3] = {-0.025, 0.3, 0.35};
  parameter SIunits.Inertia I[3,3] = {{0.015, 0, 0}, {0, 0.015, 0}, {0, 0, 0.015}};

end RrBellcrank;

within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrLCA
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 0.5182905;
  parameter SIunits.Position r_cm[3] = {0.00803455, 0.40581792, 0.09882118};
  parameter SIunits.Inertia I[3,3] = {{0.09691535, 0.0015142, 0.00039327}, {0.0015142, 0.00654926, 0.0214741}, {0.00039327, 0.0214741, 0.09315229}};

end RrLCA;

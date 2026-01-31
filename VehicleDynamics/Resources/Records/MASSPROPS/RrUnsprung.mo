within VehicleDynamics.Resources.Records.MASSPROPS;

// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rober/shared/VehicleDynamics/VehicleDynamics/Resources/JSONs/MASSPROPS/mass_props.json
// Tool: convert_massprops_json_to_record.py
// ============================================================================

record RrUnsprung
  "Auto-generated rigid body mass properties"

  import Modelica.SIunits;

  parameter SIunits.Mass m = 7.8160579;
  parameter SIunits.Position r_cm[3] = {-0.0061298, 0.60174377, 0.19797979};
  parameter SIunits.Inertia I[3,3] = {{3.24231873, -0.02844717, -0.00889662}, {-0.02844717, 0.46729195, 0.93039709}, {-0.00889662, 0.93039709, 2.93847123}};

end RrUnsprung;

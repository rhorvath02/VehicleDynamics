// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rhorvath2002/Documents/Github/VehicleDynamics/VehicleDynamics/Resources/JSONs/SUS/tune.json
// Tool: convert_suspension_json_to_record.py
// ============================================================================

within VehicleDynamics.Resources.Records.SUS;

record FrAxleBase
  "Auto-generated suspension parameter record"

  import Modelica.SIunits;

  parameter SIunits.Position upper_fore_i[3] = {0.1016, 0.237744, 0.2143252};
  parameter SIunits.Position upper_fore_i_c[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position upper_fore_i_d[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position upper_aft_i[3] = {-0.0680974, 0.2356358, 0.215138};
  parameter SIunits.Position upper_aft_i_c[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position upper_aft_i_d[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position upper_outboard[3] = {-0.0092964, 0.5420106, 0.2679954};
  parameter SIunits.Position lower_fore_i[3] = {0.1016, 0.226314, 0.08001};
  parameter SIunits.Position lower_fore_i_c[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position lower_fore_i_d[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position lower_aft_i[3] = {-0.0762, 0.226314, 0.08001};
  parameter SIunits.Position lower_aft_i_c[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position lower_aft_i_d[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position lower_outboard[3] = {0.0029972, 0.562991, 0.1139952};
  parameter SIunits.Position tie_inboard[3] = {0.05715, 0.2260092, 0.1137158};
  parameter SIunits.Position tie_inboard_c[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position tie_inboard_d[3] = {1000000000, 1000000000, 1000000000};
  parameter SIunits.Position tie_outboard[3] = {0.0569976, 0.546989, 0.1522222};
  parameter SIunits.Position wheel_center[3] = {0, 0.606110767456, 0.199898};
  parameter SIunits.Angle static_gamma = 0;
  parameter SIunits.Angle static_alpha = 0;

end FrAxleBase;

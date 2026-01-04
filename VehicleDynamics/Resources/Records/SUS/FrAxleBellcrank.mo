// ============================================================================
// AUTO-GENERATED FILE — DO NOT EDIT
// Source: /home/rhorvath2002/Documents/Github/VehicleDynamics/VehicleDynamics/Resources/JSONs/SUS/tune.json
// Tool: convert_suspension_json_to_record.py
// ============================================================================

within VehicleDynamics.Resources.Records.SUS;

record FrAxleBellcrank
  "Auto-generated suspension parameter record"

  import Modelica.SIunits;

  extends FrAxleBase;

  parameter SIunits.Position bellcrank_pivot[3] = {-0.042144464098, 0.250754351932, 0.370010000136};
  parameter SIunits.Position bellcrank_pivot_ref[3] = {-0.017822781728, 0.244001173922, 0.36717974316};
  parameter SIunits.Position bellcrank_pickup_1[3] = {-0.029010173628, 0.2971411507, 0.37219698865};
  parameter SIunits.Position bellcrank_pickup_2[3] = {-0.014493326106, 0.348410770284, 0.374614186762};
  parameter SIunits.Position bellcrank_pickup_3[3] = {-0.01102742905, 0.34553503283, 0.411259910778};
  parameter SIunits.Position rod_mount[3] = {0.006762552642, 0.525610676234, 0.134465050856};
  parameter SIunits.Position shock_mount[3] = {-0.020673469702, 0.247847085458, 0.561456926868};

end FrAxleBellcrank;

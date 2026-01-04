within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;
model RrAxleBellcrank
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  extends VehicleDynamics.Vehicle.Chassis.Suspension.RrAxleBase;
  
  final parameter VehicleDynamics.Resources.Records.SUS.RrAxleBellcrank RLBC;
  
  parameter SIunits.Position RL_bellcrank_pivot[3] = RLBC.bellcrank_pivot annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pivot_ref[3] = RLBC.bellcrank_pivot_ref annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_1[3] = RLBC.bellcrank_pickup_1 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_2[3] = RLBC.bellcrank_pickup_2 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_3[3] = RLBC.bellcrank_pickup_3 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_UCA_mount[3] = RLBC.rod_mount annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_shock_mount[3] = RLBC.shock_mount annotation(
    Dialog(group = "Geometry"));
  
  // FL bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_bellcrank_mount(r = RL_bellcrank_pivot - effective_center)  annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RL_bellcrank(pivot = RL_bellcrank_pivot, pivot_ref = RL_bellcrank_pivot_ref, pickup_1 = RL_bellcrank_pickup_1, pickup_2 = RL_bellcrank_pickup_2, pickup_3 = RL_bellcrank_pickup_3)  annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
  
  // FL outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_rod_out(r = RL_UCA_mount - RL_upper_o)  annotation(
    Placement(transformation(origin = {-110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Link RL_pushrod(inboard = RL_UCA_mount, outboard = RL_bellcrank_pickup_1, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {-90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));

  // FR bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_bellcrank_mount(r = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RR_bellcrank(
    pickup_1 = {RL_bellcrank_pickup_1[1], -RL_bellcrank_pickup_1[2], RL_bellcrank_pickup_1[3]},
    pickup_2 = {RL_bellcrank_pickup_2[1], -RL_bellcrank_pickup_2[2], RL_bellcrank_pickup_2[3]},
    pickup_3 = {RL_bellcrank_pickup_3[1], -RL_bellcrank_pickup_3[2], RL_bellcrank_pickup_3[3]},
    pivot = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]},
    pivot_ref = {RL_bellcrank_pivot_ref[1], -RL_bellcrank_pivot_ref[2], RL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // FR outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_rod_out(r = {RL_UCA_mount[1], -RL_UCA_mount[2], RL_UCA_mount[3]} - RR_upper_o) annotation(
    Placement(transformation(origin = {110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Link RR_pushrod(inboard = {RL_UCA_mount[1], -RL_UCA_mount[2], RL_UCA_mount[3]}, joint_diameter = 0.825*0.0254, link_diameter = 0.625*0.0254, outboard = {RL_bellcrank_pickup_1[1], -RL_bellcrank_pickup_1[2], RL_bellcrank_pickup_1[3]}) annotation(
    Placement(transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  
  // FL shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_shock_pickup(r = RL_shock_mount - effective_center)  annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Linkages.ShockLinkage RL_shock(spring_diameter = 0.040, start_point = RL_bellcrank_pickup_2, end_point = RL_shock_mount, rod_length_fraction = 0.5, free_length = norm(RL_shock_mount - RL_bellcrank_pickup_2)/2, spring_table = [0, 0; 1, 80000], spring_mass = 0, damper_table = [0, 0; 0.25, 100], damper_mass = 0, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));

  // FR shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_shock_pickup(r = {RL_shock_mount[1], -RL_shock_mount[2], RL_shock_mount[3]} - effective_center)  annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  Linkages.ShockLinkage RR_shock(spring_diameter = 0.040, start_point = {RL_bellcrank_pickup_2[1], -RL_bellcrank_pickup_2[2], RL_bellcrank_pickup_2[3]}, end_point = {RL_shock_mount[1], -RL_shock_mount[2], RL_shock_mount[3]}, rod_length_fraction = 0.5, free_length = norm(RL_shock_mount - RL_bellcrank_pickup_2)/2, spring_table = [0, 0; 1, 80000], spring_mass = 0, damper_table = [0, 0; 0.25, 100], damper_mass = 0, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    
equation
  connect(RL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_bellcrank_mount.frame_b, RL_bellcrank.mount_frame) annotation(
    Line(points = {{-40, 40}, {-50, 40}}, color = {95, 95, 95}));
  connect(RL_rod_out.frame_b, RL_pushrod.frame_a) annotation(
    Line(points = {{-110, 0}, {-110, 30}, {-100, 30}}, color = {95, 95, 95}));
  connect(RR_rod_out.frame_b, RR_pushrod.frame_a) annotation(
    Line(points = {{110, 0}, {110, 30}, {100, 30}}, color = {95, 95, 95}));
  connect(RR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_bellcrank_mount.frame_b, RR_bellcrank.mount_frame) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {95, 95, 95}));
  connect(RL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_shock.frame_b, RL_shock_pickup.frame_b) annotation(
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(RR_shock.frame_b, RR_shock_pickup.frame_b) annotation(
    Line(points = {{40, 70}, {30, 70}}, color = {95, 95, 95}));
  connect(RL_rod_out.frame_a, RL_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{-110, -20}, {-110, -30}, {-100, -30}}, color = {95, 95, 95}));
  connect(RR_rod_out.frame_a, RR_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{110, -20}, {110, -30}, {100, -30}}, color = {95, 95, 95}));
  connect(RL_shock.frame_a, RL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-60, 70}, {-80, 70}, {-80, 40}, {-70, 40}}, color = {95, 95, 95}));
  connect(RL_pushrod.frame_b, RL_bellcrank.pickup_1_frame) annotation(
    Line(points = {{-80, 30}, {-60, 30}}, color = {95, 95, 95}));
  connect(RR_pushrod.frame_b, RR_bellcrank.pickup_1_frame) annotation(
    Line(points = {{80, 30}, {60, 30}}, color = {95, 95, 95}));
  connect(RR_shock.frame_a, RR_bellcrank.pickup_2_frame) annotation(
    Line(points = {{60, 70}, {80, 70}, {80, 40}, {70, 40}}, color = {95, 95, 95}));
end RrAxleBellcrank;
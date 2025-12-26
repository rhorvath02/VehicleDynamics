within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;
model FrAxleBellcrank
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  extends VehicleDynamics.Vehicle.Chassis.Suspension.FrAxleBase;
  
  parameter SIunits.Position FL_bellcrank_pivot[3] = hdpts.getRealArray1D("Front.left.bellcrank.pivot", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pivot_ref[3] = hdpts.getRealArray1D("Front.left.bellcrank.pivot_ref", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_1[3] = hdpts.getRealArray1D("Front.left.bellcrank.pickup_1", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_2[3] = hdpts.getRealArray1D("Front.left.bellcrank.pickup_2", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_3[3] = hdpts.getRealArray1D("Front.left.bellcrank.pickup_3", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_LCA_mount[3] = hdpts.getRealArray1D("Front.left.push/pull rod.LCA_mount", 3) annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_shock_mount[3] = hdpts.getRealArray1D("Front.left.push/pull rod.shock_mount", 3) annotation(
    Dialog(group = "Geometry"));
  
  // FL bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_bellcrank_mount(r = FL_bellcrank_pivot - effective_center)  annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FL_bellcrank(pivot = FL_bellcrank_pivot, pivot_ref = FL_bellcrank_pivot_ref, pickup_1 = FL_bellcrank_pickup_1, pickup_2 = FL_bellcrank_pickup_2, pickup_3 = FL_bellcrank_pickup_3)  annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
  
  // FL outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_rod_out(r = FL_LCA_mount - FL_lower_o)  annotation(
    Placement(transformation(origin = {-110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Link FL_pushrod(inboard = FL_LCA_mount, outboard = FL_bellcrank_pickup_2, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  
  // FR bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_bellcrank_mount(r = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FR_bellcrank(
    pickup_1 = {FL_bellcrank_pickup_1[1], -FL_bellcrank_pickup_1[2], FL_bellcrank_pickup_1[3]},
    pickup_2 = {FL_bellcrank_pickup_2[1], -FL_bellcrank_pickup_2[2], FL_bellcrank_pickup_2[3]},
    pickup_3 = {FL_bellcrank_pickup_3[1], -FL_bellcrank_pickup_3[2], FL_bellcrank_pickup_3[3]},
    pivot = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]},
    pivot_ref = {FL_bellcrank_pivot_ref[1], -FL_bellcrank_pivot_ref[2], FL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // FR outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_rod_out(r = {FL_LCA_mount[1], -FL_LCA_mount[2], FL_LCA_mount[3]} - FR_lower_o) annotation(
    Placement(transformation(origin = {110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Link FR_pushrod(inboard = {FL_LCA_mount[1], -FL_LCA_mount[2], FL_LCA_mount[3]}, joint_diameter = 0.825*0.0254, link_diameter = 0.625*0.0254, outboard = {FL_bellcrank_pickup_2[1], -FL_bellcrank_pickup_2[2], FL_bellcrank_pickup_2[3]}) annotation(
    Placement(transformation(origin = {90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  
  // FL shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_shock_pickup(r = FL_shock_mount - effective_center)  annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Linkages.ShockLinkage FL_shock(spring_diameter = 0.040, start_point = FL_bellcrank_pickup_3, end_point = FL_shock_mount, rod_length_fraction = 0.5, free_length = norm(FL_shock_mount - FL_bellcrank_pickup_3)/2, spring_table = [0, 0; 1, 80000], spring_mass = 0, damper_table = [0, 0; 0.25, 100], damper_mass = 0, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));

  // FR shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_shock_pickup(r = {FL_shock_mount[1], -FL_shock_mount[2], FL_shock_mount[3]} - effective_center)  annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  Linkages.ShockLinkage FR_shock(spring_diameter = 0.040, start_point = {FL_bellcrank_pickup_3[1], -FL_bellcrank_pickup_3[2], FL_bellcrank_pickup_3[3]}, end_point = {FL_shock_mount[1], -FL_shock_mount[2], FL_shock_mount[3]}, rod_length_fraction = 0.5, free_length = norm(FL_shock_mount - FL_bellcrank_pickup_3)/2, spring_table = [0, 0; 1, 80000], spring_mass = 0, damper_table = [0, 0; 0.25, 100], damper_mass = 0, link_diameter = 0.625*0.0254, joint_diameter = 0.825*0.0254)  annotation(
    Placement(transformation(origin = {50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    
equation
  connect(FL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(FL_bellcrank_mount.frame_b, FL_bellcrank.mount_frame) annotation(
    Line(points = {{-40, 40}, {-50, 40}}, color = {95, 95, 95}));
  connect(FL_rod_out.frame_b, FL_pushrod.frame_a) annotation(
    Line(points = {{-110, 0}, {-110, 40}, {-100, 40}}, color = {95, 95, 95}));
  connect(FL_pushrod.frame_b, FL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-80, 40}, {-70, 40}}, color = {95, 95, 95}));
  connect(FR_rod_out.frame_b, FR_pushrod.frame_a) annotation(
    Line(points = {{110, 0}, {110, 40}, {100, 40}}, color = {95, 95, 95}));
  connect(FR_pushrod.frame_b, FR_bellcrank.pickup_2_frame) annotation(
    Line(points = {{80, 40}, {70, 40}}, color = {95, 95, 95}));
  connect(FR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_bellcrank_mount.frame_b, FR_bellcrank.mount_frame) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {95, 95, 95}));
  connect(FL_rod_out.frame_a, FL_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{-110, -20}, {-110, -70}, {-100, -70}}, color = {95, 95, 95}));
  connect(FR_rod_out.frame_a, FR_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{110, -20}, {110, -70}, {100, -70}}, color = {95, 95, 95}));
  connect(FL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(shockLinkage.frame_a, FL_bellcrank.pickup_3_frame) annotation(
    Line(points = {{-60, 70}, {-60, 50}}, color = {95, 95, 95}));
  connect(shockLinkage.frame_b, FL_shock_pickup.frame_b) annotation(
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(FL_shock.frame_a, FL_bellcrank.pickup_3_frame) annotation(
    Line(points = {{-60, 70}, {-60, 50}}, color = {95, 95, 95}));
  connect(FL_shock.frame_b, FL_shock_pickup.frame_b) annotation(
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(FR_shock.frame_a, FR_bellcrank.pickup_3_frame) annotation(
    Line(points = {{60, 70}, {60, 50}}, color = {95, 95, 95}));
  connect(FR_shock.frame_b, FR_shock_pickup.frame_b) annotation(
    Line(points = {{40, 70}, {30, 70}}, color = {95, 95, 95}));
end FrAxleBellcrank;

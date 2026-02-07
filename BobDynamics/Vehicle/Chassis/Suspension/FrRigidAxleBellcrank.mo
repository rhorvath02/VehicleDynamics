within BobDynamics.Vehicle.Chassis.Suspension;

model FrRigidAxleBellcrank
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  extends BobDynamics.Vehicle.Chassis.Suspension.Templates.FrRigidAxleBase(FR_double_wishbone(side = "Right"));
  
  final parameter BobDynamics.Resources.Records.SUS.FrAxleBellcrank FLBC;
  
  parameter SIunits.Position FL_bellcrank_pivot[3] = FLBC.bellcrank_pivot annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pivot_ref[3] = FLBC.bellcrank_pivot_ref annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_1[3] = FLBC.bellcrank_pickup_1 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_2[3] = FLBC.bellcrank_pickup_2 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_3[3] = FLBC.bellcrank_pickup_3 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_LCA_mount[3] = FLBC.rod_mount annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_shock_mount[3] = FLBC.shock_mount annotation(
    Dialog(group = "Geometry"));
  // FL bellcrank
  // FL outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_rod_out(r = FL_LCA_mount - FL_lower_o)  annotation(
    Placement(transformation(origin = {-110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  // FR bellcrank
  // FR outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_rod_out(r = {FL_LCA_mount[1], -FL_LCA_mount[2], FL_LCA_mount[3]} - FR_lower_o) annotation(
    Placement(transformation(origin = {110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  // FL shock
  // FR shock
  // Steering interface
  Modelica.Blocks.Interfaces.RealInput steer_input annotation(
    Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation=-90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_bellcrank_mount(r = FL_bellcrank_pivot - effective_center) annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FL_bellcrank(pickup_1 = FL_bellcrank_pickup_1, pickup_2 = FL_bellcrank_pickup_2, pickup_3 = FL_bellcrank_pickup_3, pivot = FL_bellcrank_pivot, pivot_ref = FL_bellcrank_pivot_ref) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_bellcrank_mount(r = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_shock_pickup(r = FL_shock_mount - effective_center) annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_shock_pickup(r = {FL_shock_mount[1], -FL_shock_mount[2], FL_shock_mount[3]} - effective_center) annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical FL_pushrod_kin(rodLength = norm(FL_bellcrank_pickup_2 - FL_LCA_mount), sphereDiameter = 0.030, rodDiameter = 0.020)  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FR_bellcrank(pickup_1 = {FL_bellcrank_pickup_1[1], -FL_bellcrank_pickup_1[2], FL_bellcrank_pickup_1[3]}, pickup_2 = {FL_bellcrank_pickup_2[1], -FL_bellcrank_pickup_2[2], FL_bellcrank_pickup_2[3]}, pickup_3 = {FL_bellcrank_pickup_3[1], -FL_bellcrank_pickup_3[2], FL_bellcrank_pickup_3[3]}, pivot = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]}, pivot_ref = {FL_bellcrank_pivot_ref[1], -FL_bellcrank_pivot_ref[2], FL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical FL_pushrod_kin1( rodLength = norm(FL_bellcrank_pickup_2 - FL_LCA_mount), sphereDiameter = 0.030, rodDiameter = 0.020) annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Linkages.TabularSpring tabularSpring(spring_table = [0, 0; 1, 70000], free_length = 7.5*0.0254, spring_diameter = 0.050)  annotation(
    Placement(transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.TabularSpring tabularSpring1(spring_table = [0, 0; 1, 70000], free_length = 7.5*0.0254, spring_diameter = 0.050) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Linkages.TabularDamper tabularDamper(free_length = 7.5*0.0254, damper_table = [0, 0; 1, 1e3])  annotation(
    Placement(transformation(origin = {-50, 130}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.TabularDamper tabularDamper1(damper_table = [0, 0; 1, 1e3], free_length = 7.5*0.0254) annotation(
    Placement(transformation(origin = {50, 130}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
equation
  connect(steer_input, FL_double_wishbone.steer_input) annotation(
    Line(points = {{0, 120}, {0, 88}, {-90, 88}, {-90, -14}}, color = {0, 0, 127}));
  connect(steer_input, FR_double_wishbone.steer_input) annotation(
    Line(points = {{0, 120}, {0, 88}, {90, 88}, {90, -14}}, color = {0, 0, 127}));
  connect(FL_rod_out.frame_a, FL_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{-110, -20}, {-110, -90}, {-70, -90}, {-70, -80}}, color = {95, 95, 95}));
  connect(FR_rod_out.frame_a, FR_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{110, -20}, {110, -92}, {70, -92}, {70, -80}}, color = {95, 95, 95}));
  connect(FL_bellcrank_mount.frame_b, FL_bellcrank.mount_frame) annotation(
    Line(points = {{-40, 40}, {-50, 40}}, color = {95, 95, 95}));
  connect(FL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(FL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(FL_pushrod_kin.frame_a, FL_rod_out.frame_b) annotation(
    Line(points = {{-100, 40}, {-110, 40}, {-110, 0}}, color = {95, 95, 95}));
  connect(FL_pushrod_kin.frame_b, FL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-80, 40}, {-70, 40}}, color = {95, 95, 95}));
  connect(FR_bellcrank_mount.frame_b, FR_bellcrank.mount_frame) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {95, 95, 95}));
  connect(FR_bellcrank.pickup_2_frame, FL_pushrod_kin1.frame_b) annotation(
    Line(points = {{70, 40}, {80, 40}}, color = {95, 95, 95}));
  connect(FL_pushrod_kin1.frame_a, FR_rod_out.frame_b) annotation(
    Line(points = {{100, 40}, {110, 40}, {110, 0}}, color = {95, 95, 95}));
  connect(FR_shock_pickup.frame_b, tabularSpring.frame_a) annotation(
    Line(points = {{30, 70}, {40, 70}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_b, FR_bellcrank.pickup_3_frame) annotation(
    Line(points = {{60, 70}, {60, 50}}, color = {95, 95, 95}));
  connect(FL_shock_pickup.frame_b, tabularSpring1.frame_a) annotation(
    Line(points = {{-30, 70}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularSpring1.frame_b, FL_bellcrank.pickup_3_frame) annotation(
    Line(points = {{-60, 70}, {-60, 50}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_a, tabularSpring1.frame_a) annotation(
    Line(points = {{-40, 130}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, tabularSpring1.frame_b) annotation(
    Line(points = {{-60, 130}, {-60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_b, tabularSpring.frame_b) annotation(
    Line(points = {{60, 130}, {60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_a, tabularSpring.frame_a) annotation(
    Line(points = {{40, 130}, {40, 70}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end FrRigidAxleBellcrank;

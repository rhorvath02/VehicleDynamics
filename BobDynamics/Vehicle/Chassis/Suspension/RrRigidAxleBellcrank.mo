within BobDynamics.Vehicle.Chassis.Suspension;

model RrRigidAxleBellcrank
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  extends BobDynamics.Vehicle.Chassis.Suspension.Templates.RrRigidAxleBase(RR_double_wishbone(side = "Right"));
  
  final parameter BobDynamics.Resources.Records.SUS.RrAxleBellcrank RLBC;
  
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
  // FL outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_rod_out(r = RL_UCA_mount - RL_upper_o)  annotation(
    Placement(transformation(origin = {-110, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  // FR bellcrank
  // FR outboard rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_rod_out(r = {RL_UCA_mount[1], -RL_UCA_mount[2], RL_UCA_mount[3]} - RR_upper_o) annotation(
    Placement(transformation(origin = {110, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  // FL shock
  // FR shock
  // Steering interface
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_bellcrank_mount(r = RL_bellcrank_pivot - effective_center) annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RL_bellcrank(pickup_1 = RL_bellcrank_pickup_1, pickup_2 = RL_bellcrank_pickup_2, pickup_3 = RL_bellcrank_pickup_3, pivot = RL_bellcrank_pivot, pivot_ref = RL_bellcrank_pivot_ref) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_bellcrank_mount(r = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_shock_pickup(r = RL_shock_mount - effective_center) annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_shock_pickup(r = {RL_shock_mount[1], -RL_shock_mount[2], RL_shock_mount[3]} - effective_center) annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical RL_pushrod_kin(rodLength = norm(RL_bellcrank_pickup_2 - RL_UCA_mount), sphereDiameter = 0.030, rodDiameter = 0.020)  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RR_bellcrank(pickup_1 = {RL_bellcrank_pickup_1[1], -RL_bellcrank_pickup_1[2], RL_bellcrank_pickup_1[3]}, pickup_2 = {RL_bellcrank_pickup_2[1], -RL_bellcrank_pickup_2[2], RL_bellcrank_pickup_2[3]}, pickup_3 = {RL_bellcrank_pickup_3[1], -RL_bellcrank_pickup_3[2], RL_bellcrank_pickup_3[3]}, pivot = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]}, pivot_ref = {RL_bellcrank_pivot_ref[1], -RL_bellcrank_pivot_ref[2], RL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical RL_pushrod_kin1( rodLength = norm(RL_bellcrank_pickup_2 - RL_UCA_mount), sphereDiameter = 0.030, rodDiameter = 0.020) annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Linkages.TabularSpring tabularSpring(spring_table = [0, 0; 1, 70000], free_length = 10.5*0.0254, spring_diameter = 0.050)  annotation(
    Placement(transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.TabularSpring tabularSpring1(spring_table = [0, 0; 1, 70000], free_length = 10.5*0.0254, spring_diameter = 0.050) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Linkages.TabularDamper tabularDamper(free_length = 10.5*0.0254, damper_table = [0, 0; 1, 1e3])  annotation(
    Placement(transformation(origin = {-50, 130}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.TabularDamper tabularDamper1(damper_table = [0, 0; 1, 1e3], free_length = 10.5*0.0254) annotation(
    Placement(transformation(origin = {50, 130}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
equation
  connect(RL_bellcrank_mount.frame_b, RL_bellcrank.mount_frame) annotation(
    Line(points = {{-40, 40}, {-50, 40}}, color = {95, 95, 95}));
  connect(RL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_pushrod_kin.frame_a, RL_rod_out.frame_b) annotation(
    Line(points = {{-100, 40}, {-110, 40}, {-110, 20}}, color = {95, 95, 95}));
  connect(RL_pushrod_kin.frame_b, RL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-80, 40}, {-70, 40}}, color = {95, 95, 95}));
  connect(RR_bellcrank_mount.frame_b, RR_bellcrank.mount_frame) annotation(
    Line(points = {{40, 40}, {50, 40}}, color = {95, 95, 95}));
  connect(RR_bellcrank.pickup_2_frame, RL_pushrod_kin1.frame_b) annotation(
    Line(points = {{70, 40}, {80, 40}}, color = {95, 95, 95}));
  connect(RL_pushrod_kin1.frame_a, RR_rod_out.frame_b) annotation(
    Line(points = {{100, 40}, {110, 40}, {110, 20}}, color = {95, 95, 95}));
  connect(RR_shock_pickup.frame_b, tabularSpring.frame_a) annotation(
    Line(points = {{30, 70}, {40, 70}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_b, RR_bellcrank.pickup_3_frame) annotation(
    Line(points = {{60, 70}, {60, 50}}, color = {95, 95, 95}));
  connect(RL_shock_pickup.frame_b, tabularSpring1.frame_a) annotation(
    Line(points = {{-30, 70}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularSpring1.frame_b, RL_bellcrank.pickup_3_frame) annotation(
    Line(points = {{-60, 70}, {-60, 50}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_a, tabularSpring1.frame_a) annotation(
    Line(points = {{-40, 130}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, tabularSpring1.frame_b) annotation(
    Line(points = {{-60, 130}, {-60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_b, tabularSpring.frame_b) annotation(
    Line(points = {{60, 130}, {60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_a, tabularSpring.frame_a) annotation(
    Line(points = {{40, 130}, {40, 70}}, color = {95, 95, 95}));
  connect(RR_rod_out.frame_a, RR_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{110, 0}, {110, -10}, {70, -10}, {70, -20}}, color = {95, 95, 95}));
  connect(RL_rod_out.frame_a, RL_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{-110, 0}, {-110, -10}, {-70, -10}, {-70, -20}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end RrRigidAxleBellcrank;

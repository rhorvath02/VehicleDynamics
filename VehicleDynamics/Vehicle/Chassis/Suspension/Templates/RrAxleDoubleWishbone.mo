within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;

model RrAxleDoubleWishbone
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  final parameter VehicleDynamics.Resources.Records.SUS.RrAxleBase RrAxle;
  final parameter VehicleDynamics.Resources.Records.SUS.RrAxleBellcrank RrAxleBC;
  
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.RrUnsprung unsprung_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.RrUCA uca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.RrLCA lca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.RrTie tie_mass;
  
  extends VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone.AxleDoubleWishboneBase(left_upper_fore_i = RrAxle.upper_fore_i,
                                                                                                     left_upper_aft_i = RrAxle.upper_aft_i,
                                                                                                     left_lower_fore_i = RrAxle.lower_fore_i,
                                                                                                     left_lower_aft_i = RrAxle.lower_aft_i,
                                                                                                     left_upper_o = RrAxle.upper_outboard,
                                                                                                     left_lower_o = RrAxle.lower_outboard,
                                                                                                     left_tie_i = RrAxle.tie_inboard,
                                                                                                     left_tie_o = RrAxle.tie_outboard,
                                                                                                     left_wheel_center = RrAxle.wheel_center,
                                                                                                     left_static_gamma = RrAxle.static_gamma,
                                                                                                     left_static_alpha = RrAxle.static_alpha,
                                                                                                     left_unsprung_mass = unsprung_mass,
                                                                                                     left_uca_mass = uca_mass,
                                                                                                     left_lca_mass = lca_mass,
                                                                                                     left_tie_mass = tie_mass);
  
  parameter SIunits.Position RL_bellcrank_pivot[3] = RrAxleBC.bellcrank_pivot annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pivot_ref[3] = RrAxleBC.bellcrank_pivot_ref annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_1[3] = RrAxleBC.bellcrank_pickup_1 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_2[3] = RrAxleBC.bellcrank_pickup_2 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_bellcrank_pickup_3[3] = RrAxleBC.bellcrank_pickup_3 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_UCA_mount[3] = RrAxleBC.rod_mount annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_shock_mount[3] = RrAxleBC.shock_mount annotation(
    Dialog(group = "Geometry"));
  
  // RL apex geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_apex(r = RL_UCA_mount - left_upper_o) annotation(
    Placement(transformation(origin = {-110, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // RL pushrod
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical RL_pushrod(rodLength = norm(RL_bellcrank_pickup_1 - RL_UCA_mount),
                                                                    sphereDiameter = joint_diameter,
                                                                    rodDiameter = link_diameter) annotation(
    Placement(transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}})));
  
  // RL bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_bellcrank_mount(r = RL_bellcrank_pivot - effective_center) annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RL_bellcrank(pickup_1 = RL_bellcrank_pickup_1,
                                                                                  pickup_2 = RL_bellcrank_pickup_2,
                                                                                  pickup_3 = RL_bellcrank_pickup_3,
                                                                                  pivot = RL_bellcrank_pivot,
                                                                                  pivot_ref = RL_bellcrank_pivot_ref) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
  
  // RL shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_shock_pickup(r = RL_shock_mount - effective_center) annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Linkages.TabularSpring tabularSpring(spring_table = [0, 0; 1, 70000],
                                       free_length = 10*0.0254,
                                       spring_diameter = 0.050) annotation(
    Placement(transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}})));
  Linkages.TabularDamper tabularDamper(free_length = 10*0.0254,
                                       damper_table = [0, 0; 1, 1e3]) annotation(
    Placement(transformation(origin = {-50, 130}, extent = {{10, -10}, {-10, 10}})));
  
  // RR apex geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_apex(r = {RL_UCA_mount[1], -RL_UCA_mount[2], RL_UCA_mount[3]} - right_upper_o) annotation(
    Placement(transformation(origin = {110, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // RR pushrod
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical RR_pushrod(rodLength = norm(RL_bellcrank_pickup_1 - RL_UCA_mount),
                                                                    sphereDiameter = joint_diameter,
                                                                    rodDiameter = link_diameter) annotation(
    Placement(transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}})));
  
  // RR bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_bellcrank_mount(r = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p RR_bellcrank(pickup_1 = {RL_bellcrank_pickup_1[1], -RL_bellcrank_pickup_1[2], RL_bellcrank_pickup_1[3]},
                                                                                  pickup_2 = {RL_bellcrank_pickup_2[1], -RL_bellcrank_pickup_2[2], RL_bellcrank_pickup_2[3]},
                                                                                  pickup_3 = {RL_bellcrank_pickup_3[1], -RL_bellcrank_pickup_3[2], RL_bellcrank_pickup_3[3]},
                                                                                  pivot = {RL_bellcrank_pivot[1], -RL_bellcrank_pivot[2], RL_bellcrank_pivot[3]},
                                                                                  pivot_ref = {RL_bellcrank_pivot_ref[1], -RL_bellcrank_pivot_ref[2], RL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // RR shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_shock_pickup(r = {RL_shock_mount[1], -RL_shock_mount[2], RL_shock_mount[3]} - effective_center) annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularSpring tabularSpring1(spring_table = [0, 0; 1, 70000],
                                                                                   free_length = 10*0.0254,
                                                                                   spring_diameter = 0.050) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularDamper tabularDamper1(damper_table = [0, 0; 1, 1e3],
                                                                                   free_length = 10*0.0254) annotation(
    Placement(transformation(origin = {50, 130}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Zero steer (for typical rear axle)
  Modelica.Blocks.Sources.RealExpression zero_steer annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(RL_apex.frame_b, RL_pushrod.frame_a) annotation(
    Line(points = {{-110, 20}, {-110, 30}, {-100, 30}}, color = {95, 95, 95}));
  connect(tabularSpring1.frame_a, RL_shock_pickup.frame_b) annotation(
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(RL_bellcrank.mount_frame, RL_bellcrank_mount.frame_b) annotation(
    Line(points = {{-50, 40}, {-40, 40}}, color = {95, 95, 95}));
  connect(RL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_apex.frame_b, RR_pushrod.frame_a) annotation(
    Line(points = {{110, 20}, {110, 30}, {100, 30}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_a, RR_shock_pickup.frame_b) annotation(
    Line(points = {{40, 70}, {30, 70}}, color = {95, 95, 95}));
  connect(RR_bellcrank.mount_frame, RR_bellcrank_mount.frame_b) annotation(
    Line(points = {{50, 40}, {40, 40}}, color = {95, 95, 95}));
  connect(RR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, tabularSpring1.frame_b) annotation(
    Line(points = {{-60, 130}, {-60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_a, tabularSpring1.frame_a) annotation(
    Line(points = {{-40, 130}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_a, tabularSpring.frame_a) annotation(
    Line(points = {{40, 130}, {40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_b, tabularSpring.frame_b) annotation(
    Line(points = {{60, 130}, {60, 70}}, color = {95, 95, 95}));
  connect(zero_steer.y, left_double_wishbone.steer_input) annotation(
    Line(points = {{-90, 59}, {-90, -14}}, color = {0, 0, 127}));
  connect(zero_steer.y, right_double_wishbone.steer_input) annotation(
    Line(points = {{-90, 59}, {-90, 20}, {90, 20}, {90, -14}}, color = {0, 0, 127}));
  connect(RL_pushrod.frame_b, RL_bellcrank.pickup_1_frame) annotation(
    Line(points = {{-80, 30}, {-60, 30}}, color = {95, 95, 95}));
  connect(tabularSpring1.frame_b, RL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-60, 70}, {-70, 70}, {-70, 40}}, color = {95, 95, 95}));
  connect(RR_pushrod.frame_b, RR_bellcrank.pickup_1_frame) annotation(
    Line(points = {{80, 30}, {60, 30}}, color = {95, 95, 95}));
  connect(RL_apex.frame_a, left_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{-110, 0}, {-70, 0}, {-70, -20}}, color = {95, 95, 95}));
  connect(RR_apex.frame_a, right_double_wishbone.upper_wishbone_frame) annotation(
    Line(points = {{110, 0}, {70, 0}, {70, -20}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_b, RR_bellcrank.pickup_2_frame) annotation(
    Line(points = {{60, 70}, {70, 70}, {70, 40}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end RrAxleDoubleWishbone;

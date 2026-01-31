within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;

model FrAxleDoubleWishbone
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  final parameter VehicleDynamics.Resources.Records.SUS.FrAxleBase FrAxle;
  final parameter VehicleDynamics.Resources.Records.SUS.FrAxleBellcrank FrAxleBC;
  
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUnsprung unsprung_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUCA uca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrLCA lca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrTie tie_mass;
  
  extends VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone.AxleDoubleWishboneBase(left_upper_fore_i = FrAxle.upper_fore_i,
                                                                                                     left_upper_aft_i = FrAxle.upper_aft_i,
                                                                                                     left_lower_fore_i = FrAxle.lower_fore_i,
                                                                                                     left_lower_aft_i = FrAxle.lower_aft_i,
                                                                                                     left_upper_o = FrAxle.upper_outboard,
                                                                                                     left_lower_o = FrAxle.lower_outboard,
                                                                                                     left_tie_i = FrAxle.tie_inboard,
                                                                                                     left_tie_o = FrAxle.tie_outboard,
                                                                                                     left_wheel_center = FrAxle.wheel_center,
                                                                                                     left_static_gamma = FrAxle.static_gamma,
                                                                                                     left_static_alpha = FrAxle.static_alpha,
                                                                                                     left_unsprung_mass = unsprung_mass,
                                                                                                     left_uca_mass = uca_mass,
                                                                                                     left_lca_mass = lca_mass,
                                                                                                     left_tie_mass = tie_mass);
  
  parameter SIunits.Position FL_bellcrank_pivot[3] = FrAxleBC.bellcrank_pivot annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pivot_ref[3] = FrAxleBC.bellcrank_pivot_ref annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_1[3] = FrAxleBC.bellcrank_pickup_1 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_2[3] = FrAxleBC.bellcrank_pickup_2 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_bellcrank_pickup_3[3] = FrAxleBC.bellcrank_pickup_3 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_LCA_mount[3] = FrAxleBC.rod_mount annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_shock_mount[3] = FrAxleBC.shock_mount annotation(
    Dialog(group = "Geometry"));
  
  // FL apex geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_apex(r = FL_LCA_mount - left_lower_o) annotation(
    Placement(transformation(origin = {-110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // FL pushrod
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical FL_pushrod(rodLength = norm(FL_bellcrank_pickup_2 - FL_LCA_mount),
                                                                    sphereDiameter = joint_diameter,
                                                                    rodDiameter = link_diameter) annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // FL bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_bellcrank_mount(r = FL_bellcrank_pivot - effective_center) annotation(
    Placement(transformation(origin = {-30, 40}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FL_bellcrank(pickup_1 = FL_bellcrank_pickup_1,
                                                                                  pickup_2 = FL_bellcrank_pickup_2,
                                                                                  pickup_3 = FL_bellcrank_pickup_3,
                                                                                  pivot = FL_bellcrank_pivot,
                                                                                  pivot_ref = FL_bellcrank_pivot_ref) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{10, -10}, {-10, 10}})));
    
  // FL shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_shock_pickup(r = FL_shock_mount - effective_center) annotation(
    Placement(transformation(origin = {-20, 70}, extent = {{10, -10}, {-10, 10}})));
  Linkages.TabularSpring tabularSpring(spring_table = [0, 0; 1, 70000],
                                       free_length = 7.5*0.0254,
                                       spring_diameter = 0.050)  annotation(
    Placement(transformation(origin = {50, 70}, extent = {{-10, -10}, {10, 10}})));
  Linkages.TabularDamper tabularDamper(free_length = 7.5*0.0254,
                                       damper_table = [0, 0; 1, 1e3])  annotation(
    Placement(transformation(origin = {-50, 130}, extent = {{10, -10}, {-10, 10}})));
  
  // FR apex geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_apex(r = {FL_LCA_mount[1], -FL_LCA_mount[2], FL_LCA_mount[3]} - right_lower_o) annotation(
    Placement(transformation(origin = {110, -10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));

  // FR pushrod
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical FR_pushrod(rodLength = norm(FL_bellcrank_pickup_2 - FL_LCA_mount),
                                                                    sphereDiameter = 0.030,
                                                                    rodDiameter = 0.020) annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  // FR bellcrank
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_bellcrank_mount(r = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]} - effective_center) annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Bellcrank3pu1p FR_bellcrank(pickup_1 = {FL_bellcrank_pickup_1[1], -FL_bellcrank_pickup_1[2], FL_bellcrank_pickup_1[3]},
                                                                                  pickup_2 = {FL_bellcrank_pickup_2[1], -FL_bellcrank_pickup_2[2], FL_bellcrank_pickup_2[3]},
                                                                                  pickup_3 = {FL_bellcrank_pickup_3[1], -FL_bellcrank_pickup_3[2], FL_bellcrank_pickup_3[3]},
                                                                                  pivot = {FL_bellcrank_pivot[1], -FL_bellcrank_pivot[2], FL_bellcrank_pivot[3]},
                                                                                  pivot_ref = {FL_bellcrank_pivot_ref[1], -FL_bellcrank_pivot_ref[2], FL_bellcrank_pivot_ref[3]}) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // FR shock
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_shock_pickup(r = {FL_shock_mount[1], -FL_shock_mount[2], FL_shock_mount[3]} - effective_center) annotation(
    Placement(transformation(origin = {20, 70}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularSpring tabularSpring1(spring_table = [0, 0; 1, 70000],
                                                                                   free_length = 7.5*0.0254,
                                                                                   spring_diameter = 0.050) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularDamper tabularDamper1(damper_table = [0, 0; 1, 1e3],
                                                                                   free_length = 7.5*0.0254) annotation(
    Placement(transformation(origin = {50, 130}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Steering interface
  Modelica.Blocks.Interfaces.RealInput steer_input annotation(
    Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation=-90)));
  
equation
  connect(FL_apex.frame_a, left_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{-110, -20}, {-110, -90}, {-70, -90}, {-70, -80}}, color = {95, 95, 95}));
  connect(FL_apex.frame_b, FL_pushrod.frame_a) annotation(
    Line(points = {{-110, 0}, {-110, 40}, {-100, 40}}, color = {95, 95, 95}));
  connect(FL_pushrod.frame_b, FL_bellcrank.pickup_2_frame) annotation(
    Line(points = {{-80, 40}, {-70, 40}}, color = {95, 95, 95}));
  connect(FL_bellcrank.pickup_3_frame, tabularSpring1.frame_b) annotation(
    Line(points = {{-60, 50}, {-60, 70}}, color = {95, 95, 95}));
  connect(tabularSpring1.frame_a, FL_shock_pickup.frame_b) annotation(
    Line(points = {{-40, 70}, {-30, 70}}, color = {95, 95, 95}));
  connect(FL_bellcrank.mount_frame, FL_bellcrank_mount.frame_b) annotation(
    Line(points = {{-50, 40}, {-40, 40}}, color = {95, 95, 95}));
  connect(FL_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{-10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(FL_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{-20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_apex.frame_a, right_double_wishbone.lower_wishbone_frame) annotation(
    Line(points = {{110, -20}, {110, -90}, {70, -90}, {70, -80}}, color = {95, 95, 95}));
  connect(FR_apex.frame_b, FR_pushrod.frame_a) annotation(
    Line(points = {{110, 0}, {110, 40}, {100, 40}}, color = {95, 95, 95}));
  connect(FR_pushrod.frame_b, FR_bellcrank.pickup_2_frame) annotation(
    Line(points = {{80, 40}, {70, 40}}, color = {95, 95, 95}));
  connect(FR_bellcrank.pickup_3_frame, tabularSpring.frame_b) annotation(
    Line(points = {{60, 50}, {60, 70}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_a, FR_shock_pickup.frame_b) annotation(
    Line(points = {{40, 70}, {30, 70}}, color = {95, 95, 95}));
  connect(FR_bellcrank.mount_frame, FR_bellcrank_mount.frame_b) annotation(
    Line(points = {{50, 40}, {40, 40}}, color = {95, 95, 95}));
  connect(FR_shock_pickup.frame_a, axle_frame) annotation(
    Line(points = {{10, 70}, {0, 70}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_bellcrank_mount.frame_a, axle_frame) annotation(
    Line(points = {{20, 40}, {0, 40}, {0, -100}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, tabularSpring1.frame_b) annotation(
    Line(points = {{-60, 130}, {-60, 70}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_a, tabularSpring1.frame_a) annotation(
    Line(points = {{-40, 130}, {-40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_a, tabularSpring.frame_a) annotation(
    Line(points = {{40, 130}, {40, 70}}, color = {95, 95, 95}));
  connect(tabularDamper1.frame_b, tabularSpring.frame_b) annotation(
    Line(points = {{60, 130}, {60, 70}}, color = {95, 95, 95}));
  connect(left_double_wishbone.steer_input, steer_input) annotation(
    Line(points = {{-90, -14}, {-90, 90}, {0, 90}, {0, 120}}, color = {0, 0, 127}));
  connect(right_double_wishbone.steer_input, steer_input) annotation(
    Line(points = {{90, -14}, {90, 90}, {0, 90}, {0, 120}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end FrAxleDoubleWishbone;

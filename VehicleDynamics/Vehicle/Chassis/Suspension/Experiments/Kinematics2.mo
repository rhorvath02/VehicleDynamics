within VehicleDynamics.Vehicle.Chassis.Suspension.Experiments;
model Kinematics2
  import Modelica.SIunits;
  parameter SIunits.Position contact_patch[3] = {0, 0.609600, 0} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_fore_i[3] = {112.268, 225.425, 214.3222}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_aft_i[3] = {-112.268, 223.0393, 215.7854}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_o[3] = {-19.9572, 530.7122, 294.64}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_i[3] = {57.15, 210.074, 97.6}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_o[3] = {55.372, 579.12, 157.2416}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_fore_i[3] = {112.776, 225.425, 80.01}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_aft_i[3] = {-112.776, 225.425, 80.01}/1000 annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_o[3] = {14.024, 554.1225, 126}/1000 annotation(
    Dialog(group = "Geometry"));
  
  DoubleWishboneBase doubleWishboneBase(upper_fore_i = upper_fore_i, upper_aft_i = upper_aft_i, lower_fore_i = lower_fore_i, lower_aft_i = lower_aft_i, upper_o = upper_o, lower_o = lower_o, tie_i = tie_i, tie_o = tie_o, contact_patch = contact_patch) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-110, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = upper_fore_i, animation = false) annotation(
    Placement(transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = upper_aft_i, animation = false) annotation(
    Placement(transformation(origin = {70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(r = lower_fore_i, animation = false) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed3(r = lower_aft_i, animation = false) annotation(
    Placement(transformation(origin = {70, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed4(r = tie_i, animation = false) annotation(
    Placement(transformation(origin = {70, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Vehicle.Chassis.Suspension.Tires.MF5p2Base MF5p2Base(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = 0.0000000, PHX2 = 0.0000000, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 8*0.0254, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, mu_s = 1, rim_R0 = 5*0.0254, rim_width = 7*0.0254, static_alpha = 0, static_gamma = 0, each v_rel = {0, 0, 0}, wheel_J = 0.2) annotation(
    Placement(transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = true, n = {0, 1, 0}) annotation(
    Placement(transformation(origin = {40, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 0.5*0.0254, duration = 1, startTime = 1) annotation(
    Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed5(r = contact_patch, animation = false) annotation(
    Placement(transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.GroundPhysics groundPhysics annotation(
    Placement(transformation(origin = {-80, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce(animation = false)  annotation(
    Placement(transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed6(r = {upper_fore_i[1], -upper_fore_i[2], upper_fore_i[3]}, animation = false) annotation(
    Placement(transformation(origin = {130, 80}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed11(r = {upper_aft_i[1], -upper_aft_i[2], upper_aft_i[3]}, animation = false) annotation(
    Placement(transformation(origin = {130, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed21(r = {lower_fore_i[1], -lower_fore_i[2], lower_fore_i[3]}, animation = false) annotation(
    Placement(transformation(origin = {130, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed31(r = {lower_aft_i[1], -lower_aft_i[2], lower_aft_i[3]}, animation = false) annotation(
    Placement(transformation(origin = {130, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed41(r = {tie_i[1], -tie_i[2], tie_i[3]}, animation = false) annotation(
    Placement(transformation(origin = {130, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Vehicle.Chassis.Suspension.DoubleWishboneBase doubleWishboneBase1(contact_patch = {contact_patch[1], -contact_patch[2], contact_patch[3]}, lower_aft_i = {lower_aft_i[1], -lower_aft_i[2], lower_aft_i[3]}, lower_fore_i = {lower_fore_i[1], -lower_fore_i[2], lower_fore_i[3]}, lower_o = {lower_o[1], -lower_o[2], lower_o[3]}, tie_i = {tie_i[1], -tie_i[2], tie_i[3]}, tie_o = {tie_o[1], -tie_o[2], tie_o[3]}, upper_aft_i = {upper_aft_i[1], -upper_aft_i[2], upper_aft_i[3]}, upper_fore_i = {upper_fore_i[1], -upper_fore_i[2], upper_fore_i[3]}, upper_o = {upper_o[1], -upper_o[2], upper_o[3]}) annotation(
    Placement(transformation(origin = {200, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cutForce1(animation = false)  annotation(
    Placement(transformation(origin = {180, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed51(r = {contact_patch[1], -contact_patch[2], contact_patch[3]}, animation = false) annotation(
    Placement(transformation(origin = {280, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.GroundPhysics groundPhysics1 annotation(
    Placement(transformation(origin = {280, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Vehicle.Chassis.Suspension.Tires.MF5p2Base MF5p2Base1(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = 0.0000000, PHX2 = 0.0000000, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 8*0.0254, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, mu_s = 1, rim_R0 = 5*0.0254, rim_width = 7*0.0254, static_alpha = 0, static_gamma = 0, each v_rel = {0, 0, 0}, wheel_J = 0.2) annotation(
    Placement(transformation(origin = {230, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic1(n = {0, 1, 0}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {160, -80}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Blocks.Math.Add add(k1 = -1, k2 = +1) annotation(
    Placement(transformation(origin = {100, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Real FL_Fy_aligned;
  Real FR_Fy_aligned;
  Real axle_weight = 1225;
  Real axle_mass = axle_weight/Modelica.Constants.g_n;
  Real CG_height = 10*0.0254;
  Real track_width = contact_patch[2] * 2;
  Real ay;

  Modelica.Blocks.Math.Gain gain(k = 3.5/2*0.0254) annotation(
    Placement(transformation(origin = {100, -120}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput pinion_torque annotation(
    Placement(transformation(origin = {100, -150}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {150, -124}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain gain1(k = (3.5*0.0254/360)^(-1)) annotation(
    Placement(transformation(origin = {-30, -120}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput hwa annotation(
    Placement(transformation(origin = {-30, -150}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-34, -148}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 8, r_CM = {0, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {-30, 59}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 8, r_CM = {0, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {230, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  FL_Fy_aligned = MF5p2Base.Fy*cos(abs(MF5p2Base.alpha));
  FR_Fy_aligned = MF5p2Base1.Fy*cos(abs(MF5p2Base1.alpha));
  ay = (FL_Fy_aligned + FR_Fy_aligned)/axle_mass;
  MF5p2Base.Fz = axle_weight/2 - axle_mass*ay*CG_height/track_width;
  MF5p2Base1.Fz = axle_weight/2 + axle_mass*ay*CG_height/track_width;
  MF5p2Base.wheel_vel = {1, 0, 0};
  MF5p2Base1.wheel_vel = {1, 0, 0};
  connect(fixed.frame_b, doubleWishboneBase.upper_fore_i_frame) annotation(
    Line(points = {{60, 80}, {0, 80}, {0, 10}}, color = {95, 95, 95}));
  connect(fixed1.frame_b, doubleWishboneBase.upper_aft_i_frame) annotation(
    Line(points = {{60, 40}, {6, 40}, {6, 10}}, color = {95, 95, 95}));
  connect(fixed3.frame_b, doubleWishboneBase.lower_aft_i_frame) annotation(
    Line(points = {{60, -40}, {30, -40}, {30, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(fixed2.frame_b, doubleWishboneBase.lower_fore_i_frame) annotation(
    Line(points = {{60, 0}, {40, 0}, {40, 6}, {10, 6}}, color = {95, 95, 95}));
  connect(prismatic.frame_a, fixed4.frame_b) annotation(
    Line(points = {{50, -80}, {60, -80}}, color = {95, 95, 95}));
  connect(ramp.y, position.s_ref) annotation(
    Line(points = {{-39, -60}, {-23, -60}}, color = {0, 0, 127}));
  connect(position.flange, prismatic.axis) annotation(
    Line(points = {{0, -60}, {32, -60}, {32, -74}}, color = {0, 127, 0}));
  connect(doubleWishboneBase.contact_patch_frame, MF5p2Base.frame_a) annotation(
    Line(points = {{0, -10}, {0, -20}, {-30, -20}, {-30, 20}}, color = {95, 95, 95}));
  connect(fixed5.frame_b, groundPhysics.frame_a) annotation(
    Line(points = {{-80, 30}, {-80, 20}}, color = {95, 95, 95}));
  connect(groundPhysics.frame_b, MF5p2Base.frame_a) annotation(
    Line(points = {{-80, 0}, {-80, -10}, {-30, -10}, {-30, 20}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, cutForce.frame_a) annotation(
    Line(points = {{30, -80}, {20, -80}, {20, -50}}, color = {95, 95, 95}));
  connect(cutForce.frame_b, doubleWishboneBase.tie_i_frame) annotation(
    Line(points = {{20, -30}, {20, -6}, {10, -6}}, color = {95, 95, 95}));
  connect(fixed6.frame_b, doubleWishboneBase1.upper_fore_i_frame) annotation(
    Line(points = {{140, 80}, {200, 80}, {200, 10}}, color = {95, 95, 95}));
  connect(fixed11.frame_b, doubleWishboneBase1.upper_aft_i_frame) annotation(
    Line(points = {{140, 40}, {194, 40}, {194, 10}}, color = {95, 95, 95}));
  connect(fixed21.frame_b, doubleWishboneBase1.lower_fore_i_frame) annotation(
    Line(points = {{140, 0}, {160, 0}, {160, 6}, {190, 6}}, color = {95, 95, 95}));
  connect(fixed31.frame_b, doubleWishboneBase1.lower_aft_i_frame) annotation(
    Line(points = {{140, -40}, {170, -40}, {170, 0}, {190, 0}}, color = {95, 95, 95}));
  connect(cutForce1.frame_b, doubleWishboneBase1.tie_i_frame) annotation(
    Line(points = {{180, -30}, {180, -6}, {190, -6}}, color = {95, 95, 95}));
  connect(groundPhysics1.frame_b, MF5p2Base1.frame_a) annotation(
    Line(points = {{280, 0}, {280, -10}, {230, -10}, {230, 20}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.contact_patch_frame, MF5p2Base1.frame_a) annotation(
    Line(points = {{200, -10}, {200, -20}, {230, -20}, {230, 20}}, color = {95, 95, 95}));
  connect(fixed51.frame_b, groundPhysics1.frame_a) annotation(
    Line(points = {{280, 30}, {280, 20}}, color = {95, 95, 95}));
  connect(prismatic1.axis, position.flange) annotation(
    Line(points = {{168, -74}, {168, -60}, {0, -60}}, color = {0, 127, 0}));
  connect(fixed41.frame_b, prismatic1.frame_a) annotation(
    Line(points = {{140, -80}, {150, -80}}, color = {95, 95, 95}));
  connect(prismatic1.frame_b, cutForce1.frame_a) annotation(
    Line(points = {{170, -80}, {180, -80}, {180, -50}}, color = {95, 95, 95}));
  connect(cutForce.force[2], add.u2) annotation(
    Line(points = {{32, -48}, {94, -48}, {94, -78}}, color = {0, 0, 127}));
  connect(cutForce1.force[2], add.u1) annotation(
    Line(points = {{170, -48}, {106, -48}, {106, -78}}, color = {0, 0, 127}));
  connect(gain.u, add.y) annotation(
    Line(points = {{100, -108}, {100, -100}}, color = {0, 0, 127}));
  connect(gain.y, pinion_torque) annotation(
    Line(points = {{100, -130}, {100, -150}}, color = {0, 0, 127}));
  connect(ramp.y, gain1.u) annotation(
    Line(points = {{-38, -60}, {-30, -60}, {-30, -108}}, color = {0, 0, 127}));
  connect(gain1.y, hwa) annotation(
    Line(points = {{-30, -130}, {-30, -150}}, color = {0, 0, 127}));
  connect(body.frame_a, MF5p2Base.frame_a) annotation(
    Line(points = {{-30, 50}, {-30, 20}}, color = {95, 95, 95}));
  connect(body1.frame_a, MF5p2Base1.frame_a) annotation(
    Line(points = {{230, 50}, {230, 20}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 2),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,evaluateAllParameters,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
    Diagram(coordinateSystem(extent = {{-140, 100}, {300, -140}})));
end Kinematics2;

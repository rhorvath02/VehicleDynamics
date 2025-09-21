model Test_Frame
import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  parameter SIunits.Position CG_location[3] = {-0.7747, 0, 0.254} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_contact_patch[3] = {0, 0.609600, 0} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_contact_patch[3] = {FL_contact_patch[1], -FL_contact_patch[2], FL_contact_patch[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_contact_patch[3] = {-1.5494, 0.609600, 0} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_contact_patch[3] = {RL_contact_patch[1], -RL_contact_patch[2], RL_contact_patch[3]} annotation(Dialog(group="Geometry"));
  // FL node parameters
  parameter SIunits.Position FL_upper_fore_i[3] = {0.086868, 0.215900, 0.200000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_upper_o[3] = {-0.006347, 0.523240, 0.287020} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_tie_i[3] = {0.041128, 0.215900, 0.117856} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_tie_o[3] = {0.056000, 0.532333, 0.164821} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_fore_i[3] = {0.087376, 0.215900, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_o[3] = {0, 0.556499, 0.124998} annotation(Dialog(group="Geometry"));
  // FR node parameters
  parameter SIunits.Position FR_upper_fore_i[3] = {FL_upper_fore_i[1], -FL_upper_fore_i[2], FL_upper_fore_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_upper_aft_i[3] = {FL_upper_aft_i[1], -FL_upper_aft_i[2], FL_upper_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_upper_o[3] = {FL_upper_o[1], -FL_upper_o[2], FL_upper_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_tie_i[3] = {FL_tie_i[1], -FL_tie_i[2], FL_tie_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_tie_o[3] = {FL_tie_o[1], -FL_tie_o[2], FL_tie_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_lower_fore_i[3] = {FL_lower_fore_i[1], -FL_lower_fore_i[2], FL_lower_fore_i[3]} annotation(Dialog(group="Geometry"));
parameter SIunits.Position FR_lower_aft_i[3] = {FL_lower_aft_i[1], -FL_lower_aft_i[2], FL_lower_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_lower_o[3] = {FL_lower_o[1], -FL_lower_o[2], FL_lower_o[3]} annotation(Dialog(group="Geometry"));

  // RL node parameters
  parameter SIunits.Position RL_upper_fore_i[3] = {-1.298905, 0.282999, 0.217500} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_upper_aft_i[3] = {-1.490977, 0.282999, 0.217500} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_upper_o[3] = {-1.574797, 0.554998, 0.289560} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_tie_i[3] = {-1.428059, 0.282999, 0.177800} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_tie_o[3] = {-1.462710, 0.587375, 0.240741} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_lower_fore_i[3] = {-1.298905, 0.282999, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_lower_aft_i[3] = {-1.490977, 0.282999, 0.090000} annotation(Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  parameter SIunits.Position RL_lower_o[3] = {-1.554983, 0.579999, 0.113030} annotation(Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  
  // RR node parameters
  parameter SIunits.Position RR_upper_fore_i[3] = {RL_upper_fore_i[1], -RL_upper_fore_i[2], RL_upper_fore_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_aft_i[3] = {RL_upper_aft_i[1], -RL_upper_aft_i[2], RL_upper_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_o[3] = {RL_upper_o[1], -RL_upper_o[2], RL_upper_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_i[3] = {RL_tie_i[1], -RL_tie_i[2], RL_tie_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_o[3] = {RL_tie_o[1], -RL_tie_o[2], RL_tie_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_fore_i[3] = {RL_lower_fore_i[1], -RL_lower_fore_i[2], RL_lower_fore_i[3]} annotation(Dialog(group="Geometry"));
parameter SIunits.Position RR_lower_aft_i[3] = {RL_lower_aft_i[1], -RL_lower_aft_i[2], RL_lower_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_o[3] = {RL_lower_o[1], -RL_lower_o[2], RL_lower_o[3]} annotation(Dialog(group="Geometry"));
  
  // Mount parameters
  parameter SIunits.Position FL_mount_node[3] = {-0.09277, 0.24013, 0.56990} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_mount_node[3] = {-0.09277, -0.24013, 0.56990} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_mount_node[3] = {-1.42764, 0.29016, 0.40877} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_mount_node[3] = {-1.42764, -0.29016, 0.40877} annotation(Dialog(group="Geometry"));
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, animateGround = true, groundColor = {0, 255, 0})  annotation(
    Placement(transformation(origin = {-170, -130}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Body.FrameFrSteerFrRrBellcrank frameFrSteer(FL_upper_fore_i = FL_upper_fore_i, FL_upper_aft_i = FL_upper_aft_i, FL_tie_i = FL_tie_i, FL_lower_fore_i = FL_lower_fore_i, FL_lower_aft_i = FL_lower_aft_i, RL_upper_fore_i = RL_upper_fore_i, RL_upper_aft_i = RL_upper_aft_i, RL_tie_i = RL_tie_i, RL_lower_fore_i = RL_lower_fore_i, RL_lower_aft_i = RL_lower_aft_i, CG_location = CG_location, FL_mount = FL_mount_node, FR_mount = FR_mount_node, RL_mount = RL_mount_node, RR_mount = RR_mount_node, front_mass = 100, rear_mass = 100)  annotation(
    Placement(transformation(extent = {{-30, -60}, {30, 60}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 0*1*0.0254, duration = 0.5, startTime = 0.125)  annotation(
    Placement(transformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Vehicle.Chassis.Suspension.DoubleWishboneBase doubleWishboneBase(upper_fore_i = FL_upper_fore_i, upper_aft_i = FL_upper_aft_i, lower_fore_i = FL_lower_fore_i, lower_aft_i = FL_lower_aft_i, upper_o = FL_upper_o, lower_o = FL_lower_o, tie_i = FL_tie_i, tie_o = FL_tie_o, contact_patch = FL_contact_patch) annotation(
    Placement(transformation(origin = {-60, 30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.DoubleWishboneBase doubleWishboneBase1(contact_patch = RL_contact_patch, lower_aft_i = RL_lower_aft_i, lower_fore_i = RL_lower_fore_i, lower_o = RL_lower_o, tie_i = RL_tie_i, tie_o = RL_tie_o, upper_aft_i = RL_upper_aft_i, upper_fore_i = RL_upper_fore_i, upper_o = RL_upper_o) annotation(
    Placement(transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.DoubleWishboneBase FR_doubleWishboneBase(upper_fore_i = FR_upper_fore_i, upper_aft_i = FR_upper_aft_i, lower_fore_i = FR_lower_fore_i, lower_aft_i = FR_lower_aft_i, upper_o = FR_upper_o, lower_o = FR_lower_o, tie_i = FR_tie_i, tie_o = FR_tie_o, contact_patch = FR_contact_patch)  annotation(
    Placement(transformation(origin = {60, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.DoubleWishboneBase RR_doubleWishboneBase1(contact_patch = RR_contact_patch, lower_aft_i = RR_lower_aft_i, lower_fore_i = RR_lower_fore_i, lower_o = RR_lower_o, tie_i = RR_tie_i, tie_o = RR_tie_o, upper_aft_i = RR_upper_aft_i, upper_fore_i = RR_upper_fore_i, upper_o = RR_upper_o) annotation(
    Placement(transformation(origin = {60, -30}, extent = {{10, -10}, {-10, 10}})));
  // FL ground interface
  Vehicle.Chassis.Suspension.Tires.MF5p2Base FL_tire(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  // FR ground interface
  Vehicle.Chassis.Suspension.Tires.MF5p2Base FR_tire(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    
  // RL ground interface
  Vehicle.Chassis.Suspension.Tires.MF5p2Base RL_tire(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  
  // RR ground interface
  Vehicle.Chassis.Suspension.Tires.MF5p2Base RR_tire(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    
  // Tires :D
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_x(n = {1, 0, 0}, useAxisFlange = false, s(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_y(n = {0, 1, 0}, s(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-80, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_z(n = {0, 0, 1}, s(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-50, -130}, extent = {{-10, -10}, {10, 10}})));
    
  Modelica.Mechanics.MultiBody.Joints.Revolute free_rot_x(n = {1, 0, 0}, phi(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {-20, -130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute free_rot_y(n = {0, 1, 0}, phi(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {0, -132}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute free_rot_z(n = {0, 0, 1}, phi(start = 0, fixed = true), useAxisFlange = true) annotation(
    Placement(transformation(origin = {0, -102}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false, r = FL_contact_patch) annotation(
    Placement(transformation(origin = {-160, 10}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.GroundPhysics ground annotation(
    Placement(transformation(origin = {-120, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(animation = false, r = FR_contact_patch) annotation(
    Placement(transformation(origin = {160, 10}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.GroundPhysics ground1 annotation(
    Placement(transformation(origin = {120, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(animation = false, r = RL_contact_patch) annotation(
    Placement(transformation(origin = {-160, -50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.GroundPhysics ground2 annotation(
    Placement(transformation(origin = {-120, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed3(animation = false, r = RR_contact_patch) annotation(
    Placement(transformation(origin = {160, -50}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.GroundPhysics ground3 annotation(
    Placement(transformation(origin = {120, -50}, extent = {{10, -10}, {-10, 10}})));

Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel FL_rod(c = 62e3 / 10, s_unstretched = norm(FL_upper_o - FL_mount_node), d = 100e1)  annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel FR_rod(c = 62e3 / 10, s_unstretched = norm(FR_upper_o - FR_mount_node), d = 100e1) annotation(
    Placement(transformation(origin = {50, 70}, extent = {{10, -10}, {-10, 10}})));
Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel RL_rod(c = 62e3 / 10, s_unstretched = norm(RL_lower_o - RL_mount_node), d = 100e1) annotation(
    Placement(transformation(origin = {-34, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel RR_rod(c = 62e3 / 10, s_unstretched = norm(RR_lower_o - RR_mount_node), d = 100e1) annotation(
    Placement(transformation(origin = {34, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed21(animation = false, r = CG_location+ {0, 0, 0.5}) annotation(
    Placement(transformation(origin = {-110, -130}, extent = {{-10, -10}, {10, 10}})));

Modelica.Mechanics.Rotational.Sources.Speed speed_yaw annotation(
    Placement(transformation(origin = {-50, -102}, extent = {{-10, -10}, {10, 10}})));
Modelica.Blocks.Sources.Ramp yaw_ramp(height = 0, duration = 0.5, startTime = 0.125)  annotation(
    Placement(transformation(origin = {-86, -102}, extent = {{-10, -10}, {10, 10}})));


equation
  connect(ramp.y, frameFrSteer.u) annotation(
    Line(points = {{0, 79}, {0, 30}}, color = {0, 0, 127}));
  connect(doubleWishboneBase.tie_i_frame, frameFrSteer.FL_tie_i_frame) annotation(
    Line(points = {{-50, 24}, {-40, 24}, {-40, 30}, {-30, 30}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.lower_fore_i_frame, frameFrSteer.FL_lower_fore_i_frame) annotation(
    Line(points = {{-50, 36}, {-36, 36}, {-36, 16}, {-30, 16}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.lower_aft_i_frame, frameFrSteer.FL_lower_aft_i_frame) annotation(
    Line(points = {{-50, 30}, {-44, 30}, {-44, 4}, {-30, 4}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.upper_aft_i_frame, frameFrSteer.FL_upper_aft_i_frame) annotation(
    Line(points = {{-54, 40}, {-52, 40}, {-52, 44}, {-30, 44}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.upper_fore_i_frame, frameFrSteer.FL_upper_fore_i_frame) annotation(
    Line(points = {{-60, 40}, {-60, 56}, {-30, 56}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.upper_fore_i_frame, frameFrSteer.RL_upper_fore_i_frame) annotation(
    Line(points = {{-60, -20}, {-60, -4}, {-30, -4}}, color = {95, 95, 95}));
  connect(frameFrSteer.RL_upper_aft_i_frame, doubleWishboneBase1.upper_aft_i_frame) annotation(
    Line(points = {{-30, -16}, {-54, -16}, {-54, -20}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.tie_i_frame, frameFrSteer.RL_tie_i_frame) annotation(
    Line(points = {{-50, -36}, {-40, -36}, {-40, -30}, {-30, -30}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.lower_aft_i_frame, frameFrSteer.RL_lower_aft_i_frame) annotation(
    Line(points = {{-50, -30}, {-46, -30}, {-46, -56}, {-30, -56}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.lower_fore_i_frame, frameFrSteer.RL_lower_fore_i_frame) annotation(
    Line(points = {{-50, -24}, {-36, -24}, {-36, -44}, {-30, -44}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.upper_aft_i_frame, frameFrSteer.FR_upper_aft_i_frame) annotation(
    Line(points = {{54, 40}, {54, 44}, {30, 44}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.upper_fore_i_frame, frameFrSteer.FR_upper_fore_i_frame) annotation(
    Line(points = {{60, 40}, {60, 56}, {30, 56}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.tie_i_frame, frameFrSteer.FR_tie_i_frame) annotation(
    Line(points = {{50, 24}, {40, 24}, {40, 30}, {30, 30}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.lower_fore_i_frame, frameFrSteer.FR_lower_fore_i_frame) annotation(
    Line(points = {{50, 36}, {36, 36}, {36, 16}, {30, 16}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.lower_aft_i_frame, frameFrSteer.FR_lower_aft_i_frame) annotation(
    Line(points = {{50, 30}, {44, 30}, {44, 4}, {30, 4}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.upper_aft_i_frame, frameFrSteer.RR_upper_aft_i_frame) annotation(
    Line(points = {{54, -20}, {54, -16}, {30, -16}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.upper_fore_i_frame, frameFrSteer.RR_upper_fore_i_frame) annotation(
    Line(points = {{60, -20}, {60, -4}, {30, -4}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.tie_i_frame, frameFrSteer.RR_tie_i_frame) annotation(
    Line(points = {{50, -36}, {40, -36}, {40, -30}, {30, -30}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.lower_aft_i_frame, frameFrSteer.RR_lower_aft_i_frame) annotation(
    Line(points = {{50, -30}, {46, -30}, {46, -56}, {30, -56}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.lower_fore_i_frame, frameFrSteer.RR_lower_fore_i_frame) annotation(
    Line(points = {{50, -24}, {36, -24}, {36, -44}, {30, -44}}, color = {95, 95, 95}));
  connect(free_y.frame_b, free_z.frame_a) annotation(
    Line(points = {{-70, -130}, {-60, -130}}, color = {95, 95, 95}));
  connect(free_z.frame_b, free_rot_x.frame_a) annotation(
    Line(points = {{-40, -130}, {-30, -130}}, color = {95, 95, 95}));
  connect(free_rot_x.frame_b, free_rot_y.frame_b) annotation(
    Line(points = {{-10, -130}, {-10, -142}, {0, -142}}, color = {95, 95, 95}));
  connect(free_rot_y.frame_a, free_rot_z.frame_a) annotation(
    Line(points = {{0, -122}, {0, -112}}, color = {95, 95, 95}));
  connect(ground.frame_a, fixed.frame_b) annotation(
    Line(points = {{-130, 10}, {-150, 10}}, color = {95, 95, 95}));
  connect(fixed1.frame_b, ground1.frame_a) annotation(
    Line(points = {{150, 10}, {130, 10}}, color = {95, 95, 95}));
  connect(fixed3.frame_b, ground3.frame_a) annotation(
    Line(points = {{150, -50}, {130, -50}}, color = {95, 95, 95}));
  connect(fixed2.frame_b, ground2.frame_a) annotation(
    Line(points = {{-150, -50}, {-130, -50}}, color = {95, 95, 95}));
  connect(FL_rod.frame_a, doubleWishboneBase.upper_wishbone_frame) annotation(
    Line(points = {{-60, 70}, {-66, 70}, {-66, 40}}, color = {95, 95, 95}));
  connect(FL_rod.frame_b, frameFrSteer.FL_mount_frame) annotation(
    Line(points = {{-40, 70}, {-20, 70}, {-20, 50}}, color = {95, 95, 95}));
  connect(FR_rod.frame_b, frameFrSteer.FR_mount_frame) annotation(
    Line(points = {{40, 70}, {20, 70}, {20, 50}}, color = {95, 95, 95}));
  connect(FR_rod.frame_a, FR_doubleWishboneBase.upper_wishbone_frame) annotation(
    Line(points = {{60, 70}, {66, 70}, {66, 40}}, color = {95, 95, 95}));
  connect(RL_rod.frame_a, doubleWishboneBase1.lower_wishbone_frame) annotation(
    Line(points = {{-44, -70}, {-54, -70}, {-54, -40}}, color = {95, 95, 95}));
  connect(RL_rod.frame_b, frameFrSteer.RL_mount_frame) annotation(
    Line(points = {{-24, -70}, {-20, -70}, {-20, -50}}, color = {95, 95, 95}));
  connect(RR_rod.frame_a, RR_doubleWishboneBase1.lower_wishbone_frame) annotation(
    Line(points = {{44, -70}, {54, -70}, {54, -40}}, color = {95, 95, 95}));
  connect(RR_rod.frame_b, frameFrSteer.RR_mount_frame) annotation(
    Line(points = {{24, -70}, {20, -70}, {20, -50}}, color = {95, 95, 95}));
  connect(FL_tire.frame_a, ground.frame_b) annotation(
    Line(points = {{-100, 10}, {-100, 9}, {-110, 9}, {-110, 10}}, color = {95, 95, 95}));
  connect(RL_tire.frame_a, ground2.frame_b) annotation(
    Line(points = {{-100, -50}, {-100, -51}, {-110, -51}, {-110, -50}}, color = {95, 95, 95}));
  connect(RR_tire.frame_a, ground3.frame_b) annotation(
    Line(points = {{100, -50}, {100, -51}, {110, -51}, {110, -50}}, color = {95, 95, 95}));
  connect(FR_tire.frame_a, ground1.frame_b) annotation(
    Line(points = {{100, 10}, {100, 11}, {110, 11}, {110, 10}}, color = {95, 95, 95}));
  connect(free_rot_z.frame_b, free_x.frame_a) annotation(
    Line(points = {{0, -92}, {0, -40}}, color = {95, 95, 95}));
  connect(free_x.frame_b, frameFrSteer.frame_a) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
  connect(fixed21.frame_b, free_y.frame_a) annotation(
    Line(points = {{-100, -130}, {-90, -130}}, color = {95, 95, 95}));
  connect(FR_tire.corner_frame, FR_doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{80, 10}, {60, 10}, {60, 20}}, color = {95, 95, 95}));
  connect(RR_tire.corner_frame, RR_doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{80, -50}, {60, -50}, {60, -40}}, color = {95, 95, 95}));
  connect(FL_tire.corner_frame, doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{-80, 10}, {-60, 10}, {-60, 20}}, color = {95, 95, 95}));
  connect(RL_tire.corner_frame, doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{-80, -50}, {-60, -50}, {-60, -40}}, color = {95, 95, 95}));
  connect(speed_yaw.flange, free_rot_z.axis) annotation(
    Line(points = {{-40, -102}, {-10, -102}}));
  connect(yaw_ramp.y, speed_yaw.w_ref) annotation(
    Line(points = {{-74, -102}, {-62, -102}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.3")),
  Diagram(coordinateSystem(extent = {{-180, 100}, {180, -140}})),
  version = "",
  Documentation(revisions = "<html><head></head><body></body></html>"));
end Test_Frame;
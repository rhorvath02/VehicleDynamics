model Test_Frame
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
  // FR node parameters
  parameter SIunits.Position RR_upper_fore_i[3] = {RL_upper_fore_i[1], -RL_upper_fore_i[2], RL_upper_fore_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_aft_i[3] = {RL_upper_aft_i[1], -RL_upper_aft_i[2], RL_upper_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_o[3] = {RL_upper_o[1], -RL_upper_o[2], RL_upper_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_i[3] = {RL_tie_i[1], -RL_tie_i[2], RL_tie_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_o[3] = {RL_tie_o[1], -RL_tie_o[2], RL_tie_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_fore_i[3] = {RL_lower_fore_i[1], -RL_lower_fore_i[2], RL_lower_fore_i[3]} annotation(Dialog(group="Geometry"));
parameter SIunits.Position RR_lower_aft_i[3] = {RL_lower_aft_i[1], -RL_lower_aft_i[2], RL_lower_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_o[3] = {RL_lower_o[1], -RL_lower_o[2], RL_lower_o[3]} annotation(Dialog(group="Geometry"));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Body.FrameFrSteer frameFrSteer(FL_upper_fore_i = FL_upper_fore_i, FL_upper_aft_i = FL_upper_aft_i, FL_tie_i = FL_tie_i, FL_lower_fore_i = FL_lower_fore_i, FL_lower_aft_i = FL_lower_aft_i, RL_upper_fore_i = RL_upper_fore_i, RL_upper_aft_i = RL_upper_aft_i, RL_tie_i = RL_tie_i, RL_lower_fore_i = RL_lower_fore_i, RL_lower_aft_i = RL_lower_aft_i, CG_location = CG_location)  annotation(
    Placement(transformation(extent = {{-30, -60}, {30, 60}})));
  Modelica.Blocks.Sources.Sine sine(amplitude = 1.5*0.0254, freqHz = 1)  annotation(
    Placement(transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone(aft_i = FL_upper_aft_i, fore_i = FL_upper_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FL_upper_o) annotation(
    Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone1(aft_i = FL_lower_aft_i, fore_i = FL_lower_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FL_lower_o) annotation(
    Placement(transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Upright upright(lower = FL_lower_o, tie = FL_tie_o, upper = FL_upper_o) annotation(
    Placement(transformation(origin = {-80, 30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Link link(inboard = FL_tie_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FL_tie_o) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch(r = FL_contact_patch - FL_lower_o) annotation(
    Placement(transformation(origin = {-90, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed(r = FL_contact_patch, animation = false)  annotation(
    Placement(transformation(origin = {-150, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = CG_location, animation = false)  annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Vehicle.GroundPhysics groundPhysics annotation(
    Placement(transformation(origin = {-122, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 16, sphereDiameter = 0.025)  annotation(
    Placement(transformation(origin = {-100, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone2(aft_i = FR_upper_aft_i, fore_i = FR_upper_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FR_upper_o) annotation(
    Placement(transformation(origin = {50, 50}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone11(aft_i = FR_lower_aft_i, fore_i = FR_lower_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FR_lower_o) annotation(
    Placement(transformation(origin = {50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.Linkages.Upright upright1(lower = FR_lower_o, tie = FR_tie_o, upper = FR_upper_o) annotation(
    Placement(transformation(origin = {80, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.Linkages.Link link1(inboard = FR_tie_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = FR_tie_o) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch1(r = FR_contact_patch - FR_lower_o) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed1(animation = false, r = FR_contact_patch) annotation(
    Placement(transformation(origin = {150, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.GroundPhysics groundPhysics1 annotation(
    Placement(transformation(origin = {120, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Body body1(m = 16, r_CM = {0, 0, 0}, sphereDiameter = 0.025) annotation(
    Placement(transformation(origin = {100, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone3(aft_i = RL_upper_aft_i, fore_i = RL_upper_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RL_upper_o) annotation(
    Placement(transformation(origin = {-50, -10}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone12(aft_i = RL_lower_aft_i, fore_i = RL_lower_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RL_lower_o) annotation(
    Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Upright upright2(lower = RL_lower_o, tie = RL_tie_o, upper = RL_upper_o) annotation(
    Placement(transformation(origin = {-80, -30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Link link2(inboard = RL_tie_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RL_tie_o) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch2(r = RL_contact_patch - RL_lower_o) annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed2(animation = false, r = RL_contact_patch) annotation(
    Placement(transformation(origin = {-150, -50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.GroundPhysics groundPhysics2 annotation(
    Placement(transformation(origin = {-122, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body2(m = 16, r_CM = {0, 0, 0}, sphereDiameter = 0.025) annotation(
    Placement(transformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone21(aft_i = RR_upper_aft_i, fore_i = RR_upper_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RR_upper_o) annotation(
    Placement(transformation(origin = {50, -10}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone111(aft_i = RR_lower_aft_i, fore_i = RR_lower_fore_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RR_lower_o) annotation(
    Placement(transformation(origin = {50, -50}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Upright upright11(lower = RR_lower_o, tie = RR_tie_o, upper = RR_upper_o) annotation(
    Placement(transformation(origin = {80, -30}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Link link11(inboard = RR_tie_i, joint_diameter = 0.030, link_diameter = 0.010, outboard = RR_tie_o) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch11(r = RR_contact_patch - RR_lower_o) annotation(
    Placement(transformation(origin = {90, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed11(animation = false, r = RR_contact_patch) annotation(
    Placement(transformation(origin = {150, -50}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.GroundPhysics groundPhysics11 annotation(
    Placement(transformation(origin = {120, -50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body11(m = 16, r_CM = {0, 0, 0}, sphereDiameter = 0.025) annotation(
    Placement(transformation(origin = {100, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
equation
  connect(sine.y, frameFrSteer.u) annotation(
    Line(points = {{-19, 80}, {-0.5, 80}, {-0.5, 30}, {0, 30}}, color = {0, 0, 127}));
  connect(wishbone.outboard_frame, upright.upper_frame) annotation(
    Line(points = {{-60, 50}, {-80, 50}, {-80, 40}}, color = {95, 95, 95}));
  connect(wishbone1.outboard_frame, upright.lower_frame) annotation(
    Line(points = {{-60, 10}, {-80, 10}, {-80, 20}}, color = {95, 95, 95}));
  connect(link.frame_b, upright.tie_frame) annotation(
    Line(points = {{-60, 30}, {-70, 30}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch.frame_a, upright.lower_frame) annotation(
    Line(points = {{-80, 10}, {-80, 20}}, color = {95, 95, 95}));
  connect(wishbone.fore_i_frame, frameFrSteer.FL_upper_fore_i_frame) annotation(
    Line(points = {{-40, 56}, {-30, 56}}, color = {95, 95, 95}));
  connect(wishbone.aft_i_frame, frameFrSteer.FL_upper_aft_i_frame) annotation(
    Line(points = {{-40, 44}, {-30, 44}}, color = {95, 95, 95}));
  connect(link.frame_a, frameFrSteer.FL_tie_i_frame) annotation(
    Line(points = {{-40, 30}, {-30, 30}}, color = {95, 95, 95}));
  connect(wishbone1.fore_i_frame, frameFrSteer.FL_lower_fore_i_frame) annotation(
    Line(points = {{-40, 16}, {-30, 16}}, color = {95, 95, 95}));
  connect(wishbone1.aft_i_frame, frameFrSteer.FL_lower_aft_i_frame) annotation(
    Line(points = {{-40, 4}, {-30, 4}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch.frame_b, groundPhysics.frame_b) annotation(
    Line(points = {{-100, 10}, {-112, 10}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed.frame_b, groundPhysics.frame_a) annotation(
    Line(points = {{-140, 10}, {-132, 10}}, color = {95, 95, 95}));
  connect(body.frame_a, upright.mass_frame) annotation(
    Line(points = {{-90, 30}, {-80, 30}}, color = {95, 95, 95}));
  connect(frameFrSteer.FR_upper_fore_i_frame, wishbone2.fore_i_frame) annotation(
    Line(points = {{30, 56}, {40, 56}}, color = {95, 95, 95}));
  connect(frameFrSteer.FR_upper_aft_i_frame, wishbone2.aft_i_frame) annotation(
    Line(points = {{30, 44}, {40, 44}}, color = {95, 95, 95}));
  connect(frameFrSteer.FR_tie_i_frame, link1.frame_a) annotation(
    Line(points = {{30, 30}, {40, 30}}, color = {95, 95, 95}));
  connect(frameFrSteer.FR_lower_fore_i_frame, wishbone11.fore_i_frame) annotation(
    Line(points = {{30, 16}, {40, 16}}, color = {95, 95, 95}));
  connect(frameFrSteer.FR_lower_aft_i_frame, wishbone11.aft_i_frame) annotation(
    Line(points = {{30, 4}, {40, 4}}, color = {95, 95, 95}));
  connect(wishbone2.outboard_frame, upright1.upper_frame) annotation(
    Line(points = {{60, 50}, {80, 50}, {80, 40}}, color = {95, 95, 95}));
  connect(link1.frame_b, upright1.tie_frame) annotation(
    Line(points = {{60, 30}, {70, 30}}, color = {95, 95, 95}));
  connect(wishbone11.outboard_frame, upright1.lower_frame) annotation(
    Line(points = {{60, 10}, {80, 10}, {80, 20}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch1.frame_a, upright1.lower_frame) annotation(
    Line(points = {{80, 10}, {80, 20}}, color = {95, 95, 95}));
  connect(groundPhysics1.frame_b, lower_to_contact_patch1.frame_b) annotation(
    Line(points = {{110, 10}, {100, 10}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed1.frame_b, groundPhysics1.frame_a) annotation(
    Line(points = {{140, 10}, {130, 10}}, color = {95, 95, 95}));
  connect(body1.frame_a, upright1.mass_frame) annotation(
    Line(points = {{90, 30}, {80, 30}}, color = {95, 95, 95}));
  connect(wishbone3.outboard_frame, upright2.upper_frame) annotation(
    Line(points = {{-60, -10}, {-80, -10}, {-80, -20}}, color = {95, 95, 95}));
  connect(wishbone12.outboard_frame, upright2.lower_frame) annotation(
    Line(points = {{-60, -50}, {-80, -50}, {-80, -40}}, color = {95, 95, 95}));
  connect(link2.frame_b, upright2.tie_frame) annotation(
    Line(points = {{-60, -30}, {-70, -30}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch2.frame_a, upright2.lower_frame) annotation(
    Line(points = {{-80, -50}, {-80, -40}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch2.frame_b, groundPhysics2.frame_b) annotation(
    Line(points = {{-100, -50}, {-112, -50}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed2.frame_b, groundPhysics2.frame_a) annotation(
    Line(points = {{-140, -50}, {-132, -50}}, color = {95, 95, 95}));
  connect(body2.frame_a, upright2.mass_frame) annotation(
    Line(points = {{-90, -30}, {-80, -30}}, color = {95, 95, 95}));
  connect(wishbone3.fore_i_frame, frameFrSteer.RL_upper_fore_i_frame) annotation(
    Line(points = {{-40, -4}, {-30, -4}}, color = {95, 95, 95}));
  connect(wishbone3.aft_i_frame, frameFrSteer.RL_upper_aft_i_frame) annotation(
    Line(points = {{-40, -16}, {-30, -16}}, color = {95, 95, 95}));
  connect(link2.frame_a, frameFrSteer.RL_tie_i_frame) annotation(
    Line(points = {{-40, -30}, {-30, -30}}, color = {95, 95, 95}));
  connect(wishbone12.fore_i_frame, frameFrSteer.RL_lower_fore_i_frame) annotation(
    Line(points = {{-40, -44}, {-30, -44}}, color = {95, 95, 95}));
  connect(wishbone12.aft_i_frame, frameFrSteer.RL_lower_aft_i_frame) annotation(
    Line(points = {{-40, -56}, {-30, -56}}, color = {95, 95, 95}));
  connect(wishbone21.outboard_frame, upright11.upper_frame) annotation(
    Line(points = {{60, -10}, {80, -10}, {80, -20}}, color = {95, 95, 95}));
  connect(link11.frame_b, upright11.tie_frame) annotation(
    Line(points = {{60, -30}, {70, -30}}, color = {95, 95, 95}));
  connect(wishbone111.outboard_frame, upright11.lower_frame) annotation(
    Line(points = {{60, -50}, {80, -50}, {80, -40}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch11.frame_a, upright11.lower_frame) annotation(
    Line(points = {{80, -50}, {80, -40}}, color = {95, 95, 95}));
  connect(groundPhysics11.frame_b, lower_to_contact_patch11.frame_b) annotation(
    Line(points = {{110, -50}, {100, -50}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed11.frame_b, groundPhysics11.frame_a) annotation(
    Line(points = {{140, -50}, {130, -50}}, color = {95, 95, 95}));
  connect(body11.frame_a, upright11.mass_frame) annotation(
    Line(points = {{90, -30}, {80, -30}}, color = {95, 95, 95}));
  connect(frameFrSteer.RR_upper_fore_i_frame, wishbone21.fore_i_frame) annotation(
    Line(points = {{30, -4}, {40, -4}}, color = {95, 95, 95}));
  connect(frameFrSteer.RR_upper_aft_i_frame, wishbone21.aft_i_frame) annotation(
    Line(points = {{30, -16}, {40, -16}}, color = {95, 95, 95}));
  connect(frameFrSteer.RR_tie_i_frame, link11.frame_a) annotation(
    Line(points = {{30, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(frameFrSteer.RR_lower_fore_i_frame, wishbone111.fore_i_frame) annotation(
    Line(points = {{30, -44}, {40, -44}}, color = {95, 95, 95}));
  connect(frameFrSteer.RR_lower_aft_i_frame, wishbone111.aft_i_frame) annotation(
    Line(points = {{30, -56}, {40, -56}}, color = {95, 95, 95}));
  connect(fixed.frame_b, frameFrSteer.frame_a) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
  annotation(
    uses(Modelica(version = "3.2.3")));
end Test_Frame;
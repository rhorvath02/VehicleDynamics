within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;

model FrDoubleWishbone
  // Modelica units
  import Modelica.SIunits;
  // Parameters - FL sus defn
  final parameter VehicleDynamics.Resources.Records.SUS.FrAxleBase FLDW;
  // Parameters
  parameter SIunits.Position upper_fore_i[3] = FLDW.upper_fore_i annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_aft_i[3] = FLDW.upper_aft_i annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_fore_i[3] = FLDW.lower_fore_i annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_aft_i[3] = FLDW.lower_aft_i annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_o[3] = FLDW.upper_outboard annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_o[3] = FLDW.lower_outboard annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_i[3] = FLDW.tie_inboard annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_o[3] = FLDW.tie_outboard annotation(Dialog(group="Geometry"));
  parameter SIunits.Position wheel_center[3] = FLDW.wheel_center annotation(Dialog(group="Geometry"));
  parameter SIunits.Angle static_gamma = FLDW.static_gamma annotation(Dialog(group="Geometry"));
  parameter SIunits.Angle static_alpha = FLDW.static_alpha annotation(Dialog(group="Geometry"));
  
  parameter SIunits.TranslationalSpringConstant[3] FUCA_fore_i_c = FLDW.upper_fore_i_c "{x, y, z}-stiffness of upper, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FUCA_aft_i_c = FLDW.upper_aft_i_c "{x, y, z}-stiffness of upper, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_fore_i_c = FLDW.lower_fore_i_c "{x, y, z}-stiffness of lower, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_aft_i_c = FLDW.lower_aft_i_c  "{x, y, z}-stiffness of lower, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] tie_i_c = FLDW.tie_inboard_c  "{x, y, z}-stiffness of inboard tie mount" annotation(Dialog(group="Mounting"));

  parameter SIunits.TranslationalDampingConstant[3] FUCA_fore_i_d = FLDW.upper_fore_i_d "{x, y, z}-damping of upper, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FUCA_aft_i_d = FLDW.upper_aft_i_d "{x, y, z}-damping of upper, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_fore_i_d = FLDW.lower_fore_i_d "{x, y, z}-damping of lower, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_aft_i_d = FLDW.lower_aft_i_d "{x, y, z}-damping of lower, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] tie_i_d = FLDW.tie_inboard_d "{x, y, z}-damping of inboard tie mount" annotation(Dialog(group="Mounting"));
  // Visual parameters
  parameter SIunits.Length link_diameter = 0.025 annotation(Dialog(tab="Animation", group="Sizing"));
  parameter SIunits.Length joint_diameter = 0.030 annotation(Dialog(tab="Animation", group="Sizing"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upper_fore_i_frame annotation(
    Placement(transformation(origin = {100, 80}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upper_aft_i_frame annotation(
    Placement(transformation(origin = {100, 40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {66, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lower_fore_i_frame annotation(
    Placement(transformation(origin = {100, -80}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lower_aft_i_frame annotation(
    Placement(transformation(origin = {100, -40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a tie_i_frame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b upper_wishbone_frame annotation(
    Placement(transformation(origin = {44, 100}, extent = {{16, -16}, {-16, 16}}, rotation = 90), iconTransformation(origin = {-100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b lower_wishbone_frame annotation(
    Placement(transformation(origin = {44, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {-100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b midpoint_frame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{16, -16}, {-16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

// Wishbones
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Wishbone upper_wishbone(fore_i = upper_fore_i, aft_i = upper_aft_i, outboard = upper_o, link_diameter = link_diameter, joint_diameter = joint_diameter, fore_i_c = FUCA_fore_i_c, aft_i_c = FUCA_aft_i_c, fore_i_d = FUCA_fore_i_d, aft_i_d = FUCA_aft_i_d) annotation(
    Placement(transformation(origin = {50, 60}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Wishbone lower_wishbone(aft_i = lower_aft_i, fore_i = lower_fore_i, outboard = lower_o, link_diameter = link_diameter, joint_diameter = joint_diameter, fore_i_c = FLCA_fore_i_c, aft_i_c = FLCA_aft_i_c, fore_i_d = FLCA_fore_i_d, aft_i_d = FLCA_aft_i_d) annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  
  // Upright
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Upright upright(lower = lower_o, upper = upper_o, tie = tie_o) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}})));
  // Tie rod
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Link link(inboard = tie_i, outboard = tie_o, link_diameter = link_diameter, joint_diameter = joint_diameter, inboard_c = tie_i_c, inboard_d = tie_i_d) annotation(
    Placement(transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}})));
  // Generalized mass
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, animation = false)  annotation(
    Placement(transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  // Connect midpoint of kingpin to center of the wheel
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = wheel_center - (upper_o + lower_o) / 2) annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{10, -10}, {-10, 10}})));
  // Steering interface
  Modelica.Blocks.Interfaces.RealInput steer_input annotation(
    Placement(transformation(origin = {-20, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-66, 120}, extent = {{-20, -20}, {20, 20}}, rotation=-90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_rack(n = {0, 1, 0}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  // ==================================================
  // === I dare you to find a better way to do this ===
  // ==================================================
  // Set gamma
  Modelica.Blocks.Sources.RealExpression static_gamma_source(y = static_gamma*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-40, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position x_angle annotation(
    Placement(transformation(origin = {-40, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_x(n = {1, 0, 0}, phi(start = static_gamma*Modelica.Constants.pi/180), useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}})));
  // Set toe (using alpha sign convention)
  Modelica.Blocks.Sources.RealExpression static_alpha_source(y = static_alpha*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Position z_angle annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_z(n = {0, 0, 1}, phi(start = static_alpha*Modelica.Constants.pi/180), useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.Translational.Sources.Position position(useSupport = true)  annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
equation
  connect(upper_fore_i_frame, upper_wishbone.fore_i_frame) annotation(
    Line(points = {{100, 80}, {70, 80}, {70, 67}, {60, 67}}));
  connect(upper_aft_i_frame, upper_wishbone.aft_i_frame) annotation(
    Line(points = {{100, 40}, {70, 40}, {70, 53}, {60, 53}}));
  connect(link.frame_b, upright.tie_frame) annotation(
    Line(points = {{30, 0}, {20, 0}}, color = {95, 95, 95}));
  connect(lower_wishbone.outboard_frame, upright.lower_frame) annotation(
    Line(points = {{40, -60}, {10, -60}, {10, -10}}, color = {95, 95, 95}));
  connect(upper_wishbone.outboard_frame, upright.upper_frame) annotation(
    Line(points = {{40, 60}, {10, 60}, {10, 10}}, color = {95, 95, 95}));
  connect(lower_aft_i_frame, lower_wishbone.aft_i_frame) annotation(
    Line(points = {{100, -40}, {70, -40}, {70, -53}, {60, -53}}));
  connect(lower_fore_i_frame, lower_wishbone.fore_i_frame) annotation(
    Line(points = {{100, -80}, {70, -80}, {70, -67}, {60, -67}}));
  connect(upright.mass_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{10, 0}, {0, 0}}, color = {95, 95, 95}));
  connect(static_gamma_source.y, x_angle.phi_ref) annotation(
    Line(points = {{-40, 50}, {-40, 42}}, color = {0, 0, 127}));
  connect(x_angle.flange, revolute_x.axis) annotation(
    Line(points = {{-40, 20}, {-40, 10}}));
  connect(static_alpha_source.y, z_angle.phi_ref) annotation(
    Line(points = {{-70, 50}, {-70, 42}}, color = {0, 0, 127}));
  connect(z_angle.flange, revolute_z.axis) annotation(
    Line(points = {{-70, 20}, {-70, 10}}));
  connect(fixedTranslation.frame_b, revolute_x.frame_a) annotation(
    Line(points = {{-20, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(revolute_x.frame_b, revolute_z.frame_a) annotation(
    Line(points = {{-50, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(revolute_z.frame_b, midpoint_frame) annotation(
    Line(points = {{-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(upper_wishbone.link_frame, upper_wishbone_frame) annotation(
    Line(points = {{44, 70}, {44, 100}}, color = {95, 95, 95}));
  connect(lower_wishbone_frame, lower_wishbone.link_frame) annotation(
    Line(points = {{44, -100}, {44, -70}}));
  connect(tie_i_frame, prismatic_rack.frame_a) annotation(
    Line(points = {{100, 0}, {80, 0}}));
  connect(prismatic_rack.frame_b, link.frame_a) annotation(
    Line(points = {{60, 0}, {50, 0}}, color = {95, 95, 95}));
  connect(position.support, prismatic_rack.support) annotation(
    Line(points = {{40, -30}, {74, -30}, {74, -6}}, color = {0, 127, 0}));
  connect(position.flange, prismatic_rack.axis) annotation(
    Line(points = {{30, -20}, {30, -6}, {62, -6}}, color = {0, 127, 0}));
  connect(steer_input, position.s_ref) annotation(
    Line(points = {{-20, -120}, {-20, -70}, {30, -70}, {30, -42}}, color = {0, 0, 127}));
  connect(body.frame_a, upright.mass_frame) annotation(
    Line(points = {{-20, -30}, {0, -30}, {0, 0}, {10, 0}}, color = {95, 95, 95}));
end FrDoubleWishbone;

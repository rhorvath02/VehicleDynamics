within VehicleDynamics.Vehicle.Chassis.Suspension;

model FrDoubleWishboneBase
  // Modelica units
  import Modelica.SIunits;
  
  // File path inputs
  parameter String hdpts_path =
  Modelica.Utilities.Files.loadResource("modelica://VehicleDynamics/Resources/JSONs/SUS/hdpts.json") "File path to hdpts json" annotation(Dialog(group="File Paths"));
  parameter String mass_props_path =
  Modelica.Utilities.Files.loadResource("modelica://VehicleDynamics/Resources/JSONs/SUS/mass_props.json") "File path to mass_props json" annotation(Dialog(group="File Paths"));
  
  // JSONs
  inner ExternData.JSONFile hdpts(fileName = hdpts_path) annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{10, -10}, {-10, 10}})));
  inner ExternData.JSONFile mass_props(fileName = mass_props_path) annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{10, -10}, {-10, 10}})));

  // Parameters
  parameter SIunits.Position upper_fore_i[3] = hdpts.getRealArray1D("Front.left.upper.fore_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_aft_i[3] = hdpts.getRealArray1D("Front.left.upper.aft_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_fore_i[3] = hdpts.getRealArray1D("Front.left.lower.fore_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_aft_i[3] = hdpts.getRealArray1D("Front.left.lower.aft_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_o[3] = hdpts.getRealArray1D("Front.left.upper.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_o[3] = hdpts.getRealArray1D("Front.left.lower.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_i[3] = hdpts.getRealArray1D("Front.left.tie.inboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_o[3] = hdpts.getRealArray1D("Front.left.tie.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position wheel_center[3] = hdpts.getRealArray1D("Front.left.tire.wheel_center", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Angle static_gamma = hdpts.getReal("Front.left.tire.static_gamma") annotation(Dialog(group="Geometry"));
  parameter SIunits.Angle static_alpha = hdpts.getReal("Front.left.tire.static_alpha") annotation(Dialog(group="Geometry"));
  
  parameter SIunits.TranslationalSpringConstant[3] FUCA_fore_i_c = hdpts.getRealArray1D("Front.left.upper.fore_i_c", 3)  "{x, y, z}-stiffness of upper, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FUCA_aft_i_c = hdpts.getRealArray1D("Front.left.upper.aft_i_c", 3)  "{x, y, z}-stiffness of upper, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_fore_i_c = hdpts.getRealArray1D("Front.left.lower.fore_i_c", 3)  "{x, y, z}-stiffness of lower, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_aft_i_c = hdpts.getRealArray1D("Front.left.lower.aft_i_c", 3)  "{x, y, z}-stiffness of lower, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] tie_i_c = hdpts.getRealArray1D("Front.left.tie.inboard_c", 3)  "{x, y, z}-stiffness of inboard tie mount" annotation(Dialog(group="Mounting"));

  parameter SIunits.TranslationalDampingConstant[3] FUCA_fore_i_d = hdpts.getRealArray1D("Front.left.upper.fore_i_d", 3)  "{x, y, z}-damping of upper, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FUCA_aft_i_d = hdpts.getRealArray1D("Front.left.upper.aft_i_d", 3)  "{x, y, z}-damping of upper, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_fore_i_d = hdpts.getRealArray1D("Front.left.lower.fore_i_d", 3)  "{x, y, z}-damping of lower, fore, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_aft_i_d = hdpts.getRealArray1D("Front.left.lower.aft_i_d", 3)  "{x, y, z}-damping of lower, aft, inboard mount" annotation(Dialog(group="Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] tie_i_d = hdpts.getRealArray1D("Front.left.tie.inboard_d", 3)  "{x, y, z}-damping of inboard tie mount" annotation(Dialog(group="Mounting"));
  
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
  Suspension.Linkages.Wishbone upper_wishbone(fore_i = upper_fore_i, aft_i = upper_aft_i, outboard = upper_o, link_diameter = link_diameter, joint_diameter = joint_diameter, fore_i_c = FUCA_fore_i_c, aft_i_c = FUCA_aft_i_c, fore_i_d = FUCA_fore_i_d, aft_i_d = FUCA_aft_i_d) annotation(
    Placement(transformation(origin = {50, 60}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone lower_wishbone(aft_i = lower_aft_i, fore_i = lower_fore_i, outboard = lower_o, link_diameter = link_diameter, joint_diameter = joint_diameter, fore_i_c = FLCA_fore_i_c, aft_i_c = FLCA_aft_i_c, fore_i_d = FLCA_fore_i_d, aft_i_d = FLCA_aft_i_d) annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  
  // Upright
  Suspension.Linkages.Upright upright(lower = lower_o, upper = upper_o, tie = tie_o) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));
  
  // Tie rod
  Suspension.Linkages.Link link(inboard = tie_i, outboard = tie_o, link_diameter = link_diameter, joint_diameter = joint_diameter, inboard_c = tie_i_c, inboard_d = tie_i_d) annotation(
    Placement(transformation(origin = {50, 0}, extent = {{10, -10}, {-10, 10}})));
  
  // Generalized mass
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, animation = false)  annotation(
    Placement(transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  
  // Connect midpoint of kindpin to center of the wheel
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = wheel_center - (upper_o + lower_o) / 2) annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{10, -10}, {-10, 10}})));
  
  // ==================================================
  // === I dare you to find a better way to do this ===
  // ==================================================
  // Set gamma
  Modelica.Blocks.Sources.RealExpression static_gamma_source(y = static_gamma*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-40, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position x_angle annotation(
    Placement(transformation(origin = {-40, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_x(n = {1, 0, 0}, phi(fixed = true, start = static_gamma*Modelica.Constants.pi/180), useAxisFlange = true) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}})));
  
  // Set toe (using alpha sign convention)
  Modelica.Blocks.Sources.RealExpression static_alpha_source(y = static_alpha*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Position z_angle annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_z(n = {0, 0, 1}, phi(fixed = true, start = static_alpha*Modelica.Constants.pi/180), useAxisFlange = true) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{10, -10}, {-10, 10}})));
    
equation
  connect(upper_fore_i_frame, upper_wishbone.fore_i_frame) annotation(
    Line(points = {{100, 80}, {70, 80}, {70, 67}, {60, 67}}));
  connect(upper_aft_i_frame, upper_wishbone.aft_i_frame) annotation(
    Line(points = {{100, 40}, {70, 40}, {70, 53}, {60, 53}}));
  connect(tie_i_frame, link.frame_a) annotation(
    Line(points = {{100, 0}, {60, 0}}));
  connect(link.frame_b, upright.tie_frame) annotation(
    Line(points = {{40, 0}, {30, 0}}, color = {95, 95, 95}));
  connect(lower_wishbone.outboard_frame, upright.lower_frame) annotation(
    Line(points = {{40, -60}, {20, -60}, {20, -10}}, color = {95, 95, 95}));
  connect(upper_wishbone.outboard_frame, upright.upper_frame) annotation(
    Line(points = {{40, 60}, {20, 60}, {20, 10}}, color = {95, 95, 95}));
  connect(lower_aft_i_frame, lower_wishbone.aft_i_frame) annotation(
    Line(points = {{100, -40}, {70, -40}, {70, -53}, {60, -53}}));
  connect(lower_fore_i_frame, lower_wishbone.fore_i_frame) annotation(
    Line(points = {{100, -80}, {70, -80}, {70, -67}, {60, -67}}));
  connect(upright.mass_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{20, 0}, {0, 0}}, color = {95, 95, 95}));
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
  connect(body.frame_a, upright.mass_frame) annotation(
    Line(points = {{0, 30}, {10, 30}, {10, 0}, {20, 0}}, color = {95, 95, 95}));
  connect(upper_wishbone.link_frame, upper_wishbone_frame) annotation(
    Line(points = {{44, 70}, {44, 100}}, color = {95, 95, 95}));
  connect(lower_wishbone_frame, lower_wishbone.link_frame) annotation(
    Line(points = {{44, -100}, {44, -70}}));
end FrDoubleWishboneBase;
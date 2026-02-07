within BobDynamics.Vehicle.Chassis.Suspension.Templates;

model FrRigidDoubleWishbone
  // Modelica units
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  // FL sus parameters
  final replaceable parameter BobDynamics.Resources.Records.SUS.FrAxleBase FLDW;
  replaceable parameter BobDynamics.Resources.Records.MASSPROPS.FrUnsprung FrUnsprungMass;
  final replaceable parameter BobDynamics.Resources.Records.MASSPROPS.FrUCA FrUCAMass;
  final replaceable parameter BobDynamics.Resources.Records.MASSPROPS.FrLCA FrLCAMass;
  final replaceable parameter BobDynamics.Resources.Records.MASSPROPS.FrTie FrTieMass;
  parameter String side = "Left" annotation(
    choices(choice = "Left", choice = "Right"));
  // Parameters
  parameter SIunits.Position upper_fore_i[3] = FLDW.upper_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_aft_i[3] = FLDW.upper_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_fore_i[3] = FLDW.lower_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_aft_i[3] = FLDW.lower_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position upper_o[3] = FLDW.upper_outboard annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position lower_o[3] = FLDW.lower_outboard annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_i[3] = FLDW.tie_inboard annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position tie_o[3] = FLDW.tie_outboard annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position wheel_center[3] = FLDW.wheel_center annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle static_gamma = FLDW.static_gamma annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle static_alpha = FLDW.static_alpha annotation(
    Dialog(group = "Geometry"));
  // Visual parameters
  parameter SIunits.Length link_diameter = 0.025 annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  parameter SIunits.Length joint_diameter = 0.030 annotation(
    Dialog(tab = "Animation", group = "Sizing"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upper_i_frame annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lower_i_frame annotation(
    Placement(transformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a tie_i_frame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b upper_wishbone_frame annotation(
    Placement(transformation(origin = {10, 100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b lower_wishbone_frame annotation(
    Placement(transformation(origin = {10, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b midpoint_frame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{16, -16}, {-16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  // Upper wishbone
  Modelica.Mechanics.MultiBody.Joints.Revolute upper_inboard_joint(n = normalize(upper_fore_i - upper_aft_i), phi(each fixed = true, start = 0), stateSelect = StateSelect.always, animation = false, w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {60, 60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper_rigid_link(height = link_diameter, r = upper_o - (upper_fore_i + upper_aft_i)/2, width = link_diameter, animation = false) annotation(
    Placement(transformation(origin = {30, 60}, extent = {{10, -10}, {-10, 10}})));
  // Lower wisbone
  Modelica.Mechanics.MultiBody.Joints.Revolute lower_inboard_joint(n = normalize(lower_fore_i - lower_aft_i), phi(each fixed = true, start = 0), stateSelect = StateSelect.always, animation = false, w(start = 0, fixed = true)) annotation(
    Placement(transformation(origin = {60, -60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_rigid_link(height = link_diameter, r = lower_o - (lower_fore_i + lower_aft_i)/2, width = link_diameter, animation = false) annotation(
    Placement(transformation(origin = {30, -60}, extent = {{10, -10}, {-10, 10}})));
  // Upright
  BobDynamics.Vehicle.Chassis.Suspension.Linkages.Upright upright(lower = lower_o, upper = upper_o, tie = tie_o) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}})));
  // Tie rod
  Modelica.Mechanics.MultiBody.Joints.SphericalSpherical sphericalSpherical(rodLength = norm(tie_o - tie_i), sphereDiameter = joint_diameter, rodDiameter = link_diameter, m = FrTieMass.m, kinematicConstraint = false) annotation(
    Placement(transformation(origin = {40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  // Wheel mass + inertia
  Modelica.Mechanics.MultiBody.Parts.Body wheel_body(animation = true, r_CM = FrUnsprungMass.r_cm - wheel_center, m = FrUnsprungMass.m, I_11 = FrUnsprungMass.I[1, 1], I_22 = FrUnsprungMass.I[2, 2], I_33 = FrUnsprungMass.I[3, 3], I_21 = FrUnsprungMass.I[2, 1], I_31 = FrUnsprungMass.I[3, 1], I_32 = FrUnsprungMass.I[3, 2], useQuaternions = false, enforceStates = true, sphereDiameter = joint_diameter, cylinderDiameter = link_diameter) annotation(
    Placement(transformation(origin = {-20, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  // UCA mass + inertia
  Modelica.Mechanics.MultiBody.Parts.Body UCA_body(animation = true, r_CM = FrUCAMass.r_cm - FLDW.upper_outboard, m = FrUCAMass.m, I_11 = FrUCAMass.I[1, 1], I_22 = FrUCAMass.I[2, 2], I_33 = FrUCAMass.I[3, 3], I_21 = FrUCAMass.I[2, 1], I_31 = FrUCAMass.I[3, 1], I_32 = FrUCAMass.I[3, 2], useQuaternions = false, sphereDiameter = joint_diameter, cylinderDiameter = link_diameter) annotation(
    Placement(transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  // LCA mass + inertia
  Modelica.Mechanics.MultiBody.Parts.Body LCA_body(r_CM = FrLCAMass.r_cm - FLDW.lower_outboard, m = FrLCAMass.m, I_11 = FrLCAMass.I[1, 1], I_22 = FrLCAMass.I[2, 2], I_33 = FrLCAMass.I[3, 3], I_21 = FrLCAMass.I[2, 1], I_31 = FrLCAMass.I[3, 1], I_32 = FrLCAMass.I[3, 2], w_0_fixed = false, useQuaternions = false, sphereDiameter = joint_diameter, cylinderDiameter = link_diameter) annotation(
    Placement(transformation(origin = {-10, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  // Public steering interface
  Modelica.Blocks.Interfaces.RealInput steer_input annotation(
    Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {-66, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  BobDynamics.Vehicle.Chassis.Suspension.Joints.xyzSphericalCompliant upper_xyzSphericalCompliant(r_rel(start = {0, 0, 0}, each fixed = true), diameter = joint_diameter) annotation(
    Placement(transformation(origin = {10, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  BobDynamics.Vehicle.Chassis.Suspension.Joints.xyzSphericalCompliant lower_xyzSphericalCompliant(r_rel(start = {0, 0, 0}, each fixed = true), diameter = joint_diameter) annotation(
    Placement(transformation(origin = {10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
protected
  // Connect midpoint of kingpin to center of the wheel
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = wheel_center - (upper_o + lower_o)/2, animation = false) annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{10, -10}, {-10, 10}})));
  // Private steering interface
  Modelica.Mechanics.Translational.Sources.Position position(useSupport = true) annotation(
    Placement(transformation(origin = {74, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic_rack(n = {0, 1, 0}, useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  // ==================================================
  // === I dare you to find a better way to do this ===
  // ==================================================
  // Set gamma
  Modelica.Blocks.Sources.RealExpression static_gamma_source(y = static_gamma*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-40, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position x_angle(exact = true) annotation(
    Placement(transformation(origin = {-40, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_x(animation = false, n = {1, 0, 0}, phi(start = static_gamma*Modelica.Constants.pi/180), useAxisFlange = true) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}})));
  // Set toe (using alpha sign convention)
  Modelica.Blocks.Sources.RealExpression static_alpha_source(y = static_alpha*Modelica.Constants.pi/180) annotation(
    Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Position z_angle(exact = true) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute_z(animation = false, n = {0, 0, 1}, phi(start = static_alpha*Modelica.Constants.pi/180), useAxisFlange = true) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{10, -10}, {-10, 10}})));
  // Rod visualizers
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape upper_fore_shape(shapeType = "cylinder", r = upper_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(upper_i_frame.R, (upper_fore_i - upper_aft_i)/2), lengthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(upper_wishbone_frame.R, upper_o - upper_fore_i), length = norm(upper_o - upper_fore_i), width = link_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor) annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape upper_aft_shape(shapeType = "cylinder", r = upper_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(upper_i_frame.R, (upper_aft_i - upper_fore_i)/2), lengthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(upper_wishbone_frame.R, upper_o - upper_aft_i), length = norm(upper_o - upper_aft_i), width = link_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor) annotation(
    Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape lower_fore_shape(shapeType = "cylinder", r = lower_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(lower_i_frame.R, (lower_fore_i - lower_aft_i)/2), lengthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(lower_wishbone_frame.R, lower_o - lower_fore_i), length = norm(lower_o - lower_fore_i), width = link_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape lower_aft_shape(shapeType = "cylinder", r = lower_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(lower_i_frame.R, (lower_aft_i - lower_fore_i)/2), lengthDirection = Modelica.Mechanics.MultiBody.Frames.resolve1(lower_wishbone_frame.R, lower_o - lower_aft_i), length = norm(lower_o - lower_aft_i), width = link_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.RodColor) annotation(
    Placement(transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}})));
  // Joint visualizers
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape upper_fore_i_shape(shapeType = "sphere", r = upper_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(upper_i_frame.R, (upper_fore_i - upper_aft_i)/2), r_shape = joint_diameter/2*normalize(upper_aft_i - upper_fore_i), length = joint_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor) annotation(
    Placement(transformation(origin = {50, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape upper_aft_i_shape(shapeType = "sphere", r = upper_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(upper_i_frame.R, (upper_aft_i - upper_fore_i)/2), r_shape = joint_diameter/2*normalize(upper_aft_i - upper_fore_i), length = joint_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor) annotation(
    Placement(transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape lower_fore_i_shape(shapeType = "sphere", r = lower_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(lower_i_frame.R, (lower_fore_i - lower_aft_i)/2), r_shape = joint_diameter/2*normalize(lower_aft_i - lower_fore_i), length = joint_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor) annotation(
    Placement(transformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape lower_aft_i_shape(shapeType = "sphere", r = lower_i_frame.r_0 + Modelica.Mechanics.MultiBody.Frames.resolve1(lower_i_frame.R, (lower_aft_i - lower_fore_i)/2), r_shape = joint_diameter/2*normalize(lower_aft_i - lower_fore_i), length = joint_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor) annotation(
    Placement(transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}})));
equation
  if side == "Left" then
    connect(prismatic_rack.frame_b, sphericalSpherical.frame_a);
    connect(sphericalSpherical.frame_b, upright.tie_frame);
  else
    connect(prismatic_rack.frame_b, sphericalSpherical.frame_b);
    connect(sphericalSpherical.frame_a, upright.tie_frame);
  end if;
  connect(tie_i_frame, prismatic_rack.frame_a) annotation(
    Line(points = {{100, 0}, {80, 0}}));
  connect(upright.mass_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{10, 0}, {0, 0}}, color = {95, 95, 95}));
  connect(steer_input, position.s_ref) annotation(
    Line(points = {{120, -30}, {86, -30}}, color = {0, 0, 127}));
  connect(upper_i_frame, upper_inboard_joint.frame_a) annotation(
    Line(points = {{100, 60}, {70, 60}}));
  connect(upper_inboard_joint.frame_b, upper_rigid_link.frame_a) annotation(
    Line(points = {{50, 60}, {40, 60}}, color = {95, 95, 95}));
  connect(upper_rigid_link.frame_b, upper_xyzSphericalCompliant.frame_a) annotation(
    Line(points = {{20, 60}, {10, 60}, {10, 40}}, color = {95, 95, 95}));
  connect(upper_xyzSphericalCompliant.frame_b, upright.upper_frame) annotation(
    Line(points = {{10, 20}, {10, 10}}, color = {95, 95, 95}));
  connect(upper_wishbone_frame, upper_rigid_link.frame_b) annotation(
    Line(points = {{10, 100}, {10, 60}, {20, 60}}));
  connect(lower_i_frame, lower_inboard_joint.frame_a) annotation(
    Line(points = {{100, -60}, {70, -60}}));
  connect(lower_inboard_joint.frame_b, lower_rigid_link.frame_a) annotation(
    Line(points = {{50, -60}, {40, -60}}, color = {95, 95, 95}));
  connect(lower_rigid_link.frame_b, lower_xyzSphericalCompliant.frame_a) annotation(
    Line(points = {{20, -60}, {10, -60}, {10, -40}}, color = {95, 95, 95}));
  connect(lower_xyzSphericalCompliant.frame_b, upright.lower_frame) annotation(
    Line(points = {{10, -20}, {10, -10}}, color = {95, 95, 95}));
  connect(lower_wishbone_frame, lower_rigid_link.frame_b) annotation(
    Line(points = {{10, -100}, {10, -60}, {20, -60}}));
  connect(static_gamma_source.y, x_angle.phi_ref) annotation(
    Line(points = {{-40, 49}, {-40, 41}}, color = {0, 0, 127}));
  connect(x_angle.flange, revolute_x.axis) annotation(
    Line(points = {{-40, 20}, {-40, 10}}));
  connect(revolute_x.frame_b, revolute_z.frame_a) annotation(
    Line(points = {{-50, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(static_alpha_source.y, z_angle.phi_ref) annotation(
    Line(points = {{-70, 50}, {-70, 42}}, color = {0, 0, 127}));
  connect(z_angle.flange, revolute_z.axis) annotation(
    Line(points = {{-70, 20}, {-70, 10}}));
  connect(fixedTranslation.frame_b, revolute_x.frame_a) annotation(
    Line(points = {{-20, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(revolute_z.frame_b, midpoint_frame) annotation(
    Line(points = {{-80, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(wheel_body.frame_a, fixedTranslation.frame_b) annotation(
    Line(points = {{-20, -20}, {-20, 0}}, color = {95, 95, 95}));
  connect(prismatic_rack.support, position.support) annotation(
    Line(points = {{74, -6}, {74, -20}}, color = {0, 127, 0}));
  connect(position.flange, prismatic_rack.axis) annotation(
    Line(points = {{64, -30}, {62, -30}, {62, -6}}, color = {0, 127, 0}));
  connect(upper_rigid_link.frame_b, UCA_body.frame_a) annotation(
    Line(points = {{20, 60}, {0, 60}}, color = {95, 95, 95}));
  connect(lower_rigid_link.frame_b, LCA_body.frame_a) annotation(
    Line(points = {{20, -60}, {0, -60}}, color = {95, 95, 95}));
end FrRigidDoubleWishbone;

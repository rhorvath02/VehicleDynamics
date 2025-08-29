within Vehicle.Chassis.Suspension;
model DoubleWishbone
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Units.SI;
  parameter SI.Length wheel_diameter = 16*0.0254 "Nominal diameter of tire";
  parameter SI.Position contact_patch[3] = {0, 0.609600, 0} "Position of contact patch";
  parameter SI.Position upper_fore_i[3] = {0.086868, 0.215900, 0.200000} "Position of upper-fore-inboard pickup";
  parameter SI.Position upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} "Position of upper-aft-inboard pickup";
  parameter SI.Position upper_o[3] = {-0.006347, 0.523240, 0.287020} "Position of upper-outboard pickup";
  parameter SI.Position tie_i[3] = {0.041128, 0.215900, 0.117856} "Position of tie-inboard pickup";
  parameter SI.Position tie_o[3] = {0.056000, 0.532333, 0.164821} "Position of tie-outboard pickup";
  parameter SI.Position lower_fore_i[3] = {0.087376, 0.215900, 0.090000} "Position of lower-fore-inboard pickup";
  parameter SI.Position lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} "Position of lower-aft-inboard pickup";
  parameter SI.Position lower_o[3] = {0, 0.556499, 0.124998} "Position of lower-fore-inboard pickup";
  parameter SI.Mass small_mass = 1e-3 "Mass of bodies inserted for state selection";
  parameter SI.Length constraint_diameter = 0.25*0.0254 "Diameter of constraints (internal links)";
  parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links";
  parameter SI.Length joint_diameter = 0.825*0.0254 "Diameter of joints";
  final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
  final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
  final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-190, -190}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteUpper(n = normalize(upper_fore_i - upper_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
    Placement(transformation(origin = {150, 170}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteLower(n = normalize(lower_fore_i - lower_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
    Placement(transformation(origin = {150, -170}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationUpper(r = (upper_o - upper_fore_i) + r_upper_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {110, 170}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationLower(r = (lower_o - lower_fore_i) + r_lower_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {110, -170}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower(animation = false, r = r_lower_mount) annotation(
    Placement(transformation(origin = {190, -170}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedUpper(animation = false, r = r_upper_mount) annotation(
    Placement(transformation(origin = {190, 170}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  //Modelica.Mechanics.MultiBody.Frames.resolveRelative(v1=upper_o - lower_o, R1=Modelica.Mechanics.MultiBody.Frames.from_nxy(normalize(upper_o - lower_o)), R2=Modelica.Mechanics.MultiBody.Frames.from_nxy(normalize(tie_o - lower_o)));
  final parameter Frames.Orientation R_world = Frames.from_nxy({1, 0, 0}, {0, 1, 0});
  final parameter Real lower_to_upper_o[3] = upper_o - lower_o;
  final parameter Real n_x_initial[3] = normalize(lower_to_upper_o);
  final parameter Real n_y_initial[3] = normalize({lower_to_upper_o[1], 1, -(lower_to_upper_o[1]^2 + lower_to_upper_o[2])/lower_to_upper_o[3]});
  final parameter Frames.Orientation R_kingpin_initial = Frames.from_nxy(n_x_initial, n_y_initial);
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = upper_o - lower_o, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant sphericalCompliant1(mass = 1e-3)  annotation(
    Placement(transformation(origin = {90, -130}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = tie_o - lower_o, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {10, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation11(r = contact_patch - lower_o, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {10, -130}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Sources.ContactPatch contactPatch annotation(
    Placement(transformation(origin = {-70, -150}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower1(animation = false, r = contact_patch) annotation(
    Placement(transformation(origin = {-110, -170}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 2*0.0254, f = 1)  annotation(
    Placement(transformation(origin = {-110, -110}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant sphericalCompliant11(mass = 1e-3) annotation(
    Placement(transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant sphericalCompliant12(mass = 1e-3) annotation(
    Placement(transformation(origin = {-29, -151}, extent = {{-10, -10}, {10, 10}})));
  Sources.SteeringRack steeringRack annotation(
    Placement(transformation(origin = {-110, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower11(animation = false, r = tie_i) annotation(
    Placement(transformation(origin = {-150, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 0, f = 1) annotation(
    Placement(transformation(origin = {-150, 130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation12(r = tie_o - tie_i, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant sphericalCompliant111(mass = 1e-3) annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Spherical spherical(sphereDiameter = joint_diameter)  annotation(
    Placement(transformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(fixedTranslationUpper.frame_a, revoluteUpper.frame_b) annotation(
    Line(points = {{120, 170}, {140, 170}}, color = {95, 95, 95}));
  connect(fixedTranslationLower.frame_a, revoluteLower.frame_b) annotation(
    Line(points = {{120, -170}, {140, -170}}, color = {95, 95, 95}));
  connect(fixedLower.frame_b, revoluteLower.frame_a) annotation(
    Line(points = {{180, -170}, {160, -170}}, color = {95, 95, 95}));
  connect(fixedUpper.frame_b, revoluteUpper.frame_a) annotation(
    Line(points = {{180, 170}, {160, 170}}, color = {95, 95, 95}));
  connect(sphericalCompliant1.frame_a, fixedTranslationLower.frame_b) annotation(
    Line(points = {{90, -140}, {90, -170}, {100, -170}}, color = {95, 95, 95}));
  connect(sphericalCompliant1.frame_b, fixedTranslation.frame_a) annotation(
    Line(points = {{90, -120}, {90, -40}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, fixedTranslation1.frame_a) annotation(
    Line(points = {{90, -40}, {90, -70}, {20, -70}}, color = {95, 95, 95}));
  connect(fixedTranslation11.frame_a, fixedTranslation.frame_a) annotation(
    Line(points = {{10, -120}, {10, -100}, {90, -100}, {90, -40}}, color = {95, 95, 95}));
  connect(fixedLower1.frame_b, contactPatch.frame_a) annotation(
    Line(points = {{-110, -160}, {-110, -150}, {-80, -150}}, color = {95, 95, 95}));
  connect(sine.y, contactPatch.realInput) annotation(
    Line(points = {{-99, -110}, {-77, -110}, {-77, -140}}, color = {0, 0, 127}));
  connect(fixedTranslation.frame_b, sphericalCompliant11.frame_a) annotation(
    Line(points = {{90, -20}, {90, 40}}, color = {95, 95, 95}));
  connect(sphericalCompliant11.frame_b, fixedTranslationUpper.frame_b) annotation(
    Line(points = {{90, 60}, {90, 170}, {100, 170}}, color = {95, 95, 95}));
  connect(sphericalCompliant12.frame_b, fixedTranslation11.frame_b) annotation(
    Line(points = {{-18, -150}, {10, -150}, {10, -140}}, color = {95, 95, 95}));
  connect(sphericalCompliant12.frame_a, contactPatch.frame_b) annotation(
    Line(points = {{-38, -150}, {-60, -150}}, color = {95, 95, 95}));
  connect(fixedLower11.frame_b, steeringRack.frame_a) annotation(
    Line(points = {{-150, 80}, {-150, 90}, {-120, 90}}, color = {95, 95, 95}));
  connect(sine1.y, steeringRack.realInput) annotation(
    Line(points = {{-139, 130}, {-117, 130}, {-117, 100}}, color = {0, 0, 127}));
  connect(steeringRack.frame_b, spherical.frame_a) annotation(
    Line(points = {{-100, 90}, {-80, 90}}, color = {95, 95, 95}));
  connect(spherical.frame_b, fixedTranslation12.frame_a) annotation(
    Line(points = {{-60, 90}, {-50, 90}, {-50, 40}}, color = {95, 95, 95}));
  connect(fixedTranslation12.frame_b, sphericalCompliant111.frame_a) annotation(
    Line(points = {{-50, 20}, {-50, -70}, {-40, -70}}, color = {95, 95, 95}));
  connect(sphericalCompliant111.frame_b, fixedTranslation1.frame_b) annotation(
    Line(points = {{-20, -70}, {0, -70}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})));
end DoubleWishbone;

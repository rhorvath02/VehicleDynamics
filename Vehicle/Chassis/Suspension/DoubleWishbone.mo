within Vehicle.Chassis.Suspension;
model DoubleWishbone
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.Units.SI;

  parameter SI.Length wheel_diameter = 16*0.0254 "Nominal diameter of tire";
  parameter SI.Mass joint_mass = 1e-6 "Joint mass";
  parameter SI.TranslationalSpringConstant joint_radial_stiffness = 1e9 "Joint radial stiffness";
  parameter SI.TranslationalDampingConstant joint_radial_damping = 1e12 "Joint radial damping";
  
  // Node positions
  parameter SI.Position contact_patch[3] = {0, 0.609600, 0} "Position of contact patch";
  parameter SI.Position upper_fore_i[3] = {0.086868, 0.215900, 0.200000} "Position of upper-fore-inboard pickup";
  parameter SI.Position upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} "Position of upper-aft-inboard pickup";
  parameter SI.Position upper_o[3] = {-0.006347, 0.523240, 0.287020} "Position of upper-outboard pickup";
  parameter SI.Position tie_i[3] = {0.041128, 0.215900, 0.117856} "Position of tie-inboard pickup";
  parameter SI.Position tie_o[3] = {0.056000, 0.532333, 0.164821} "Position of tie-outboard pickup";
  parameter SI.Position lower_fore_i[3] = {0.087376, 0.215900, 0.090000} "Position of lower-fore-inboard pickup";
  parameter SI.Position lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} "Position of lower-aft-inboard pickup";
  parameter SI.Position lower_o[3] = {0, 0.556499, 0.124998} "Position of lower-fore-inboard pickup";
  
  // Animation parameters
  parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links" annotation(Dialog(tab="Animation"));
  parameter SI.Length joint_diameter = 0.825*0.0254 "Diameter of joints" annotation(Dialog(tab="Animation"));
  parameter SI.Length constraint_diameter = 0.25*0.0254 "Diameter of constraints (internal links)" annotation(Dialog(tab="Animation"));

  // Pre-computed vectors
  final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
  final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
  final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  // Define gravity
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-190, -190}, extent = {{-10, -10}, {10, 10}})));

  // ===========================
  // === Mechanical elements ===
  // ===========================

protected
  // Inboard pickups
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower(animation = false, r = r_lower_mount) annotation(
    Placement(transformation(origin = {190, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedUpper(animation = false, r = r_upper_mount) annotation(
    Placement(transformation(origin = {190, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  // Inboard joints
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteUpper(n = normalize(upper_fore_i - upper_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
    Placement(transformation(origin = {150, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteLower(n = normalize(lower_fore_i - lower_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
    Placement(transformation(origin = {150, -90}, extent = {{10, -10}, {-10, 10}})));

  // Linkages
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upperLink(r = (upper_o - upper_fore_i) + r_upper_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {110, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerLink(r = (lower_o - lower_fore_i) + r_lower_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {110, -90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tieRod(r = tie_o - tie_i, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {-10, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));

  // Internal links (wheel assembly)
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightKingpinLink(r = upper_o - lower_o, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightTieLink(r = tie_o - lower_o, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {10, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightContactPatchLink(r = contact_patch - lower_o, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {10, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));

public
  // Outboard joints
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant lowerOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping)  annotation(
    Placement(transformation(origin = {70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant upperOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // ======================================
  // === Sources, signals, and fixtures ===
  // ======================================

  // Contact patch
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedContactPatch(animation = false, r = contact_patch) annotation(
    Placement(transformation(origin = {-130, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine jounceSignal(amplitude = 3*0.0254, f = 1/10, startTime = 10)  annotation(
    Placement(transformation(origin = {-170, -50}, extent = {{-10, -10}, {10, 10}})));
  Sources.ContactPatch contactPatch annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant contactPatchJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {-30, -90}, extent = {{-10, -10}, {10, 10}})));
    
  // Steering rack
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedSteeringRack(animation = false, r = tie_i) annotation(
    Placement(transformation(origin = {-130, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine rackSignal(amplitude = 0, f = 1/10, startTime = 10) annotation(
    Placement(transformation(origin = {-170, 110}, extent = {{-10, -10}, {10, 10}})));
  Sources.SteeringRack steeringRack annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Joints.Spherical steeringInboardJoint(sphereDiameter = joint_diameter)  annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant steeringOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(upperLink.frame_a, revoluteUpper.frame_b) annotation(
    Line(points = {{120, 90}, {140, 90}}, color = {95, 95, 95}));
  connect(lowerLink.frame_a, revoluteLower.frame_b) annotation(
    Line(points = {{120, -90}, {140, -90}}, color = {95, 95, 95}));
  connect(fixedLower.frame_b, revoluteLower.frame_a) annotation(
    Line(points = {{180, -90}, {160, -90}}, color = {95, 95, 95}));
  connect(fixedUpper.frame_b, revoluteUpper.frame_a) annotation(
    Line(points = {{180, 90}, {160, 90}}, color = {95, 95, 95}));
  connect(lowerOutboardJoint.frame_a, lowerLink.frame_b) annotation(
    Line(points = {{70, -60}, {70, -90}, {100, -90}}, color = {95, 95, 95}));
  connect(lowerOutboardJoint.frame_b, uprightKingpinLink.frame_a) annotation(
    Line(points = {{70, -40}, {70, 0}}, color = {95, 95, 95}));
  connect(fixedContactPatch.frame_b, contactPatch.frame_a) annotation(
    Line(points = {{-120, -90}, {-80, -90}}, color = {95, 95, 95}));
  connect(jounceSignal.y, contactPatch.realInput) annotation(
    Line(points = {{-159, -50}, {-77, -50}, {-77, -80}}, color = {0, 0, 127}));
  connect(uprightKingpinLink.frame_b, upperOutboardJoint.frame_a) annotation(
    Line(points = {{70, 20}, {70, 40}}, color = {95, 95, 95}));
  connect(upperOutboardJoint.frame_b, upperLink.frame_b) annotation(
    Line(points = {{70, 60}, {70, 90}, {100, 90}}, color = {95, 95, 95}));
  connect(contactPatchJoint.frame_b, uprightContactPatchLink.frame_b) annotation(
    Line(points = {{-20, -90}, {10, -90}, {10, -60}}, color = {95, 95, 95}));
  connect(contactPatchJoint.frame_a, contactPatch.frame_b) annotation(
    Line(points = {{-40, -90}, {-60, -90}}, color = {95, 95, 95}));
  connect(fixedSteeringRack.frame_b, steeringRack.frame_a) annotation(
    Line(points = {{-120, 70}, {-100, 70}}, color = {95, 95, 95}));
  connect(rackSignal.y, steeringRack.realInput) annotation(
    Line(points = {{-159, 110}, {-97, 110}, {-97, 80}}, color = {0, 0, 127}));
  connect(steeringRack.frame_b, steeringInboardJoint.frame_a) annotation(
    Line(points = {{-80, 70}, {-60, 70}}, color = {95, 95, 95}));
  connect(steeringInboardJoint.frame_b, tieRod.frame_a) annotation(
    Line(points = {{-40, 70}, {-20, 70}}, color = {95, 95, 95}));
  connect(tieRod.frame_b, steeringOutboardJoint.frame_a) annotation(
    Line(points = {{0, 70}, {9, 70}, {9, 60}, {10, 60}}, color = {95, 95, 95}));
  connect(steeringOutboardJoint.frame_b, uprightTieLink.frame_b) annotation(
    Line(points = {{10, 40}, {10, 20}}, color = {95, 95, 95}));
  connect(uprightContactPatchLink.frame_a, uprightKingpinLink.frame_a) annotation(
    Line(points = {{10, -40}, {10, -20}, {70, -20}, {70, 0}}, color = {95, 95, 95}));
  connect(uprightTieLink.frame_a, uprightKingpinLink.frame_a) annotation(
    Line(points = {{10, 0}, {10, -20}, {70, -20}, {70, 0}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})));
end DoubleWishbone;

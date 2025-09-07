within Vehicle.Chassis.Suspension;

model DoubleWishboneV3
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.Units.SI;
  output SI.Position contact_patch_probe[3];
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
  parameter SI.Position pushrod_o[3] = {-0.01368377, 0.49897726, 0.30564884} "Position of pushrod outboard node";
  parameter SI.Position pushrod_i[3] = {-0.04415429, 0.39924560, 0.40746267} "Position of pushrod inboard node";
  // Animation parameters
  parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links" annotation(
    Dialog(tab = "Animation"));
  parameter SI.Length joint_diameter = 0.825*0.0254 "Diameter of joints" annotation(
    Dialog(tab = "Animation"));
  parameter SI.Length constraint_diameter = 0.25*0.0254 "Diameter of constraints (internal links)" annotation(
    Dialog(tab = "Animation"));
  // Pre-computed vectors
  final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
  final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
  final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  // Define gravity
  // ===========================
  // === Mechanical elements ===
  // ===========================
  // Outboard joints
  Chassis.Suspension.Joints.SphericalCompliant lowerOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Chassis.Suspension.Joints.SphericalCompliant upperOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  // ======================================
  // === Sources, signals, and fixtures ===
  // ======================================
  // Contact patch
  Sources.ContactPatch contactPatch annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}})));
  Chassis.Suspension.Joints.SphericalCompliant contactPatchJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {-30, -90}, extent = {{-10, -10}, {10, 10}})));
  // Steering rack
  Sources.SteeringRack steeringRack annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Spherical steeringInboardJoint(sphereDiameter = joint_diameter) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
  Chassis.Suspension.Joints.SphericalCompliant steeringOutboardJoint(mass = joint_mass, radial_stiffness = joint_radial_stiffness, radial_damping = joint_radial_damping) annotation(
    Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {200, 90}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a1 annotation(
    Placement(transformation(origin = {200, -90}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a2 annotation(
    Placement(transformation(origin = {200, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a3 annotation(
    Placement(transformation(origin = {0, -200}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput JounceInput annotation(
    Placement(transformation(origin = {-140, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-66, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput SteerInput annotation(
    Placement(transformation(origin = {-220, 96}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-66, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(fixedRotationAtFrame_a = true, c = 60e3, s_unstretched = norm({-0.04415429, 0.39924560, 0.40746267} - {-0.09276858, 0.24012839, 0.56990187}), diameter_a = constraint_diameter, diameter_b = constraint_diameter*1.5, width = joint_diameter)  annotation(
    Placement(transformation(origin = {160, 170}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a4 annotation(
    Placement(transformation(origin = {180, 200}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {66, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
protected
  // Inboard pickups
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
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightKingpinLink(r = upper_o - lower_o, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightTieLink(r = tie_o - lower_o, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {10, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation uprightContactPatchLink(r = contact_patch - lower_o, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {10, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerLink1(height = link_diameter, r = pushrod_o - upper_o, width = link_diameter, animation = false) annotation(
    Placement(transformation(origin = {90, 130}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant upperOutboardJoint1(mass = joint_mass, radial_damping = joint_radial_damping, radial_stiffness = joint_radial_stiffness) annotation(
    Placement(transformation(origin = {100, 170}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lowerLink11(height = link_diameter, r = pushrod_i -pushrod_o, width = link_diameter) annotation(
    Placement(transformation(origin = {130, 170}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
equation
  contact_patch_probe = contactPatchJoint.frame_b.r_0 - contactPatchJoint.frame_a.r_0;
  connect(upperLink.frame_a, revoluteUpper.frame_b) annotation(
    Line(points = {{120, 90}, {140, 90}}, color = {95, 95, 95}));
  connect(lowerLink.frame_a, revoluteLower.frame_b) annotation(
    Line(points = {{120, -90}, {140, -90}}, color = {95, 95, 95}));
  connect(lowerOutboardJoint.frame_a, lowerLink.frame_b) annotation(
    Line(points = {{70, -60}, {70, -90}, {100, -90}}, color = {95, 95, 95}));
  connect(lowerOutboardJoint.frame_b, uprightKingpinLink.frame_a) annotation(
    Line(points = {{70, -40}, {70, 0}}, color = {95, 95, 95}));
  connect(uprightKingpinLink.frame_b, upperOutboardJoint.frame_a) annotation(
    Line(points = {{70, 20}, {70, 40}}, color = {95, 95, 95}));
  connect(upperOutboardJoint.frame_b, upperLink.frame_b) annotation(
    Line(points = {{70, 60}, {70, 90}, {100, 90}}, color = {95, 95, 95}));
  connect(contactPatchJoint.frame_b, uprightContactPatchLink.frame_b) annotation(
    Line(points = {{-20, -90}, {10, -90}, {10, -60}}, color = {95, 95, 95}));
  connect(contactPatchJoint.frame_a, contactPatch.frame_b) annotation(
    Line(points = {{-40, -90}, {-60, -90}}, color = {95, 95, 95}));
  connect(steeringRack.frame_b, steeringInboardJoint.frame_a) annotation(
    Line(points = {{-90, 80}, {-70, 80}, {-70, 70}, {-60, 70}}, color = {95, 95, 95}));
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
  connect(revoluteUpper.frame_a, frame_a) annotation(
    Line(points = {{160, 90}, {200, 90}}, color = {95, 95, 95}));
  connect(revoluteLower.frame_a, frame_a1) annotation(
    Line(points = {{160, -90}, {200, -90}}, color = {95, 95, 95}));
  connect(steeringRack.frame_a, frame_a2) annotation(
    Line(points = {{-90, 100}, {-90, 140}, {180, 140}, {180, 0}, {200, 0}}, color = {95, 95, 95}));
  connect(contactPatch.frame_a, frame_a3) annotation(
    Line(points = {{-80, -90}, {-100, -90}, {-100, -140}, {0, -140}, {0, -200}}, color = {95, 95, 95}));
  connect(SteerInput, steeringRack.realInput) annotation(
    Line(points = {{-220, 96}, {-100, 96}}, color = {0, 0, 127}));
  connect(JounceInput, contactPatch.realInput) annotation(
    Line(points = {{-140, -220}, {-140, -60}, {-76, -60}, {-76, -80}}, color = {0, 0, 127}));
  connect(lowerLink1.frame_a, upperLink.frame_b) annotation(
    Line(points = {{90, 120}, {90, 90}, {100, 90}}, color = {95, 95, 95}));
  connect(lowerLink11.frame_a, upperOutboardJoint1.frame_a) annotation(
    Line(points = {{120, 170}, {110, 170}}, color = {95, 95, 95}));
  connect(upperOutboardJoint1.frame_b, lowerLink1.frame_b) annotation(
    Line(points = {{90, 170}, {90, 140}}, color = {95, 95, 95}));
  connect(lowerLink11.frame_b, springDamperParallel.frame_a) annotation(
    Line(points = {{140, 170}, {150, 170}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_b, frame_a4) annotation(
    Line(points = {{170, 170}, {180, 170}, {180, 200}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -200}, {200, 200}})));
end DoubleWishboneV3;

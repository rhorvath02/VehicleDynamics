within VehicleDynamics.TestVehicle.TestChassis.TestSuspension;

model TestFrDoubleWishboneBase
  // Modelica linalg
  import Modelica.Math.Vectors.normalize;
  // Custom linalg
  import VehicleDynamics.Utilities.Math.Vector.cross;
  import VehicleDynamics.Utilities.Math.Vector.dot;
  // Fixed inboard elements
  Modelica.Mechanics.MultiBody.Parts.Fixed upper_fore_i(r = hdpts.getRealArray1D("Front.left.upper.fore_i", 3)) annotation(
    Placement(transformation(origin = {90, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed upper_aft_i(r = hdpts.getRealArray1D("Front.left.upper.aft_i", 3)) annotation(
    Placement(transformation(origin = {90, 60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed lower_fore_i(r = hdpts.getRealArray1D("Front.left.lower.fore_i", 3)) annotation(
    Placement(transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed lower_aft_i(r = hdpts.getRealArray1D("Front.left.lower.aft_i", 3)) annotation(
    Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed tie_i(r = hdpts.getRealArray1D("Front.left.tie.inboard", 3)) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  // Double wishbone
  VehicleDynamics.Vehicle.Chassis.Suspension.FrDoubleWishboneBase frDoubleWishboneBase annotation(
    Placement(transformation(origin = {1.42109e-14, 0}, extent = {{-45, -45}, {45, 45}})));
  // Read JSON
  parameter String hdpts_path = Modelica.Utilities.Files.loadResource("modelica://VehicleDynamics/Resources/JSONs/SUS/hdpts.json") "File path to hdpts json" annotation(
    Dialog(group = "File Paths"));
  // Align wheel to desired origin
  final parameter Real[3] kp_midpoint = (hdpts.getRealArray1D("Front.left.upper.outboard", 3) + hdpts.getRealArray1D("Front.left.lower.outboard", 3))/2;
  final parameter Real[3] kp_mid_to_wheel_center = hdpts.getRealArray1D("Front.left.tire.wheel_center", 3) - kp_midpoint;
  inner ExternData.JSONFile hdpts(fileName = hdpts_path) annotation(
    Placement(transformation(origin = {-210, 90}, extent = {{10, -10}, {-10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-210, -90}, extent = {{-10, -10}, {10, 10}})));
  // Zero torque application to fully define system
  Vehicle.Chassis.Tires.MF5p2Tire mF5p2Tire annotation(
    Placement(transformation(origin = {-160, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 0, startTime = 0) annotation(
    Placement(transformation(origin = {-210, 32}, extent = {{-10, -10}, {10, 10}})));

  final parameter Real static_alpha = hdpts.getReal("Front.left.tire.static_alpha");
  final parameter Real static_gamma = hdpts.getReal("Front.left.tire.static_gamma");

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = kp_mid_to_wheel_center) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteX(n = {1, 0, 0}, phi(start = static_gamma * Modelica.Constants.pi / 180, fixed = true), useAxisFlange = true) annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Rotational.Sources.Position position annotation(
    Placement(transformation(origin = {-100, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression static_gamma_source(y = static_gamma*Modelica.Constants.pi/180)  annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revoluteY(useAxisFlange = true, n = {0, 0, 1}, phi(start = static_alpha*Modelica.Constants.pi/180, fixed = true))  annotation(
    Placement(transformation(origin = {-130, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.Rotational.Sources.Position position1 annotation(
    Placement(transformation(origin = {-130, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.RealExpression static_alpha_source(y = static_alpha*Modelica.Constants.pi/180)  annotation(
    Placement(transformation(origin = {-130, 60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.GroundPhysics groundPhysics annotation(
    Placement(transformation(origin = {-160, -40}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(upper_fore_i.frame_b, frDoubleWishboneBase.upper_fore_i_frame) annotation(
    Line(points = {{80, 90}, {0, 90}, {0, 46}}, color = {95, 95, 95}));
  connect(upper_aft_i.frame_b, frDoubleWishboneBase.upper_aft_i_frame) annotation(
    Line(points = {{80, 60}, {30, 60}, {30, 46}}, color = {95, 95, 95}));
  connect(lower_fore_i.frame_b, frDoubleWishboneBase.lower_fore_i_frame) annotation(
    Line(points = {{80, 30}, {46, 30}}, color = {95, 95, 95}));
  connect(lower_aft_i.frame_b, frDoubleWishboneBase.lower_aft_i_frame) annotation(
    Line(points = {{80, 0}, {46, 0}}, color = {95, 95, 95}));
  connect(tie_i.frame_b, frDoubleWishboneBase.tie_i_frame) annotation(
    Line(points = {{80, -30}, {46, -30}}, color = {95, 95, 95}));
  connect(ramp.y, mF5p2Tire.hub_torque) annotation(
    Line(points = {{-199, 32}, {-162.5, 32}, {-162.5, 12}}, color = {0, 0, 127}));
  connect(frDoubleWishboneBase.midpoint_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{-44, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, revoluteX.frame_a) annotation(
    Line(points = {{-80, 0}, {-90, 0}}, color = {95, 95, 95}));
  connect(position.flange, revoluteX.axis) annotation(
    Line(points = {{-100, 20}, {-100, 10}}));
  connect(static_gamma_source.y, position.phi_ref) annotation(
    Line(points = {{-100, 49}, {-100, 42}}, color = {0, 0, 127}));
  connect(revoluteX.frame_b, revoluteY.frame_a) annotation(
    Line(points = {{-110, 0}, {-120, 0}}, color = {95, 95, 95}));
  connect(mF5p2Tire.chassis_frame, revoluteY.frame_b) annotation(
    Line(points = {{-150, 0}, {-140, 0}}, color = {95, 95, 95}));
  connect(revoluteY.axis, position1.flange) annotation(
    Line(points = {{-130, 10}, {-130, 20}}));
  connect(static_alpha_source.y, position1.phi_ref) annotation(
    Line(points = {{-130, 50}, {-130, 42}}, color = {0, 0, 127}));
  connect(groundPhysics.frame_b, mF5p2Tire.cp_frame) annotation(
    Line(points = {{-160, -30}, {-160, -10}}, color = {95, 95, 95}));
  connect(groundPhysics.frame_a, world.frame_b) annotation(
    Line(points = {{-170, -40}, {-190, -40}, {-190, -90}, {-200, -90}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-220, 100}, {100, -100}})));
end TestFrDoubleWishboneBase;
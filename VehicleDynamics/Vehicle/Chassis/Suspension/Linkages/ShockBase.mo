within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model ShockBase
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  // Kinematic elements
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  // User parameters
  parameter Boolean animation = true "Show sphere in animation";
  parameter SIunits.TranslationalSpringConstant spring_constant = 62e3 "Shock stiffness";
  parameter SIunits.TranslationalDampingConstant damping_constant = 1e3 "Shock damping";
  parameter SIunits.Length free_length = 0.2 "Spring free length";
  parameter SIunits.Length compressed_length = 0.1 "Spring compressed length (1UP)";
  parameter SIunits.Length diameter = 0.825*0.0254 "Diameter of bearing";
  parameter SIunits.Mass mass = 2.48e-3 "Mass of bearing";
  Real dampingForce = 0;
  Real r_rel[3];
  // Visualization
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_a(shapeType = "sphere", length = diameter, width = diameter, height = diameter, animation = animation, r_shape = {-diameter/2, 0, 0}) annotation(
    Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_b(animation = animation, height = diameter, length = diameter, r_shape = {-diameter/2, 0, 0}, shapeType = "sphere", width = diameter) annotation(
    Placement(transformation(origin = {70, 20}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  // Spring
  Modelica.Mechanics.MultiBody.Forces.Spring spring(c = spring_constant, s_unstretched = free_length) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  // Damper
  Modelica.Mechanics.MultiBody.Forces.Damper damper(d = damping_constant)  annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = mass, animation = false) annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}})));
  // Add DOF
  Modelica.Mechanics.MultiBody.Joints.Prismatic spring_dir(n = {0, 0, 1}, s(start = compressed_length, fixed = true)) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
equation
  r_rel = frame_b.r_0 - frame_a.r_0;
  connect(shape_a.frame_a, frame_a) annotation(
    Line(points = {{-80, 20}, {-80, 19.5}, {-100, 19.5}, {-100, 0}}, color = {95, 95, 95}));
  connect(shape_b.frame_a, frame_b) annotation(
    Line(points = {{80, 20}, {100, 20}, {100, 0}}, color = {95, 95, 95}));
  connect(body.frame_a, frame_a) annotation(
    Line(points = {{-80, -50}, {-100, -50}, {-100, 0}}, color = {95, 95, 95}));
  connect(frame_a, spring.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(spring.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(spring_dir.frame_a, frame_a) annotation(
    Line(points = {{-10, 30}, {-40, 30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(spring_dir.frame_b, frame_b) annotation(
    Line(points = {{10, 30}, {40, 30}, {40, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(damper.frame_a, frame_a) annotation(
    Line(points = {{-10, -30}, {-40, -30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(damper.frame_b, frame_b) annotation(
    Line(points = {{10, -30}, {40, -30}, {40, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 10}, {100, -10}}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {192, 192, 192}), Ellipse(extent = {{-60, -60}, {60, 60}}, fillPattern = FillPattern.Solid, fillColor = {192, 192, 192}, lineColor = {0, 0, 0}, closure = EllipseClosure.Radial, startAngle = 60, endAngle = 300), Ellipse(extent = {{-44, -44}, {44, 44}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, closure = EllipseClosure.Radial, startAngle = 55, endAngle = 305), Ellipse(extent = {{-44, -44}, {44, 44}}, startAngle = 60, endAngle = 300, lineColor = {0, 0, 0}, closure = EllipseClosure.None), Ellipse(extent = {{-26, 26}, {26, -26}}, fillPattern = FillPattern.Sphere, fillColor = {192, 192, 192}, lineColor = {0, 0, 0}), Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}, color = {255, 0, 0}), Text(extent = {{-150, 110}, {150, 70}}, textString = "%name", textColor = {0, 0, 255})}));
end ShockBase;
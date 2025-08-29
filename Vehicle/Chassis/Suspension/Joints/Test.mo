within Vehicle.Chassis.Suspension.Joints;

model Test
  SphericalCompliant sphericalCompliant(diameter = 0.050, radial_stiffness = 1e6) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Blocks.Sources.Sine forceX(amplitude = 1e6/5, f = 1) annotation(
    Placement(transformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine forceY(amplitude = 0, f = 1) annotation(
    Placement(transformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine forceZ(amplitude = 0, f = 1) annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, g = 0) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, 1}) annotation(
    Placement(transformation(origin = {60, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
equation
  connect(forceX.y, force.force[1]) annotation(
    Line(points = {{-78, 50}, {-60, 50}, {-60, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(forceY.y, force.force[2]) annotation(
    Line(points = {{-78, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(forceZ.y, force.force[3]) annotation(
    Line(points = {{-78, -50}, {-60, -50}, {-60, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(fixed.frame_b, fixedTranslation.frame_a) annotation(
    Line(points = {{80, 0}, {70, 0}}, color = {95, 95, 95}));
  connect(force.frame_b, sphericalCompliant.frame_a) annotation(
    Line(points = {{-20, 0}, {0, 0}}, color = {95, 95, 95}));
  connect(sphericalCompliant.frame_b, fixedTranslation.frame_b) annotation(
    Line(points = {{20, 0}, {50, 0}}, color = {95, 95, 95}));
end Test;

within VehicleDynamics.Vehicle.Chassis.Suspension.Sources;

model ContactPatch
  import Modelica.Units.SI;
  
  extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  
  Modelica.Mechanics.Translational.Sources.Position position(exact = false) annotation(
    Placement(transformation(origin = {0, 54}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Blocks.Interfaces.RealInput realInput annotation(
    Placement(transformation(origin = {-80, 120}, extent = {{20, -20}, {-20, 20}}, rotation = 90), iconTransformation(origin = {-66, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticX(animation = false) annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticY(n = {0, 1, 0}, animation = false) annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticZ(n = {0, 0, 1}, useAxisFlange = true, animation = true) annotation(
    Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add annotation(
    Placement(transformation(origin = {-50, 54}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y = 0) annotation(
    Placement(transformation(origin = {-90, 48}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(position.flange, prismaticZ.axis) annotation(
    Line(points = {{10, 54}, {48, 54}, {48, 6}}, color = {0, 127, 0}));
  connect(prismaticX.frame_b, prismaticY.frame_a) annotation(
    Line(points = {{-30, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(prismaticY.frame_b, prismaticZ.frame_a) annotation(
    Line(points = {{10, 0}, {30, 0}}, color = {95, 95, 95}));
  connect(frame_a, prismaticX.frame_a) annotation(
    Line(points = {{-100, 0}, {-50, 0}}));
  connect(realExpression.y, add.u2) annotation(
    Line(points = {{-78, 48}, {-62, 48}}, color = {0, 0, 127}));
  connect(add.y, position.s_ref) annotation(
    Line(points = {{-38, 54}, {-12, 54}}, color = {0, 0, 127}));
  connect(realInput, add.u1) annotation(
    Line(points = {{-80, 120}, {-80, 60}, {-62, 60}}, color = {0, 0, 127}));
  connect(prismaticZ.frame_b, frame_b) annotation(
    Line(points = {{50, 0}, {100, 0}}, color = {95, 95, 95}));
end ContactPatch;
within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model TabularSpring
  import VehicleDynamics.Utilities.Math.Vector.dot;
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // User parameters
  parameter Boolean animation = true "Enable animation" annotation(Dialog(group="Animation"));
  parameter Real[:, 2] force_curve = [0, 0; 1, 0] annotation(Dialog(group="Initialization"));
  parameter SIunits.Length free_length "Free spring length" annotation(Dialog(group="Initialization"));
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Spring interpolation
  Modelica.Blocks.Tables.CombiTable1D spring_table(table = force_curve, columns = {2}, smoothness = Modelica.Blocks.Types.Smoothness.ContinuousDerivative, extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.Internal.BasicForce basic_force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameAB.world) annotation(
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  
  Modelica.Mechanics.MultiBody.Interfaces.ZeroPosition zero_position annotation(
    Placement(transformation(origin = {30, -20},extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Blocks.Sources.RealExpression force_expression[3](y = -force*e_spring)  annotation(
    Placement(transformation(origin = {-30, -20}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Position position(useSupport = true)  annotation(
    Placement(transformation(origin = {-4, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Ramp ramp(height = 10, duration = 10)  annotation(
    Placement(transformation(origin = {-70, 90}, extent = {{-10, -10}, {10, 10}})));

protected
  Real r_rel[3];
  Real length;
  Real e_spring[3];
  Real force;

equation
  // Spring kinematics
  r_rel = frame_b.r_0 - frame_a.r_0;
  length = Modelica.Math.Vectors.norm(r_rel) - free_length;

  // Spring axis
  if length > 1e-10 then
    e_spring = r_rel / length;
  else
    e_spring = {1, 0, 0};
  end if;
  
  // Spring force generation
  spring_table.u[1] = length;
  force = spring_table.y[1];
  
  connect(frame_a, basic_force.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(basic_force.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(zero_position.frame_resolve, basic_force.frame_resolve) annotation(
    Line(points = {{20, -20}, {4, -20}, {4, -10}}, color = {95, 95, 95}));
  connect(prismatic.frame_a, frame_a) annotation(
    Line(points = {{-10, 30}, {-40, 30}, {-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, frame_b) annotation(
    Line(points = {{10, 30}, {40, 30}, {40, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(force_expression.y, basic_force.force) annotation(
    Line(points = {{-18, -20}, {-6, -20}, {-6, -12}}, color = {0, 0, 127}, thickness = 0.5));
  connect(position.flange, prismatic.support) annotation(
    Line(points = {{-4, 60}, {-4, 36}}, color = {0, 127, 0}));
  connect(ramp.y, position.s_ref) annotation(
    Line(points = {{-58, 90}, {-4, 90}, {-4, 82}}, color = {0, 0, 127}));
  connect(position.support, prismatic.axis) annotation(
    Line(points = {{6, 70}, {8, 70}, {8, 36}}, color = {0, 127, 0}));
  annotation(experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end TabularSpring;

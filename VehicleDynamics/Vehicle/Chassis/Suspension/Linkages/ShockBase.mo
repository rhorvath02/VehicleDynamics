within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model ShockBase
  import VehicleDynamics.Utilities.Math.Vector.dot;
  import Modelica.Mechanics.MultiBody.Frames;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // User parameters
  parameter Boolean animation = true "Show sphere in animation";
  parameter Real[:, 2] force_curve = [0, 0; 1, 500];
  
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
    
protected
  Real L;
  Real e[3];
  Real r_rel[3];
  Real v_rel;
  Real F;
equation
  r_rel = frame_b.r_0 - frame_a.r_0;
  L = Modelica.Math.Vectors.norm(r_rel);
  e = if L > 1e-6 then r_rel/L else {1, 0, 0};
  v_rel = dot(der(frame_b.r_0) - der(frame_a.r_0), e);
  spring_table.u[1] = L;
  F = spring_table.y[1] + 0*v_rel;
  basic_force.force = F*e;
  connect(frame_a, basic_force.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(basic_force.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(zero_position.frame_resolve, basic_force.frame_resolve) annotation(
    Line(points = {{20, -20}, {4, -20}, {4, -10}}, color = {95, 95, 95}));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 10}, {100, -10}}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {192, 192, 192}), Ellipse(extent = {{-60, -60}, {60, 60}}, fillPattern = FillPattern.Solid, fillColor = {192, 192, 192}, lineColor = {0, 0, 0}, closure = EllipseClosure.Radial, startAngle = 60, endAngle = 300), Ellipse(extent = {{-44, -44}, {44, 44}}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, closure = EllipseClosure.Radial, startAngle = 55, endAngle = 305), Ellipse(extent = {{-44, -44}, {44, 44}}, startAngle = 60, endAngle = 300, lineColor = {0, 0, 0}, closure = EllipseClosure.None), Ellipse(extent = {{-26, 26}, {26, -26}}, fillPattern = FillPattern.Sphere, fillColor = {192, 192, 192}, lineColor = {0, 0, 0}), Line(points = {{-100, 0}, {-58, 0}, {-43, -30}, {-13, 30}, {17, -30}, {47, 30}, {62, 0}, {100, 0}}, color = {255, 0, 0}), Text(extent = {{-150, 110}, {150, 70}}, textString = "%name", textColor = {0, 0, 255})}));
end ShockBase;
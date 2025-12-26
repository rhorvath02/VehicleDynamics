within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model TabularSpring "Tabular translational spring with optional mass"
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.TranslationalSpringConstant force_table[:,2] "Table of Force vs Compression (change in length)" annotation(
    Dialog(group = "Spring Parameters"));
  parameter SIunits.Length free_length "Free length of spring" annotation(Dialog(group = "Spring Parameters"));
  parameter SIunits.Mass spring_mass = 0 "Spring mass" annotation(Dialog(group = "Spring Parameters"));
  parameter SIunits.Position start_point[3] "Initial point of spring" annotation(Dialog(group = "Spring Parameters"));
  parameter SIunits.Position end_point[3] "Terminal point of spring" annotation(Dialog(group = "Spring Parameters"));
  
  parameter SIunits.Length spring_diameter "Diameter of smallest possible cylinder enclosing spring" annotation(Dialog(group = "Animation"));
  parameter Real eps = 1e-12 "regularization (m)" annotation(Dialog(group="Numerical"));
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Force generation
  Modelica.Mechanics.Translational.Sources.Force2 force annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = false, n = normalize(end_point - start_point), useAxisFlange = true) annotation(Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}))); 
  
  // Visualization
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape(
    shapeType = "spring",
    R = Modelica.Mechanics.MultiBody.Frames.nullRotation(),
    r = prismatic.frame_a.r_0,
    lengthDirection = e_spring,
    widthDirection  = w_spring,
    length = spring_length,
    width  = spring_diameter / 2,
    height = spring_diameter/10,
    extra  = 6) annotation(Placement(transformation(origin = {90, -90}, extent = {{-10, -10}, {10, 10}})));
    
protected
  Real[3] r_rel;
  Real[3] e_spring;
  Real spring_length;
  
  // Orientation
  Real[3] ref;
  Real[3] w_spring;
  
  Real defl;
  Real defl_abs;
  Real sgn;
  Real F_spring;

  Modelica.Blocks.Sources.RealExpression realExpression(y = defl_abs) annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1D combiTable1D(columns = {2}, extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, table = force_table) annotation(
    Placement(transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}})));initial equation
  norm(prismatic.frame_b.r_0 - prismatic.frame_a.r_0) = free_length;

equation
  // Relative vector along prismatic element
  r_rel = prismatic.frame_b.r_0 - prismatic.frame_a.r_0;
  spring_length = norm(r_rel);
  e_spring = if spring_length > 1e-9 then r_rel/spring_length else {1, 0, 0};
  ref = if abs(e_spring[3]) < 0.9 then {0, 0, 1} else {1, 0, 0};
  w_spring = normalize(cross(ref, e_spring));
  defl = free_length - spring_length;
  
  // Smooth abs and smooth sign
  defl_abs = sqrt(defl*defl + eps*eps);
  sgn = defl/defl_abs;
  
  // Apply sign
  F_spring = sgn*combiTable1D.y[1];
  force.f = F_spring;
  connect(frame_a, prismatic.frame_a) annotation(
    Line(points = {{-100, 0}, {-8, 0}}));
  connect(prismatic.frame_b, frame_b) annotation(
    Line(points = {{12, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(realExpression.y, combiTable1D.u[1]) annotation(
    Line(points = {{-79, 90}, {-62, 90}}, color = {0, 0, 127}));
  connect(force.flange_a, prismatic.support) annotation(
    Line(points = {{-10, 30}, {-20, 30}, {-20, 6}, {-2, 6}}, color = {0, 127, 0}));
  connect(force.flange_b, prismatic.axis) annotation(
    Line(points = {{10, 30}, {20, 30}, {20, 6}, {10, 6}}, color = {0, 127, 0}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end TabularSpring;
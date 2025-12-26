within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model TabularSpringDamperParallel
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Geometry
  parameter SIunits.Position start_point[3] "Initial point of spring" annotation(Dialog(group = "Geometry"));
  parameter SIunits.Position end_point[3] "Terminal point of spring" annotation(Dialog(group = "Geometry"));
  
  // Spring params
  parameter SIunits.TranslationalSpringConstant spring_table[:,2] "Table of force vs compression (change in length)" annotation(Dialog(group = "Spring Parameters"));
  parameter SIunits.Length free_length "Free length of spring" annotation(Dialog(group = "Spring Parameters"));
  
  // Damper params
  parameter SIunits.TranslationalDampingConstant damper_table[:,2] "Table of force vs relative velocity" annotation(Dialog(group="Damper Parameters"));
  
  // Visual params
  parameter SIunits.Length spring_diameter "Diameter of smallest possible cylinder enclosing spring" annotation(Dialog(group = "Animation"));
  
  // Numerical
  parameter Real eps = 1e-12 "regularization (m)" annotation(Dialog(group="Numerical"));
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Force generation
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = false, n = normalize(end_point - start_point), useAxisFlange = true, s(start = norm(end_point - start_point), fixed = true)) annotation(Placement(transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}))); 
  Modelica.Mechanics.Translational.Sources.Force2 force annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}})));
  
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
    
public
  Real[3] r_rel;
  Real[3] e_spring;
  Real spring_length;
  
  // Orientation
  Real[3] ref;
  Real[3] w_spring;
  
  Real defl;
  Real defl_abs;
  Real vel;
  Real sgn;
  Real F_spring;
  Real F_damper;
  
  // Spring force interpolation
  Modelica.Blocks.Sources.RealExpression spring_source(y = defl_abs) annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1D spring_combi_table_1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = spring_table)
    annotation(Placement(transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}})));

  // Damper force interpolation
  Modelica.Blocks.Sources.RealExpression damper_source(y = vel) annotation(
    Placement(transformation(origin = {-90, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1D damper_combi_table_1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = damper_table)
    annotation(Placement(transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape inner_cylinder(height = spring_diameter/4, length = norm(end_point - start_point)*3/4, lengthDirection = normalize(end_point - start_point), shapeType = "cylinder", width = spring_diameter/4) annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape outer_cylinder(height = spring_diameter/2, length = norm(end_point - start_point)*3/4, lengthDirection = -normalize(end_point - start_point), shapeType = "cylinder", width = spring_diameter/2) annotation(
    Placement(transformation(origin = {70, 30}, extent = {{10, -10}, {-10, 10}})));
initial equation
  norm(prismatic.frame_b.r_0 - prismatic.frame_a.r_0) = free_length;

equation
  // Relative vector
  r_rel = prismatic.frame_b.r_0 - prismatic.frame_a.r_0;
  
  // Extract desired quantities
  spring_length = norm(r_rel);
  
  if spring_length > 1e-9 then
    e_spring = r_rel / spring_length;
  else
    e_spring = {1, 0, 0};
  end if;
  
  ref = if abs(e_spring[3]) < 0.9 then {0, 0, 1} else {1, 0, 0};
  w_spring = normalize(cross(ref, e_spring));
  defl = free_length - spring_length;
  vel = prismatic.v;
  
  // Smooth abs and smooth sign
  defl_abs = sqrt(defl*defl + eps*eps);
  sgn = defl / defl_abs;
  
  // Apply sign
  F_spring = sgn * spring_combi_table_1D.y[1];
  F_damper = -1 * damper_combi_table_1D.y[1];
  
  force.f = F_spring + F_damper;
  
  connect(frame_a, prismatic.frame_a) annotation(
    Line(points = {{-100, 0}, {-8, 0}}));
  connect(prismatic.frame_b, frame_b) annotation(
    Line(points = {{12, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(force.flange_a, prismatic.support) annotation(
    Line(points = {{-10, 30}, {-20, 30}, {-20, 6}, {-2, 6}}, color = {0, 127, 0}));
  connect(force.flange_b, prismatic.axis) annotation(
    Line(points = {{10, 30}, {20, 30}, {20, 6}, {10, 6}}, color = {0, 127, 0}));
  connect(spring_source.y, spring_combi_table_1D.u[1]) annotation(
    Line(points = {{-78, 90}, {-62, 90}}, color = {0, 0, 127}));
  connect(damper_source.y, damper_combi_table_1D.u[1]) annotation(
    Line(points = {{-78, 60}, {-62, 60}}, color = {0, 0, 127}));
  connect(inner_cylinder.frame_a, frame_a) annotation(
    Line(points = {{-80, 30}, {-90, 30}, {-90, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(outer_cylinder.frame_a, frame_b) annotation(
    Line(points = {{80, 30}, {90, 30}, {90, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end TabularSpringDamperParallel;

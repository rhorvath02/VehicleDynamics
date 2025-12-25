within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model TabularDamper "Tabular translational damper with velocity-force curve"
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.TranslationalDampingConstant force_table[:,2]
    "Table of Force vs Relative Velocity (m/s, N)"
    annotation(Dialog(group = "Damper Parameters"));
  parameter SIunits.Length initial_length "Initial length of damper" 
    annotation(Dialog(group = "Damper Parameters"));
  parameter SIunits.Mass damper_mass = 0
    "Optional damper mass (unused, placeholder)"
    annotation(Dialog(group = "Damper Parameters"));
  parameter SIunits.Position start_point[3]
    "Initial point of damper"
    annotation(Dialog(group = "Damper Parameters"));
  parameter SIunits.Position end_point[3]
    "Terminal point of damper"  
    annotation(Dialog(group = "Damper Parameters"));
  
  parameter SIunits.Length outer_diameter=0.008
    "Diameter of smallest possible cylinder which can fully enclose the damper"
    annotation(Dialog(group = "Animation"));
    
  parameter SIunits.Length inner_diameter=0.004
    "Diameter of smallest possible cylinder (by volume) which can enclose any part of the damper"
    annotation(Dialog(group = "Animation"));
    
  parameter Real eps = 1e-12 "regularization (m/s)";
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0},
      extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-100, 0},
      extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0},
      extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {100, 0},
      extent = {{-16, -16}, {16, 16}})));
  
  // Force generation
  Modelica.Mechanics.Translational.Sources.Force2 force annotation(
    Placement(transformation(origin = {0, 30},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(
    animation = false,
    n = normalize(end_point - start_point),
    useAxisFlange = true, s(start = initial_length, fixed = true))
    annotation(Placement(transformation(origin = {2, 0},
      extent = {{-10, -10}, {10, 10}}))); 
  
    Modelica.Mechanics.MultiBody.Visualizers.FixedShape inner_cylinder(shapeType = "cylinder", r_shape = (end_point - start_point)*1/2, lengthDirection = normalize(end_point - start_point), length = norm(end_point - start_point)*3/4, width = inner_diameter, height = inner_diameter)  annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sensors.RelSpeedSensor relSpeedSensor annotation(
    Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
protected
  Real[3] r_rel;
  Real[3] v_rel;
  Real[3] e_damper;
  Real damper_length;
  
  // Orientation
  Real[3] ref;
  Real[3] w_damper;
  
  Real v_axial;
  Real v_abs;
  Real sgn;
  Real F_damper;
  
  Modelica.Blocks.Sources.RealExpression realExpression(y = v_abs) annotation(
    Placement(transformation(origin = {-90, 90},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1D combiTable1D(
    columns = {2},
    extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
    smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
    table = force_table)
    annotation(Placement(transformation(origin = {-50, 90},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape outer_cylinder(height = outer_diameter, length = -norm(end_point - start_point)*3/4, lengthDirection = -normalize(end_point - start_point), r_shape = -(end_point - start_point)*1/2, shapeType = "cylinder", width = outer_diameter) annotation(
    Placement(transformation(origin = {70, 30}, extent = {{10, -10}, {-10, 10}})));

equation
// Relative kinematics along damper axis
  r_rel = prismatic.frame_b.r_0 - prismatic.frame_a.r_0;
  v_rel = der(r_rel);
  damper_length = norm(r_rel);
  e_damper = if damper_length > 1e-9 then r_rel/damper_length else {1, 0, 0};
  ref = if abs(e_damper[3]) < 0.9 then {0, 0, 1} else {1, 0, 0};
  w_damper = normalize(cross(ref, e_damper));
// Axial velocity
  v_axial = v_rel*e_damper;
// Smooth abs and sign
  v_abs = sqrt(v_axial*v_axial + eps*eps);
  sgn = v_axial / v_abs;
// Table lookup + sign
  F_damper = sgn*combiTable1D.y[1];
  force.f = F_damper;
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
  connect(relSpeedSensor.flange_b, force.flange_a) annotation(
    Line(points = {{-10, 50}, {-20, 50}, {-20, 30}, {-10, 30}}, color = {0, 127, 0}));
  connect(relSpeedSensor.flange_a, force.flange_b) annotation(
    Line(points = {{10, 50}, {20, 50}, {20, 30}, {10, 30}}, color = {0, 127, 0}));
  connect(inner_cylinder.frame_a, frame_a) annotation(
    Line(points = {{-80, 30}, {-100, 30}, {-100, 0}}, color = {95, 95, 95}));
  connect(outer_cylinder.frame_a, frame_b) annotation(
    Line(points = {{80, 30}, {100, 30}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1,
      Tolerance = 1e-06, Interval = 0.002));
end TabularDamper;

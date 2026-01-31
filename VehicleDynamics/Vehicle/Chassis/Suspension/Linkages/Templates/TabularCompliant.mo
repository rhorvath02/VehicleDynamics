within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Templates;

model TabularCompliant "Tabular translational spring with optional mass"
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Length free_length "Free length of spring" annotation(Dialog(group = "Spring Parameters"));
  parameter Real eps = 1e-12 "regularization (m)" annotation(Dialog(group="Numerical"));
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Force generation
  Modelica.Mechanics.Translational.Sources.Force2 force annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}))); 
  Modelica.Mechanics.MultiBody.Forces.LineForceWithMass lineForceWithMass annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  
  // Public interface
  Real defl_abs;
  Real sgn;
  
protected
  Real[3] r_rel;
  Real length;
  Real defl;

equation
  // Deflection calc
  r_rel = frame_b.r_0 - frame_a.r_0;
  length = norm(r_rel);
  defl = free_length - length;
  
  // Smooth abs and smooth sign
  defl_abs = sqrt(defl*defl + eps*eps);
  sgn = defl/defl_abs;
  
  connect(frame_a, lineForceWithMass.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(force.flange_a, lineForceWithMass.flange_a) annotation(
    Line(points = {{-10, 30}, {-20, 30}, {-20, 10}, {-6, 10}}, color = {0, 127, 0}));
  connect(force.flange_b, lineForceWithMass.flange_b) annotation(
    Line(points = {{10, 30}, {20, 30}, {20, 10}, {6, 10}}, color = {0, 127, 0}));
  connect(lineForceWithMass.frame_b, frame_b) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end TabularCompliant;

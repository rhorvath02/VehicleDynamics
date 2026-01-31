within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model TabularDamper "Tabular translational damper with velocity-force curve"
  extends VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.Templates.TabularCompliant;
  
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  import VehicleDynamics.Utilities.Math.Vector.dot;
    
  // Parameters
  parameter SIunits.TranslationalDampingConstant damper_table[:,2] "Table of Force vs Relative Velocity (m/s, N)" annotation(Dialog(group = "Damper Parameters"));
  
  parameter SIunits.Length outer_diameter=0.008 "Diameter of smallest possible cylinder which can fully enclose the damper" annotation(
    Dialog(group = "Animation"));
    
  parameter SIunits.Length inner_diameter=0.004 "Diameter of smallest possible cylinder (by volume) which can enclose any part of the damper" annotation(
    Dialog(group = "Animation"));
  Modelica.Blocks.Math.Product product annotation(
    Placement(transformation(origin = {-20, 80}, extent = {{-10, -10}, {10, 10}})));
protected
  Real[3] v_rel;
  Real[3] e_damper;
  
  Real v_axial;
  Real v_abs;
  Real vel_sgn;
  
  Modelica.Blocks.Sources.RealExpression vel_expression(y = v_abs) annotation(
    Placement(transformation(origin = {-90, 90},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1D combiTable1D(columns = {2},
                                                   extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints,
                                                   smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments,
                                                   table = damper_table) annotation(
                                                   Placement(transformation(origin = {-60, 90},
      extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression sgn_expression(y = -vel_sgn) annotation(
    Placement(transformation(origin = {-60, 70},
      extent = {{-10, -10}, {10, 10}})));

  
equation
  // Damper axis
  e_damper = if length > 1e-9 then r_rel/length else {1, 0, 0};
  
  // Relative velocity
  v_rel = der(frame_b.r_0) - der(frame_a.r_0);
  v_axial = dot(v_rel, e_damper);
  
  // Safeguarded values
  v_abs = sqrt(v_axial*v_axial + eps*eps);
  vel_sgn = v_axial/v_abs;
  connect(vel_expression.y, combiTable1D.u[1]) annotation(
    Line(points = {{-78, 90}, {-72, 90}}, color = {0, 0, 127}));
  connect(combiTable1D.y[1], product.u1) annotation(
    Line(points = {{-48, 90}, {-40, 90}, {-40, 86}, {-32, 86}}, color = {0, 0, 127}));
  connect(sgn_expression.y, product.u2) annotation(
    Line(points = {{-48, 70}, {-40, 70}, {-40, 74}, {-32, 74}}, color = {0, 0, 127}));
  connect(product.y, force.f) annotation(
    Line(points = {{-8, 80}, {0, 80}, {0, 34}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 1,
      Tolerance = 1e-06, Interval = 0.002));
end TabularDamper;

within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model ShockLinkage
  import VehicleDynamics.Utilities.Math.Vector.cross;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Geometry params
  parameter SIunits.Position start_point[3] "Initial point of shock linkage" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position end_point[3] "Final point of shock linkage" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Length rod_length_fraction "Length fraction of effective rigid member on shock linkage" annotation(
    Dialog(group = "Geometry"));
  
  // Spring params
  parameter SIunits.Length free_length "Spring free length" annotation(
    Dialog(group = "Spring Params"));
  parameter SIunits.TranslationalSpringConstant spring_table[:, 2] "Table of spring force vs deflection (change in length)" annotation(
    Dialog(group = "Spring Params"));
  parameter SIunits.Mass spring_mass "Spring mass" annotation(
    Dialog(group = "Spring Params"));
  
  // Damper params
  parameter SIunits.TranslationalDampingConstant damper_table[:, 2] "Table of damper force vs relative velocity" annotation(Dialog(group = "Damper Params"));
  parameter SIunits.Mass damper_mass "Damper mass" annotation(Dialog(group = "Damper Params"));
  
  // Animation
  parameter SIunits.Length link_diameter "Link diameter" annotation(
    Dialog(group = "Animation"));
  parameter SIunits.Length joint_diameter "Joint diameter" annotation(
    Dialog(group = "Animation"));
  parameter SIunits.Length spring_diameter "Spring diameter" annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  // Shock geometry calcs
  final parameter Real[3] shock_start = (end_point - start_point) * rod_length_fraction + start_point;
  final parameter Real[3] shock_end = end_point;
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Rigid rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = shock_start - start_point) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  
  // Tabular spring
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularSpring tabularSpring(
    force_table = spring_table,
    free_length = free_length,
    spring_mass = spring_mass,
    start_point = shock_start,
    end_point = shock_end,
    spring_diameter = free_length / 5)  annotation(
    Placement(transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}})));
  // Tabular damper
  VehicleDynamics.Vehicle.Chassis.Suspension.Linkages.TabularDamper tabularDamper(force_table = damper_table, damper_mass = damper_mass, start_point = shock_start, end_point = shock_end) annotation(
    Placement(transformation(origin = {50, -20}, extent = {{-10, -10}, {10, 10}})));

// Damper connection
equation
  connect(frame_a, fixedTranslation.frame_a) annotation(
    Line(points = {{-100, 0}, {-60, 0}}));
  connect(tabularSpring.frame_a, fixedTranslation.frame_b) annotation(
    Line(points = {{40, 20}, {30, 20}, {30, 0}, {-40, 0}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_b, frame_b) annotation(
    Line(points = {{60, 20}, {70, 20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, tabularDamper.frame_a) annotation(
    Line(points = {{-40, 0}, {30, 0}, {30, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, frame_b) annotation(
    Line(points = {{60, -20}, {70, -20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end ShockLinkage;
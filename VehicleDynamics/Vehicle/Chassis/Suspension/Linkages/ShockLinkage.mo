within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model ShockLinkage
  import VehicleDynamics.Utilities.Math.Vector.cross;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  parameter SIunits.Position start_point[3] "Initial point of shock linkage" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position end_point[3] "Final point of shock linkage" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Length rod_length "Length of effective rigid member on shock linkage" annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Length free_length "Spring free length" annotation(
    Dialog(group = "Rate Behavior"));
  parameter SIunits.TranslationalSpringConstant force_table[:, 2] "Table of Force vs Deflection (change in length)" annotation(
    Dialog(group = "Rate Behavior"));
  parameter SIunits.Mass spring_mass "Spring mass" annotation(
    Dialog(group = "Mass"));

// Animation
  parameter SIunits.Length link_diameter "Link diameter" annotation(
    Dialog(group = "Animation"));
  parameter SIunits.Length joint_diameter "Joint diameter" annotation(
    Dialog(group = "Animation"));
  parameter SIunits.Length spring_diameter "Spring diameter" annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));

// Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  // Joints
  // Rigid rod
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = (end_point - start_point)/norm(end_point - start_point)*rod_length) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  // Tabular spring
  Modelica.Mechanics.MultiBody.Visualizers.Advanced.Shape shape(shapeType = "spring", R = Modelica.Mechanics.MultiBody.Frames.nullRotation(), r = prismatic.frame_a.r_0, lengthDirection = e_spring, widthDirection = w_spring, length = spring_length, width = spring_diameter, height = spring_diameter/5, extra = 6) annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  TabularSpring tabularSpring annotation(
    Placement(transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}})));
  TabularDamper tabularDamper annotation(
    Placement(transformation(origin = {50, -20}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(frame_a, fixedTranslation.frame_a) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(tabularSpring.frame_a, fixedTranslation.frame_b) annotation(
    Line(points = {{40, 20}, {30, 20}, {30, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_a, fixedTranslation.frame_b) annotation(
    Line(points = {{40, -20}, {30, -20}, {30, 0}, {10, 0}}, color = {95, 95, 95}));
  connect(tabularSpring.frame_b, frame_b) annotation(
    Line(points = {{60, 20}, {70, 20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(tabularDamper.frame_b, frame_b) annotation(
    Line(points = {{60, -20}, {70, -20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
end ShockLinkage;

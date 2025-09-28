within VehicleDynamics.Vehicle.Chassis.Suspension.Linkages;

model Pushrod
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;

  parameter SIunits.Position outboard_node[3] "Outboard wishbone node coordinates";
  parameter SIunits.Position outboard_rod[3] "Inboard node coordinates";
  parameter SIunits.Position outboard_shock[3] "Outboard shock node coordinates";
  parameter SIunits.Position inboard_shock[3] "Inboard shock node coordinates";
  parameter SIunits.Length spring_length "Spring free length";
  parameter SIunits.Length compressed_length "Spring compressed length (1UP)";
  parameter SIunits.TranslationalSpringConstant spring_constant "Spring rate";
  parameter SIunits.Length link_diameter "Link diameter";
  parameter SIunits.Length joint_diameter "Joint diameter" annotation(
    Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  // Joints
  Joints.SphericalCompliant to_link(diameter = joint_diameter) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  // Link
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation to_rod_link(r = outboard_rod - outboard_node, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation rod_link(r = outboard_shock - outboard_rod, width = link_diameter, height = link_diameter) annotation(
    Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}})));
  // Shock
  Vehicle.Chassis.Suspension.Linkages.ShockBase spring_damper_parallel(compressed_length = compressed_length)  annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant to_link1(diameter = joint_diameter) annotation(
    Placement(transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
equation
  connect(frame_a, to_rod_link.frame_a) annotation(
    Line(points = {{-100, 0}, {-90, 0}}));
  connect(to_rod_link.frame_b, to_link.frame_a) annotation(
    Line(points = {{-70, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(to_link.frame_b, rod_link.frame_a) annotation(
    Line(points = {{-40, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(spring_damper_parallel.frame_a, rod_link.frame_b) annotation(
    Line(points = {{20, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(to_link1.frame_b, frame_b) annotation(
    Line(points = {{90, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(to_link1.frame_a, spring_damper_parallel.frame_b) annotation(
    Line(points = {{70, 0}, {40, 0}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Line(points = {{-80, 0}, {80, 0}}, color = {0, 0, 0}, thickness = 5), Ellipse(extent = {{-90, -10}, {-70, 10}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid), Ellipse(extent = {{70, -10}, {90, 10}}, lineColor = {0, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid)}));
end Pushrod;
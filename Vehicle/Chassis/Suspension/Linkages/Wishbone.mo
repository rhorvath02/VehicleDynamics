within Vehicle.Chassis.Suspension.Linkages;
model Wishbone
  import Modelica.Units.SI;
  
  parameter SI.Position fore_i[3] "Fore inboard node coordinates";
  parameter SI.Position aft_i[3] "Aft inboard node coordinates";
  parameter SI.Position outboard[3] "Outboard node coordinates";
  
  parameter SI.Length link_diameter "Link diameter";
  parameter SI.Length joint_diameter "Joint diameter";
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a fore_i_frame annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a aft_i_frame annotation(
    Placement(transformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b outboard_frame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

  // Links
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fore_link(r = outboard - fore_i, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {0, 60}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation aft_link(r = outboard - aft_i, width = link_diameter, height = link_diameter)  annotation(
    Placement(transformation(origin = {0, -60}, extent = {{10, -10}, {-10, 10}})));
  
  // Joints
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant fore_i_joint(r_rel(each fixed = true), diameter = joint_diameter)  annotation(
    Placement(transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant aft_i_joint(r_rel(each fixed = true), diameter = joint_diameter)  annotation(
    Placement(transformation(origin = {70, -60}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant outboard_joint(r_rel(each fixed = true), diameter = joint_diameter)  annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  
equation
  connect(aft_i_joint.frame_a, aft_link.frame_a) annotation(
    Line(points = {{60, -60}, {10, -60}}, color = {95, 95, 95}));
  connect(fore_i_joint.frame_a, fore_link.frame_a) annotation(
    Line(points = {{60, 60}, {10, 60}}, color = {95, 95, 95}));
  connect(fore_link.frame_b, outboard_joint.frame_b) annotation(
    Line(points = {{-10, 60}, {-40, 60}, {-40, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(aft_link.frame_b, outboard_joint.frame_b) annotation(
    Line(points = {{-10, -60}, {-40, -60}, {-40, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(aft_i_frame, aft_i_joint.frame_b) annotation(
    Line(points = {{100, -60}, {80, -60}}));
  connect(fore_i_frame, fore_i_joint.frame_b) annotation(
    Line(points = {{100, 60}, {80, 60}}));
  connect(outboard_joint.frame_a, outboard_frame) annotation(
    Line(points = {{-80, 0}, {-100, 0}}, color = {95, 95, 95}));

annotation(
  Icon(
    graphics = {
    Line(
          points = {{-80, 0}, {80, 66}},
          color = {0, 0, 0},
          thickness = 5
        ),
    Line(
          points = {{-80, 0}, {80, -66}},
          color = {0, 0, 0},
          thickness = 5
        ),
    Ellipse(
          extent = {{-90, -10}, {-70, 10}},
          lineColor = {0, 0, 0},
          fillColor = {255, 0, 0},
          fillPattern = FillPattern.Solid
        ),
    Ellipse(
          extent = {{70, 56}, {90, 76}},
          lineColor = {0, 0, 0},
          fillColor = {255, 0, 0},
          fillPattern = FillPattern.Solid
        ),
    Ellipse(
      extent = {{70, -76}, {90, -56}},
      lineColor = {0, 0, 0},
      fillColor = {255, 0, 0},
      fillPattern = FillPattern.Solid
    )
        }));
end Wishbone;

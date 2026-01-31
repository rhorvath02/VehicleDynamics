within VehicleDynamics.TestVehicle.TestChassis.TestSuspension.TestTemplates;

model TestRrDoubleWishboneBase
  // Modelica linalg
  import Modelica.Math.Vectors.normalize;
  
  // Custom linalg
  import VehicleDynamics.Utilities.Math.Vector.cross;
  import VehicleDynamics.Utilities.Math.Vector.dot;
  
  // Environment
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-140, -90}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics groundPhysics annotation(
    Placement(transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}})));
  
  // Double wishbone and supports
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.RrDoubleWishbone rrDoubleWishboneBase annotation(
    Placement(transformation(origin = {1.42109e-14, 0}, extent = {{-45, -45}, {45, 45}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed upper_fore_i(r = rrDoubleWishboneBase.upper_fore_i) annotation(
    Placement(transformation(origin = {90, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed upper_aft_i(r = rrDoubleWishboneBase.upper_aft_i) annotation(
    Placement(transformation(origin = {90, 60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed lower_fore_i(r = rrDoubleWishboneBase.lower_fore_i) annotation(
    Placement(transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed lower_aft_i(r = rrDoubleWishboneBase.lower_aft_i) annotation(
    Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed tie_i(r = rrDoubleWishboneBase.tie_i) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  // Torque application to fully define system
  Vehicle.Chassis.Tires.MF5p2Tire mF5p2Tire annotation(
    Placement(transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp(duration = 1, height = 0, startTime = 0) annotation(
    Placement(transformation(origin = {-140, 32}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(upper_fore_i.frame_b, rrDoubleWishboneBase.upper_fore_i_frame) annotation(
    Line(points = {{80, 90}, {0, 90}, {0, 46}}, color = {95, 95, 95}));
  connect(upper_aft_i.frame_b, rrDoubleWishboneBase.upper_aft_i_frame) annotation(
    Line(points = {{80, 60}, {30, 60}, {30, 46}}, color = {95, 95, 95}));
  connect(lower_fore_i.frame_b, rrDoubleWishboneBase.lower_fore_i_frame) annotation(
    Line(points = {{80, 30}, {46, 30}}, color = {95, 95, 95}));
  connect(lower_aft_i.frame_b, rrDoubleWishboneBase.lower_aft_i_frame) annotation(
    Line(points = {{80, 0}, {46, 0}}, color = {95, 95, 95}));
  connect(tie_i.frame_b, rrDoubleWishboneBase.tie_i_frame) annotation(
    Line(points = {{80, -30}, {46, -30}}, color = {95, 95, 95}));
  connect(ramp.y, mF5p2Tire.hub_torque) annotation(
    Line(points = {{-129, 32}, {-92.5, 32}, {-92.5, 12}}, color = {0, 0, 127}));
  connect(groundPhysics.frame_b, mF5p2Tire.cp_frame) annotation(
    Line(points = {{-90, -30}, {-90, -10}}, color = {95, 95, 95}));
  connect(mF5p2Tire.chassis_frame, rrDoubleWishboneBase.midpoint_frame) annotation(
    Line(points = {{-80, 0}, {-44, 0}}, color = {95, 95, 95}));
  connect(world.frame_b, groundPhysics.frame_a) annotation(
    Line(points = {{-130, -90}, {-120, -90}, {-120, -40}, {-100, -40}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-220, 100}, {100, -100}})));
end TestRrDoubleWishboneBase;

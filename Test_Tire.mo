model Test_Tire
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Tires.BaseMF52 baseMF52(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false, r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Sine sine_input(amplitude = 15*Modelica.Constants.pi/180, freqHz = 1) annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position angle annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));

Vehicle.GroundPhysics ground annotation(
    Placement(transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Sensors.CutForce cut_force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

Modelica.Mechanics.MultiBody.Joints.Prismatic free_x(n = {1, 0, 0}, useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_z(n = {0, 0, 1}) annotation(
    Placement(transformation(origin = {-20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute free_yaw(n = {0, 0, 1}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
equation
  connect(sine_input.y, angle.phi_ref) annotation(
    Line(points = {{-59, 0}, {-52, 0}}, color = {0, 0, 127}));
  connect(fixed.frame_b, ground.frame_a) annotation(
    Line(points = {{30, -80}, {30, -70}}, color = {95, 95, 95}));
  connect(ground.frame_b, cut_force.frame_a) annotation(
    Line(points = {{30, -50}, {30, -40}}, color = {95, 95, 95}));
  connect(fixed.frame_b, free_x.frame_a) annotation(
    Line(points = {{30, -80}, {-20, -80}}, color = {95, 95, 95}));
  connect(free_yaw.frame_b, baseMF52.frame_a) annotation(
    Line(points = {{10, 30}, {30, 30}}, color = {95, 95, 95}));
  connect(free_z.frame_b, free_yaw.frame_a) annotation(
    Line(points = {{-20, -8}, {-20, 30}, {-10, 30}}, color = {95, 95, 95}));
  connect(free_x.frame_b, free_z.frame_a) annotation(
    Line(points = {{-20, -60}, {-20, -28}}, color = {95, 95, 95}));
  connect(angle.flange, free_yaw.axis) annotation(
    Line(points = {{-30, 0}, {0, 0}, {0, 20}}));
  connect(cut_force.frame_b, baseMF52.frame_a) annotation(
    Line(points = {{30, -20}, {30, 30}}, color = {95, 95, 95}));
end Test_Tire;
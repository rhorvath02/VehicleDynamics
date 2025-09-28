within VehicleDynamics.TestVehicle.TestChassis.TestSuspension.TestTires;
model TestMF5p2Base
  // Gravity vector
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Tire with zero toe and camber
  Vehicle.Chassis.Suspension.Tires.MF5p2Base MF5p2Base(static_gamma = 0, static_alpha = 0)  annotation(
    Placement(transformation(origin = {30, 40}, extent = {{-10, -10}, {10, 10}})));
  
  // Reference point (origin)
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false, r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Steer input
  Modelica.Blocks.Sources.Sine sine_input(amplitude = 15*Modelica.Constants.pi/180, freqHz = 1, startTime = 5) annotation(
    Placement(transformation(origin = {-80, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position angle annotation(
    Placement(transformation(origin = {-50, 60}, extent = {{-10, -10}, {10, 10}})));

  // Ground physics
  Vehicle.GroundPhysics ground annotation(
      Placement(transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Normal force sensor
  Modelica.Mechanics.MultiBody.Sensors.CutForce cut_force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world, animation = false) annotation(
    Placement(transformation(origin = {30, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Free only x, z, and yaw
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_x(n = {1, 0, 0}, useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_z(n = {0, 0, 1}) annotation(
    Placement(transformation(origin = {-20, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute free_yaw(n = {0, 0, 1}, useAxisFlange = true) annotation(
    Placement(transformation(origin = {0, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Constant velocity
  Modelica.Mechanics.Translational.Sources.Speed v_x annotation(
    Placement(transformation(origin = {-50, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Blocks.Sources.Constant vel_source(k = 0)  annotation(
    Placement(transformation(origin = {-80, -50}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  
  // Sprung mass
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 100, r_0(each fixed = true, start = {0, 0, 0.25}), r_CM = {0, 0, 0}, sphereDiameter = 0.05) annotation(
    Placement(transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  connect(sine_input.y, angle.phi_ref) annotation(
    Line(points = {{-69, 60}, {-62, 60}}, color = {0, 0, 127}));
  connect(fixed.frame_b, ground.frame_a) annotation(
    Line(points = {{30, -80}, {30, -70}}, color = {95, 95, 95}));
  connect(ground.frame_b, cut_force.frame_a) annotation(
    Line(points = {{30, -50}, {30, -40}}, color = {95, 95, 95}));
  connect(fixed.frame_b, free_x.frame_a) annotation(
    Line(points = {{30, -80}, {-20, -80}}, color = {95, 95, 95}));
  connect(free_yaw.frame_b, MF5p2Base.frame_a) annotation(
    Line(points = {{10, 30}, {30, 30}}, color = {95, 95, 95}));
  connect(free_z.frame_b, free_yaw.frame_a) annotation(
    Line(points = {{-20, -8}, {-20, 30}, {-10, 30}}, color = {95, 95, 95}));
  connect(free_x.frame_b, free_z.frame_a) annotation(
    Line(points = {{-20, -60}, {-20, -28}}, color = {95, 95, 95}));
  connect(angle.flange, free_yaw.axis) annotation(
    Line(points = {{-40, 60}, {0, 60}, {0, 40}}));
  connect(cut_force.frame_b, MF5p2Base.frame_a) annotation(
    Line(points = {{30, -20}, {30, 30}}, color = {95, 95, 95}));
  connect(vel_source.y, v_x.v_ref) annotation(
    Line(points = {{-69, -50}, {-62, -50}}, color = {0, 0, 127}));
  connect(v_x.flange, free_x.axis) annotation(
    Line(points = {{-40, -50}, {-33, -50}, {-33, -62}, {-26, -62}}, color = {0, 127, 0}));
  connect(body.frame_a, MF5p2Base.corner_frame) annotation(
    Line(points = {{30, 60}, {30, 50}}, color = {95, 95, 95}));
annotation(
  experiment(StartTime = 0, StopTime = 10));
end TestMF5p2Base;
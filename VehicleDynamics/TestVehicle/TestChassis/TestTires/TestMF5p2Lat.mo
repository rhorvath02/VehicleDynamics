within VehicleDynamics.TestVehicle.TestChassis.TestTires;

model TestMF5p2Lat
  // Gravity vector
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  // Test tire
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire TestTire(R0 = 8*0.0254, rim_width = 7*0.0254, rim_R0 = 5*0.0254, tire_c = 1e8, tire_d = 1e8, wheel_J = 0.2, wheel_m = 10, initial_velocity = 2) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  // Ground interface
  Modelica.Mechanics.MultiBody.Parts.Fixed FixedSupport annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics Ground annotation(
    Placement(transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}})));
  // Wheel constraints
  VehicleDynamics.Utilities.Mechanics.Multibody.TranslationalJoint FreeTranslation(z0 = TestTire.R0, vx0 = 2)  annotation(
    Placement(transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute FreeYaw(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  // Steer input
  Modelica.Mechanics.Rotational.Sources.Position YawInput annotation(
    Placement(transformation(origin = {-60, -10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp SteerSource(height = 15*Modelica.Constants.pi/180, duration = 5, startTime = 1)  annotation(
    Placement(transformation(origin = {-90, -10}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(FixedSupport.frame_b, Ground.frame_a) annotation(
    Line(points = {{0, -60}, {0, -40}, {10, -40}}, color = {95, 95, 95}));
  connect(Ground.frame_b, TestTire.cp_frame) annotation(
    Line(points = {{20, -30}, {20, -20.5}, {0, -20.5}, {0, 0}}, color = {95, 95, 95}));
  connect(FixedSupport.frame_b, FreeTranslation.frame_a) annotation(
    Line(points = {{0, -60}, {-30, -60}, {-30, -50}}, color = {95, 95, 95}));
  connect(FreeTranslation.frame_b, FreeYaw.frame_a) annotation(
    Line(points = {{-30, -30}, {-30, -20}}, color = {95, 95, 95}));
  connect(FreeYaw.frame_b, TestTire.chassis_frame) annotation(
    Line(points = {{-30, 0}, {-30, 10}, {-10, 10}}, color = {95, 95, 95}));
  connect(SteerSource.y, YawInput.phi_ref) annotation(
    Line(points = {{-79, -10}, {-72, -10}}, color = {0, 0, 127}));
  connect(YawInput.flange, FreeYaw.axis) annotation(
    Line(points = {{-50, -10}, {-40, -10}}));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001));
end TestMF5p2Lat;
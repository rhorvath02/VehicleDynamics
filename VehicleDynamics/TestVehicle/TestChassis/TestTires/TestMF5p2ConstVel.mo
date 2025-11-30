within VehicleDynamics.TestVehicle.TestChassis.TestTires;

model TestMF5p2ConstVel
  // Gravity vector
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Test tire
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire TestTire(R0 = 8*0.0254, rim_width = 7*0.0254, rim_R0 = 5*0.0254, tire_c = 1e8, tire_d = 1e8, wheel_J = 0.2, wheel_m = 3, initial_velocity = 2)  annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
  
  // Ground interface
  Modelica.Mechanics.MultiBody.Parts.Fixed FixedSupport annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics Ground annotation(
    Placement(transformation(origin = {20, -40}, extent = {{-10, -10}, {10, 10}})));
  
  // Wheel constraints
  Modelica.Mechanics.MultiBody.Joints.Planar PlanarJoint(v_x(start = 2, fixed = true))  annotation(
    Placement(transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic PrismaticJoint(n = {0, 0, 1}, s(start = TestTire.R0, fixed = true)) annotation(
    Placement(transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  // Initial conditions
equation
  connect(FixedSupport.frame_b, Ground.frame_a) annotation(
    Line(points = {{0, -60}, {0, -40.5}, {10, -40.5}, {10, -40}}, color = {95, 95, 95}));
  connect(Ground.frame_b, TestTire.cp_frame) annotation(
    Line(points = {{20, -30}, {20, -20.5}, {0, -20.5}, {0, 0}}, color = {95, 95, 95}));
  connect(PlanarJoint.frame_a, FixedSupport.frame_b) annotation(
    Line(points = {{-30, -50}, {-30, -60}, {0, -60}}, color = {95, 95, 95}));
  connect(PlanarJoint.frame_b, PrismaticJoint.frame_a) annotation(
    Line(points = {{-30, -30}, {-30, -20}}, color = {95, 95, 95}));
  connect(PrismaticJoint.frame_b, TestTire.chassis_frame) annotation(
    Line(points = {{-30, 0}, {-30, 10}, {-10, 10}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001));
end TestMF5p2ConstVel;
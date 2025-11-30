within VehicleDynamics.TestVehicle.TestChassis.TestSuspension.TestTires;

model TestMF5p2Long
  // Gravity vector
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Test tire
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire TestTire(R0 = 8*0.0254, rim_width = 7*0.0254, rim_R0 = 5*0.0254, tire_c = 1e8, tire_d = 1e8, wheel_J = 0.2, wheel_m = 3)  annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Ground interface
  Modelica.Mechanics.MultiBody.Parts.Fixed FixedSupport annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics Ground annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}})));
  
  // Wheel constraints
  Modelica.Mechanics.MultiBody.Joints.Planar PlanarJoint annotation(
    Placement(transformation(origin = {-30, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic PrismaticJoint(n = {0, 0, 1}, s(start = TestTire.R0 + 0.25, fixed = true)) annotation(
    Placement(transformation(origin = {-30, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Torque input
  Modelica.Mechanics.MultiBody.Forces.WorldTorque TorqueInput annotation(
    Placement(transformation(origin = {30, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression Mx annotation(
    Placement(transformation(origin = {70, 40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.Ramp My(height = 10, duration = 2, startTime = 2)  annotation(
    Placement(transformation(origin = {70, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.RealExpression Mz annotation(
    Placement(transformation(origin = {70, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    
equation
  connect(FixedSupport.frame_b, Ground.frame_a) annotation(
    Line(points = {{0, -60}, {0, -50}}, color = {95, 95, 95}));
  connect(Ground.frame_b, TestTire.cp_frame) annotation(
    Line(points = {{0, -30}, {0, 0}}, color = {95, 95, 95}));
  connect(PlanarJoint.frame_a, FixedSupport.frame_b) annotation(
    Line(points = {{-30, -50}, {-30, -60}, {0, -60}}, color = {95, 95, 95}));
  connect(PlanarJoint.frame_b, PrismaticJoint.frame_a) annotation(
    Line(points = {{-30, -30}, {-30, -20}}, color = {95, 95, 95}));
  connect(Mx.y, TorqueInput.torque[1]) annotation(
    Line(points = {{60, 40}, {50, 40}, {50, 10}, {42, 10}}, color = {0, 0, 127}));
  connect(My.y, TorqueInput.torque[2]) annotation(
    Line(points = {{59, 10}, {42, 10}}, color = {0, 0, 127}));
  connect(Mz.y, TorqueInput.torque[3]) annotation(
    Line(points = {{60, -20}, {50, -20}, {50, 10}, {42, 10}}, color = {0, 0, 127}));
  connect(TorqueInput.frame_b, TestTire.hub_frame) annotation(
    Line(points = {{20, 10}, {10, 10}}, color = {95, 95, 95}));
  connect(TestTire.chassis_frame, PrismaticJoint.frame_b) annotation(
    Line(points = {{-10, 10}, {-30, 10}, {-30, 0}}, color = {95, 95, 95}));

annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001));
end TestMF5p2Long;
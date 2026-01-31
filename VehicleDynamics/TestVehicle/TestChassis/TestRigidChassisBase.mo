within VehicleDynamics.TestVehicle.TestChassis;
model TestRigidChassisBase
  inner Modelica.Mechanics.MultiBody.World world(n={0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  // Chassis
  VehicleDynamics.Vehicle.Chassis.RigidChassis chassisBase annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  // Wheel torques
  Modelica.Blocks.Sources.Constant FL_torque_in(k = 0) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant FR_torque_in(k = 0) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Ramp RL_torque_in(height = 100, duration = 3, startTime = 1) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp RR_torque_in(height = 100, duration = 3, startTime = 1) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}})));
  // Steer input
  Modelica.Blocks.Sources.Ramp steer_input(duration = 2, height = 0.25*0.0254*0, startTime = 3.5) annotation(
    Placement(transformation(origin = {-20, 50}, extent = {{-10, -10}, {10, 10}})));

  Real a_body[3];
  
  output Real a_x;

equation
  a_body = Modelica.Mechanics.MultiBody.Frames.resolve2(chassisBase.sprung_mass.frame_a.R, chassisBase.sprung_mass.a_0);
  a_x = a_body[1];
  
  connect(world.frame_b, chassisBase.world_frame) annotation(
    Line(points = {{-80, -90}, {0, -90}, {0, -20}}, color = {95, 95, 95}));
  connect(RL_torque_in.y, chassisBase.RL_torque) annotation(
    Line(points = {{-38, -30}, {-24, -30}, {-24, -14}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, chassisBase.RR_torque) annotation(
    Line(points = {{40, -30}, {24, -30}, {24, -14}}, color = {0, 0, 127}));
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-24, 30}, {-24, 14}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 14}}, color = {0, 0, 127}));
  connect(steer_input.y, chassisBase.rack_input) annotation(
    Line(points = {{-8, 50}, {0, 50}, {0, 24}}, color = {0, 0, 127}));  
annotation(
    experiment(StartTime = 0, StopTime = 6, Tolerance = 1e-06, Interval = 0.001),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=1000",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"));
end TestRigidChassisBase;

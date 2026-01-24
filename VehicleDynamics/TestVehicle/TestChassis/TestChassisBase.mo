within VehicleDynamics.TestVehicle.TestChassis;
model TestChassisBase
  inner Modelica.Mechanics.MultiBody.World world(n={0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Chassis
  VehicleDynamics.Vehicle.Chassis.ChassisBase chassisBase annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  
  // Wheel torques
  Modelica.Blocks.Sources.Constant FL_torque_in(k = 0) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant FR_torque_in(k = 0) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Ramp RL_torque_in(height = 100, duration = 3, startTime = 1) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp RR_torque_in(height = 100, duration = 3, startTime = 1) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  // Steer input
  Modelica.Blocks.Sources.Ramp steer_input(duration = 2, height = 0.25*0.0254, startTime = 3.5) annotation(
    Placement(transformation(origin = {-20, 60}, extent = {{-10, -10}, {10, 10}})));

  Real a_body[3];

equation
  a_body = Modelica.Mechanics.MultiBody.Frames.resolve2(chassisBase.sprung_mass.frame_a.R, chassisBase.sprung_mass.a_0);
  
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-38, 29.8125}, {-24, 29.8125}, {-24, 13}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 13}}, color = {0, 0, 127}));
  connect(RL_torque_in.y, chassisBase.RL_torque) annotation(
    Line(points = {{-38, -30}, {-38, -30.25}, {-24, -30.25}, {-24, -13}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, chassisBase.RR_torque) annotation(
    Line(points = {{40, -30}, {24, -30}, {24, -13}}, color = {0, 0, 127}));
  connect(steer_input.y, chassisBase.rack_input) annotation(
    Line(points = {{-8, 60}, {0, 60}, {0, 24}}, color = {0, 0, 127}));

annotation(
    experiment(StartTime = 0, StopTime = 6, Tolerance = 1e-06, Interval = 0.001),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"));
end TestChassisBase;

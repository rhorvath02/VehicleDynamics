within VehicleDynamics.TestVehicle.TestChassis;
model TestChassisBase
  inner Modelica.Mechanics.MultiBody.World world(n={0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  VehicleDynamics.Vehicle.Chassis.ChassisBase chassisBase annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}})));
  
  Modelica.Blocks.Sources.Constant FL_torque_in(k = 0) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant FR_torque_in(k = 0) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Exponentials RL_torque_in(outMax=275, offset=10, riseTime=1, startTime=1) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Exponentials RR_torque_in(outMax=275, offset=10, riseTime=1, startTime=1) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  Real a_body[3];
  
  Real Ax;
  Real Ay;
  Real Az;

equation
  a_body =
    Modelica.Mechanics.MultiBody.Frames.resolve2(
      chassisBase.body.frame_a.R,
      chassisBase.body.a_0);

  Ax = a_body[1];
  Ay = a_body[2];
  Az = a_body[3];
  
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-38, 29.8125}, {-24, 29.8125}, {-24, 13}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 13}}, color = {0, 0, 127}));
  connect(RL_torque_in.y, chassisBase.RL_torque) annotation(
    Line(points = {{-38, -30}, {-38, -30.25}, {-24, -30.25}, {-24, -13}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, chassisBase.RR_torque) annotation(
    Line(points = {{40, -30}, {24, -30}, {24, -13}}, color = {0, 0, 127}));
annotation(
    experiment(StartTime = 0, StopTime = 15, Tolerance = 1e-06, Interval = 0.005),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian -d=initialization --maxSizeLinearTearing=500 ",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"));
end TestChassisBase;
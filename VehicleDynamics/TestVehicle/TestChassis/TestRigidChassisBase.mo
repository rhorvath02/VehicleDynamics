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

// Steer input
  Real a_body[3];
  
  output Real a_x;
  output Real a_y;
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(table = [0, 0; 1, 0; 2, 50*1.5; 3, 100*1.5; 4, 105*1.5], extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1)  annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y = time - 1)  annotation(
    Placement(transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds1(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, table = [0, 0; 1, 0; 2, 50*1.5; 3, 100*1.5; 4, 105*1.5], smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression realExpression1(y = time - 1) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds2(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1, table = [0, 0; 1, 0; 2, 0; 3, 0; 4, 0.625*0.0254; 5, 1.25*0.0254; 6, 1.30*0.0254]) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y = time) annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}})));
equation
  a_body = Modelica.Mechanics.MultiBody.Frames.resolve2(chassisBase.sprung_mass.frame_a.R, chassisBase.sprung_mass.a_0);
  a_x = a_body[1];
  a_y = a_body[2];
  connect(world.frame_b, chassisBase.world_frame) annotation(
    Line(points = {{-80, -90}, {0, -90}, {0, -20}}, color = {95, 95, 95}));
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-24, 30}, {-24, 14}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 14}}, color = {0, 0, 127}));
  connect(realExpression.y, combiTable1Ds.u) annotation(
    Line(points = {{-78, -30}, {-62, -30}}, color = {0, 0, 127}));
  connect(combiTable1Ds.y[1], chassisBase.RL_torque) annotation(
    Line(points = {{-39, -30}, {-24, -30}, {-24, -14}}, color = {0, 0, 127}));
  connect(realExpression1.y, combiTable1Ds1.u) annotation(
    Line(points = {{80, -30}, {62, -30}}, color = {0, 0, 127}));
  connect(combiTable1Ds1.y[1], chassisBase.RR_torque) annotation(
    Line(points = {{40, -30}, {24, -30}, {24, -14}}, color = {0, 0, 127}));
  connect(realExpression2.y, combiTable1Ds2.u) annotation(
    Line(points = {{-78, 70}, {-62, 70}}, color = {0, 0, 127}));
  connect(combiTable1Ds2.y[1], chassisBase.rack_input) annotation(
    Line(points = {{-38, 70}, {0, 70}, {0, 24}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 12, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=1000",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*", noEventEmit = "()"));
end TestRigidChassisBase;

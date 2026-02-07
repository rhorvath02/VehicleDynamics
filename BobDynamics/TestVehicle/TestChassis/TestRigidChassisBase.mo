within BobDynamics.TestVehicle.TestChassis;
model TestRigidChassisBase
  import Modelica.Math.Vectors.norm;
  
  final inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Chassis
  BobDynamics.Vehicle.Chassis.RigidChassis chassisBase annotation(
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
  output Real vel;
  
  Modelica.Blocks.Sources.RealExpression realExpression(y = norm({chassisBase.sprung_mass.v_0[1], chassisBase.sprung_mass.v_0[2], 0}))  annotation(
    Placement(transformation(origin = {-64, -52}, extent = {{-10, -10}, {10, 10}})));
  final Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds2(extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1, table = [0.0, 0.0; 1.0, 0.0; 11, 1.0*0.0254; 21, 1.0*0.0254]) annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y = time- 8) annotation(
    Placement(transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.LimPID PID(yMax = 250, k = 30, Ti = 0.5, Td = 0, initType = Modelica.Blocks.Types.InitPID.InitialOutput, y_start = 0)  annotation(
    Placement(transformation(origin = {-24, -52}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  Modelica.Blocks.Sources.RealExpression realExpression3(y = target_speed) annotation(
    Placement(transformation(origin = {-44, -72}, extent = {{-10, -10}, {10, 10}})));
  
  Real target_speed(start = 15);
  
equation
  target_speed = 15;
  
  a_body = Modelica.Mechanics.MultiBody.Frames.resolve2(chassisBase.sprung_mass.frame_a.R, chassisBase.sprung_mass.a_0);
  a_x = a_body[1];
  a_y = a_body[2];
  vel = norm(chassisBase.sprung_mass.v_0);
  connect(world.frame_b, chassisBase.world_frame) annotation(
    Line(points = {{-80, -90}, {0, -90}, {0, -20}}, color = {95, 95, 95}));
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-24, 30}, {-24, 14}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 14}}, color = {0, 0, 127}));
  connect(realExpression2.y, combiTable1Ds2.u) annotation(
    Line(points = {{-78, 70}, {-62, 70}}, color = {0, 0, 127}));
  connect(combiTable1Ds2.y[1], chassisBase.rack_input) annotation(
    Line(points = {{-38, 70}, {0, 70}, {0, 24}}, color = {0, 0, 127}));
  connect(realExpression3.y, PID.u_s) annotation(
    Line(points = {{-33, -72}, {-25, -72}, {-25, -64}}, color = {0, 0, 127}));
  connect(realExpression.y, PID.u_m) annotation(
    Line(points = {{-53, -52}, {-37, -52}}, color = {0, 0, 127}));
  connect(PID.y, chassisBase.RL_torque) annotation(
    Line(points = {{-24, -41}, {-24, -14}}, color = {0, 0, 127}));
  connect(PID.y, chassisBase.RR_torque) annotation(
    Line(points = {{-24, -40}, {-24, -30}, {40, -30}, {40, -14}, {24, -14}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.02),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=2000",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*", noEventEmit = "()"));
end TestRigidChassisBase;

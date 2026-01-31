within VehicleDynamics.TestVehicle.TestChassis.TestSuspension;

model TestFrRigidAxleBellcrank
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  Vehicle.Chassis.Suspension.FrRigidAxleBellcrank Fr_axle annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Sources.Constant FL_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {-30, 17}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant FR_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {30, 17}, extent = {{10, -10}, {-10, 10}})));
  
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {40, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed constrain_axle(r = Fr_axle.effective_center) annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  output Real test;
  Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(table = [0, 0; 1, 0; 2, 0; 3, 1*0.0254; 4, 1.125*0.0254], smoothness = Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint, tableOnFile = false)  annotation(
    Placement(transformation(origin = {-30, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y = time)  annotation(
    Placement(transformation(origin = {-70, 50}, extent = {{-10, -10}, {10, 10}})));
equation
  test = Fr_axle.FL_cp.r_0[2];
  connect(FL_torque_in.y, Fr_axle.FL_torque) annotation(
    Line(points = {{-19, 17}, {-12, 17}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, Fr_axle.FR_torque) annotation(
    Line(points = {{19, 17}, {12, 17}}, color = {0, 0, 127}));
  connect(FL_ground.frame_b, Fr_axle.FL_cp) annotation(
    Line(points = {{-40, -20}, {-40, -10}, {-9, -10}, {-9, 0}}, color = {95, 95, 95}));
  connect(FR_ground.frame_b, Fr_axle.FR_cp) annotation(
    Line(points = {{40, -20}, {40, -12}, {9, -12}, {9, 0}}, color = {95, 95, 95}));
  connect(constrain_axle.frame_b, Fr_axle.axle_frame) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
  connect(combiTable1Ds.y[1], Fr_axle.steer_input) annotation(
    Line(points = {{-18, 50}, {0, 50}, {0, 22}}, color = {0, 0, 127}));
  connect(realExpression.y, combiTable1Ds.u) annotation(
    Line(points = {{-58, 50}, {-42, 50}}, color = {0, 0, 127}));
  connect(world.frame_b, FL_ground.frame_a) annotation(
    Line(points = {{-80, -90}, {-70, -90}, {-70, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(world.frame_b, FR_ground.frame_a) annotation(
    Line(points = {{-80, -90}, {70, -90}, {70, -30}, {50, -30}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 6, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end TestFrRigidAxleBellcrank;

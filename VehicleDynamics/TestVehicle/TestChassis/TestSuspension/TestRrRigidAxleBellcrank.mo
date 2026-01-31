within VehicleDynamics.TestVehicle.TestChassis.TestSuspension;

model TestRrRigidAxleBellcrank
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  Vehicle.Chassis.Suspension.RrRigidAxleBellcrank Rr_axle annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Sources.Constant RL_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {-30, 17}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant RR_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {30, 17}, extent = {{10, -10}, {-10, 10}})));
  
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {40, -30}, extent = {{10, -10}, {-10, 10}})));
  
  output Real test;
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = Rr_axle.effective_center, animation = false)  annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  test = Rr_axle.RL_cp.r_0[2];
  connect(RL_torque_in.y, Rr_axle.RL_torque) annotation(
    Line(points = {{-19, 17}, {-12, 17}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, Rr_axle.RR_torque) annotation(
    Line(points = {{19, 17}, {12, 17}}, color = {0, 0, 127}));
  connect(RL_ground.frame_b, Rr_axle.RL_cp) annotation(
    Line(points = {{-40, -20}, {-40, -10}, {-9, -10}, {-9, 0}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, Rr_axle.RR_cp) annotation(
    Line(points = {{40, -20}, {40, -12}, {9, -12}, {9, 0}}, color = {95, 95, 95}));
  connect(world.frame_b, RL_ground.frame_a) annotation(
    Line(points = {{-80, -90}, {-70, -90}, {-70, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(world.frame_b, RR_ground.frame_a) annotation(
    Line(points = {{-80, -90}, {70, -90}, {70, -30}, {50, -30}}, color = {95, 95, 95}));
  connect(fixed.frame_b, Rr_axle.axle_frame) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 6, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"));
end TestRrRigidAxleBellcrank;

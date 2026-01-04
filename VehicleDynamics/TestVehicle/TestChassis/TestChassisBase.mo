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
  Modelica.Blocks.Sources.Ramp RL_torque_in(height= 275, duration=2, startTime=1) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp RR_torque_in(height= 275, duration=2, startTime=1) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

equation
  connect(FL_torque_in.y, chassisBase.FL_torque) annotation(
    Line(points = {{-38, 30}, {-38, 29.8125}, {-24, 29.8125}, {-24, 13}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, chassisBase.FR_torque) annotation(
    Line(points = {{40, 30}, {24, 30}, {24, 13}}, color = {0, 0, 127}));
  connect(RL_torque_in.y, chassisBase.RL_torque) annotation(
    Line(points = {{-38, -30}, {-38, -30.25}, {-24, -30.25}, {-24, -13}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, chassisBase.RR_torque) annotation(
    Line(points = {{40, -30}, {24, -30}, {24, -13}}, color = {0, 0, 127}));
annotation(
    experiment(StartTime = 0, StopTime = 4.5, Tolerance = 1e-06, Interval = 0.001));
end TestChassisBase;
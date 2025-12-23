within VehicleDynamics.TestVehicle.TestChassis.TestSuspension;
model TestFrAxleBase
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  Vehicle.Chassis.Suspension.FrAxleBellcrank Fr_axle annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Sources.Constant FL_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant FR_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {40, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {40, -30}, extent = {{10, -10}, {-10, 10}})));
  
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_fixed(r = {0, 0.609600, -1.5*0.0254})  annotation(
    Placement(transformation(origin = {-78, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FR_fixed(r = {0, -0.609600, -1.5*0.0254})  annotation(
    Placement(transformation(origin = {80, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  Modelica.Mechanics.MultiBody.Parts.Fixed constrain_axle(r = {0, 0, 0.2032})  annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  connect(FL_torque_in.y, Fr_axle.FL_torque) annotation(
    Line(points = {{-28, 10}, {-20, 10}, {-20, 17}, {-12, 17}}, color = {0, 0, 127}));
  connect(FR_torque_in.y, Fr_axle.FR_torque) annotation(
    Line(points = {{30, 10}, {20, 10}, {20, 17}, {12, 17}}, color = {0, 0, 127}));
  connect(FL_ground.frame_b, Fr_axle.FL_cp) annotation(
    Line(points = {{-40, -20}, {-40, -10}, {-9, -10}, {-9, 0}}, color = {95, 95, 95}));
  connect(FR_ground.frame_b, Fr_axle.FR_cp) annotation(
    Line(points = {{40, -20}, {40, -12}, {9, -12}, {9, 0}}, color = {95, 95, 95}));
  connect(FL_fixed.frame_b, FL_ground.frame_a) annotation(
    Line(points = {{-68, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(FR_fixed.frame_b, FR_ground.frame_a) annotation(
    Line(points = {{70, -30}, {50, -30}}, color = {95, 95, 95}));
  connect(constrain_axle.frame_b, Fr_axle.axle_frame) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
end TestFrAxleBase;
within VehicleDynamics.TestVehicle.TestChassis.TestSuspension.TestTemplates;

model TestRrAxleBase
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.RrAxleBase Rr_axle annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Sources.Constant RL_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant RR_torque_in(k = 0)  annotation(
    Placement(transformation(origin = {40, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-40, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {40, -30}, extent = {{10, -10}, {-10, 10}})));
  
  Modelica.Mechanics.MultiBody.Parts.Fixed RL_fixed(r = {0, 0.609600, -1.5*0.0254*0})  annotation(
    Placement(transformation(origin = {-78, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RR_fixed(r = {0, -0.609600, -1.5*0.0254*0})  annotation(
    Placement(transformation(origin = {80, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed constrain_axle(r = {0, 0, 0.2032}) annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  connect(RL_torque_in.y, Rr_axle.RL_torque) annotation(
    Line(points = {{-28, 10}, {-20, 10}, {-20, 17}, {-12, 17}}, color = {0, 0, 127}));
  connect(RR_torque_in.y, Rr_axle.RR_torque) annotation(
    Line(points = {{30, 10}, {20, 10}, {20, 17}, {12, 17}}, color = {0, 0, 127}));
  connect(RL_ground.frame_b, Rr_axle.RL_cp) annotation(
    Line(points = {{-40, -20}, {-40, -10}, {-9, -10}, {-9, 0}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, Rr_axle.RR_cp) annotation(
    Line(points = {{40, -20}, {40, -12}, {9, -12}, {9, 0}}, color = {95, 95, 95}));
  connect(RL_fixed.frame_b, RL_ground.frame_a) annotation(
    Line(points = {{-68, -30}, {-50, -30}}, color = {95, 95, 95}));
  connect(RR_fixed.frame_b, RR_ground.frame_a) annotation(
    Line(points = {{70, -30}, {50, -30}}, color = {95, 95, 95}));
  connect(constrain_axle.frame_b, Rr_axle.axle_frame) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
end TestRrAxleBase;

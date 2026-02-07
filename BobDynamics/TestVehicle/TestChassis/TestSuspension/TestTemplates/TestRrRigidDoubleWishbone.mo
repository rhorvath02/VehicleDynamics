within BobDynamics.TestVehicle.TestChassis.TestSuspension.TestTemplates;

model TestRrRigidDoubleWishbone
  // Modelica linalg
  import Modelica.Math.Vectors.normalize;
  // Custom linalg
  import BobDynamics.Utilities.Math.Vector.cross;
  import BobDynamics.Utilities.Math.Vector.dot;
  // Environment
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, enableAnimation = true) annotation(
    Placement(transformation(origin = {-140, -90}, extent = {{-10, -10}, {10, 10}})));
  
  BobDynamics.Vehicle.Chassis.Suspension.Templates.RrRigidDoubleWishbone RrRigidDoubleWishbone annotation(
    Placement(transformation(origin = {1.42109e-14, 0}, extent = {{-45, -45}, {45, 45}})));
  
protected
  // Supports
  Modelica.Mechanics.MultiBody.Parts.Fixed upper_i(r = (RrRigidDoubleWishbone.upper_fore_i + RrRigidDoubleWishbone.upper_aft_i) / 2, animation = false) annotation(
    Placement(transformation(origin = {90, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed lower_i(r = (RrRigidDoubleWishbone.lower_fore_i + RrRigidDoubleWishbone.lower_aft_i) / 2, animation = false) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed tie_i(r = RrRigidDoubleWishbone.tie_i, animation = false) annotation(
    Placement(transformation(origin = {90, 0}, extent = {{10, -10}, {-10, 10}})));
  // Ground
  Utilities.Mechanics.Multibody.GroundPhysics groundPhysics(d = 10000)  annotation(
    Placement(transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}})));
  // Placeholder tire
  // Torque application to fully define system
public
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire mF5p2Tire annotation(
    Placement(transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Sources.Ramp torque_ramp(duration = 1, height = 0, startTime = 0) annotation(
    Placement(transformation(origin = {-140, 32}, extent = {{-10, -10}, {10, 10}})));  equation
  connect(lower_i.frame_b, RrRigidDoubleWishbone.lower_i_frame) annotation(
    Line(points = {{80, -30}, {46, -30}}, color = {95, 95, 95}));
  connect(tie_i.frame_b, RrRigidDoubleWishbone.tie_i_frame) annotation(
    Line(points = {{80, 0}, {46, 0}}, color = {95, 95, 95}));
  connect(upper_i.frame_b, RrRigidDoubleWishbone.upper_i_frame) annotation(
    Line(points = {{80, 30}, {46, 30}}, color = {95, 95, 95}));
  connect(world.frame_b, groundPhysics.frame_a) annotation(
    Line(points = {{-130, -90}, {-120, -90}, {-120, -40}, {-100, -40}}, color = {95, 95, 95}));
  connect(RrRigidDoubleWishbone.midpoint_frame, mF5p2Tire.chassis_frame) annotation(
    Line(points = {{-44, 0}, {-80, 0}}, color = {95, 95, 95}));
  connect(mF5p2Tire.cp_frame, groundPhysics.frame_b) annotation(
    Line(points = {{-90, -10}, {-90, -30}}, color = {95, 95, 95}));
  connect(torque_ramp.y, mF5p2Tire.hub_torque) annotation(
    Line(points = {{-128, 32}, {-92, 32}, {-92, 12}}, color = {0, 0, 127}));
  annotation(
    Diagram(coordinateSystem(extent = {{-220, 100}, {100, -100}})),
  experiment(StartTime = 0, StopTime = 2, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end TestRrRigidDoubleWishbone;

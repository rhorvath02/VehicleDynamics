within VehicleDynamics.Vehicle.Chassis;

model RigidChassis
  import Modelica.Math.Vectors.norm;
  VehicleDynamics.Vehicle.Chassis.Suspension.FrRigidAxleBellcrank FrAxle annotation(
    Placement(transformation(origin = {0, 47}, extent = {{-20, -20}, {20, 20}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.RrRigidAxleBellcrank RrAxle annotation(
    Placement(transformation(origin = {0, -47}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
  // Torque inputs
  Modelica.Blocks.Interfaces.RealInput FL_torque annotation(
    Placement(transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput FR_torque annotation(
    Placement(transformation(origin = {120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -0), iconTransformation(origin = {120, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput RL_torque annotation(
    Placement(transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput RR_torque annotation(
    Placement(transformation(origin = {120, -60}, extent = {{20, -20}, {-20, 20}}, rotation = -0), iconTransformation(origin = {120, -66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  // Steering input
  Modelica.Blocks.Interfaces.RealInput rack_input annotation(
    Placement(transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  // Sprung mass
  Modelica.Mechanics.MultiBody.Parts.Body sprung_mass(r_CM = (RrAxle.effective_center - FrAxle.effective_center)/2, m = 200, I_11 = 30, I_22 = 40, I_33 = 50, r_0(start = {0, 0, 0.19}, each fixed = true)) annotation(
    Placement(transformation(origin = {20, 10}, extent = {{-10, -10}, {10, 10}})));
  // World frame for "grounding"
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b world_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
protected
  // Ground elements
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  // Front to rear connection
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = RrAxle.effective_center - FrAxle.effective_center) annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90)));
equation
  connect(FL_torque, FrAxle.FL_torque) annotation(
    Line(points = {{-120, 60}, {-24, 60}}, color = {0, 0, 127}));
  connect(rack_input, FrAxle.steer_input) annotation(
    Line(points = {{0, 120}, {0, 72}}, color = {0, 0, 127}));
  connect(FR_torque, FrAxle.FR_torque) annotation(
    Line(points = {{120, 60}, {24, 60}}, color = {0, 0, 127}));
  connect(RL_torque, RrAxle.RL_torque) annotation(
    Line(points = {{-120, -60}, {-24, -60}}, color = {0, 0, 127}));
  connect(RR_torque, RrAxle.RR_torque) annotation(
    Line(points = {{120, -60}, {24, -60}}, color = {0, 0, 127}));
  connect(FrAxle.FL_cp, FL_ground.frame_b) annotation(
    Line(points = {{-20, 48}, {-50, 48}, {-50, 20}}, color = {95, 95, 95}));
  connect(FrAxle.FR_cp, FR_ground.frame_b) annotation(
    Line(points = {{20, 48}, {50, 48}, {50, 20}}, color = {95, 95, 95}));
  connect(FL_ground.frame_a, world_frame) annotation(
    Line(points = {{-60, 10}, {-80, 10}, {-80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_ground.frame_a, world_frame) annotation(
    Line(points = {{60, 10}, {80, 10}, {80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_ground.frame_b, RrAxle.RL_cp) annotation(
    Line(points = {{-50, -40}, {-50, -46}, {-20, -46}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, RrAxle.RR_cp) annotation(
    Line(points = {{50, -40}, {50, -46}, {20, -46}}, color = {95, 95, 95}));
  connect(RL_ground.frame_a, world_frame) annotation(
    Line(points = {{-60, -30}, {-80, -30}, {-80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_ground.frame_a, world_frame) annotation(
    Line(points = {{60, -30}, {80, -30}, {80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(FrAxle.axle_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{0, 28}, {0, 10}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, RrAxle.axle_frame) annotation(
    Line(points = {{0, -10}, {0, -26}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, sprung_mass.frame_a) annotation(
    Line(points = {{0, 10}, {10, 10}}, color = {95, 95, 95}));
end RigidChassis;

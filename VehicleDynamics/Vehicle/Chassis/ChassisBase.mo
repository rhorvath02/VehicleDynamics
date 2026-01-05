within VehicleDynamics.Vehicle.Chassis;
model ChassisBase
  import Modelica.Math.Vectors.norm;
  
  // TODO: add contents
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.FrAxleBellcrank FrAxle annotation(
    Placement(transformation(origin = {0, 47}, extent = {{-20, -20}, {20, 20}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.RrAxleBellcrank RrAxle annotation(
    Placement(transformation(origin = {0, -47}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
  // Torque inputs
  Modelica.Blocks.Interfaces.RealInput FL_torque annotation(
    Placement(transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput RL_torque annotation(
    Placement(transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput RR_torque annotation(
    Placement(transformation(origin = {120, -60}, extent = {{20, -20}, {-20, 20}}, rotation = -0), iconTransformation(origin = {120, -66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_fixed(r = {0, 0.609600, 0}, animation = false) annotation(
    Placement(transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed FR_fixed(r = {0, -0.609600, 0}, animation = false) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RL_fixed(r = {-61*0.0254, 0.609600, 0}, animation = false) annotation(
    Placement(transformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed RR_fixed(r = {-61*0.0254, -0.609600, 0}, animation = false) annotation(
    Placement(transformation(origin = {90, -40}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Interfaces.RealInput FR_torque annotation(
    Placement(transformation(origin = {120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -0), iconTransformation(origin = {120, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = (RrAxle.effective_center - FrAxle.effective_center) / 2, m=200, I_11=30, I_22=40, I_33=50, r_0(start = {0, 0, 0.187959})) annotation(
    Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = RrAxle.effective_center - FrAxle.effective_center) annotation(
    Placement(transformation( extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Ramp ramp1(duration = 3, height = 0.5*0.0254, startTime = 5) annotation(
    Placement(transformation(origin = {-30, 90}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(FL_torque, FrAxle.FL_torque) annotation(
    Line(points = {{-120, 60}, {-24, 60}}, color = {0, 0, 127}));
  connect(FR_torque, FrAxle.FR_torque) annotation(
    Line(points = {{120, 60}, {24, 60}}, color = {0, 0, 127}));
  connect(RL_torque, RrAxle.RL_torque) annotation(
    Line(points = {{-120, -60}, {-24, -60}}, color = {0, 0, 127}));
  connect(RR_torque, RrAxle.RR_torque) annotation(
    Line(points = {{120, -60}, {24, -60}}, color = {0, 0, 127}));
  connect(FL_fixed.frame_b, FL_ground.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(FL_ground.frame_b, FrAxle.FL_cp) annotation(
    Line(points = {{-50, 20}, {-50, 28}, {-18, 28}}, color = {95, 95, 95}));
  connect(FR_fixed.frame_b, FR_ground.frame_a) annotation(
    Line(points = {{80, 10}, {60, 10}}, color = {95, 95, 95}));
  connect(FR_ground.frame_b, FrAxle.FR_cp) annotation(
    Line(points = {{50, 20}, {50, 28}, {18, 28}}, color = {95, 95, 95}));
  connect(RL_fixed.frame_b, RL_ground.frame_a) annotation(
    Line(points = {{-80, -40}, {-60, -40}}, color = {95, 95, 95}));
  connect(RL_ground.frame_b, RrAxle.RL_cp) annotation(
    Line(points = {{-50, -30}, {-50, -26}, {-18, -26}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, RrAxle.RR_cp) annotation(
    Line(points = {{50, -30}, {50, -26}, {18, -26}}, color = {95, 95, 95}));
  connect(RR_fixed.frame_b, RR_ground.frame_a) annotation(
    Line(points = {{80, -40}, {60, -40}}, color = {95, 95, 95}));
  connect(body.frame_a, FrAxle.axle_frame) annotation(
    Line(points = {{40, 50}, {40, 50.5}, {0, 50.5}, {0, 28}}, color = {95, 95, 95}));
  connect(FrAxle.axle_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{0, 28}, {0, 10}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, RrAxle.axle_frame) annotation(
    Line(points = {{0, -10}, {0, -26}}, color = {95, 95, 95}));
  connect(ramp1.y, FrAxle.steer_input) annotation(
    Line(points = {{-18, 90}, {0, 90}, {0, 72}}, color = {0, 0, 127}));
end ChassisBase;
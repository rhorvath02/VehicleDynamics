within BobDynamics.Vehicle.Chassis;

model RigidChassis
  // Modelica linalg
  import Modelica.Math.Vectors.norm;
  
  BobDynamics.Vehicle.Chassis.Suspension.FrAxleDWPushBCARB FrAxle(final link_diameter = 0.025,
                                                                  final joint_diameter = 0.030)  annotation(
    Placement(transformation(origin = {0, 47}, extent = {{-20, -20}, {20, 20}})));
  BobDynamics.Vehicle.Chassis.Suspension.RrAxleDWPullBCARB RrAxle(final link_diameter = 0.025,
                                                                  final joint_diameter = 0.030)  annotation(
    Placement(transformation(origin = {0, -47}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
  final Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = RrAxle.effective_center - FrAxle.effective_center) annotation(
    Placement(transformation( extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  
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
  final Modelica.Mechanics.MultiBody.Parts.Body sprung_mass(r_CM = (RrAxle.effective_center - FrAxle.effective_center)/2,
                                                            m = 200,
                                                            I_11 = 30,
                                                            I_22 = 40,
                                                            I_33 = 50,
                                                            r_0(start = {0, 0, 0.19696}, each fixed = true),
                                                            enforceStates = true,
                                                            useQuaternions = false) annotation(
    Placement(transformation(origin = {20, 10}, extent = {{-10, -10}, {10, 10}})));
  
  // World frame for "grounding"
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b world_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  
protected
  // Ground elements
  BobDynamics.Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  BobDynamics.Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  BobDynamics.Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

equation
  connect(FL_torque, FrAxle.left_torque) annotation(
    Line(points = {{-120, 60}, {-24, 60}}, color = {0, 0, 127}));
  connect(rack_input, FrAxle.steer_input) annotation(
    Line(points = {{0, 120}, {0, 72}}, color = {0, 0, 127}));
  connect(FR_torque, FrAxle.right_torque) annotation(
    Line(points = {{120, 60}, {24, 60}}, color = {0, 0, 127}));
  connect(RL_torque, RrAxle.left_torque) annotation(
    Line(points = {{-120, -60}, {-24, -60}}, color = {0, 0, 127}));
  connect(RR_torque, RrAxle.right_torque) annotation(
    Line(points = {{120, -60}, {24, -60}}, color = {0, 0, 127}));
  connect(FrAxle.left_cp, FL_ground.frame_b) annotation(
    Line(points = {{-20, 48}, {-50, 48}, {-50, 20}}, color = {95, 95, 95}));
  connect(FrAxle.right_cp, FR_ground.frame_b) annotation(
    Line(points = {{20, 48}, {50, 48}, {50, 20}}, color = {95, 95, 95}));
  connect(FL_ground.frame_a, world_frame) annotation(
    Line(points = {{-60, 10}, {-80, 10}, {-80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(FR_ground.frame_a, world_frame) annotation(
    Line(points = {{60, 10}, {80, 10}, {80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(RL_ground.frame_b, RrAxle.left_cp) annotation(
    Line(points = {{-50, -40}, {-50, -46}, {-20, -46}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, RrAxle.right_cp) annotation(
    Line(points = {{50, -40}, {50, -46}, {20, -46}}, color = {95, 95, 95}));
  connect(RL_ground.frame_a, world_frame) annotation(
    Line(points = {{-60, -30}, {-80, -30}, {-80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(RR_ground.frame_a, world_frame) annotation(
    Line(points = {{60, -30}, {80, -30}, {80, -80}, {0, -80}, {0, -100}}, color = {95, 95, 95}));
  connect(FrAxle.axle_frame, fixedTranslation.frame_a) annotation(
    Line(points = {{0, 28}, {0, 10}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, RrAxle.axle_frame) annotation(
    Line(points = {{0, -10}, {0, -26}}, color = {95, 95, 95}));
  connect(sprung_mass.frame_a, fixedTranslation.frame_a) annotation(
    Line(points = {{10, 10}, {0, 10}}, color = {95, 95, 95}));
end RigidChassis;

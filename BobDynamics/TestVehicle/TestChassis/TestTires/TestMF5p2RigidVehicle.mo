within BobDynamics.TestVehicle.TestChassis.TestTires;
model TestMF5p2RigidVehicle
  import Modelica.Math.Vectors.norm;
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  // Outputs
  Real body_accels[3];
  Real normal_loads[4];
  // Tires
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire FL_tire  annotation(
    Placement(transformation(origin = {-70, 80}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire FR_tire  annotation(
    Placement(transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}})));
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire RL_tire  annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire RR_tire  annotation(
    Placement(transformation(origin = {50, -20}, extent = {{-10, -10}, {10, 10}})));
  
  
protected
  // System inputs
  Modelica.Blocks.Sources.Ramp rack_disp(height = 10*Modelica.Constants.pi/180, duration = 0.5, startTime = 10)  annotation(
    Placement(transformation(origin = {-70, 130}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Sources.Ramp FL_torque(height = 0, duration = 1, startTime = 1) annotation(
    Placement(transformation(origin = {-90, 100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp FR_torque(height = 0, duration = 1, startTime = 1) annotation(
    Placement(transformation(origin = {90, 100}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

// Ground interfaces
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_fixed(r = {1, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FR_fixed(r = {1, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RL_fixed(r = {-1, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RR_fixed(r = {-1, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {90, -60}, extent = {{10, -10}, {-10, 10}})));

  Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 40}, extent = {{10, -10}, {-10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}})));
      // Vehicle geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_track(r = {0, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-20, 80}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_track(r = {0, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_track(r = {0, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_track(r = {0, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation Fr_to_Rr_track(r = {-2, 0, 0}, animation = false)  annotation(
    Placement(transformation(origin = {0, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation Fr_to_CG(r = {-0.78, 0, 0}, animation = false)  annotation(
    Placement(transformation(origin = {20, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  // Vehicle mass
  Modelica.Mechanics.MultiBody.Parts.Body Body(r_CM = {0, 0, 0}, m = 150, r_0(start = {0, 0, FL_tire.R0}, each fixed = true), I_11 = 40, I_22 = 50, I_33 = 60, animation = false, w_a(start = {0, 0, 0}, each fixed = true), v_0(start = {0, 0, 0}, each fixed = true))  annotation(
    Placement(transformation(origin = {20, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  // Steering interface
  Modelica.Mechanics.MultiBody.Joints.Revolute left_revolute(useAxisFlange = true, animation = false, w(start = 0, each fixed = true))  annotation(
    Placement(transformation(origin = {-44, 80}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute right_revolute(useAxisFlange = true, animation = false, w(start = 0, each fixed = true))  annotation(
    Placement(transformation(origin = {44, 80}, extent = {{-10, -10}, {10, 10}})));
    
  Modelica.Mechanics.Rotational.Sources.Position left_steer(useSupport = true)  annotation(
    Placement(transformation(origin = {-44, 106}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position right_steer(useSupport = true) annotation(
    Placement(transformation(origin = {44, 106}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));public
  Modelica.Blocks.Sources.RealExpression realExpression(y = norm({Body.v_0[1], Body.v_0[2], 0})) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Continuous.LimPID PID(Td = 0, Ti = 5, initType = Modelica.Blocks.Types.InitPID.InitialOutput, k = 30, yMax = 250, y_start = 0) annotation(
    Placement(transformation(origin = {0, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 270)));
  Modelica.Blocks.Sources.RealExpression realExpression3(y = 20) annotation(
    Placement(transformation(origin = {0, -80}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
equation
  body_accels = Modelica.Mechanics.MultiBody.Frames.resolve2(Body.frame_a.R, Body.a_0);
  normal_loads[1] = FL_tire.Fz;
  normal_loads[2] = FR_tire.Fz;
  normal_loads[3] = RL_tire.Fz;
  normal_loads[4] = RR_tire.Fz;
  connect(FL_fixed.frame_b, FL_ground.frame_a) annotation(
    Line(points = {{-80, 40}, {-60, 40}}, color = {95, 95, 95}));
  connect(FL_ground.frame_b, FL_tire.cp_frame) annotation(
    Line(points = {{-50, 50}, {-50, 60}, {-70, 60}, {-70, 70}}, color = {95, 95, 95}));
  connect(FR_fixed.frame_b, FR_ground.frame_a) annotation(
    Line(points = {{80, 40}, {60, 40}}, color = {95, 95, 95}));
  connect(FR_ground.frame_b, FR_tire.cp_frame) annotation(
    Line(points = {{50, 50}, {50, 60}, {70, 60}, {70, 70}}, color = {95, 95, 95}));
  connect(RL_fixed.frame_b, RL_ground.frame_a) annotation(
    Line(points = {{-80, -60}, {-60, -60}}, color = {95, 95, 95}));
  connect(RL_ground.frame_b, RL_tire.cp_frame) annotation(
    Line(points = {{-50, -50}, {-50, -30}}, color = {95, 95, 95}));
  connect(RR_fixed.frame_b, RR_ground.frame_a) annotation(
    Line(points = {{80, -60}, {60, -60}}, color = {95, 95, 95}));
  connect(RR_ground.frame_b, RR_tire.cp_frame) annotation(
    Line(points = {{50, -50}, {50, -30}}, color = {95, 95, 95}));
  connect(Fr_to_Rr_track.frame_a, FL_track.frame_a) annotation(
    Line(points = {{0, 40}, {0, 80}, {-10, 80}}, color = {95, 95, 95}));
  connect(Fr_to_Rr_track.frame_a, FR_track.frame_a) annotation(
    Line(points = {{0, 40}, {0, 80}, {10, 80}}, color = {95, 95, 95}));
  connect(Fr_to_Rr_track.frame_b, RL_track.frame_a) annotation(
    Line(points = {{0, 20}, {0, -20}, {-10, -20}}, color = {95, 95, 95}));
  connect(RL_track.frame_b, RL_tire.chassis_frame) annotation(
    Line(points = {{-30, -20}, {-40, -20}}, color = {95, 95, 95}));
  connect(Fr_to_Rr_track.frame_b, RR_track.frame_a) annotation(
    Line(points = {{0, 20}, {0, -20}, {10, -20}}, color = {95, 95, 95}));
  connect(RR_track.frame_b, RR_tire.chassis_frame) annotation(
    Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
  connect(Fr_to_CG.frame_a, Fr_to_Rr_track.frame_a) annotation(
    Line(points = {{20, 40}, {20, 60}, {0, 60}, {0, 40}}, color = {95, 95, 95}));
  connect(Body.frame_a, Fr_to_CG.frame_b) annotation(
    Line(points = {{20, 10}, {20, 20}}, color = {95, 95, 95}));
  connect(FL_track.frame_b, left_revolute.frame_a) annotation(
    Line(points = {{-30, 80}, {-34, 80}}, color = {95, 95, 95}));
  connect(left_revolute.frame_b, FL_tire.chassis_frame) annotation(
    Line(points = {{-54, 80}, {-60, 80}}, color = {95, 95, 95}));
  connect(FR_track.frame_b, right_revolute.frame_a) annotation(
    Line(points = {{30, 80}, {34, 80}}, color = {95, 95, 95}));
  connect(right_revolute.frame_b, FR_tire.chassis_frame) annotation(
    Line(points = {{54, 80}, {60, 80}}, color = {95, 95, 95}));
  connect(left_steer.support, left_revolute.support) annotation(
    Line(points = {{-34, 106}, {-34, 90}, {-38, 90}}));
  connect(rack_disp.y, left_steer.phi_ref) annotation(
    Line(points = {{-58, 130}, {-44, 130}, {-44, 118}}, color = {0, 0, 127}));
  connect(left_steer.flange, left_revolute.axis) annotation(
    Line(points = {{-44, 96}, {-44, 90}}));
  connect(right_steer.support, right_revolute.support) annotation(
    Line(points = {{34, 106}, {34, 90}, {38, 90}}));
  connect(right_steer.flange, right_revolute.axis) annotation(
    Line(points = {{44, 96}, {44, 90}}));
  connect(right_steer.phi_ref, rack_disp.y) annotation(
    Line(points = {{44, 118}, {44, 130}, {-58, 130}}, color = {0, 0, 127}));
  connect(FL_torque.y, FL_tire.hub_torque) annotation(
    Line(points = {{-78, 100}, {-72, 100}, {-72, 92}}, color = {0, 0, 127}));
  connect(FR_torque.y, FR_tire.hub_torque) annotation(
    Line(points = {{80, 100}, {72, 100}, {72, 92}}, color = {0, 0, 127}));
  connect(PID.y, RL_tire.hub_torque) annotation(
    Line(points = {{0, -39}, {0, 0}, {-52, 0}, {-52, -8}}, color = {0, 0, 127}));
  connect(PID.y, RR_tire.hub_torque) annotation(
    Line(points = {{0, -39}, {0, 0}, {52, 0}, {52, -8}}, color = {0, 0, 127}));
  connect(realExpression.y, PID.u_m) annotation(
    Line(points = {{-20, -59}, {-20, -50}, {-12, -50}}, color = {0, 0, 127}));
  connect(realExpression3.y, PID.u_s) annotation(
    Line(points = {{0, -69}, {0, -62}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 16, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
  __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "cvode", variableFilter = ".*"));
end TestMF5p2RigidVehicle;

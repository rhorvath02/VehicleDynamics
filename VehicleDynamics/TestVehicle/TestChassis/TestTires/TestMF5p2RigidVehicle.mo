within VehicleDynamics.TestVehicle.TestChassis.TestTires;
model TestMF5p2RigidVehicle
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Outputs
  Real body_accels[3];
  output Real Ax;
  output Real Ay;
  
  Real mu_eff_total;
  Real eps = 1e-6;
  
  Real Fz_FL;
  Real Fz_FR;
  Real Fz_RL;
  Real Fz_RR;
  
  Real Fz_Fr;
  Real Fz_Rr;
  
  Real Fz_Lf;
  Real Fz_Rt;
  
  Real Fz_tot;
  Real Ay_calc;
  
  parameter String tir_path =
  Modelica.Utilities.Files.loadResource(
    "modelica://VehicleDynamics/Resources/JSONs/SUS/placeholder.tir");
    
  // Tires
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire FL_tire  annotation(
    Placement(transformation(origin = {-70, 80}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire FR_tire  annotation(
    Placement(transformation(origin = {70, 80}, extent = {{-10, -10}, {10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire RL_tire  annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire RR_tire  annotation(
    Placement(transformation(origin = {50, -20}, extent = {{-10, -10}, {10, 10}})));
  
  // Ground Interfaces
  Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_fixed(r = {1, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FR_fixed(r = {1, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RL_fixed(r = {-1, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RR_fixed(r = {-1, -1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {90, -60}, extent = {{10, -10}, {-10, 10}})));

  // Vehicle Geometry
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_track(r = {0, 1, 0})  annotation(
    Placement(transformation(origin = {-20, 80}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_track(r = {0, -1, 0})  annotation(
    Placement(transformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_track(r = {0, 1, 0})  annotation(
    Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_track(r = {0, -1, 0})  annotation(
    Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation Fr_to_Rr_track(r = {-2, 0, 0})  annotation(
    Placement(transformation(origin = {0, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation Fr_to_CG(r = {-0.78, 0, 0})  annotation(
    Placement(transformation(origin = {20, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  
  // Vehicle Mass
  Modelica.Mechanics.MultiBody.Parts.Body Body(r_CM = {0, 0, 0}, m = 150, r_0(start = {0, 0, FL_tire.R0}, each fixed = true), I_11 = 40, I_22 = 50, I_33 = 60)  annotation(
    Placement(transformation(origin = {20, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Ramp ramp(height = 50, duration = 1, startTime = 1)  annotation(
    Placement(transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp1(duration = 1, height = 50, startTime = 1) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {-44, 80}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange = true)  annotation(
    Placement(transformation(origin = {44, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp2(height = 15*Modelica.Constants.pi/180, duration = 0.5, startTime = 2)  annotation(
    Placement(transformation(origin = {-70, 130}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position position(useSupport = true)  annotation(
    Placement(transformation(origin = {-44, 106}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position position1(useSupport = true) annotation(
    Placement(transformation(origin = {44, 106}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Ramp ramp3(duration = 1, height = 0, startTime = 1) annotation(
    Placement(transformation(origin = {-90, 100}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp4(duration = 1, height = 0, startTime = 1) annotation(
    Placement(transformation(origin = {90, 100}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
    
equation
  mu_eff_total = sqrt((FL_tire.Fx + FR_tire.Fx + RL_tire.Fx + RR_tire.Fx)^2 + (FL_tire.Fy + FR_tire.Fy + RL_tire.Fy + RR_tire.Fy)^2)/max(FL_tire.Fz + FR_tire.Fz + RL_tire.Fz + RR_tire.Fz, eps);
  body_accels = Modelica.Mechanics.MultiBody.Frames.resolve2(Body.frame_a.R, Body.a_0);
  Ax = body_accels[1]/9.81;
  Ay = body_accels[2]/9.81;
  Fz_FL = FL_tire.Fz;
  Fz_FR = FR_tire.Fz;
  Fz_RL = RL_tire.Fz;
  Fz_RR = RR_tire.Fz;
  Fz_Fr = Fz_FL + Fz_FR;
  Fz_Rr = Fz_RL + Fz_RR;
  Fz_Lf = Fz_FL + Fz_RL;
  Fz_Rt = Fz_FR + Fz_RR;
  Fz_tot = Fz_FL + Fz_FR + Fz_RL + Fz_RR;
  Ay_calc = (FL_tire.Fy + FR_tire.Fy + RL_tire.Fy + RR_tire.Fy)/154/9.81;
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
  connect(FL_track.frame_b, revolute.frame_a) annotation(
    Line(points = {{-30, 80}, {-34, 80}}, color = {95, 95, 95}));
  connect(revolute.frame_b, FL_tire.chassis_frame) annotation(
    Line(points = {{-54, 80}, {-60, 80}}, color = {95, 95, 95}));
  connect(FR_track.frame_b, revolute1.frame_a) annotation(
    Line(points = {{30, 80}, {34, 80}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, FR_tire.chassis_frame) annotation(
    Line(points = {{54, 80}, {60, 80}}, color = {95, 95, 95}));
  connect(position.support, revolute.support) annotation(
    Line(points = {{-34, 106}, {-34, 90}, {-38, 90}}));
  connect(ramp2.y, position.phi_ref) annotation(
    Line(points = {{-58, 130}, {-44, 130}, {-44, 118}}, color = {0, 0, 127}));
  connect(position.flange, revolute.axis) annotation(
    Line(points = {{-44, 96}, {-44, 90}}));
  connect(position1.support, revolute1.support) annotation(
    Line(points = {{34, 106}, {34, 90}, {38, 90}}));
  connect(position1.flange, revolute1.axis) annotation(
    Line(points = {{44, 96}, {44, 90}}));
  connect(position1.phi_ref, ramp2.y) annotation(
    Line(points = {{44, 118}, {44, 130}, {-58, 130}}, color = {0, 0, 127}));
  connect(ramp.y, RL_tire.hub_torque) annotation(
    Line(points = {{-79, 10}, {-52.5, 10}, {-52.5, -8}, {-52, -8}}, color = {0, 0, 127}));
  connect(ramp1.y, RR_tire.hub_torque) annotation(
    Line(points = {{80, 10}, {52, 10}, {52, -8}}, color = {0, 0, 127}));
  connect(ramp3.y, FL_tire.hub_torque) annotation(
    Line(points = {{-78, 100}, {-72, 100}, {-72, 92}}, color = {0, 0, 127}));
  connect(ramp4.y, FR_tire.hub_torque) annotation(
    Line(points = {{80, 100}, {72, 100}, {72, 92}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-08, Interval = 0.002));
end TestMF5p2RigidVehicle;
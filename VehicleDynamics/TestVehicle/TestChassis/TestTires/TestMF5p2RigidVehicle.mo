within VehicleDynamics.TestVehicle.TestChassis.TestTires;
model TestMF5p2RigidVehicle
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Tires
  Vehicle.Chassis.Tires.MF5p2Tire FL_tire annotation(
    Placement(transformation(origin = {-50, 80}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Tires.MF5p2Tire FR_tire annotation(
    Placement(transformation(origin = {50, 80}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Tires.MF5p2Tire RL_tire annotation(
    Placement(transformation(origin = {-50, -20}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Tires.MF5p2Tire RR_tire annotation(
    Placement(transformation(origin = {50, -20}, extent = {{-10, -10}, {10, 10}})));
  
  // Ground Interfaces
  Utilities.Mechanics.Multibody.GroundPhysics FL_ground annotation(
    Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_fixed(r = {1, 1, 0})  annotation(
    Placement(transformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics FR_ground annotation(
    Placement(transformation(origin = {50, 40}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FR_fixed(r = {1, -1, 0})  annotation(
    Placement(transformation(origin = {90, 40}, extent = {{10, -10}, {-10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RL_ground annotation(
    Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RL_fixed(r = {-1, 1, 0})  annotation(
    Placement(transformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.GroundPhysics RR_ground annotation(
    Placement(transformation(origin = {50, -60}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed RR_fixed(r = {-1, -1, 0})  annotation(
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
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation Fr_to_CG(r = {-1, 0, 0})  annotation(
    Placement(transformation(origin = {20, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  
  // Vehicle Mass
  Modelica.Mechanics.MultiBody.Parts.Body Body(r_CM = {0, 0, 0}, m = 50, r_0(start = {0, 0, FL_tire.R0}, each fixed = true))  annotation(
    Placement(transformation(origin = {20, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    
equation
  connect(FL_fixed.frame_b, FL_ground.frame_a) annotation(
    Line(points = {{-80, 40}, {-60, 40}}, color = {95, 95, 95}));
  connect(FL_ground.frame_b, FL_tire.cp_frame) annotation(
    Line(points = {{-50, 50}, {-50, 70}}, color = {95, 95, 95}));
  connect(FR_fixed.frame_b, FR_ground.frame_a) annotation(
    Line(points = {{80, 40}, {60, 40}}, color = {95, 95, 95}));
  connect(FR_ground.frame_b, FR_tire.cp_frame) annotation(
    Line(points = {{50, 50}, {50, 70}}, color = {95, 95, 95}));
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
  connect(FL_track.frame_b, FL_tire.chassis_frame) annotation(
    Line(points = {{-30, 80}, {-40, 80}}, color = {95, 95, 95}));
  connect(Fr_to_Rr_track.frame_a, FR_track.frame_a) annotation(
    Line(points = {{0, 40}, {0, 80}, {10, 80}}, color = {95, 95, 95}));
  connect(FR_track.frame_b, FR_tire.chassis_frame) annotation(
    Line(points = {{30, 80}, {40, 80}}, color = {95, 95, 95}));
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
end TestMF5p2RigidVehicle;
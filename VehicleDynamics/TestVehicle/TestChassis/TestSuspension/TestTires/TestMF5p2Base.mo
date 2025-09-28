within VehicleDynamics.TestVehicle.TestChassis.TestSuspension.TestTires;

model TestMF5p2Base
  // Gravity vector
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  // Tire with zero toe and camber
  Vehicle.Chassis.Suspension.Tires.MF5p2Base MF5p2Base(static_gamma = 0, static_alpha = 0, PCX1 = 1.530410, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PHX1 = 0.0000000, PHX2 = 0.0000000, PVX1 = 0.0000000, PVX2 = 0.0000000, RBX1 = 8.151136, RBX2 = 5.388063, RCX1 = 1.122399, REX1 = 0.052014, REX2 = -0.898450, RHX1 = 0.0, PCY1 = 1.53041, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCY1 = 1.044, REY1 = 0.048, REY2 = 0.025, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, QBZ1 = 8.22843, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QBZ10 = -1.72926, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, LFZO = 1, LCX = 1, LMUX = 1, LEX = 1, LKX = 1, LHX = 1, LVX = 1, LXAL = 1, LGAX = 1, LCY = 1, LMUY = 1, LEY = 1, LKY = 1, LHY = 1, LVY = 1, LGAY = 1, LKYG = 1, LTR = 1, LRES = 1, LCZ = 1, LGAZ = 1, LYKA = 1, LVYKA = 1, LS = 1, LSGKP = 1, LSGAL = 1, LGYR = 1, LMX = 1, LVMX = 1, LMY = 1, LIP = 1, FNOMIN = 654, R0 = 8*0.0254, rim_width = 7*0.0254, rim_R0 = 5*0.0254, mu_s = 1, each v_rel = {5, 0, 0}, wheel_J = 0.2) annotation(
    Placement(transformation(origin = {30, 30}, extent = {{-10, -10}, {10, 10}})));
  // Reference point (origin)
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false, r = {0, 0, 0}) annotation(
    Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  // Steer input
  // Ground physics
  // Normal force sensor
  // Free only x, z, and yaw
  Modelica.Mechanics.MultiBody.Joints.Prismatic free_x(n = {1, 0, 0}, useAxisFlange = true, boxWidth = 0.005, boxHeight = 0.005, animation = false, v(start = 5, fixed = true)) annotation(
    Placement(transformation(origin = {30, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  // Constant velocity
  // Sprung mass
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 50, r_CM = {0, 0, 0}, sphereDiameter = 0.05) annotation(
    Placement(transformation(origin = {70, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(useAxisFlange = true) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sources.Position position annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp1(height = 25*Modelica.Constants.pi/180, duration = 2, startTime = 0) annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.WorldTorque torque annotation(
    Placement(transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 0, duration = 0)  annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression zero_expr annotation(
    Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression zero_expr1 annotation(
    Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Speed speed annotation(
    Placement(transformation(origin = {-10, -52}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const(k = 5)  annotation(
    Placement(transformation(origin = {-50, -52}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(body.frame_a, MF5p2Base.corner_frame) annotation(
    Line(points = {{60, 60}, {60, 61}, {30, 61}, {30, 40}}, color = {95, 95, 95}));
  connect(position.flange, revolute.axis) annotation(
    Line(points = {{0, 0}, {20, 0}}));
  connect(ramp1.y, position.phi_ref) annotation(
    Line(points = {{-39, 0}, {-23, 0}}, color = {0, 0, 127}));
  connect(fixed.frame_b, free_x.frame_a) annotation(
    Line(points = {{30, -80}, {30, -70}}, color = {95, 95, 95}));
  connect(revolute.frame_b, MF5p2Base.frame_a) annotation(
    Line(points = {{30, 10}, {30, 20}}, color = {95, 95, 95}));
  connect(torque.frame_b, MF5p2Base.hub_frame) annotation(
    Line(points = {{0, 30}, {20, 30}}, color = {95, 95, 95}));
  connect(zero_expr.y, torque.torque[1]) annotation(
    Line(points = {{-38, 70}, {-22, 70}, {-22, 30}}, color = {0, 0, 127}));
  connect(zero_expr1.y, torque.torque[3]) annotation(
    Line(points = {{-38, 50}, {-22, 50}, {-22, 30}}, color = {0, 0, 127}));
  connect(ramp.y, torque.torque[2]) annotation(
    Line(points = {{-38, 30}, {-22, 30}}, color = {0, 0, 127}));
  connect(free_x.frame_b, revolute.frame_a) annotation(
    Line(points = {{30, -50}, {30, -10}}, color = {95, 95, 95}));
  connect(speed.flange, free_x.axis) annotation(
    Line(points = {{0, -52}, {24, -52}}, color = {0, 127, 0}));
  connect(const.y, speed.v_ref) annotation(
    Line(points = {{-38, -52}, {-22, -52}}, color = {0, 0, 127}));
  annotation(
    experiment(StartTime = 0, StopTime = 3));
end TestMF5p2Base;

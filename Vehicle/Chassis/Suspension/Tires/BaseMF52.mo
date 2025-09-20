within Vehicle.Chassis.Suspension.Tires;

model BaseMF52
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import Modelica.SIunits;
  
  // Read tir file
  parameter String tir_path = "/home/rhorvath/Documents/Github/VehicleDynamics/JSONs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir";
  inner ExternData.TIRFile tir_file(fileName=tir_path)  annotation(
      Placement(transformation(origin = {-90, -90}, extent = {{10, -10}, {-10, 10}})));
  
  // Static angles
  parameter SIunits.Angle static_gamma;
  parameter SIunits.Angle static_alpha;
  
  SIunits.Angle gamma = 0;
  SIunits.Force Fx;
  SIunits.Force Fy;
  SIunits.Torque Mx;
  SIunits.Torque My;
  SIunits.Torque Mz;
  SIunits.Length pneu_trail;
  SIunits.Length pneu_scrub;
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));

  Real Fz;
  Real wheel_vel[3];
  Real wheel_heading[3];
  Real alpha;
  Real kappa;
  
  Real v_x_wheel;
  
  // Pure longitudinal slip coefficients
  parameter Real PCX1 = tir_file.getReal("PCX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PDX1 = tir_file.getReal("PDX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PDX2 = tir_file.getReal("PDX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PDX3 = tir_file.getReal("PDX3", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PEX1 = tir_file.getReal("PEX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PEX2 = tir_file.getReal("PEX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PEX3 = tir_file.getReal("PEX3", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PEX4 = tir_file.getReal("PEX4", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PKX1 = tir_file.getReal("PKX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PKX2 = tir_file.getReal("PKX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PKX3 = tir_file.getReal("PKX3", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PHX1 = tir_file.getReal("PHX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PHX2 = tir_file.getReal("PHX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PVX1 = tir_file.getReal("PVX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  parameter Real PVX2 = tir_file.getReal("PVX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fx"));
  // Combined longitudinal slip coefficients
  parameter Real RBX1 = tir_file.getReal("RBX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  parameter Real RBX2 = tir_file.getReal("RBX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  parameter Real RCX1 = tir_file.getReal("RCX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  parameter Real REX1 = tir_file.getReal("REX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  parameter Real REX2 = tir_file.getReal("REX2", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  parameter Real RHX1 = tir_file.getReal("RHX1", "LONGITUDINAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fx"));
  // Pure lateral slip coefficients
  parameter Real PCY1 = tir_file.getReal("PCY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PDY1 = tir_file.getReal("PDY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PDY2 = tir_file.getReal("PDY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PDY3 = tir_file.getReal("PDY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PEY1 = tir_file.getReal("PEY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PEY2 = tir_file.getReal("PEY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PEY3 = tir_file.getReal("PEY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PEY4 = tir_file.getReal("PEY4", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PKY1 = tir_file.getReal("PKY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PKY2 = tir_file.getReal("PKY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PKY3 = tir_file.getReal("PKY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PHY1 = tir_file.getReal("PHY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PHY2 = tir_file.getReal("PHY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PHY3 = tir_file.getReal("PHY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PVY1 = tir_file.getReal("PVY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PVY2 = tir_file.getReal("PVY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PVY3 = tir_file.getReal("PVY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  parameter Real PVY4 = tir_file.getReal("PVY4", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Fy"));
  // Combined lateral slip coefficients
  parameter Real RBY1 = tir_file.getReal("RBY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RBY2 = tir_file.getReal("RBY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RBY3 = tir_file.getReal("RBY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RCY1 = tir_file.getReal("RCY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real REY1 = tir_file.getReal("REY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real REY2 = tir_file.getReal("REY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RHY1 = tir_file.getReal("RHY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RHY2 = tir_file.getReal("RHY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY1 = tir_file.getReal("RVY1", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY2 = tir_file.getReal("RVY2", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY3 = tir_file.getReal("RVY3", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY4 = tir_file.getReal("RVY4", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY5 = tir_file.getReal("RVY5", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  parameter Real RVY6 = tir_file.getReal("RVY6", "LATERAL_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Fy"));
  // Overturning coefficients
  parameter Real QSX1 = tir_file.getReal("QSX1", "OVERTURNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All Mx"));
  parameter Real QSX2 = tir_file.getReal("QSX2", "OVERTURNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All Mx"));
  parameter Real QSX3 = tir_file.getReal("QSX3", "OVERTURNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All Mx"));
  // Rolling resistance coefficients
  parameter Real QSY1 = tir_file.getReal("QSY1", "ROLLING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All My"));
  parameter Real QSY2 = tir_file.getReal("QSY2", "ROLLING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All My"));
  parameter Real QSY3 = tir_file.getReal("QSY3", "ROLLING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All My"));
  parameter Real QSY4 = tir_file.getReal("QSY4", "ROLLING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "All My"));
  // Pure aligning coefficients
  parameter Real QBZ1 = tir_file.getReal("QBZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ2 = tir_file.getReal("QBZ2", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ3 = tir_file.getReal("QBZ3", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ4 = tir_file.getReal("QBZ4", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ5 = tir_file.getReal("QBZ5", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ9 = tir_file.getReal("QBZ9", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QBZ10 = tir_file.getReal("QBZ10", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QCZ1 = tir_file.getReal("QCZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ1 = tir_file.getReal("QDZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ2 = tir_file.getReal("QDZ2", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ3 = tir_file.getReal("QDZ3", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ4 = tir_file.getReal("QDZ4", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ6 = tir_file.getReal("QDZ6", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ7 = tir_file.getReal("QDZ7", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ8 = tir_file.getReal("QDZ8", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QDZ9 = tir_file.getReal("QDZ9", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QEZ1 = tir_file.getReal("QEZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QEZ2 = tir_file.getReal("QEZ2", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QEZ3 = tir_file.getReal("QEZ3", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QEZ4 = tir_file.getReal("QEZ4", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QEZ5 = tir_file.getReal("QEZ5", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QHZ1 = tir_file.getReal("QHZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QHZ2 = tir_file.getReal("QHZ2", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QHZ3 = tir_file.getReal("QHZ3", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  parameter Real QHZ4 = tir_file.getReal("QHZ4", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Pure Mz"));
  // Combined aligning coefficients
  parameter Real SSZ1 = tir_file.getReal("SSZ1", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Mz"));
  parameter Real SSZ2 = tir_file.getReal("SSZ2", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Mz"));
  parameter Real SSZ3 = tir_file.getReal("SSZ3", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Mz"));
  parameter Real SSZ4 = tir_file.getReal("SSZ4", "ALIGNING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Combined Mz"));
  // Scaling coefficients
  parameter Real LFZO = tir_file.getReal("LFZO", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LCX = tir_file.getReal("LCX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LMUX = tir_file.getReal("LMUX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LEX = tir_file.getReal("LEX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LKX = tir_file.getReal("LKX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LHX = tir_file.getReal("LHX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LVX = tir_file.getReal("LVX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LXAL = tir_file.getReal("LXAL", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LGAX = tir_file.getReal("LGAX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LCY = tir_file.getReal("LCY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LMUY = tir_file.getReal("LMUY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LEY = tir_file.getReal("LEY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LKY = tir_file.getReal("LKY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LHY = tir_file.getReal("LHY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LVY = tir_file.getReal("LVY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LGAY = tir_file.getReal("LGAY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LKYG = tir_file.getReal("LKYG", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LTR = tir_file.getReal("LTR", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LRES = tir_file.getReal("LRES", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LCZ = tir_file.getReal("LCZ", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LGAZ = tir_file.getReal("LGAZ", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LYKA = tir_file.getReal("LYKA", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LVYKA = tir_file.getReal("LVYKA", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LS = tir_file.getReal("LS", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LSGKP = tir_file.getReal("LSGKP", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LSGAL = tir_file.getReal("LSGAL", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LGYR = tir_file.getReal("LGYR", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LMX = tir_file.getReal("LMX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LVMX = tir_file.getReal("LVMX", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LMY = tir_file.getReal("LMY", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  parameter Real LIP = tir_file.getReal("LIP", "SCALING_COEFFICIENTS") annotation(
    Dialog(tab = "Tire Coeffs", group = "Scaling"));
  
  parameter SIunits.Force FNOMIN = tir_file.getReal("FNOMIN", "VERTICAL") annotation(
    Dialog(group = "Conditions"));
  parameter SIunits.Length R0 = tir_file.getReal("UNLOADED_RADIUS", "DIMENSION") annotation(
    Dialog(group = "Dimensions"));
  parameter SIunits.Length rim_width = tir_file.getReal("RIM_WIDTH", "DIMENSION") annotation(
    Dialog(group = "Dimensions"));
  parameter SIunits.Length rim_R0 = tir_file.getReal("RIM_RADIUS", "DIMENSION") annotation(
    Dialog(group = "Dimensions"));
  
  Modelica.Mechanics.MultiBody.Parts.FixedRotation set_toe(rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence, angles = {static_gamma, 0, static_alpha}) annotation(
    Placement(transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, R0*0.95}) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Visualizers.VoluminousWheel voluminousWheel(rTire = R0, rRim = rim_R0, width = rim_width) annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation heading_reference(r = {1, 0, 0}) annotation(
    Placement(transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_resolve) annotation(
    Placement(transformation(origin = {-30, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  Modelica.Blocks.Sources.RealExpression tire_forces[3](y = {Fx, Fy, 0}) annotation(
    Placement(transformation(origin = {-70, -44}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.RealExpression tire_torques[3](y = {Mx, My, Mz}) annotation(
    Placement(transformation(origin = {-70, -56}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(animation = false, n = {0, 1, 0}, useAxisFlange = false) annotation(
    Placement(transformation(origin = {-20, 30}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Body wheel_inertia(r_CM = {0, 0, 0}, m = 8)  annotation(
    Placement(transformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
equation
  Fz = frame_a.f[3];
  wheel_vel = der(frame_a.r_0);
//  wheel_heading = heading_reference.frame_b.r_0 - heading_reference.frame_a.r_0;
  wheel_heading = {1, 0, 0};
  alpha = Utilities.Math.Vector.angle_between(wheel_vel, wheel_heading, {0, 0, 1});
  v_x_wheel = Utilities.Math.Vector.dot(wheel_vel, wheel_heading);
  
  kappa = 0;
    
//  if norm({wheel_vel[1], wheel_vel[2], 0}) < 1 then
//    kappa = 0;
//  else
//    kappa = (der(revolute.phi)*R0 - v_x_wheel)/v_x_wheel;
//  end if;
  
  Fx = MF52.Fx_eval(Fz, alpha, kappa, gamma, PCX1, PDX1, PDX2, PDX3, PEX1, PEX2, PEX3, PEX4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2, RBX1, RBX2, RCX1, REX1, REX2, RHX1, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  Fy = MF52.Fy_eval(Fz, alpha, kappa, gamma, PCY1, PDY1, PDY2, PDY3, PEY1, PEY2, PEY3, PEY4, PKY1, PKY2, PKY3, PHY1, PHY2, PHY3, PVY1, PVY2, PVY3, PVY4, RBY1, RBY2, RBY3, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  Mx = MF52.Mx_eval(Fz, Fy, alpha, kappa, gamma, QSX1, QSX2, QSX3, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  My = MF52.My_eval(Fz, alpha, kappa, gamma, QSY1, QSY2, QSY3, QSY4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  (Mz, pneu_trail, pneu_scrub) = MF52.Mz_eval(Fz, Fx, Fy, alpha, kappa, gamma, QBZ1, QBZ2, QBZ3, QBZ4, QBZ5, QBZ9, QBZ10, QCZ1, QDZ1, QDZ2, QDZ3, QDZ4, QDZ6, QDZ7, QDZ8, QDZ9, QEZ1, QEZ2, QEZ3, QEZ4, QEZ5, QHZ1, QHZ2, QHZ3, QHZ4, SSZ1, SSZ2, SSZ3, SSZ4, PCY1, PDY1, PDY2, PDY3, PKY1, PKY2, PKY3, PHY1, PHY2, PHY3, PVY1, PVY2, PVY3, PVY4, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6, PKX1, PKX2, PKX3, LFZO, LKX, LCY, LMUY, LKY, LHY, LVY, LGAY, LTR, LRES, LGAZ, LVYKA, LS, FNOMIN, R0);
  connect(frame_a, set_toe.frame_a) annotation(
    Line(points = {{0, -100}, {0, -40}}));
  connect(set_toe.frame_b, fixedTranslation.frame_a) annotation(
    Line(points = {{0, -20}, {0, 0}}, color = {95, 95, 95}));
  connect(heading_reference.frame_a, frame_a) annotation(
    Line(points = {{20, -70}, {0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_b, frame_a) annotation(
    Line(points = {{-20, -50}, {0, -50}, {0, -100}}, color = {95, 95, 95}));
  connect(forceAndTorque.frame_resolve, frame_a) annotation(
    Line(points = {{-30, -60}, {-30, -70}, {0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(tire_forces.y, forceAndTorque.force) annotation(
    Line(points = {{-58, -44}, {-42, -44}}, color = {0, 0, 127}, thickness = 0.5));
  connect(tire_torques.y, forceAndTorque.torque) annotation(
    Line(points = {{-58, -56}, {-42, -56}}, color = {0, 0, 127}, thickness = 0.5));
  connect(voluminousWheel.frame_a, revolute.frame_a) annotation(
    Line(points = {{-40, 30}, {-30, 30}}, color = {95, 95, 95}));
  connect(revolute.frame_b, fixedTranslation.frame_b) annotation(
    Line(points = {{-10, 30}, {0, 30}, {0, 20}}, color = {95, 95, 95}));
  connect(frame_b, revolute.frame_a) annotation(
    Line(points = {{-100, 0}, {-34, 0}, {-34, 30}, {-30, 30}}));
  connect(wheel_inertia.frame_a, revolute.frame_a) annotation(
    Line(points = {{-80, 50}, {-35, 50}, {-35, 30}, {-30, 30}}, color = {95, 95, 95}));
end BaseMF52;
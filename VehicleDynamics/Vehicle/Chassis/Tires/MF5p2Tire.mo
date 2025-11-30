within VehicleDynamics.Vehicle.Chassis.Tires;

model MF5p2Tire
  import VehicleDynamics.Utilities.Math.Vector.angle_between;
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.Fx_eval;
  import Modelica.SIunits;
  
  parameter SIunits.Length rim_width=7*0.0254 annotation(
    Dialog(group = "Dimensions"));
  parameter SIunits.Length rim_R0=5*0.0254 annotation(
    Dialog(group = "Dimensions"));
  parameter SIunits.Inertia wheel_J=1 annotation(
    Dialog(group = "Properties"));
  parameter SIunits.Mass wheel_m=1 annotation(
    Dialog(group = "Properties"));
  
  parameter String tir_path = "/home/rhorvath/Documents/Github/VehicleDynamics/JSONs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir";
  inner ExternData.TIRFile tir_file(fileName=tir_path)  annotation(
      Placement(transformation(origin = {-90, 90}, extent = {{10, -10}, {-10, 10}})));
      
  // General parameters
  parameter Real R0=tir_file.getReal("UNLOADED_RADIUS", "DIMENSION") "Unloaded tire radius" annotation(
    Dialog(group = "Dimensions"));
  parameter Real tire_c=tir_file.getReal("VERTICAL_STIFFNESS", "VERTICAL")  annotation(
    Dialog(group = "Properties"));
  parameter Real tire_d=tir_file.getReal("VERTICAL_DAMPING", "VERTICAL")  annotation(
    Dialog(group = "Properties"));

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
  
  Real v_min = 0.2;
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a cp_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b hub_frame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  // 2DOF Tire physics
  TirePhysics.Tire2DOF tire2DOF(R0 = R0, rim_width = rim_width, rim_R0 = rim_R0, tire_c = tire_c, tire_d = tire_d, wheel_J = wheel_J, wheel_m = wheel_m)  annotation(
    Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  // Tire states
  Real Fz;
  Real kappa_kin;
  Real alpha_kin;
  Real gamma;
  
  // Informational states
  Real Vsx;
  Real global_heading[3];
  Real ground_heading[3];
  Real ground_heading_dir[3];
  Real global_vel[3];
  Real ground_vel[3];
  Real ground_vel_dir[3];
  Real global_vert_dir[3];
  Real global_lat_dir[3];
  
  // Tire forces
  SIunits.Force Fx;
  SIunits.Force Fy;
  // Force application
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)  annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}})));
  // Fx source
  Modelica.Blocks.Sources.RealExpression realExpression(y = Fx)  annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-10, -10}, {10, 10}})));
  // Fy source
  Modelica.Blocks.Sources.RealExpression realExpression1(y = Fy)  annotation(
    Placement(transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}})));
  // Fz source (zero because spring force reaction accounts for this)
  Modelica.Blocks.Sources.RealExpression realExpression2 annotation(
    Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}})));
  // Transient parameters
  Real kappa_rel;
  Real u(start=0);
  Real sigma_kappa = 0.05;
  Modelica.Mechanics.MultiBody.Parts.Body wheel_body( m = 1e-3, r_CM = {0, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {30, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassis_frame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
equation
  // Normal load
  Fz = cp_frame.f[3];
  
  // Heading
  global_heading = Modelica.Mechanics.MultiBody.Frames.resolve1(cp_frame.R, {1, 0, 0});
  ground_heading = {global_heading[1], global_heading[2], 0};
  ground_heading_dir = normalize(ground_heading);
  
  // Velocity
  global_vel = tire2DOF.wheel_vel.v;
  ground_vel = {global_vel[1], global_vel[2], 0};
//  if norm(ground_vel) > v_min then
//    ground_vel_dir = normalize(ground_vel);
//  else
//    ground_vel_dir = ground_heading_dir;
//  end if;

  ground_vel_dir = ground_heading_dir;
    
  // Inclination angle
  global_vert_dir = Modelica.Mechanics.MultiBody.Frames.resolve2(cp_frame.R, {0, 0, 1});
  global_lat_dir = Modelica.Mechanics.MultiBody.Frames.resolve2(cp_frame.R, {0, 1, 0});
  gamma = angle_between(global_vert_dir, {0,0,1}, ground_heading_dir);
  
  // Longitudinal calcs
  Vsx = tire2DOF.wheel_vel.v[1] - tire2DOF.tire_Re.s_rel*tire2DOF.wheel_speed.w;
  kappa_kin = -1*Vsx/max(abs(tire2DOF.wheel_vel.v[1]), v_min);
  
  // Lateral calcs
  alpha_kin = angle_between(ground_heading_dir, ground_vel_dir, {0, 0, 1});

//  alpha_kin = 0;
// Longitudinal transient calcs
  der(u) + 1/sigma_kappa*abs(tire2DOF.wheel_vel.v[1])*u = -Vsx;
  kappa_rel = u/sigma_kappa;
  
  // MF calcs
  Fx = MF52.Fx_eval(Fz, alpha_kin, kappa_rel, gamma, PCX1, PDX1, PDX2, PDX3, PEX1, PEX2, PEX3, PEX4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2, RBX1, RBX2, RCX1, REX1, REX2, RHX1, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  Fy = 0;
//  Fy = MF52.Fy_eval(Fz, alpha_kin, kappa_rel, gamma, PCY1, PDY1, PDY2, PDY3, PEY1, PEY2, PEY3, PEY4, PKY1, PKY2, PKY3, PHY1, PHY2, PHY3, PVY1, PVY2, PVY3, PVY4, RBY1, RBY2, RBY3, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6, LFZO, LCX, LMUX, LEX, LKX, LHX, LVX, LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY, LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ, LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR, LMX, LVMX, LMY, LIP, FNOMIN, R0);
  connect(tire2DOF.hub_frame, hub_frame) annotation(
    Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(tire2DOF.cp_frame, cp_frame) annotation(
    Line(points = {{0, -10}, {0, -100}}, color = {95, 95, 95}));
  connect(force.frame_b, tire2DOF.cp_frame) annotation(
    Line(points = {{-20, -70}, {0, -70}, {0, -10}}, color = {95, 95, 95}));
  connect(realExpression.y, force.force[1]) annotation(
    Line(points = {{-58, -50}, {-50, -50}, {-50, -70}, {-42, -70}}, color = {0, 0, 127}));
  connect(realExpression1.y, force.force[2]) annotation(
    Line(points = {{-58, -70}, {-42, -70}}, color = {0, 0, 127}));
  connect(realExpression2.y, force.force[3]) annotation(
    Line(points = {{-58, -90}, {-50, -90}, {-50, -70}, {-42, -70}}, color = {0, 0, 127}));
  connect(wheel_body.frame_a, cp_frame) annotation(
    Line(points = {{20, -70}, {0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(chassis_frame, tire2DOF.chassis_frame) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  annotation(
  Icon(
  coordinateSystem(extent={{-100,-100},{100,100}}),
  graphics = {
        Text(
          extent={{-200,-60},{-100,-20}},
          textString="Frame",
          fontSize=12,
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Center
        ),
        Text(
          extent={{100,-60},{200,-20}},
          textString="T_in",
          fontSize=12,
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Center
        ),
        Text(
          extent={{40,-140},{80,-100}},
          textString="CP",
          fontSize=12,
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Center
        ),
        // Outer tire
        Ellipse(
          extent={{-90,-90},{90,90}},
          fillPattern=FillPattern.Solid,
          fillColor={40,40,40},
          lineThickness=3
        ),

        // Inner rim
        Ellipse(
          extent={{-45,-45},{45,45}},
          fillPattern=FillPattern.Solid,
          fillColor={200,200,200},
          lineThickness=2
        ),

        // Hub
        Ellipse(
          extent={{-15,-15},{15,15}},
          fillPattern=FillPattern.Solid,
          fillColor={160,160,160},
          lineThickness=2
        ),

        // Spokes (default black)
        Line(points={{0,0},{0,45}},   thickness=2),
        Line(points={{0,0},{0,-45}},  thickness=2),
        Line(points={{0,0},{45,0}},   thickness=2),
        Line(points={{0,0},{-45,0}},  thickness=2),
        Line(points={{0,0},{32,32}},  thickness=2),
        Line(points={{0,0},{-32,-32}}, thickness=2),
        Line(points={{0,0},{32,-32}}, thickness=2),
        Line(points={{0,0},{-32,32}}, thickness=2)
      }
      ),
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end MF5p2Tire;
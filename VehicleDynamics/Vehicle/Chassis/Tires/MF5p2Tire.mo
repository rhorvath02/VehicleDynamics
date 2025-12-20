within VehicleDynamics.Vehicle.Chassis.Tires;

model MF5p2Tire
  // Custom linalg
  import VehicleDynamics.Utilities.Math.Vector.angle_between;
  import VehicleDynamics.Utilities.Math.Vector.cross;
  import VehicleDynamics.Utilities.Math.Vector.dot;
  
  // Modelica linalg
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  
  // Modelica units
  import Modelica.SIunits;
  
  // MF52 functions
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.Fx_eval;
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.Fy_eval;
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.Mx_eval;
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.My_eval;
  import VehicleDynamics.Vehicle.Chassis.Tires.MF52.Mz_eval;
  
  
  // Parameters - Initial conditions
  parameter SIunits.Velocity initial_velocity = 0
    "Initial translational velocity"
    annotation(Dialog(group = "Initial Conditions"));
  
  // Parameters - Dimensions
  parameter SIunits.Length rim_width = 7*0.0254 "Rim width"
    annotation(Dialog(group = "Dimensions"));
  parameter SIunits.Length rim_R0 = 5*0.0254 "Rim unloaded static radius"
    annotation(Dialog(group = "Dimensions"));
  
  // Parameters - Mass properties
  parameter SIunits.Inertia wheel_inertia[3, 3] = [0, 0, 0; 0, 0.2, 0; 0, 0, 0] "Wheel + hub inertia tensor (y-axis as spindle)";
    annotation(Dialog(group = "Mass Properties"));
  parameter SIunits.Mass wheel_m = 1 ""
    annotation(Dialog(group = "Mass Properties"));
  
  // Numerical stability
  parameter Real v_min = 0.1 "Low-speed threshold for force gating (m/s)" annotation(Dialog(group = "Numerical Conditions"));
  parameter Real eps = 1e-6 "Small constant to prevenet division by zero" annotation(Dialog(group = "Numerical Conditions"));

  // Read tire model
  parameter String tir_path "File path to .tir";
  inner ExternData.TIRFile tir_file(fileName = tir_path)
    annotation(Placement(transformation(origin = {-90, 90},
                                         extent = {{10, -10}, {-10, 10}})));

  // General .tir parameters
  final parameter Real R0 =
    tir_file.getReal("UNLOADED_RADIUS", "DIMENSION") "Unloaded tire radius";
  final parameter Real tire_c =
    tir_file.getReal("VERTICAL_STIFFNESS", "VERTICAL") "Wheel vertical stiffness";
  final parameter Real tire_d =
    tir_file.getReal("VERTICAL_DAMPING", "VERTICAL") "Wheel vertical damping";
  final parameter Real FNOMIN =
    tir_file.getReal("FNOMIN", "VERTICAL") "Nominal normal load, FZ0";

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
  
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a cp_frame
    annotation(Placement(transformation(origin = {0, -100},
                                         extent = {{-16, -16}, {16, 16}},
                                         rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b chassis_frame
    annotation(Placement(transformation(origin = {-100, 0},
                                         extent = {{-16, -16}, {16, 16}})));

  // 2DOF Tire physics
  TirePhysics.Tire2DOF tire2DOF(
    R0 = R0,
    rim_width = rim_width,
    rim_R0 = rim_R0,
    tire_c = tire_c,
    tire_d = tire_d,
    wheel_m = wheel_m,
    wheel_inertia = wheel_inertia)
    annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  
  // Torque input
  Modelica.Mechanics.MultiBody.Forces.Torque input_torque annotation(
    Placement(transformation(origin = {22, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Blocks.Interfaces.RealInput hub_torque annotation(
    Placement(transformation(origin = {16, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {25, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Sources.RealExpression zeroX(y = 0)  annotation(
    Placement(transformation(origin = {-10, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression zeroZ(y = 0)  annotation(
    Placement(transformation(origin = {-10, 80}, extent = {{-10, -10}, {10, 10}})));

  // MF52 force application
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(
    resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.world)
    annotation(Placement(transformation(origin = {-30, -70},
                                         extent = {{-10, -10}, {10, 10}})));
  
  // Body for state selection... Don't worry about it  
  Modelica.Mechanics.MultiBody.Parts.Body wheel_body(
    m = 1e-3,
    r_CM = {0, 0, 0},
    animation = false)
    annotation(Placement(transformation(origin = {30, -70},
                                         extent = {{-10, -10}, {10, 10}})));

  // World / ground unit vectors
  Real[3] e_z = {0, 0, 1};
  Real[3] e_xw;
  Real[3] e_xg;
  Real[3] e_yg;

  // Velocity vectors
  Real[3] v_cp;
  Real[3] v_g;
  
  // Velocity components
  Real Vx;
  Real Vy;

  // Slip quantities
  Real alpha;
  Real kappa;

  // Loads / forces
  Real Fx;
  Real Fy;
  Real Fz;
  
  Real Fy_raw;
  
  // Relaxation lengths
  Real sigma_kappa = 0.05;
  Real sigma_alpha = 0.3;
  
  // Diagnostics
  Real[3] F_world;
  Real P_contact;
  Real mu_eff_i;
  
  // Longitudinal slip
  Real Vsx;
  Real w_speed;
  
protected
  // Transient slip states
  Real u(start = 0);

initial equation
  tire2DOF.hub_axis.w = initial_velocity / R0;

equation
  // Normal load
  Fz = cp_frame.f[3];

  // World basis
  e_xw = Modelica.Mechanics.MultiBody.Frames.resolve1(cp_frame.R, {1, 0, 0});
  e_xg = normalize({e_xw[1], e_xw[2], 0});
  e_yg = normalize(cross(e_z, e_xg));

  // Contact velocity
  v_cp = tire2DOF.wheel_vel.v;
  v_g  = {v_cp[1], v_cp[2], 0};

  Vx = dot(v_g, e_xg);
  Vy = dot(v_g, e_yg);
  
  // Lateral slip
  alpha = atan2(-Vy, abs(Vx));
  
  // Longitudinal slip
  Vsx = Vx - R0 * tire2DOF.hub_axis.w;
  der(u) + (abs(Vx)/sigma_kappa) * u = -Vsx;
  kappa = u / sigma_kappa;
  
  // Low-speed gating
  w_speed = abs(Vx) / (abs(Vx) + v_min);

  // MF52 calls
  Fy_raw =
    MF52.Fy_eval(
      Fz, alpha, 0, 0,
      PCY1, PDY1, PDY2, PDY3,
      PEY1, PEY2, PEY3, PEY4,
      PKY1, PKY2, PKY3,
      PHY1, PHY2, PHY3,
      PVY1, PVY2, PVY3, PVY4,
      RBY1, RBY2, RBY3,
      RCY1, REY1, REY2, RHY1, RHY2,
      RVY1, RVY2, RVY3, RVY4, RVY5, RVY6,
      LFZO, LCX, LMUX, LEX, LKX, LHX, LVX,
      LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY,
      LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ,
      LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR,
      LMX, LVMX, LMY, LIP, FNOMIN, R0);

  Fy = Fy_raw;
  
  Fx =
  MF52.Fx_eval(
    Fz, alpha, kappa, 0,
    PCX1, PDX1, PDX2, PDX3,
    PEX1, PEX2, PEX3, PEX4,
    PKX1, PKX2, PKX3,
    PHX1, PHX2,
    PVX1, PVX2,
    RBX1, RBX2, RCX1, REX1, REX2, RHX1,
    LFZO, LCX, LMUX, LEX, LKX, LHX, LVX,
    LXAL, LGAX, LCY, LMUY, LEY, LKY, LHY,
    LVY, LGAY, LKYG, LTR, LRES, LCZ, LGAZ,
    LYKA, LVYKA, LS, LSGKP, LSGAL, LGYR,
    LMX, LVMX, LMY, LIP, FNOMIN, R0);


  // ------------------------------------------------------------
  // Apply force
  // ------------------------------------------------------------
  F_world = Fx*e_xg + Fy*e_yg;
  force.force = F_world;

  P_contact = dot(F_world, v_g);
  
  mu_eff_i = sqrt(Fx^2 + Fy^2) / max(abs(Fz), eps);
// ------------------------------------------------------------
// Connections
// ------------------------------------------------------------
  connect(cp_frame, tire2DOF.cp_frame) annotation(
    Line(points = {{0, -100}, {0, -10}}));
  connect(force.frame_b, cp_frame) annotation(
    Line(points = {{-20, -70}, {0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(wheel_body.frame_a, cp_frame) annotation(
    Line(points = {{20, -70}, {0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(chassis_frame, tire2DOF.chassis_frame) annotation(
    Line(points = {{-100, 0}, {-10, 0}}));
  connect(hub_torque, input_torque.torque[2]) annotation(
    Line(points = {{16, 120}, {16, 42}}, color = {0, 0, 127}));
  connect(zeroX.y, input_torque.torque[1]) annotation(
    Line(points = {{2, 60}, {16, 60}, {16, 42}}, color = {0, 0, 127}));
  connect(zeroZ.y, input_torque.torque[3]) annotation(
    Line(points = {{2, 80}, {16, 80}, {16, 42}}, color = {0, 0, 127}));
  connect(input_torque.frame_a, tire2DOF.chassis_frame) annotation(
    Line(points = {{12, 30}, {-30, 30}, {-30, 0}, {-10, 0}}, color = {95, 95, 95}));
  connect(input_torque.frame_b, tire2DOF.hub_frame) annotation(
    Line(points = {{32, 30}, {50, 30}, {50, 0}, {10, 0}}, color = {95, 95, 95}));

annotation(
  Icon(
  coordinateSystem(extent={{-100,-100},{100,100}}),
  graphics = {
        Text(
          extent={{-120,-160},{120,-120}},
          textString="%name",
          fontSize=14,
          horizontalAlignment=TextAlignment.Center
        ),

        Text(
          extent={{-200,-60},{-100,-20}},
          textString="Frame",
          fontSize=12,
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Center
        ),
        Text(
          extent={{-100,110},{0,140}},
          textString="T_in",
          fontSize=12,
          textColor={0,0,255},
          horizontalAlignment=TextAlignment.Center
        ),
        Text(
          extent={{30,-120},{70,-80}},
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
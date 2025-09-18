model Test_Frame
  import Modelica.SIunits;
  
  parameter SIunits.Position CG_location[3] = {-0.7747, 0, 0.254} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_contact_patch[3] = {0, 0.609600, 0} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_contact_patch[3] = {FL_contact_patch[1], -FL_contact_patch[2], FL_contact_patch[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_contact_patch[3] = {-1.5494, 0.609600, 0} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_contact_patch[3] = {RL_contact_patch[1], -RL_contact_patch[2], RL_contact_patch[3]} annotation(Dialog(group="Geometry"));
  // FL node parameters
  parameter SIunits.Position FL_upper_fore_i[3] = {0.086868, 0.215900, 0.200000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_upper_o[3] = {-0.006347, 0.523240, 0.287020} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_tie_i[3] = {0.041128, 0.215900, 0.117856} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_tie_o[3] = {0.056000, 0.532333, 0.164821} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_fore_i[3] = {0.087376, 0.215900, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FL_lower_o[3] = {0, 0.556499, 0.124998} annotation(Dialog(group="Geometry"));
  // FR node parameters
  parameter SIunits.Position FR_upper_fore_i[3] = {FL_upper_fore_i[1], -FL_upper_fore_i[2], FL_upper_fore_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_upper_aft_i[3] = {FL_upper_aft_i[1], -FL_upper_aft_i[2], FL_upper_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_upper_o[3] = {FL_upper_o[1], -FL_upper_o[2], FL_upper_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_tie_i[3] = {FL_tie_i[1], -FL_tie_i[2], FL_tie_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_tie_o[3] = {FL_tie_o[1], -FL_tie_o[2], FL_tie_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_lower_fore_i[3] = {FL_lower_fore_i[1], -FL_lower_fore_i[2], FL_lower_fore_i[3]} annotation(Dialog(group="Geometry"));
parameter SIunits.Position FR_lower_aft_i[3] = {FL_lower_aft_i[1], -FL_lower_aft_i[2], FL_lower_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position FR_lower_o[3] = {FL_lower_o[1], -FL_lower_o[2], FL_lower_o[3]} annotation(Dialog(group="Geometry"));

// RL node parameters
  parameter SIunits.Position RL_upper_fore_i[3] = {-1.298905, 0.282999, 0.217500} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_upper_aft_i[3] = {-1.490977, 0.282999, 0.217500} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_upper_o[3] = {-1.574797, 0.554998, 0.289560} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_tie_i[3] = {-1.428059, 0.282999, 0.177800} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_tie_o[3] = {-1.462710, 0.587375, 0.240741} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_lower_fore_i[3] = {-1.298905, 0.282999, 0.090000} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RL_lower_aft_i[3] = {-1.490977, 0.282999, 0.090000} annotation(Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  parameter SIunits.Position RL_lower_o[3] = {-1.554983, 0.579999, 0.113030} annotation(Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  // RR node parameters
  // FR node parameters
  parameter SIunits.Position RR_upper_fore_i[3] = {RL_upper_fore_i[1], -RL_upper_fore_i[2], RL_upper_fore_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_aft_i[3] = {RL_upper_aft_i[1], -RL_upper_aft_i[2], RL_upper_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_upper_o[3] = {RL_upper_o[1], -RL_upper_o[2], RL_upper_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_i[3] = {RL_tie_i[1], -RL_tie_i[2], RL_tie_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_tie_o[3] = {RL_tie_o[1], -RL_tie_o[2], RL_tie_o[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_fore_i[3] = {RL_lower_fore_i[1], -RL_lower_fore_i[2], RL_lower_fore_i[3]} annotation(Dialog(group="Geometry"));
parameter SIunits.Position RR_lower_aft_i[3] = {RL_lower_aft_i[1], -RL_lower_aft_i[2], RL_lower_aft_i[3]} annotation(Dialog(group="Geometry"));
  parameter SIunits.Position RR_lower_o[3] = {RL_lower_o[1], -RL_lower_o[2], RL_lower_o[3]} annotation(Dialog(group="Geometry"));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-170, -130}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Body.FrameFrSteer frameFrSteer(FL_upper_fore_i = FL_upper_fore_i, FL_upper_aft_i = FL_upper_aft_i, FL_tie_i = FL_tie_i, FL_lower_fore_i = FL_lower_fore_i, FL_lower_aft_i = FL_lower_aft_i, RL_upper_fore_i = RL_upper_fore_i, RL_upper_aft_i = RL_upper_aft_i, RL_tie_i = RL_tie_i, RL_lower_fore_i = RL_lower_fore_i, RL_lower_aft_i = RL_lower_aft_i, CG_location = CG_location)  annotation(
    Placement(transformation(extent = {{-30, -60}, {30, 60}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed(r = FL_contact_patch, animation = false) annotation(
    Placement(transformation(origin = {-160, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = CG_location, animation = false)  annotation(
    Placement(transformation(origin = {-110, -110}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.GroundPhysics groundPhysics annotation(
    Placement(transformation(origin = {-130, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed1(animation = false, r = FR_contact_patch) annotation(
    Placement(transformation(origin = {160, 10}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.GroundPhysics groundPhysics1 annotation(
    Placement(transformation(origin = {130, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed2(animation = false, r = RL_contact_patch) annotation(
    Placement(transformation(origin = {-160, -50}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.GroundPhysics groundPhysics2 annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed FL_contact_patch_fixed11(animation = false, r = RR_contact_patch) annotation(
    Placement(transformation(origin = {160, -50}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.GroundPhysics groundPhysics11 annotation(
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));
  Vehicle.Chassis.Suspension.Tires.BaseMF52 baseMF52(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = -0.000126, PHX2 = 0.000467, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 0.2045, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, rim_R0 = 0.127, rim_width = 0.1778, static_alpha = 0, static_gamma = 0) annotation(
    Placement(transformation(origin = {-100, 30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Tires.BaseMF52 baseMF521(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = -0.000126, PHX2 = 0.000467, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 0.2045, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, rim_R0 = 0.127, rim_width = 0.1778, static_alpha = 0, static_gamma = 0) annotation(
    Placement(transformation(origin = {-100, -30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Tires.BaseMF52 baseMF522(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = -0.000126, PHX2 = 0.000467, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 0.2045, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, rim_R0 = 0.127, rim_width = 0.1778, static_alpha = 0, static_gamma = 0) annotation(
    Placement(transformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Tires.BaseMF52 baseMF523(FNOMIN = 654, LCX = 1, LCY = 1, LCZ = 1, LEX = 1, LEY = 1, LFZO = 1, LGAX = 1, LGAY = 1, LGAZ = 1, LGYR = 1, LHX = 1, LHY = 1, LIP = 1, LKX = 1, LKY = 1, LKYG = 1, LMUX = 1, LMUY = 1, LMX = 1, LMY = 1, LRES = 1, LS = 1, LSGAL = 1, LSGKP = 1, LTR = 1, LVMX = 1, LVX = 1, LVY = 1, LVYKA = 1, LXAL = 1, LYKA = 1, PCX1 = 1.530410, PCY1 = 1.53041, PDX1 = 2.597991, PDX2 = -0.618826, PDX3 = 11.156379, PDY1 = -2.40275, PDY2 = 0.343535, PDY3 = 3.89743, PEX1 = 0, PEX2 = 0.141806, PEX3 = -1.934950, PEX4 = 0.044722, PEY1 = 0.00, PEY2 = -0.280762, PEY3 = 0.70403, PEY4 = -0.478297, PHX1 = -0.000126, PHX2 = 0.000467, PHY1 = 0.000000000, PHY2 = 0.000000000, PHY3 = 0.000000000, PKX1 = 55.079922, PKX2 = -0.000017, PKX3 = -0.161850, PKY1 = -53.2421, PKY2 = 2.38205, PKY3 = 1.36502, PVX1 = 0.0000000, PVX2 = 0.0000000, PVY1 = 0.000000000, PVY2 = 0.000000000, PVY3 = 0.000000000, PVY4 = 0.000000000, QBZ1 = 8.22843, QBZ10 = -1.72926, QBZ2 = 2.98676, QBZ3 = -3.57739, QBZ4 = -0.429117, QBZ5 = 0.433125, QBZ9 = 0, QCZ1 = 1.41359, QDZ1 = 0.152526, QDZ2 = -0.0381101, QDZ3 = 0.387762, QDZ4 = -3.95699, QDZ6 = 0.00604966, QDZ7 = -0.000116241, QDZ8 = -2.33359, QDZ9 = -0.0379755, QEZ1 = -0.239731, QEZ2 = 1.29253, QEZ3 = -1.21298, QEZ4 = 0.197579, QEZ5 = 0.244, QHZ1 = -0.00101749, QHZ2 = 0.000378319, QHZ3 = -0.0405191, QHZ4 = 0.0185463, QSX1 = -0.0130807, QSX2 = 0.00, QSX3 = 0.0587803, QSY1 = 0, QSY2 = 0, QSY3 = 0, QSY4 = 0, R0 = 0.2045, RBX1 = 8.151136, RBX2 = 5.388063, RBY1 = 14.628, RBY2 = 10.400, RBY3 = -0.00441045, RCX1 = 1.122399, RCY1 = 1.044, REX1 = 0.052014, REX2 = -0.898450, REY1 = 0.048, REY2 = 0.025, RHX1 = 0.0, RHY1 = 0.009, RHY2 = 0.0023097, RVY1 = 4.78297e-06, RVY2 = 0.0127967, RVY3 = -0.498917, RVY4 = 18.2625, RVY5 = 2.72152, RVY6 = -10.5225, SSZ1 = 0, SSZ2 = 0, SSZ3 = 0, SSZ4 = 0, rim_R0 = 0.127, rim_width = 0.1778, static_alpha = 0, static_gamma = 0) annotation(
    Placement(transformation(origin = {100, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp ramp(height = 1.5*0.0254, duration = 0.5)  annotation(
    Placement(transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {0, -89}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Translational.Sources.Speed speed annotation(
    Placement(transformation(origin = {-30, -81}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant const(k = 25) annotation(
    Placement(transformation(origin = {-70, -81}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic1(n = {0, 1, 0}, animation = false)  annotation(
    Placement(transformation(origin = {-30, -110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(animation = false)  annotation(
    Placement(transformation(origin = {-70, -110}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.DoubleWishboneBase doubleWishboneBase(upper_fore_i = FL_upper_fore_i, upper_aft_i = FL_upper_aft_i, lower_fore_i = FL_lower_fore_i, lower_aft_i = FL_lower_aft_i, upper_o = FL_upper_o, lower_o = FL_lower_o, tie_i = FL_tie_i, tie_o = FL_tie_o, contact_patch = FL_contact_patch) annotation(
    Placement(transformation(origin = {-60, 30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.DoubleWishboneBase doubleWishboneBase1(contact_patch = RL_contact_patch, lower_aft_i = RL_lower_aft_i, lower_fore_i = RL_lower_fore_i, lower_o = RL_lower_o, tie_i = RL_tie_i, tie_o = RL_tie_o, upper_aft_i = RL_upper_aft_i, upper_fore_i = RL_upper_fore_i, upper_o = RL_upper_o) annotation(
    Placement(transformation(origin = {-60, -30}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.DoubleWishboneBase FR_doubleWishboneBase(upper_fore_i = FR_upper_fore_i, upper_aft_i = FR_upper_aft_i, lower_fore_i = FR_lower_fore_i, lower_aft_i = FR_lower_aft_i, upper_o = FR_upper_o, lower_o = FR_lower_o, tie_i = FR_tie_i, tie_o = FR_tie_o, contact_patch = FR_contact_patch)  annotation(
    Placement(transformation(origin = {60, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.DoubleWishboneBase RR_doubleWishboneBase1(contact_patch = RR_contact_patch, lower_aft_i = RR_lower_aft_i, lower_fore_i = RR_lower_fore_i, lower_o = RR_lower_o, tie_i = RR_tie_i, tie_o = RR_tie_o, upper_aft_i = RR_upper_aft_i, upper_fore_i = RR_upper_fore_i, upper_o = RR_upper_o) annotation(
    Placement(transformation(origin = {60, -30}, extent = {{10, -10}, {-10, 10}})));
equation
  connect(FL_contact_patch_fixed.frame_b, groundPhysics.frame_a) annotation(
    Line(points = {{-150, 10}, {-140, 10}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed1.frame_b, groundPhysics1.frame_a) annotation(
    Line(points = {{150, 10}, {140, 10}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed11.frame_b, groundPhysics11.frame_a) annotation(
    Line(points = {{150, -50}, {140, -50}}, color = {95, 95, 95}));
  connect(ramp.y, frameFrSteer.u) annotation(
    Line(points = {{-18, 80}, {0, 80}, {0, 30}}, color = {0, 0, 127}));
  connect(speed.flange, prismatic.axis) annotation(
    Line(points = {{-20, -81}, {-6, -81}}, color = {0, 127, 0}));
  connect(const.y, speed.v_ref) annotation(
    Line(points = {{-59, -81}, {-43, -81}}, color = {0, 0, 127}));
  connect(prismatic.frame_b, frameFrSteer.frame_a) annotation(
    Line(points = {{0, -79}, {0, 0}}, color = {95, 95, 95}));
  connect(prismatic1.frame_b, prismatic.frame_a) annotation(
    Line(points = {{-20, -110}, {0, -110}, {0, -99}}, color = {95, 95, 95}));
  connect(revolute.frame_b, prismatic1.frame_a) annotation(
    Line(points = {{-60, -110}, {-40, -110}}, color = {95, 95, 95}));
  connect(fixed.frame_b, revolute.frame_a) annotation(
    Line(points = {{-100, -110}, {-80, -110}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.tie_i_frame, frameFrSteer.FL_tie_i_frame) annotation(
    Line(points = {{-50, 24}, {-40, 24}, {-40, 30}, {-30, 30}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.lower_fore_i_frame, frameFrSteer.FL_lower_fore_i_frame) annotation(
    Line(points = {{-50, 36}, {-36, 36}, {-36, 16}, {-30, 16}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.lower_aft_i_frame, frameFrSteer.FL_lower_aft_i_frame) annotation(
    Line(points = {{-50, 30}, {-44, 30}, {-44, 4}, {-30, 4}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.upper_aft_i_frame, frameFrSteer.FL_upper_aft_i_frame) annotation(
    Line(points = {{-54, 40}, {-52, 40}, {-52, 44}, {-30, 44}}, color = {95, 95, 95}));
  connect(doubleWishboneBase.upper_fore_i_frame, frameFrSteer.FL_upper_fore_i_frame) annotation(
    Line(points = {{-60, 40}, {-60, 56}, {-30, 56}}, color = {95, 95, 95}));
  connect(groundPhysics.frame_b, doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{-120, 10}, {-60, 10}, {-60, 20}}, color = {95, 95, 95}));
  connect(baseMF52.frame_a, doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{-100, 20}, {-100, 10}, {-60, 10}, {-60, 20}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.upper_fore_i_frame, frameFrSteer.RL_upper_fore_i_frame) annotation(
    Line(points = {{-60, -20}, {-60, -4}, {-30, -4}}, color = {95, 95, 95}));
  connect(frameFrSteer.RL_upper_aft_i_frame, doubleWishboneBase1.upper_aft_i_frame) annotation(
    Line(points = {{-30, -16}, {-54, -16}, {-54, -20}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.tie_i_frame, frameFrSteer.RL_tie_i_frame) annotation(
    Line(points = {{-50, -36}, {-40, -36}, {-40, -30}, {-30, -30}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.lower_aft_i_frame, frameFrSteer.RL_lower_aft_i_frame) annotation(
    Line(points = {{-50, -30}, {-46, -30}, {-46, -56}, {-30, -56}}, color = {95, 95, 95}));
  connect(doubleWishboneBase1.lower_fore_i_frame, frameFrSteer.RL_lower_fore_i_frame) annotation(
    Line(points = {{-50, -24}, {-36, -24}, {-36, -44}, {-30, -44}}, color = {95, 95, 95}));
  connect(FL_contact_patch_fixed2.frame_b, groundPhysics2.frame_a) annotation(
    Line(points = {{-150, -50}, {-140, -50}}, color = {95, 95, 95}));
  connect(groundPhysics2.frame_b, doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{-120, -50}, {-60, -50}, {-60, -40}}, color = {95, 95, 95}));
  connect(baseMF521.frame_a, doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{-100, -40}, {-100, -50}, {-60, -50}, {-60, -40}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.upper_aft_i_frame, frameFrSteer.FR_upper_aft_i_frame) annotation(
    Line(points = {{54, 40}, {54, 44}, {30, 44}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.upper_fore_i_frame, frameFrSteer.FR_upper_fore_i_frame) annotation(
    Line(points = {{60, 40}, {60, 56}, {30, 56}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.tie_i_frame, frameFrSteer.FR_tie_i_frame) annotation(
    Line(points = {{50, 24}, {40, 24}, {40, 30}, {30, 30}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.lower_fore_i_frame, frameFrSteer.FR_lower_fore_i_frame) annotation(
    Line(points = {{50, 36}, {36, 36}, {36, 16}, {30, 16}}, color = {95, 95, 95}));
  connect(FR_doubleWishboneBase.lower_aft_i_frame, frameFrSteer.FR_lower_aft_i_frame) annotation(
    Line(points = {{50, 30}, {44, 30}, {44, 4}, {30, 4}}, color = {95, 95, 95}));
  connect(groundPhysics1.frame_b, FR_doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{120, 10}, {60, 10}, {60, 20}}, color = {95, 95, 95}));
  connect(baseMF523.frame_a, FR_doubleWishboneBase.contact_patch_frame) annotation(
    Line(points = {{100, 20}, {100, 10}, {60, 10}, {60, 20}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.upper_aft_i_frame, frameFrSteer.RR_upper_aft_i_frame) annotation(
    Line(points = {{54, -20}, {54, -16}, {30, -16}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.upper_fore_i_frame, frameFrSteer.RR_upper_fore_i_frame) annotation(
    Line(points = {{60, -20}, {60, -4}, {30, -4}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.tie_i_frame, frameFrSteer.RR_tie_i_frame) annotation(
    Line(points = {{50, -36}, {40, -36}, {40, -30}, {30, -30}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.lower_aft_i_frame, frameFrSteer.RR_lower_aft_i_frame) annotation(
    Line(points = {{50, -30}, {46, -30}, {46, -56}, {30, -56}}, color = {95, 95, 95}));
  connect(RR_doubleWishboneBase1.lower_fore_i_frame, frameFrSteer.RR_lower_fore_i_frame) annotation(
    Line(points = {{50, -24}, {36, -24}, {36, -44}, {30, -44}}, color = {95, 95, 95}));
  connect(groundPhysics11.frame_b, RR_doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{120, -50}, {60, -50}, {60, -40}}, color = {95, 95, 95}));
  connect(baseMF522.frame_a, RR_doubleWishboneBase1.contact_patch_frame) annotation(
    Line(points = {{100, -40}, {100, -50}, {60, -50}, {60, -40}}, color = {95, 95, 95}));
  annotation(
    uses(Modelica(version = "3.2.3")),
  Diagram(coordinateSystem(extent = {{-180, 100}, {180, -140}})),
  version = "");
end Test_Frame;

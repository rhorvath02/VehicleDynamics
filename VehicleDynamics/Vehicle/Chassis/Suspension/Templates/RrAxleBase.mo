within VehicleDynamics.Vehicle.Chassis.Suspension.Templates;

partial model RrAxleBase
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Position RL_upper_fore_i[3] = RL_double_wishbone.upper_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_upper_aft_i[3] = RL_double_wishbone.upper_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_lower_fore_i[3] = RL_double_wishbone.lower_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_lower_aft_i[3] = RL_double_wishbone.lower_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_upper_o[3] = RL_double_wishbone.upper_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_lower_o[3] = RL_double_wishbone.lower_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_tie_i[3] = RL_double_wishbone.tie_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_tie_o[3] = RL_double_wishbone.tie_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RL_wheel_center[3] = RL_double_wishbone.wheel_center annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle RL_static_gamma = RL_double_wishbone.static_gamma annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle RL_static_alpha = RL_double_wishbone.static_alpha annotation(
    Dialog(group = "Geometry"));
  
  parameter SIunits.Position RR_upper_fore_i[3] = {RL_upper_fore_i[1], -RL_upper_fore_i[2], RL_upper_fore_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_upper_aft_i[3] = {RL_upper_aft_i[1], -RL_upper_aft_i[2], RL_upper_aft_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_lower_fore_i[3] = {RL_lower_fore_i[1], -RL_lower_fore_i[2], RL_lower_fore_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_lower_aft_i[3] = {RL_lower_aft_i[1], -RL_lower_aft_i[2], RL_lower_aft_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_upper_o[3] = {RL_upper_o[1], -RL_upper_o[2], RL_upper_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_lower_o[3] = {RL_lower_o[1], -RL_lower_o[2], RL_lower_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_tie_i[3] = {RL_tie_i[1], -RL_tie_i[2], RL_tie_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_tie_o[3] = {RL_tie_o[1], -RL_tie_o[2], RL_tie_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position RR_wheel_center[3] = {RL_wheel_center[1], -RL_wheel_center[2], RL_wheel_center[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle RR_static_gamma = -1*RL_static_gamma annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle RR_static_alpha = -1*RL_static_alpha annotation(
    Dialog(group = "Geometry"));
  
  parameter SIunits.TranslationalSpringConstant[3] RUCA_fore_i_c = RL_double_wishbone.RUCA_fore_i_c "{x, y, z}-stiffness of upper, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] RUCA_aft_i_c = RL_double_wishbone.RUCA_aft_i_c "{x, y, z}-stiffness of upper, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] RLCA_fore_i_c = RL_double_wishbone.RLCA_fore_i_c "{x, y, z}-stiffness of lower, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] RLCA_aft_i_c = RL_double_wishbone.RLCA_aft_i_c "{x, y, z}-stiffness of lower, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] tie_i_c = RL_double_wishbone.tie_i_c "{x, y, z}-stiffness of inboard tie mount" annotation(
    Dialog(group = "Mounting"));
  
  parameter SIunits.TranslationalDampingConstant[3] RUCA_fore_i_d = RL_double_wishbone.RUCA_fore_i_d "{x, y, z}-damping of upper, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] RUCA_aft_i_d = RL_double_wishbone.RUCA_aft_i_d "{x, y, z}-damping of upper, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] RLCA_fore_i_d = RL_double_wishbone.RLCA_fore_i_d "{x, y, z}-damping of lower, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] RLCA_aft_i_d = RL_double_wishbone.RLCA_aft_i_d "{x, y, z}-damping of lower, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] tie_i_d = RL_double_wishbone.tie_i_d "{x, y, z}-damping of inboard tie mount" annotation(
    Dialog(group = "Mounting"));
  
  // Interface frame
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a axle_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_cp annotation(
    Placement(transformation(origin = {-120, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {-90, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_cp annotation(
    Placement(transformation(origin = {120, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {90, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  
  // Wheel torque inputs
  Modelica.Blocks.Interfaces.RealInput RL_torque annotation(
    Placement(transformation(origin = {-160, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput RR_torque annotation(
    Placement(transformation(origin = {160, 20}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {120, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  
  // Double wishbone assemblies
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.RrDoubleWishbone RL_double_wishbone annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-30, -30}, {30, 30}})));
  VehicleDynamics.Vehicle.Chassis.Suspension.Templates.RrDoubleWishbone RR_double_wishbone(upper_fore_i = RR_upper_fore_i, upper_aft_i = RR_upper_aft_i, lower_fore_i = RR_lower_fore_i, lower_aft_i = RR_lower_aft_i, upper_o = RR_upper_o, lower_o = RR_lower_o, tie_i = RR_tie_i, tie_o = RR_tie_o, wheel_center = RR_wheel_center, static_gamma = RR_static_gamma, static_alpha = RR_static_alpha, RUCA_fore_i_c = {RUCA_fore_i_c[1], -RUCA_fore_i_c[2], RUCA_fore_i_c[3]}, RUCA_aft_i_c = {RUCA_aft_i_c[1], -RUCA_aft_i_c[2], RUCA_aft_i_c[3]}, RLCA_fore_i_c = {RLCA_fore_i_c[1], -RLCA_fore_i_c[2], RLCA_fore_i_c[3]}, RLCA_aft_i_c = {RLCA_aft_i_c[1], -RLCA_aft_i_c[2], RLCA_aft_i_c[3]}, tie_i_c = {tie_i_c[1], -tie_i_c[2], tie_i_c[3]}, RUCA_fore_i_d = {RUCA_fore_i_d[1], -RUCA_fore_i_d[2], RUCA_fore_i_d[3]}, RUCA_aft_i_d = {RUCA_aft_i_d[1], -RUCA_aft_i_d[2], RUCA_aft_i_d[3]}, RLCA_fore_i_d = {RLCA_fore_i_d[1], -RLCA_fore_i_d[2], RLCA_fore_i_d[3]}, RLCA_aft_i_d = {RLCA_aft_i_d[1], -RLCA_aft_i_d[2], RLCA_aft_i_d[3]}, tie_i_d = {tie_i_d[1], -tie_i_d[2], tie_i_d[3]}) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{30, -30}, {-30, 30}})));
  
  // Tires
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire RL_tire annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{10, -10}, {-10, 10}})));
  VehicleDynamics.Vehicle.Chassis.Tires.MF5p2Tire RR_tire annotation(
    Placement(transformation(origin = {130, -50}, extent = {{-10, -10}, {10, 10}})));
  
  // Define effective center
  final parameter Real[3] effective_center = {RL_wheel_center[1], 0, RL_wheel_center[3]};
  
  // Fixed geometry from effective center to nodes
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_upper_fore_i_trans(r = RL_upper_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-30, 20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_upper_aft_i_trans(r = RL_upper_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_lower_fore_i_trans(r = RL_lower_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_lower_aft_i_trans(r = RL_lower_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RL_tie_i_trans(r = RL_tie_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_upper_fore_i_trans(r = RR_upper_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_upper_aft_i_trans(r = RR_upper_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_lower_fore_i_trans(r = RR_lower_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_lower_aft_i_trans(r = RR_lower_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RR_tie_i_trans(r = RR_tie_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(RL_tie_i_trans.frame_b, RL_double_wishbone.tie_i_frame) annotation(
    Line(points = {{-30, -70}, {-40, -70}}, color = {95, 95, 95}));
  connect(RL_lower_aft_i_trans.frame_b, RL_double_wishbone.lower_aft_i_frame) annotation(
    Line(points = {{-30, -50}, {-40, -50}}, color = {95, 95, 95}));
  connect(RL_lower_fore_i_trans.frame_b, RL_double_wishbone.lower_fore_i_frame) annotation(
    Line(points = {{-30, -30}, {-40, -30}}, color = {95, 95, 95}));
  connect(RL_upper_aft_i_trans.frame_b, RL_double_wishbone.upper_aft_i_frame) annotation(
    Line(points = {{-40, 0}, {-50, 0}, {-50, -20}}, color = {95, 95, 95}));
  connect(RL_upper_fore_i_trans.frame_b, RL_double_wishbone.upper_fore_i_frame) annotation(
    Line(points = {{-40, 20}, {-70, 20}, {-70, -20}}, color = {95, 95, 95}));
  connect(RR_tie_i_trans.frame_b, RR_double_wishbone.tie_i_frame) annotation(
    Line(points = {{30, -70}, {40, -70}}, color = {95, 95, 95}));
  connect(RR_lower_aft_i_trans.frame_b, RR_double_wishbone.lower_aft_i_frame) annotation(
    Line(points = {{30, -50}, {40, -50}}, color = {95, 95, 95}));
  connect(RR_lower_fore_i_trans.frame_b, RR_double_wishbone.lower_fore_i_frame) annotation(
    Line(points = {{30, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(RR_upper_aft_i_trans.frame_b, RR_double_wishbone.upper_aft_i_frame) annotation(
    Line(points = {{40, 0}, {50, 0}, {50, -20}}, color = {95, 95, 95}));
  connect(RR_upper_fore_i_trans.frame_b, RR_double_wishbone.upper_fore_i_frame) annotation(
    Line(points = {{40, 20}, {70, 20}, {70, -20}}, color = {95, 95, 95}));
  connect(axle_frame, RL_tie_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -70}, {-10, -70}}));
  connect(axle_frame, RR_tie_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -70}, {10, -70}}));
  connect(axle_frame, RL_lower_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -50}, {-10, -50}}));
  connect(axle_frame, RR_lower_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -50}, {10, -50}}));
  connect(axle_frame, RL_lower_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -30}, {-10, -30}}));
  connect(axle_frame, RR_lower_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -30}, {10, -30}}));
  connect(axle_frame, RL_upper_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 0}, {-20, 0}}));
  connect(axle_frame, RR_upper_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 0}, {20, 0}}));
  connect(axle_frame, RL_upper_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 20}, {-20, 20}}));
  connect(axle_frame, RR_upper_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 20}, {20, 20}}));
  connect(RL_tire.chassis_frame, RL_double_wishbone.midpoint_frame) annotation(
    Line(points = {{-120, -50}, {-100, -50}}, color = {95, 95, 95}));
  connect(RR_tire.chassis_frame, RR_double_wishbone.midpoint_frame) annotation(
    Line(points = {{120, -50}, {100, -50}}, color = {95, 95, 95}));
  connect(RL_torque, RL_tire.hub_torque) annotation(
    Line(points = {{-160, 20}, {-132.5, 20}, {-132.5, -38}}, color = {0, 0, 127}));
  connect(RR_torque, RR_tire.hub_torque) annotation(
    Line(points = {{160, 20}, {132.5, 20}, {132.5, -38}}, color = {0, 0, 127}));
  connect(RL_cp, RL_tire.cp_frame) annotation(
    Line(points = {{-120, -100}, {-120, -80}, {-130, -80}, {-130, -60}, {-130, -60}}));
  connect(RR_cp, RR_tire.cp_frame) annotation(
    Line(points = {{120, -100}, {120, -80}, {130, -80}, {130, -60}, {130, -60}}));
  annotation(
    Diagram(coordinateSystem(extent = {{-140, 100}, {140, -100}})));
end RrAxleBase;

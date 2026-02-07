within BobDynamics.Vehicle.Chassis.Suspension.Templates;

partial model FrAxleBase
  import Modelica.SIunits;
  
  // Parameters
  parameter SIunits.Position FL_upper_fore_i[3] = FL_double_wishbone.upper_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_upper_aft_i[3] = FL_double_wishbone.upper_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_lower_fore_i[3] = FL_double_wishbone.lower_fore_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_lower_aft_i[3] = FL_double_wishbone.lower_aft_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_upper_o[3] = FL_double_wishbone.upper_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_lower_o[3] = FL_double_wishbone.lower_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_tie_i[3] = FL_double_wishbone.tie_i annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_tie_o[3] = FL_double_wishbone.tie_o annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FL_wheel_center[3] = FL_double_wishbone.wheel_center annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle FL_static_gamma = FL_double_wishbone.static_gamma annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle FL_static_alpha = FL_double_wishbone.static_alpha annotation(
    Dialog(group = "Geometry"));
  
  parameter SIunits.Position FR_upper_fore_i[3] = {FL_upper_fore_i[1], -FL_upper_fore_i[2], FL_upper_fore_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_upper_aft_i[3] = {FL_upper_aft_i[1], -FL_upper_aft_i[2], FL_upper_aft_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_lower_fore_i[3] = {FL_lower_fore_i[1], -FL_lower_fore_i[2], FL_lower_fore_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_lower_aft_i[3] = {FL_lower_aft_i[1], -FL_lower_aft_i[2], FL_lower_aft_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_upper_o[3] = {FL_upper_o[1], -FL_upper_o[2], FL_upper_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_lower_o[3] = {FL_lower_o[1], -FL_lower_o[2], FL_lower_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_tie_i[3] = {FL_tie_i[1], -FL_tie_i[2], FL_tie_i[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_tie_o[3] = {FL_tie_o[1], -FL_tie_o[2], FL_tie_o[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Position FR_wheel_center[3] = {FL_wheel_center[1], -FL_wheel_center[2], FL_wheel_center[3]} annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle FR_static_gamma = -1*FL_static_gamma annotation(
    Dialog(group = "Geometry"));
  parameter SIunits.Angle FR_static_alpha = -1*FL_static_alpha annotation(
    Dialog(group = "Geometry"));
  
  parameter SIunits.TranslationalSpringConstant[3] FUCA_fore_i_c = FL_double_wishbone.FUCA_fore_i_c "{x, y, z}-stiffness of upper, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FUCA_aft_i_c = FL_double_wishbone.FUCA_aft_i_c "{x, y, z}-stiffness of upper, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_fore_i_c = FL_double_wishbone.FLCA_fore_i_c "{x, y, z}-stiffness of lower, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] FLCA_aft_i_c = FL_double_wishbone.FLCA_aft_i_c "{x, y, z}-stiffness of lower, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalSpringConstant[3] tie_i_c = FL_double_wishbone.tie_i_c "{x, y, z}-stiffness of inboard tie mount" annotation(
    Dialog(group = "Mounting"));
  
  parameter SIunits.TranslationalDampingConstant[3] FUCA_fore_i_d = FL_double_wishbone.FUCA_fore_i_d "{x, y, z}-damping of upper, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FUCA_aft_i_d = FL_double_wishbone.FUCA_aft_i_d "{x, y, z}-damping of upper, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_fore_i_d = FL_double_wishbone.FLCA_fore_i_d "{x, y, z}-damping of lower, fore, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] FLCA_aft_i_d = FL_double_wishbone.FLCA_aft_i_d "{x, y, z}-damping of lower, aft, inboard mount" annotation(
    Dialog(group = "Mounting"));
  parameter SIunits.TranslationalDampingConstant[3] tie_i_d = FL_double_wishbone.tie_i_d "{x, y, z}-damping of inboard tie mount" annotation(
    Dialog(group = "Mounting"));
  
  // Interface frame
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a axle_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_cp annotation(
    Placement(transformation(origin = {-120, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {-90, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_cp annotation(
    Placement(transformation(origin = {120, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {90, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  
  // Wheel torque inputs
  Modelica.Blocks.Interfaces.RealInput FL_torque annotation(
    Placement(transformation(origin = {-160, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 66}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput FR_torque annotation(
    Placement(transformation(origin = {160, 20}, extent = {{20, -20}, {-20, 20}}), iconTransformation(origin = {120, 66}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  
  // Double wishbone assemblies
  BobDynamics.Vehicle.Chassis.Suspension.Templates.FrDoubleWishbone FL_double_wishbone annotation(
    Placement(transformation(origin = {-70, -50}, extent = {{-30, -30}, {30, 30}})));
  BobDynamics.Vehicle.Chassis.Suspension.Templates.FrDoubleWishbone FR_double_wishbone(upper_fore_i = FR_upper_fore_i, upper_aft_i = FR_upper_aft_i, lower_fore_i = FR_lower_fore_i, lower_aft_i = FR_lower_aft_i, upper_o = FR_upper_o, lower_o = FR_lower_o, tie_i = FR_tie_i, tie_o = FR_tie_o, wheel_center = FR_wheel_center, static_gamma = FR_static_gamma, static_alpha = FR_static_alpha, FUCA_fore_i_c = {FUCA_fore_i_c[1], -FUCA_fore_i_c[2], FUCA_fore_i_c[3]}, FUCA_aft_i_c = {FUCA_aft_i_c[1], -FUCA_aft_i_c[2], FUCA_aft_i_c[3]}, FLCA_fore_i_c = {FLCA_fore_i_c[1], -FLCA_fore_i_c[2], FLCA_fore_i_c[3]}, FLCA_aft_i_c = {FLCA_aft_i_c[1], -FLCA_aft_i_c[2], FLCA_aft_i_c[3]}, tie_i_c = {tie_i_c[1], -tie_i_c[2], tie_i_c[3]}, FUCA_fore_i_d = {FUCA_fore_i_d[1], -FUCA_fore_i_d[2], FUCA_fore_i_d[3]}, FUCA_aft_i_d = {FUCA_aft_i_d[1], -FUCA_aft_i_d[2], FUCA_aft_i_d[3]}, FLCA_fore_i_d = {FLCA_fore_i_d[1], -FLCA_fore_i_d[2], FLCA_fore_i_d[3]}, FLCA_aft_i_d = {FLCA_aft_i_d[1], -FLCA_aft_i_d[2], FLCA_aft_i_d[3]}, tie_i_d = {tie_i_d[1], -tie_i_d[2], tie_i_d[3]}) annotation(
    Placement(transformation(origin = {70, -50}, extent = {{30, -30}, {-30, 30}})));
  
  // Tires
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire FL_tire annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{10, -10}, {-10, 10}})));
  BobDynamics.Vehicle.Chassis.Tires.MF5p2Tire FR_tire annotation(
    Placement(transformation(origin = {130, -50}, extent = {{-10, -10}, {10, 10}})));
  
  // Define effective center
  final parameter Real[3] effective_center = {FL_wheel_center[1], 0, FL_wheel_center[3]};
  
  // Fixed geometry from effective center to nodes
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_upper_fore_i_trans(r = FL_upper_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-30, 20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_upper_aft_i_trans(r = FL_upper_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_lower_fore_i_trans(r = FL_lower_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_lower_aft_i_trans(r = FL_lower_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -50}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FL_tie_i_trans(r = FL_tie_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_upper_fore_i_trans(r = FR_upper_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_upper_aft_i_trans(r = FR_upper_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_lower_fore_i_trans(r = FR_lower_fore_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_lower_aft_i_trans(r = FR_lower_aft_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation FR_tie_i_trans(r = FR_tie_i - effective_center, animation = false) annotation(
    Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(FL_tie_i_trans.frame_b, FL_double_wishbone.tie_i_frame) annotation(
    Line(points = {{-30, -70}, {-40, -70}}, color = {95, 95, 95}));
  connect(FL_lower_aft_i_trans.frame_b, FL_double_wishbone.lower_aft_i_frame) annotation(
    Line(points = {{-30, -50}, {-40, -50}}, color = {95, 95, 95}));
  connect(FL_lower_fore_i_trans.frame_b, FL_double_wishbone.lower_fore_i_frame) annotation(
    Line(points = {{-30, -30}, {-40, -30}}, color = {95, 95, 95}));
  connect(FL_upper_aft_i_trans.frame_b, FL_double_wishbone.upper_aft_i_frame) annotation(
    Line(points = {{-40, 0}, {-50, 0}, {-50, -20}}, color = {95, 95, 95}));
  connect(FL_upper_fore_i_trans.frame_b, FL_double_wishbone.upper_fore_i_frame) annotation(
    Line(points = {{-40, 20}, {-70, 20}, {-70, -20}}, color = {95, 95, 95}));
  connect(FR_tie_i_trans.frame_b, FR_double_wishbone.tie_i_frame) annotation(
    Line(points = {{30, -70}, {40, -70}}, color = {95, 95, 95}));
  connect(FR_lower_aft_i_trans.frame_b, FR_double_wishbone.lower_aft_i_frame) annotation(
    Line(points = {{30, -50}, {40, -50}}, color = {95, 95, 95}));
  connect(FR_lower_fore_i_trans.frame_b, FR_double_wishbone.lower_fore_i_frame) annotation(
    Line(points = {{30, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(FR_upper_aft_i_trans.frame_b, FR_double_wishbone.upper_aft_i_frame) annotation(
    Line(points = {{40, 0}, {50, 0}, {50, -20}}, color = {95, 95, 95}));
  connect(FR_upper_fore_i_trans.frame_b, FR_double_wishbone.upper_fore_i_frame) annotation(
    Line(points = {{40, 20}, {70, 20}, {70, -20}}, color = {95, 95, 95}));
  connect(axle_frame, FL_tie_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -70}, {-10, -70}}));
  connect(axle_frame, FR_tie_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -70}, {10, -70}}));
  connect(axle_frame, FL_lower_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -50}, {-10, -50}}));
  connect(axle_frame, FR_lower_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -50}, {10, -50}}));
  connect(axle_frame, FL_lower_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -30}, {-10, -30}}));
  connect(axle_frame, FR_lower_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, -30}, {10, -30}}));
  connect(axle_frame, FL_upper_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 0}, {-20, 0}}));
  connect(axle_frame, FR_upper_aft_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 0}, {20, 0}}));
  connect(axle_frame, FL_upper_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 20}, {-20, 20}}));
  connect(axle_frame, FR_upper_fore_i_trans.frame_a) annotation(
    Line(points = {{0, -100}, {0, 20}, {20, 20}}));
  connect(FL_tire.chassis_frame, FL_double_wishbone.midpoint_frame) annotation(
    Line(points = {{-120, -50}, {-100, -50}}, color = {95, 95, 95}));
  connect(FR_tire.chassis_frame, FR_double_wishbone.midpoint_frame) annotation(
    Line(points = {{120, -50}, {100, -50}}, color = {95, 95, 95}));
  connect(FL_torque, FL_tire.hub_torque) annotation(
    Line(points = {{-160, 20}, {-132.5, 20}, {-132.5, -38}}, color = {0, 0, 127}));
  connect(FR_torque, FR_tire.hub_torque) annotation(
    Line(points = {{160, 20}, {132.5, 20}, {132.5, -38}}, color = {0, 0, 127}));
  connect(FL_cp, FL_tire.cp_frame) annotation(
    Line(points = {{-120, -100}, {-120, -80}, {-130, -80}, {-130, -60}, {-130, -60}}));
  connect(FR_cp, FR_tire.cp_frame) annotation(
    Line(points = {{120, -100}, {120, -80}, {130, -80}, {130, -60}, {130, -60}}));
  annotation(
    Diagram(coordinateSystem(extent = {{-140, 100}, {140, -100}})));
end FrAxleBase;

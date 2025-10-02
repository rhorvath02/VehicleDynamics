within VehicleDynamics.Vehicle.Chassis.Suspension;

model DoubleWishboneBase
  import Modelica.SIunits;
  
  //JSON
//  inner ExternData.JSONFile hdpts(fileName = "/home/rhorvath/Documents/Github/VehicleDynamics/Model Definitions/Nightwatch.json") annotation(
//    Placement(transformation(origin = {-90, 90}, extent = {{10, -10}, {-10, 10}})));
  
  // Parameters
  parameter SIunits.Position upper_fore_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.fore_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_aft_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.aft_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_fore_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.fore_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_aft_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.aft_i", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position upper_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position lower_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tie.inboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position tie_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tie.outboard", 3) annotation(Dialog(group="Geometry"));
  parameter SIunits.Position contact_patch[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tire.contact_patch", 3) annotation(Dialog(group="Geometry"));
  
  parameter SIunits.Length link_diameter = 0.025 annotation(Dialog(group="Animation"));
  parameter SIunits.Length joint_diameter = 0.030 annotation(Dialog(group="Animation"));
  // Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upper_fore_i_frame annotation(
    Placement(transformation(origin = {100, 80}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a upper_aft_i_frame annotation(
    Placement(transformation(origin = {100, 40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {66, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lower_fore_i_frame annotation(
    Placement(transformation(origin = {100, -80}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a lower_aft_i_frame annotation(
    Placement(transformation(origin = {100, -40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a tie_i_frame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b upper_wishbone_frame annotation(
    Placement(transformation(origin = {40, 100}, extent = {{16, -16}, {-16, 16}}, rotation = 90), iconTransformation(origin = {-66, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b lower_wishbone_frame annotation(
    Placement(transformation(origin = {40, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {66, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b contact_patch_frame annotation(
    Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b mass_frame annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  
  // Wishbones
  Suspension.Linkages.Wishbone upper_wishbone(fore_i = upper_fore_i, aft_i = upper_aft_i, outboard = upper_o, link_diameter = link_diameter, joint_diameter = joint_diameter) annotation(
    Placement(transformation(origin = {30, 60}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone lower_wishbone(aft_i = lower_aft_i, fore_i = lower_fore_i, outboard = lower_o, link_diameter = link_diameter, joint_diameter = joint_diameter) annotation(
    Placement(transformation(origin = {30, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
  
  // Upright
  Suspension.Linkages.Upright upright(lower = lower_o, upper = upper_o, tie = tie_o) annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}})));
  
  // Tie rod
  Suspension.Linkages.Link link(inboard = tie_i, outboard = tie_o, link_diameter = link_diameter, joint_diameter = joint_diameter) annotation(
    Placement(transformation(origin = {20, 0}, extent = {{10, -10}, {-10, 10}})));
  
  // Contact patch
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch(r = contact_patch - lower_o) annotation(
    Placement(transformation(origin = {-30, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  
equation
  connect(upper_fore_i_frame, upper_wishbone.fore_i_frame) annotation(
    Line(points = {{100, 80}, {70, 80}, {70, 66}, {40, 66}}));
  connect(upper_aft_i_frame, upper_wishbone.aft_i_frame) annotation(
    Line(points = {{100, 40}, {70, 40}, {70, 54}, {40, 54}}));
  connect(tie_i_frame, link.frame_a) annotation(
    Line(points = {{100, 0}, {30, 0}}));
  connect(lower_wishbone_frame, lower_wishbone.link_frame) annotation(
    Line(points = {{40, -100}, {40, -80}, {24, -80}, {24, -70}}));
  connect(upper_wishbone_frame, upper_wishbone.link_frame) annotation(
    Line(points = {{40, 100}, {40, 80}, {24, 80}, {24, 70}}));
  connect(link.frame_b, upright.tie_frame) annotation(
    Line(points = {{10, 0}, {0, 0}}, color = {95, 95, 95}));
  connect(lower_wishbone.outboard_frame, upright.lower_frame) annotation(
    Line(points = {{20, -60}, {-10, -60}, {-10, -10}}, color = {95, 95, 95}));
  connect(upper_wishbone.outboard_frame, upright.upper_frame) annotation(
    Line(points = {{20, 60}, {-10, 60}, {-10, 10}}, color = {95, 95, 95}));
  connect(upright.mass_frame, mass_frame) annotation(
    Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch.frame_a, upright.lower_frame) annotation(
    Line(points = {{-30, -40}, {-30, -20}, {-10, -20}, {-10, -10}}, color = {95, 95, 95}));
  connect(contact_patch_frame, lower_to_contact_patch.frame_b) annotation(
    Line(points = {{0, -100}, {0, -80}, {-30, -80}, {-30, -60}}));
  connect(lower_aft_i_frame, lower_wishbone.aft_i_frame) annotation(
    Line(points = {{100, -40}, {70, -40}, {70, -54}, {40, -54}}));
  connect(lower_fore_i_frame, lower_wishbone.fore_i_frame) annotation(
    Line(points = {{100, -80}, {70, -80}, {70, -66}, {40, -66}}));
end DoubleWishboneBase;

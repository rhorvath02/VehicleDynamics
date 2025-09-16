within Vehicle.Chassis.Suspension;
model DoubleWishboneBase

  Suspension.Linkages.Wishbone wishbone(fore_i = upper_fore_i, aft_i = upper_aft_i, outboard = upper_o, link_diameter = 0.010, joint_diameter = 0.030) annotation(
    Placement(transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = upper_fore_i, animation = false)  annotation(
    Placement(transformation(origin = {50, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = upper_aft_i, animation = false) annotation(
    Placement(transformation(origin = {50, 30}, extent = {{10, -10}, {-10, 10}})));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-150, -90}, extent = {{-10, -10}, {10, 10}})));
  Vehicle.Chassis.Suspension.Linkages.Wishbone wishbone1(aft_i = lower_aft_i, fore_i = lower_fore_i, outboard = lower_o, link_diameter = 0.010, joint_diameter = 0.030) annotation(
    Placement(transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(animation = false, r = lower_fore_i) annotation(
    Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed11(animation = false, r = lower_aft_i) annotation(
    Placement(transformation(origin = {50, -90}, extent = {{10, -10}, {-10, 10}})));
  Suspension.Linkages.Upright upright(lower = lower_o, upper = upper_o, tie = tie_o)  annotation(
    Placement(transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}})));
  Suspension.Linkages.Link link(inboard = tie_i, outboard = tie_o, link_diameter = 0.010, joint_diameter = 0.030)  annotation(
    Placement(transformation(origin = {-10, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed12(animation = false, r = tie_i) annotation(
    Placement(transformation(origin = {30, 0}, extent = {{10, -10}, {-10, 10}})));
  
  inner ExternData.JSONFile hdpts(fileName = "/home/rhorvath/Documents/Github/VehicleDynamics/Model Definitions/Nightwatch.json")  annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{10, -10}, {-10, 10}})));
    
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower_to_contact_patch(r = contact_patch - lower_o) annotation(
    Placement(transformation(origin = {-60, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  
  parameter Real upper_fore_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.fore_i", 3);
  parameter Real upper_aft_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.aft_i", 3);
  parameter Real lower_fore_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.fore_i", 3);
  parameter Real lower_aft_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.aft_i", 3);
  parameter Real upper_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.upper.outboard", 3);
  parameter Real lower_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.lower.outboard", 3);
  parameter Real tie_i[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tie.inboard", 3);
  parameter Real tie_o[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tie.outboard", 3);
  parameter Real contact_patch[3] = hdpts.getRealArray1D("Hardpoints.Front.left.tire.contact_patch", 3);
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed111(animation = false, r = contact_patch) annotation(
    Placement(transformation(origin = {-130, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Vehicle.Chassis.Suspension.Joints.SphericalCompliant from_link(diameter = 0.030, r_rel(each fixed = true)) annotation(
    Placement(transformation(origin = {-90, -70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
equation
  connect(fixed.frame_b, wishbone.fore_i_frame) annotation(
    Line(points = {{40, 90}, {20, 90}, {20, 66}, {10, 66}}, color = {95, 95, 95}));
  connect(fixed1.frame_b, wishbone.aft_i_frame) annotation(
    Line(points = {{40, 30}, {20, 30}, {20, 54}, {10, 54}}, color = {95, 95, 95}));
  connect(fixed2.frame_b, wishbone1.fore_i_frame) annotation(
    Line(points = {{40, -30}, {20, -30}, {20, -54}, {10, -54}}, color = {95, 95, 95}));
  connect(fixed11.frame_b, wishbone1.aft_i_frame) annotation(
    Line(points = {{40, -90}, {20, -90}, {20, -66}, {10, -66}}, color = {95, 95, 95}));
  connect(wishbone.outboard_frame, upright.upper_frame) annotation(
    Line(points = {{-10, 60}, {-40, 60}, {-40, 10}}, color = {95, 95, 95}));
  connect(wishbone1.outboard_frame, upright.lower_frame) annotation(
    Line(points = {{-10, -60}, {-40, -60}, {-40, -10}}, color = {95, 95, 95}));
  connect(link.frame_b, upright.tie_frame) annotation(
    Line(points = {{-20, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(lower_to_contact_patch.frame_a, upright.lower_frame) annotation(
    Line(points = {{-60, -40}, {-60, -20}, {-40, -20}, {-40, -10}}, color = {95, 95, 95}));
  connect(fixed111.frame_b, from_link.frame_b) annotation(
    Line(points = {{-120, -70}, {-100, -70}}, color = {95, 95, 95}));
  connect(from_link.frame_a, lower_to_contact_patch.frame_b) annotation(
    Line(points = {{-80, -70}, {-60, -70}, {-60, -60}}, color = {95, 95, 95}));
  connect(fixed12.frame_b, link.frame_a) annotation(
    Line(points = {{20, 0}, {0, 0}}, color = {95, 95, 95}));
end DoubleWishboneBase;
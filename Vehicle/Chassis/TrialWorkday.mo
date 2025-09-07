within Vehicle.Chassis;
model TrialWorkday
  import Modelica.Units.SI;
  
  // Node positions
  parameter SI.Position contact_patch[3] = {0, 0.609600, 0} "Position of contact patch";
  parameter SI.Position upper_fore_i[3] = {0.086868, 0.215900, 0.200000} "Position of upper-fore-inboard pickup";
  parameter SI.Position upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} "Position of upper-aft-inboard pickup";
  parameter SI.Position upper_o[3] = {-0.006347, 0.523240, 0.287020} "Position of upper-outboard pickup";
  parameter SI.Position tie_i[3] = {0.041128, 0.215900, 0.117856} "Position of tie-inboard pickup";
  parameter SI.Position tie_o[3] = {0.056000, 0.532333, 0.164821} "Position of tie-outboard pickup";
  parameter SI.Position lower_fore_i[3] = {0.087376, 0.215900, 0.090000} "Position of lower-fore-inboard pickup";
  parameter SI.Position lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} "Position of lower-aft-inboard pickup";
  parameter SI.Position lower_o[3] = {0, 0.556499, 0.124998} "Position of lower-fore-inboard pickup";
  parameter SI.Position shock_i[3] = {-0.09276858, 0.24012839, 0.56990187} "Position of inboard shock pickup";
  // Pre-computed vectors
  final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
  final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
  final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
  final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));

  // TODO: add contents
  Suspension.DoubleWishboneV3 doubleWishboneV3 annotation(
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = r_upper_mount, animation = false)  annotation(
    Placement(transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = tie_i, animation = false)  annotation(
    Placement(transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed11(r = r_lower_mount, animation = false)  annotation(
    Placement(transformation(origin = {10, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed12(r = contact_patch, animation = false) annotation(
    Placement(transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Sine sine(amplitude = 3*0.0254, f = 1)  annotation(
    Placement(transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine sine1(amplitude = 0, f = 1)  annotation(
    Placement(transformation(origin = {-90, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(animation = false, r = shock_i) annotation(
    Placement(transformation(origin = {10, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Vehicle.Chassis.Suspension.DoubleWishboneV3 doubleWishboneV31(contact_patch = {0, -0.609600, 0}, upper_fore_i = {0.086868, -0.215900, 0.200000}, upper_aft_i = {-0.095250, -0.215900, 0.200000}, upper_o = {-0.006347, -0.523240, 0.287020}, tie_i = {0.041128, -0.215900, 0.117856}, tie_o = {0.056000, -0.532333, 0.164821}, lower_fore_i = {0.087376, -0.215900, 0.090000}, lower_aft_i = {-0.095250, -0.215900, 0.090000}, lower_o = {0, -0.556499, 0.124998}, pushrod_o = {-0.01368377, -0.49897726, 0.30564884}, pushrod_i = {-0.04415429, -0.39924560, 0.40746267})  annotation(
    Placement(transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed3(animation = false, r = {r_upper_mount[1], -r_upper_mount[2], r_upper_mount[3]}) annotation(
    Placement(transformation(origin = {150, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed13(animation = false, r = {tie_i[1], -tie_i[2], tie_i[3]}) annotation(
    Placement(transformation(origin = {150, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed111(animation = false, r = {r_lower_mount[1], -r_lower_mount[2], r_lower_mount[3]}) annotation(
    Placement(transformation(origin = {150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed121(animation = false, r = {contact_patch[1], -contact_patch[2], contact_patch[3]}) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Sine sine2(amplitude = 3*0.0254, f = 1) annotation(
    Placement(transformation(origin = {50, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Sine sine11(amplitude = 0, f = 1) annotation(
    Placement(transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed21(animation = false, r = {shock_i[1], -shock_i[2], shock_i[3]}) annotation(
    Placement(transformation(origin = {150, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
equation
  connect(fixed11.frame_b, doubleWishboneV3.frame_a1) annotation(
    Line(points = {{0, -10}, {-20, -10}, {-20, 24}, {-40, 24}}, color = {95, 95, 95}));
  connect(fixed.frame_b, doubleWishboneV3.frame_a) annotation(
    Line(points = {{0, 70}, {-20, 70}, {-20, 36}, {-40, 36}}, color = {95, 95, 95}));
  connect(fixed1.frame_b, doubleWishboneV3.frame_a2) annotation(
    Line(points = {{0, 30}, {-40, 30}}, color = {95, 95, 95}));
  connect(fixed12.frame_b, doubleWishboneV3.frame_a3) annotation(
    Line(points = {{-50, -20}, {-50, 20}}, color = {95, 95, 95}));
  connect(sine.y, doubleWishboneV3.JounceInput) annotation(
    Line(points = {{-78, 10}, {-56, 10}, {-56, 20}}, color = {0, 0, 127}));
  connect(sine1.y, doubleWishboneV3.SteerInput) annotation(
    Line(points = {{-78, 50}, {-56, 50}, {-56, 40}}, color = {0, 0, 127}));
  connect(fixed2.frame_b, doubleWishboneV3.frame_a4) annotation(
    Line(points = {{0, 110}, {-44, 110}, {-44, 40}}, color = {95, 95, 95}));
  connect(fixed111.frame_b, doubleWishboneV31.frame_a1) annotation(
    Line(points = {{140, -10}, {120, -10}, {120, 24}, {100, 24}}, color = {95, 95, 95}));
  connect(fixed3.frame_b, doubleWishboneV31.frame_a) annotation(
    Line(points = {{140, 70}, {120, 70}, {120, 36}, {100, 36}}, color = {95, 95, 95}));
  connect(fixed13.frame_b, doubleWishboneV31.frame_a2) annotation(
    Line(points = {{140, 30}, {100, 30}}, color = {95, 95, 95}));
  connect(fixed121.frame_b, doubleWishboneV31.frame_a3) annotation(
    Line(points = {{90, -20}, {90, 20}}, color = {95, 95, 95}));
  connect(sine2.y, doubleWishboneV31.JounceInput) annotation(
    Line(points = {{61, 10}, {83, 10}, {83, 20}}, color = {0, 0, 127}));
  connect(sine11.y, doubleWishboneV31.SteerInput) annotation(
    Line(points = {{61, 50}, {83, 50}, {83, 40}}, color = {0, 0, 127}));
  connect(fixed21.frame_b, doubleWishboneV31.frame_a4) annotation(
    Line(points = {{140, 110}, {96, 110}, {96, 40}}, color = {95, 95, 95}));
end TrialWorkday;

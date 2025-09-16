within Vehicle.Chassis.Body;
model Frame
  
  // FL Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_upper_fore_i_frame annotation(
    Placement(transformation(origin = {-100, 220}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 530}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_upper_aft_i_frame annotation(
    Placement(transformation(origin = {-100, 180}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 398}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_tie_i_frame annotation(
    Placement(transformation(origin = {-100, 120}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 298}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_lower_fore_i_frame annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 198}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FL_lower_aft_i_frame annotation(
    Placement(transformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 66}, extent = {{-16, -16}, {16, 16}})));
    
  // FR Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_upper_fore_i_frame annotation(
    Placement(transformation(origin = {100, 220}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 530}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_upper_aft_i_frame annotation(
    Placement(transformation(origin = {100, 180}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 398}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_tie_i_frame annotation(
    Placement(transformation(origin = {100, 120}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 298}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_lower_fore_i_frame annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 198}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b FR_lower_aft_i_frame annotation(
    Placement(transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 66}, extent = {{-16, -16}, {16, 16}})));
  
  // RL Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_upper_fore_i_frame annotation(
    Placement(transformation(origin = {-100, -20}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -66}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_upper_aft_i_frame annotation(
    Placement(transformation(origin = {-100, -60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -198}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_tie_i_frame annotation(
    Placement(transformation(origin = {-100, -120}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -298}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_lower_fore_i_frame annotation(
    Placement(transformation(origin = {-100, -180}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -398}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RL_lower_aft_i_frame annotation(
    Placement(transformation(origin = {-100, -220}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, -530}, extent = {{-16, -16}, {16, 16}})));
  
  // RR Frames
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_upper_fore_i_frame annotation(
    Placement(transformation(origin = {100, -20}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -66}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_upper_aft_i_frame annotation(
    Placement(transformation(origin = {100, -60}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -198}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_tie_i_frame annotation(
    Placement(transformation(origin = {100, -120}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -298}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_lower_fore_i_frame annotation(
    Placement(transformation(origin = {100, -180}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -398}, extent = {{-16, -16}, {16, 16}})));  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b RR_lower_aft_i_frame annotation(
    Placement(transformation(origin = {100, -220}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, -530}, extent = {{-16, -16}, {16, 16}})));
   
annotation(
    Diagram(coordinateSystem(extent = {{-100, -240}, {100, 240}},
    preserveAspectRatio=false)),
    Placement(transformation(
      extent={{-50,-20},{50,20}}
    )));
end Frame;
within Vehicle;
model GroundPhysics
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
    
  parameter Real c = 1e8 "Stiffness";
  parameter Real d = 1e4 "Damping";
  
protected
  Real r_rel_z "Relative z-displacement";
  Real v_rel_z "Relative z-velocity";
  Real f_z "Normal force";
  
equation
  r_rel_z = frame_b.r_0[3] - frame_a.r_0[3];
  v_rel_z = der(r_rel_z);
  
  // Only apply force when compressed (r_rel[3] < 0)
  f_z = if r_rel_z < 0 then -c * r_rel_z - d * v_rel_z else 0;
  
  frame_a.f = {0, 0, f_z};
  frame_b.f = -frame_a.f;
  
  frame_a.t = {0, 0, 0};
  frame_b.t = {0, 0, 0};
  
end GroundPhysics;
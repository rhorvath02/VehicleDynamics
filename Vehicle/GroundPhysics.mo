within Vehicle;
model GroundPhysics
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
    Placement(transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}})));
    
  parameter Real c = 1e5 "Stiffness";
  parameter Real d = 1e2 "Damping";
  
protected
  Real r_rel[3] "Relative displacement";
  Real v_rel[3] "Relative velocity";
  Real f "Internal force";

equation
  r_rel = frame_b.r_0 - frame_a.r_0;
  v_rel = der(r_rel);

  // Only apply force when compressed (r_rel[3] < 0)
  f = if r_rel[3] < 0 then
        c*(-r_rel[3]) + d*(-v_rel[3])
      else 0;

  frame_a.f = {0, 0, f};
  frame_b.f = {0, 0, -f};
  
  frame_a.t = {0, 0, 0};
  frame_b.t = {0, 0, 0};
end GroundPhysics;

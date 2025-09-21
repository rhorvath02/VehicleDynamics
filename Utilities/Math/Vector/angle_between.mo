within Utilities.Math.Vector;

function angle_between
  import Utilities.Math.Vector.cross;
  import Utilities.Math.Vector.dot;
  
  import Modelica.Math.Vectors.normalize;
  import Modelica.Math.Vectors.norm;
  
  input Real a[3];
  input Real b[3];
  input Real n[3]; // axis of rotation
  output Real theta;
  
protected 
  Real dot_ab;
  Real cross_ab[3];
  Real sign_theta;

algorithm
  
  if norm(a) < 0.25 or norm(b) < 0.25 then
    dot_ab := 0;
    cross_ab := {0, 0, 0};
    sign_theta := 1;
    theta := 0;
  else
    // Compute dot and clamp to avoid domain errors in acos
    dot_ab := dot(normalize(a), normalize(b));
    dot_ab := max(-1.0, min(1.0, dot_ab));
  
    // Compute cross and sign
    cross_ab := cross(normalize(a), normalize(b));
    sign_theta := if dot(n, cross_ab) >= 0 then 1 else -1;
  
    // Final angle
    theta := sign_theta * acos(dot_ab);
  end if;
  
end angle_between;

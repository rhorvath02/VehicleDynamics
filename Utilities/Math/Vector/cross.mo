within Utilities.Math.Vector;

function cross
  input Real[3] a;
  input Real[3] b;
  output Real[3] result;
algorithm
  result[1] := a[2]*b[3] - a[3]*b[2];
  result[2] := a[3]*b[1] - a[1]*b[3];
  result[3] := a[1]*b[2] - a[2]*b[1];
end cross;

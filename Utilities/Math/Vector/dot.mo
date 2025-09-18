within Utilities.Math.Vector;

function dot
  input Real[:] a;
  input Real[:] b;
  output Real result;
protected 
  Integer n;
algorithm
  n := size(a, 1);
  assert(n == size(b, 1), "Vectors must be the same length");
  result := 0;
  for i in 1:n loop
    result := result + a[i] * b[i];
  end for;
end dot;

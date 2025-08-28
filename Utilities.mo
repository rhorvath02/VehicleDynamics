package Utilities
  function dot3
    input Real a[3];
    input Real b[3];
    output Real result;
  algorithm
    result := a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
  end dot3;
end Utilities;
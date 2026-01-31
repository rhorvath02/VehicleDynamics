within VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone;

model LeftDoubleWishbone
  extends VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone.DoubleWishboneBase(
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUnsprung
      unsprung_mass,
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUCA
      uca_mass,
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrLCA
      lca_mass,
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrTie
      tie_mass
  );

equation
  connect(prismatic_rack.frame_b, sphericalSpherical.frame_a) annotation(
    Line(points = {{60, 0}, {50, 0}}, color = {95, 95, 95}));
  connect(sphericalSpherical.frame_b, upright.tie_frame) annotation(
    Line(points = {{30, 0}, {20, 0}}, color = {95, 95, 95}));
end LeftDoubleWishbone;

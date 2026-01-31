within VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone;

model RightDoubleWishbone
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUnsprung left_unsprung_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUCA left_uca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrLCA left_lca_mass;
  final parameter VehicleDynamics.Resources.Records.MASSPROPS.FrTie left_tie_mass;
  
  extends VehicleDynamics.Vehicle.Chassis.Suspension.Templates.DoubleWishbone.DoubleWishboneBase(
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUnsprung unsprung_mass(
      r_cm = {
        left_unsprung_mass.r_cm[1],
       -left_unsprung_mass.r_cm[2],
        left_unsprung_mass.r_cm[3]
      },
      I = {
        { left_unsprung_mass.I[1,1], -left_unsprung_mass.I[1,2],  left_unsprung_mass.I[1,3] },
        { -left_unsprung_mass.I[2,1], left_unsprung_mass.I[2,2], -left_unsprung_mass.I[2,3] },
        { left_unsprung_mass.I[3,1], -left_unsprung_mass.I[3,2],  left_unsprung_mass.I[3,3] }
      }
    ),
  
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrUCA uca_mass(
      r_cm = {
        left_uca_mass.r_cm[1],
       -left_uca_mass.r_cm[2],
        left_uca_mass.r_cm[3]
      },
      I = {
        { left_uca_mass.I[1,1], -left_uca_mass.I[1,2],  left_uca_mass.I[1,3] },
        { -left_uca_mass.I[2,1], left_uca_mass.I[2,2], -left_uca_mass.I[2,3] },
        { left_uca_mass.I[3,1], -left_uca_mass.I[3,2],  left_uca_mass.I[3,3] }
      }
    ),
  
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrLCA lca_mass(
      r_cm = {
        left_lca_mass.r_cm[1],
       -left_lca_mass.r_cm[2],
        left_lca_mass.r_cm[3]
      },
      I = {
        { left_lca_mass.I[1,1], -left_lca_mass.I[1,2],  left_lca_mass.I[1,3] },
        { -left_lca_mass.I[2,1], left_lca_mass.I[2,2], -left_lca_mass.I[2,3] },
        { left_lca_mass.I[3,1], -left_lca_mass.I[3,2],  left_lca_mass.I[3,3] }
      }
    ),
  
    redeclare parameter VehicleDynamics.Resources.Records.MASSPROPS.FrTie tie_mass(
      r_cm = {
        left_tie_mass.r_cm[1],
       -left_tie_mass.r_cm[2],
        left_tie_mass.r_cm[3]
      },
      I = {
        { left_tie_mass.I[1,1], -left_tie_mass.I[1,2],  left_tie_mass.I[1,3] },
        { -left_tie_mass.I[2,1], left_tie_mass.I[2,2], -left_tie_mass.I[2,3] },
        { left_tie_mass.I[3,1], -left_tie_mass.I[3,2],  left_tie_mass.I[3,3] }
      }
    )
  );

equation
  connect(sphericalSpherical.frame_a, upright.tie_frame) annotation(
    Line(points = {{50, 0}, {50, -20}, {20, -20}, {20, 0}}, color = {95, 95, 95}));
  connect(sphericalSpherical.frame_b, prismatic_rack.frame_b) annotation(
    Line(points = {{30, 0}, {30, 20}, {60, 20}, {60, 0}}, color = {95, 95, 95}));
end RightDoubleWishbone;

package Vehicle
  package Chassis
    package Suspension
      package Tests
        package Linkages
          model TestCompliantLink
            inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
              Placement(transformation(origin = {-30, -30}, extent = {{-10, -10}, {10, 10}})));
            Modelica.Mechanics.MultiBody.Parts.Fixed Fixed(r = {0, 0, 0}) annotation(
              Placement(transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
            Vehicle.Chassis.Suspension.Linkages.CompliantLink CompliantLink1(r = {0.25, 0, -0.25}) annotation(
              Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
            Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, sphereDiameter = 0.825*0.0254) annotation(
              Placement(transformation(origin = {30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          equation
            connect(Fixed.frame_b, CompliantLink1.frame_a) annotation(
              Line(points = {{-20, 0}, {-10, 0}}, color = {95, 95, 95}));
            connect(CompliantLink1.frame_b, body.frame_a) annotation(
              Line(points = {{10, 0}, {20, 0}}, color = {95, 95, 95}));
            annotation(
              Diagram(coordinateSystem(extent = {{-40, 20}, {40, -40}})));
          end TestCompliantLink;

          model TestCompliantWishbone
            inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
              Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}})));
            Vehicle.Chassis.Suspension.Linkages.CompliantWishbone CompliantWishbone(fore_inboard_outboard = {-0.25, 0.5, 0}, aft_inboard_outboard = {0, 0.5, 0}) annotation(
              Placement(transformation(extent = {{-10, -10}, {10, 10}})));
            Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = {0, 0, 0}) annotation(
              Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
            Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(r = {0.25, 0, 0}) annotation(
              Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
            Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, sphereDiameter = 0.825*0.0254) annotation(
              Placement(transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          equation
            connect(CompliantWishbone.frame_a_fore, fixed2.frame_b) annotation(
              Line(points = {{10, 6}, {30, 6}, {30, 30}, {40, 30}}, color = {95, 95, 95}));
            connect(CompliantWishbone.frame_a_aft, fixed1.frame_b) annotation(
              Line(points = {{10, -6}, {30, -6}, {30, -30}, {40, -30}}, color = {95, 95, 95}));
            connect(body.frame_a, CompliantWishbone.frame_b_outer) annotation(
              Line(points = {{-20, 0}, {-10, 0}}, color = {95, 95, 95}));
            annotation(
              Diagram(coordinateSystem(extent = {{-60, 60}, {60, -60}})));
          end TestCompliantWishbone;
        end Linkages;
      end Tests;

      package Sources
        model ContactPatchSource
          import Modelica.Units.SI;
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
            Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.Translational.Sources.Position position(exact = false) annotation(
            Placement(transformation(origin = {10, 54}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Interfaces.RealInput realInput annotation(
            Placement(transformation(origin = {-120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
            Placement(transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticX(animation = false) annotation(
            Placement(transformation(origin = {-40, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticY(n = {0, 1, 0}, animation = false) annotation(
            Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticZ(n = {0, 0, 1}, useAxisFlange = true, animation = true) annotation(
            Placement(transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Blocks.Math.Add add annotation(
            Placement(transformation(origin = {-50, 54}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Blocks.Sources.RealExpression realExpression(y = z_initial) annotation(
            Placement(transformation(origin = {-90, 48}, extent = {{-10, -10}, {10, 10}})));
          SI.Position z_initial = 0 annotation(
            Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
          //initial equation
          //  z_initial = frame_b.r_0[3];
        equation
          connect(realInput, add.u1) annotation(
            Line(points = {{-120, 60}, {-62, 60}}, color = {0, 0, 127}));
          connect(realExpression.y, add.u2) annotation(
            Line(points = {{-78, 48}, {-62, 48}}, color = {0, 0, 127}));
          connect(add.y, position.s_ref) annotation(
            Line(points = {{-38, 54}, {-2, 54}}, color = {0, 0, 127}));
          connect(position.flange, prismaticZ.axis) annotation(
            Line(points = {{20, 54}, {48, 54}, {48, 6}}, color = {0, 127, 0}));
          connect(prismaticX.frame_b, prismaticY.frame_a) annotation(
            Line(points = {{-30, 0}, {-10, 0}}, color = {95, 95, 95}));
          connect(prismaticY.frame_b, prismaticZ.frame_a) annotation(
            Line(points = {{10, 0}, {30, 0}}, color = {95, 95, 95}));
          connect(fixed.frame_b, prismaticX.frame_a) annotation(
            Line(points = {{-80, 0}, {-50, 0}}, color = {95, 95, 95}));
          connect(prismaticZ.frame_b, frame_b) annotation(
            Line(points = {{50, 0}, {100, 0}}, color = {95, 95, 95}));
        end ContactPatchSource;

        model RackSource
          import Modelica.Units.SI;
          parameter Real rack_axis[3] = {0, 1, 0} "Axis specifying direction of rack travel";
          parameter SI.Length joint_diameter = 0.825*0.0254 "Inboard pickup point diameter";
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
            Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {0, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
          Modelica.Mechanics.Translational.Sources.Position position annotation(
            Placement(transformation(origin = {-50, 60}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Interfaces.RealInput realInput annotation(
            Placement(transformation(origin = {-120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
            Placement(transformation(origin = {-60, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticRack(n = rack_axis, useAxisFlange = true) annotation(
            Placement(transformation(extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Joints.Spherical spherical annotation(
            Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
        equation
          connect(position.s_ref, realInput) annotation(
            Line(points = {{-62, 60}, {-120, 60}}, color = {0, 0, 127}));
          connect(position.flange, prismaticRack.axis) annotation(
            Line(points = {{-40, 60}, {8, 60}, {8, 6}}, color = {0, 127, 0}));
          connect(frame_a, prismaticRack.frame_a) annotation(
            Line(points = {{-60, -100}, {-60, 0}, {-10, 0}}));
          connect(prismaticRack.frame_b, spherical.frame_a) annotation(
            Line(points = {{10, 0}, {40, 0}}, color = {95, 95, 95}));
          connect(spherical.frame_b, frame_b) annotation(
            Line(points = {{60, 0}, {100, 0}}, color = {95, 95, 95}));
        end RackSource;

        model SteerSource
          import Modelica.Units.SI;
          parameter SI.Length joint_diameter = 0.825*0.0254 "Contact patch point diameter";
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
            Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.Translational.Sources.Position position(exact = false) annotation(
            Placement(transformation(origin = {-20, 54}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Interfaces.RealInput realInput annotation(
            Placement(transformation(origin = {-120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
            Placement(transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticX(animation = false) annotation(
            Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticY(n = {0, 1, 0}, animation = false, useAxisFlange = true) annotation(
            Placement(transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticZ(n = {0, 0, 1}, useAxisFlange = false) annotation(
            Placement(transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
          Modelica.Blocks.Math.Add add annotation(
            Placement(transformation(origin = {-50, 54}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Blocks.Sources.RealExpression realExpression(y = y_initial) annotation(
            Placement(transformation(origin = {-90, 48}, extent = {{-10, -10}, {10, 10}})));
          SI.Position y_initial = 0 annotation(
            Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
          //initial equation
          //  z_initial = frame_b.r_0[3];
          Modelica.Mechanics.MultiBody.Joints.Spherical spherical(sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}})));
        equation
          connect(realInput, add.u1) annotation(
            Line(points = {{-120, 60}, {-62, 60}}, color = {0, 0, 127}));
          connect(realExpression.y, add.u2) annotation(
            Line(points = {{-78, 48}, {-62, 48}}, color = {0, 0, 127}));
          connect(add.y, position.s_ref) annotation(
            Line(points = {{-38, 54}, {-32, 54}}, color = {0, 0, 127}));
          connect(prismaticX.frame_b, prismaticY.frame_a) annotation(
            Line(points = {{-40, 0}, {-20, 0}}, color = {95, 95, 95}));
          connect(prismaticY.frame_b, prismaticZ.frame_a) annotation(
            Line(points = {{0, 0}, {20, 0}}, color = {95, 95, 95}));
          connect(prismaticZ.frame_b, spherical.frame_a) annotation(
            Line(points = {{40, 0}, {60, 0}}, color = {95, 95, 95}));
          connect(spherical.frame_b, frame_b) annotation(
            Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(fixed.frame_b, prismaticX.frame_a) annotation(
            Line(points = {{-80, 0}, {-60, 0}}, color = {95, 95, 95}));
          connect(position.flange, prismaticY.axis) annotation(
            Line(points = {{-10, 54}, {-2, 54}, {-2, 6}}, color = {0, 127, 0}));
        end SteerSource;
      end Sources;

      package Linkages
        model CompliantLink
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Math.Vectors.normalize;
          import Modelica.Math.Vectors.norm;
          import Modelica.Units.SI;
          // Kinematic elements
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
          // User parameters
          parameter Boolean animation = true "Show link in animation";
          parameter SI.TranslationalSpringConstant axial_stiffness = 1e9 "Axial stiffness of link";
          parameter SI.Position r[3] = {1, 0, 0} "Vector from frame_a to frame_b, resolved in frame_a";
          parameter SI.Length diameter = 0.625*0.0254 "Diameter of link";
          parameter SI.Mass link_mass = 1e-3 "Mass of link";
          // Internal force and torque
          Modelica.Mechanics.MultiBody.Forces.Force force(animation = false) annotation(
            Placement(transformation(origin = {0, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Forces.Torque torque(animation = false) annotation(
            Placement(transformation(origin = {30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Sources.RealExpression forceExpression[3](y = -forceInternal) annotation(
            Placement(transformation(origin = {-6, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Blocks.Sources.RealExpression torqueExpression[3](y = -torqueInternal) annotation(
            Placement(transformation(origin = {24, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          // Body for states
          // Visualization
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_a(shapeType = "cylinder", length = norm(r), width = diameter, height = diameter, lengthDirection = normalize(r_rel), animation = animation) annotation(
            Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_b(shapeType = "cylinder", length = norm(r), width = diameter, height = diameter, lengthDirection = -normalize(r_rel), animation = false) annotation(
            Placement(transformation(origin = {70, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          // Force and torque variables
          SI.Force forceInternal[3];
          SI.Torque torqueInternal[3];
          SI.TranslationalSpringConstant c_local[3] = {axial_stiffness, 0, 0};
          // Kinematic variables
          SI.Length r_rel[3];
          SI.Length track_length;
        initial equation
          frame_b.r_0 = frame_a.r_0 + r;
        equation
          r_rel = frame_b.r_0 - frame_a.r_0;
          track_length = norm(r_rel);
          forceInternal = normalize(r_rel)*(norm(r_rel) - norm(r))*axial_stiffness;
          torqueInternal = {0, 0, 0};
          connect(forceExpression.y, force.force) annotation(
            Line(points = {{-6, -49}, {-6, -42}}, color = {0, 0, 127}, thickness = 0.5));
          connect(torqueExpression.y, torque.torque) annotation(
            Line(points = {{24, -39}, {24, -12}}, color = {0, 0, 127}, thickness = 0.5));
          connect(frame_a, force.frame_a) annotation(
            Line(points = {{-100, 0}, {-60, 0}, {-60, -30}, {-10, -30}}));
          connect(force.frame_b, frame_b) annotation(
            Line(points = {{10, -30}, {60, -30}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(frame_a, torque.frame_a) annotation(
            Line(points = {{-100, 0}, {20, 0}}));
          connect(torque.frame_b, frame_b) annotation(
            Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(shape_a.frame_a, frame_a) annotation(
            Line(points = {{-80, 20}, {-80, 19.5}, {-100, 19.5}, {-100, 0}}, color = {95, 95, 95}));
          connect(shape_b.frame_a, frame_b) annotation(
            Line(points = {{80, 20}, {80, 20.25}, {100, 20.25}, {100, 0}}, color = {95, 95, 95}));
          annotation(
            Icon(graphics = {Rectangle(extent = {{-99, 5}, {101, -5}}, fillPattern = FillPattern.Solid, fillColor = {255, 0, 255}), Text(extent = {{-150, 85}, {150, 45}}, textString = "%name", textColor = {0, 0, 255})}),
            Diagram(coordinateSystem(extent = {{-120, 40}, {120, -80}})));
        end CompliantLink;

        model CompliantWishbone
          import Modelica.Units.SI;
          parameter SI.Position fore_inboard_outboard[3] = {-0.125, 0.5, 0} "Vector from fore inboard node to outboard node, resolved in frame_a_fore";
          parameter SI.Position aft_inboard_outboard[3] = {0.125, 0.5, 0} "Vector from aft inboard node to outboard node, resolved in frame_a_aft";
          parameter SI.TranslationalSpringConstant fore_axial_stiffness = 1e9 "Axial stiffness of fore link";
          parameter SI.TranslationalSpringConstant aft_axial_stiffness = 1e9 "Axial stiffness of aft link";
          parameter SI.Length fore_diameter = 0.625*0.0254 "Diameter of fore link";
          parameter SI.Length aft_diameter = 0.625*0.0254 "Diameter of aft link";
          // Kinematics
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_outer annotation(
            Placement(transformation(origin = {-100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_fore annotation(
            Placement(transformation(origin = {100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 66}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_aft annotation(
            Placement(transformation(origin = {100, -80}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, -66}, extent = {{10, -10}, {-10, 10}})));
          // Links
          Vehicle.Chassis.Suspension.Linkages.CompliantLink fore_link(r = fore_inboard_outboard, axial_stiffness = fore_axial_stiffness, diameter = fore_diameter) annotation(
            Placement(transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Vehicle.Chassis.Suspension.Linkages.CompliantLink aft_link(r = aft_inboard_outboard, diameter = aft_diameter, axial_stiffness = aft_axial_stiffness) annotation(
            Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        equation
          connect(fore_link.frame_a, frame_a_fore) annotation(
            Line(points = {{10, 40}, {60, 40}, {60, 80}, {100, 80}}, color = {95, 95, 95}));
          connect(aft_link.frame_a, frame_a_aft) annotation(
            Line(points = {{10, -40}, {60, -40}, {60, -80}, {100, -80}}, color = {95, 95, 95}));
          connect(fore_link.frame_b, frame_b_outer) annotation(
            Line(points = {{-10, 40}, {-60, 40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
          connect(aft_link.frame_b, frame_b_outer) annotation(
            Line(points = {{-10, -40}, {-60, -40}, {-60, 0}, {-100, 0}}, color = {95, 95, 95}));
        end CompliantWishbone;

        model CompliantWheel
          import Modelica.Units.SI;
          parameter SI.Position r_lower_to_upper[3] = {0, 0, 1};
          parameter SI.Position r_lower_to_tie[3] = {0.25, 0, 0.5};
          parameter SI.Position r_lower_to_contact_patch[3] = {0, 0.125, -0.125};
          // Kinematics
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b_contact_patch annotation(
            Placement(transformation(origin = {0, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_upper annotation(
            Placement(transformation(origin = {100, 80}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 66}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_tie annotation(
            Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a_lower annotation(
            Placement(transformation(origin = {100, -80}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, -66}, extent = {{10, -10}, {-10, 10}})));
          // Set translation relations with compliant links
          Vehicle.Chassis.Suspension.Linkages.CompliantLink lower_to_upper(r = r_lower_to_upper, animation = true) annotation(
            Placement(transformation(origin = {30, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
          Vehicle.Chassis.Suspension.Linkages.CompliantLink lower_to_tie(r = r_lower_to_tie, animation = true) annotation(
            Placement(transformation(origin = {70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
          Vehicle.Chassis.Suspension.Linkages.CompliantLink lower_to_contact_patch(r = r_lower_to_contact_patch, animation = true) annotation(
            Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
          Vehicle.Chassis.Suspension.Linkages.CompliantLink contact_patch_to_tie(r = r_lower_to_tie - r_lower_to_contact_patch, animation = true) annotation(
            Placement(transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation contact_patch_to_upper(r = r_lower_to_upper - r_lower_to_contact_patch, animation = true) annotation(
            Placement(transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Vehicle.Chassis.Suspension.Linkages.CompliantLink upper_to_tie(r = r_lower_to_tie - r_lower_to_upper, animation = true) annotation(
            Placement(transformation(origin = {70, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        equation
          connect(frame_a_lower, lower_to_upper.frame_a) annotation(
            Line(points = {{100, -80}, {30, -80}, {30, -40}}));
          connect(lower_to_upper.frame_b, frame_a_upper) annotation(
            Line(points = {{30, -20}, {30, 80}, {100, 80}}, color = {95, 95, 95}));
          connect(frame_a_lower, lower_to_tie.frame_a) annotation(
            Line(points = {{100, -80}, {70, -80}, {70, -40}}));
          connect(lower_to_tie.frame_b, frame_a_tie) annotation(
            Line(points = {{70, -20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(frame_a_upper, upper_to_tie.frame_a) annotation(
            Line(points = {{100, 80}, {70, 80}, {70, 40}}));
          connect(upper_to_tie.frame_b, frame_a_tie) annotation(
            Line(points = {{70, 20}, {70, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(lower_to_contact_patch.frame_b, frame_b_contact_patch) annotation(
            Line(points = {{20, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
          connect(lower_to_contact_patch.frame_a, frame_a_lower) annotation(
            Line(points = {{40, -90}, {70, -90}, {70, -80}, {100, -80}}, color = {95, 95, 95}));
          connect(frame_b_contact_patch, contact_patch_to_upper.frame_a) annotation(
            Line(points = {{0, -100}, {0, 40}}));
          connect(contact_patch_to_upper.frame_b, frame_a_upper) annotation(
            Line(points = {{0, 60}, {0, 80}, {100, 80}}, color = {95, 95, 95}));
          connect(frame_b_contact_patch, contact_patch_to_tie.frame_a) annotation(
            Line(points = {{0, -100}, {0, -40}, {10, -40}, {10, -30}}));
          connect(contact_patch_to_tie.frame_b, frame_a_tie) annotation(
            Line(points = {{10, -10}, {10, 0}, {100, 0}}, color = {95, 95, 95}));
        end CompliantWheel;

        model SphericalBearing
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Math.Vectors.normalize;
          import Modelica.Math.Vectors.norm;
          import Modelica.Units.SI;
          // Kinematic elements
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
          // User parameters
          parameter Boolean animation = true "Show link in animation";
          parameter SI.TranslationalSpringConstant radial_stiffness = 1e9 "Radial stiffness";
          parameter SI.Length diameter = 0.825*0.0254 "Diameter of bearing";
          parameter SI.Mass mass = 1 "Mass of bearing";
          // Internal force and torque
          Modelica.Mechanics.MultiBody.Forces.Force force(animation = false) annotation(
            Placement(transformation(origin = {0, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Forces.Torque torque(animation = false) annotation(
            Placement(transformation(origin = {30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Sources.RealExpression forceExpression[3](y = -forceInternal) annotation(
            Placement(transformation(origin = {-6, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Blocks.Sources.RealExpression torqueExpression[3](y = -torqueInternal) annotation(
            Placement(transformation(origin = {24, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          // Visualization
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_a(shapeType = "sphere", length = diameter, width = diameter, height = diameter, animation = animation, r_shape = {-diameter/2, 0, 0}) annotation(
            Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
          // Force and torque variables
          SI.Force forceInternal[3];
          SI.Torque torqueInternal[3];
          // Kinematic variables
          SI.Length r_rel[3];
        equation
          r_rel = frame_b.r_0 - frame_a.r_0;
          forceInternal = normalize(r_rel)*radial_stiffness;
          torqueInternal = {0, 0, 0};
          connect(forceExpression.y, force.force) annotation(
            Line(points = {{-6, -49}, {-6, -42}}, color = {0, 0, 127}, thickness = 0.5));
          connect(torqueExpression.y, torque.torque) annotation(
            Line(points = {{24, -39}, {24, -12}}, color = {0, 0, 127}, thickness = 0.5));
          connect(frame_a, force.frame_a) annotation(
            Line(points = {{-100, 0}, {-60, 0}, {-60, -30}, {-10, -30}}));
          connect(force.frame_b, frame_b) annotation(
            Line(points = {{10, -30}, {60, -30}, {60, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(frame_a, torque.frame_a) annotation(
            Line(points = {{-100, 0}, {20, 0}}));
          connect(torque.frame_b, frame_b) annotation(
            Line(points = {{40, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(shape_a.frame_a, frame_a) annotation(
            Line(points = {{-80, 20}, {-80, 19.5}, {-100, 19.5}, {-100, 0}}, color = {95, 95, 95}));
          annotation(
            Icon(graphics = {Rectangle(extent = {{-99, 5}, {101, -5}}, fillPattern = FillPattern.Solid, fillColor = {255, 0, 255}), Text(extent = {{-150, 85}, {150, 45}}, textString = "%name", textColor = {0, 0, 255})}),
            Diagram(coordinateSystem(extent = {{-120, 40}, {120, -80}})));
        end SphericalBearing;

        model PrismaticBearing
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Math.Vectors.normalize;
          import Modelica.Math.Vectors.norm;
          import Modelica.Units.SI;
          // Kinematic elements
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
          // User parameters
          parameter Boolean animation = true "Show link in animation";
          parameter SI.TranslationalSpringConstant radial_stiffness = 1e9 "Radial stiffness";
          parameter SI.Length diameter = 0.825*0.0254 "Diameter of bearing";
          parameter SI.Mass mass = 1 "Mass of bearing";
          // Internal force and torque
          Modelica.Mechanics.MultiBody.Forces.Force force(animation = false) annotation(
            Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Sources.RealExpression forceExpression[3](y = -forceInternal) annotation(
            Placement(transformation(origin = {-6, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          // Visualization
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape shape_a(shapeType = "sphere", length = diameter, width = diameter, height = diameter, animation = animation, r_shape = {-diameter/2, 0, 0}) annotation(
            Placement(transformation(origin = {-70, 20}, extent = {{-10, -10}, {10, 10}})));
          // Force and torque variables
          SI.Force forceInternal[3];
          SI.Torque torqueInternal[3];
          // Kinematic variables
          SI.Length r_rel[3];
        equation
          r_rel = frame_b.r_0 - frame_a.r_0;
          forceInternal = normalize(r_rel)*radial_stiffness;
          torqueInternal = {0, 0, 0};
          connect(forceExpression.y, force.force) annotation(
            Line(points = {{-6, -19}, {-6, -12}}, color = {0, 0, 127}, thickness = 0.5));
          connect(frame_a, force.frame_a) annotation(
            Line(points = {{-100, 0}, {-10, 0}}));
          connect(force.frame_b, frame_b) annotation(
            Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(shape_a.frame_a, frame_a) annotation(
            Line(points = {{-80, 20}, {-80, 19.5}, {-100, 19.5}, {-100, 0}}, color = {95, 95, 95}));
          annotation(
            Icon(graphics = {Rectangle(extent = {{-99, 5}, {101, -5}}, fillPattern = FillPattern.Solid, fillColor = {255, 0, 255}), Text(extent = {{-150, 85}, {150, 45}}, textString = "%name", textColor = {0, 0, 255})}),
            Diagram(coordinateSystem(extent = {{-120, 40}, {120, -80}})));
        end PrismaticBearing;

        model SphericalSpherical
          extends Modelica.Mechanics.MultiBody.Joints.SphericalSpherical;
          import Modelica.Units.SI;
          import Modelica.Math.Vectors.normalize;
          import Modelica.Mechanics.MultiBody.Visualizers;
          import Modelica.Mechanics.MultiBody.Types;
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Mechanics.MultiBody.Frames.resolveRelative;
          extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
          parameter Real alpha = 0.5 "Fraction along the link from frame_a to frame_b";
          parameter Real upper_o[3] = {0, 1, 1} "Lorem Ipsum";
          parameter Real lower_o[3] = {0, 1, 0} "Lorem Ipsum";
        
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_C annotation(
            Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
        
        protected
          SI.Position r_rel[3];
          // vector from frame_a to frame_b
          SI.Position r_C[3];
          // position of new frame_C
          Modelica.Mechanics.MultiBody.Frames.Orientation R_C;
          Real n[3];
          Real nx_guess[3] = {0, 0, 1};
        
        equation
        // Relative vector between joints
          r_rel = frame_b.r_0 - frame_a.r_0;
          n = normalize(r_rel);
        // Position of new frame along the connecting rod
          r_C = frame_a.r_0 + alpha*r_rel;
          R_C = Frames.from_nxy(n, nx_guess);
        // Assign to frame_C
          frame_C.r_0 = r_C;
          frame_C.R = R_C;
        end SphericalSpherical;
      end Linkages;

      package Wheels
        model Wheel
          import Modelica.Units.SI;
          parameter SI.Length tire_diameter = 16*0.0254 "Tire diameter";
          parameter SI.Length rim_diameter = 10*0.0254 "Rim diameter";
          parameter SI.Length rim_width = 7*0.0254 "Rim width";
          parameter SI.Angle static_gamma "Static inclination angle";
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
            Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(shapeType = "cylinder", r_shape = {0, -rim_width/2, 0}, length = rim_width, width = tire_diameter, height = tire_diameter, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}) annotation(
            Placement(transformation(origin = {30, -10}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation(r = {0, 0, tire_diameter/2}, n = {1, 0, 0}, angle = static_gamma) annotation(
            Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          connect(frame_b, fixedRotation.frame_a) annotation(
            Line(points = {{0, -100}, {0, -80}}));
          connect(fixedShape.frame_a, fixedRotation.frame_b) annotation(
            Line(points = {{20, -10}, {0, -10}, {0, -60}}, color = {95, 95, 95}));
        end Wheel;
      end Wheels;

      package KinFMU
        model KinCorner
          import Modelica.Units.SI;
          parameter SI.Position upper_fore_i[3] = {0.5, 0, 1} "Position of upper-fore-inboard pickup";
          parameter SI.Position upper_aft_i[3] = {0, 0, 1} "Position of upper-aft-inboard pickup";
          parameter SI.Position upper_o[3] = {0, 1, 1} "Position of upper-outboard pickup";
          parameter SI.Position tie_i[3] = {0.5, 0, 0.5} "Position of tie-inboard pickup";
          parameter SI.Position tie_o[3] = {0.5, 1, 0.5} "Position of tie-outboard pickup";
          parameter SI.Position lower_fore_i[3] = {0.5, 0, 0} "Position of lower-fore-inboard pickup";
          parameter SI.Position lower_aft_i[3] = {0, 0, 0} "Position of lower-aft-inboard pickup";
          parameter SI.Position lower_o[3] = {0, 1, 0} "Position of lower-fore-inboard pickup";
          parameter SI.Mass small_mass = 1e-3 "Mass of bodies inserted for state selection";
          parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links";
          parameter SI.Length spherical_diameter = 0.825*0.0254 "Diameter of spherical bearings";
          Modelica.Mechanics.MultiBody.Parts.Fixed upper_fore_i_pos(r = upper_fore_i, animation = false) annotation(
            Placement(transformation(origin = {110, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed upper_aft_i_pos(r = upper_aft_i, animation = false) annotation(
            Placement(transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed tie_i_pos(r = tie_i, animation = false) annotation(
            Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed lower_fore_i_pos(r = lower_fore_i, animation = true) annotation(
            Placement(transformation(origin = {130, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
            Placement(transformation(origin = {-110, -110}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = upper_o - upper_fore_i, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {70, 110}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = upper_o - upper_aft_i, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {70, 70}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = tie_o - tie_i, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {70, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = lower_o - lower_fore_i, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {70, -70}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation4(r = lower_o - lower_aft_i, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {70, -110}, extent = {{10, -10}, {-10, 10}})));
          Linkages.SphericalBearing sphericalBearing annotation(
            Placement(transformation(origin = {40, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Vehicle.Chassis.Suspension.Linkages.SphericalBearing sphericalBearing1 annotation(
            Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1) annotation(
            Placement(transformation(origin = {-10, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1, r_CM = {0, 0, 0}) annotation(
            Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Vehicle.Chassis.Suspension.Linkages.SphericalBearing sphericalBearing11 annotation(
            Placement(transformation(origin = {100, -70}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.Fixed lower_aft_i_pos(r = lower_aft_i, animation = false) annotation(
            Placement(transformation(origin = {110, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Body body2(m = 10, r_CM = {0, 0, 0}) annotation(
            Placement(transformation(origin = {90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          connect(upper_fore_i_pos.frame_b, fixedTranslation.frame_a) annotation(
            Line(points = {{100, 110}, {80, 110}}, color = {95, 95, 95}));
          connect(upper_aft_i_pos.frame_b, fixedTranslation1.frame_a) annotation(
            Line(points = {{100, 70}, {80, 70}}, color = {95, 95, 95}));
          connect(fixedTranslation2.frame_a, tie_i_pos.frame_b) annotation(
            Line(points = {{80, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(fixedTranslation4.frame_a, lower_aft_i_pos.frame_b) annotation(
            Line(points = {{80, -110}, {100, -110}}, color = {95, 95, 95}));
          connect(fixedTranslation1.frame_b, sphericalBearing.frame_a) annotation(
            Line(points = {{60, 70}, {40, 70}, {40, 80}}, color = {95, 95, 95}));
          connect(sphericalBearing.frame_b, fixedTranslation.frame_b) annotation(
            Line(points = {{40, 100}, {40, 110}, {60, 110}}, color = {95, 95, 95}));
          connect(fixedTranslation4.frame_b, sphericalBearing1.frame_a) annotation(
            Line(points = {{60, -110}, {30, -110}, {30, -100}}, color = {95, 95, 95}));
          connect(sphericalBearing1.frame_b, fixedTranslation3.frame_b) annotation(
            Line(points = {{30, -80}, {30, -70}, {60, -70}}, color = {95, 95, 95}));
          connect(body.frame_a, sphericalBearing1.frame_a) annotation(
            Line(points = {{0, -110}, {30, -110}, {30, -100}}, color = {95, 95, 95}));
          connect(body1.frame_a, sphericalBearing.frame_a) annotation(
            Line(points = {{0, 70}, {40, 70}, {40, 80}}, color = {95, 95, 95}));
          connect(fixedTranslation3.frame_a, sphericalBearing11.frame_a) annotation(
            Line(points = {{80, -70}, {90, -70}}, color = {95, 95, 95}));
          connect(sphericalBearing11.frame_b, lower_fore_i_pos.frame_b) annotation(
            Line(points = {{110, -70}, {120, -70}}, color = {95, 95, 95}));
          connect(body2.frame_a, sphericalBearing11.frame_a) annotation(
            Line(points = {{90, -40}, {90, -70}}, color = {95, 95, 95}));
        end KinCorner;

        model KinCornerV2
          import Modelica.Math.Vectors.normalize;
          import Modelica.Math.Vectors.norm;
          import Modelica.Units.SI;
          parameter SI.Length wheel_diameter = 16*0.0254 "Nominal diameter of tire";
          parameter SI.Position contact_patch[3] = {0, 0.609600, 0} "Position of contact patch";
          parameter SI.Position upper_fore_i[3] = {0.086868, 0.215900, 0.200000} "Position of upper-fore-inboard pickup";
          parameter SI.Position upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} "Position of upper-aft-inboard pickup";
          parameter SI.Position upper_o[3] = {-0.006347, 0.523240, 0.287020} "Position of upper-outboard pickup";
          parameter SI.Position tie_i[3] = {0.041128, 0.215900, 0.117856} "Position of tie-inboard pickup";
          parameter SI.Position tie_o[3] = {0.056000, 0.532333, 0.164821} "Position of tie-outboard pickup";
          parameter SI.Position lower_fore_i[3] = {0.087376, 0.215900, 0.090000} "Position of lower-fore-inboard pickup";
          parameter SI.Position lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} "Position of lower-aft-inboard pickup";
          parameter SI.Position lower_o[3] = {0, 0.556499, 0.124998} "Position of lower-fore-inboard pickup";
          parameter SI.Mass small_mass = 1e-3 "Mass of bodies inserted for state selection";
          parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links";
          parameter SI.Length joint_diameter = 0.825*0.0254 "Diameter of joints";
          final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
          final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
          final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
            Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
          final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
            Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
          inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
            Placement(transformation(origin = {-110, -90}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower(r = r_lower_mount, animation = false) annotation(
            Placement(transformation(origin = {130, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Joints.Revolute revoluteUpper(n = normalize(upper_fore_i - upper_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {90, 90}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Joints.Revolute revoluteLower(n = normalize(lower_fore_i - lower_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {94, -30}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationUpper(r = (upper_o - upper_fore_i) + r_upper_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {50, 90}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationLower(r = (lower_o - lower_fore_i) + r_lower_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {50, -30}, extent = {{10, -10}, {-10, 10}})));
          Sources.RackSource rackSource annotation(
            Placement(transformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedRack(animation = false, r = tie_i) annotation(
            Placement(transformation(origin = {90, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Blocks.Sources.Sine sine1(amplitude = 0, f = 1) annotation(
            Placement(transformation(origin = {130, 50}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationUpper1(height = link_diameter, r = tie_o - tie_i, width = link_diameter) annotation(
            Placement(transformation(origin = {60, 50}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedUpper(r = r_upper_mount, animation = false) annotation(
            Placement(transformation(origin = {130, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = lower_o - contact_patch) annotation(
            Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Joints.Spherical spherical11(sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {10, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
          Sources.ContactPatchSource contactPatchSource annotation(
            Placement(transformation(origin = {-50, -90}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Blocks.Sources.Sine sine11(amplitude = 0, f = 1) annotation(
            Placement(transformation(origin = {-90, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation11(r = tie_o - lower_o) annotation(
            Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation111(r = upper_o - lower_o) annotation(
            Placement(transformation(origin = {-30, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.Body body(r_CM = {0, 0, 0}, m = 1, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-30, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1, r_CM = {0, 0, 0}, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Body body2(m = 1, r_CM = {0, 0, 0}, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Body body3(m = 1, r_CM = {0, 0, 0}, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {10, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Joints.SphericalSpherical sphericalSpherical(rodLength = norm(upper_o - lower_o)) annotation(
            Placement(transformation(origin = {40, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          connect(fixedUpper.frame_b, revoluteUpper.frame_a) annotation(
            Line(points = {{120, 90}, {100, 90}}, color = {95, 95, 95}));
          connect(fixedLower.frame_b, revoluteLower.frame_a) annotation(
            Line(points = {{120, -30}, {104, -30}}, color = {95, 95, 95}));
          connect(fixedTranslationUpper.frame_a, revoluteUpper.frame_b) annotation(
            Line(points = {{60, 90}, {80, 90}}, color = {95, 95, 95}));
          connect(fixedTranslationLower.frame_a, revoluteLower.frame_b) annotation(
            Line(points = {{60, -30}, {84, -30}}, color = {95, 95, 95}));
          connect(fixedRack.frame_b, rackSource.frame_b) annotation(
            Line(points = {{90, 30}, {90, 40}}, color = {95, 95, 95}));
          connect(sine1.y, rackSource.realInput) annotation(
            Line(points = {{119, 50}, {100, 50}}, color = {0, 0, 127}));
          connect(rackSource.frame_a, fixedTranslationUpper1.frame_a) annotation(
            Line(points = {{80, 50}, {70, 50}}, color = {95, 95, 95}));
          connect(spherical11.frame_a, fixedTranslationLower.frame_b) annotation(
            Line(points = {{20, -30}, {40, -30}}, color = {95, 95, 95}));
          connect(spherical11.frame_b, fixedTranslation1.frame_b) annotation(
            Line(points = {{0, -30}, {0, -60}}, color = {95, 95, 95}));
          connect(fixedTranslation11.frame_a, spherical11.frame_b) annotation(
            Line(points = {{0, 0}, {0, -30}}, color = {95, 95, 95}));
          connect(sine11.y, contactPatchSource.realInput) annotation(
            Line(points = {{-78, -50}, {-70, -50}, {-70, -90}, {-60, -90}}, color = {0, 0, 127}));
          connect(contactPatchSource.frame_b, fixedTranslation1.frame_a) annotation(
            Line(points = {{-40, -90}, {0, -90}, {0, -80}}, color = {95, 95, 95}));
          connect(spherical11.frame_b, fixedTranslation111.frame_a) annotation(
            Line(points = {{0, -30}, {-30, -30}, {-30, 0}}, color = {95, 95, 95}));
          connect(body.frame_a, spherical11.frame_b) annotation(
            Line(points = {{-20, -50}, {0, -50}, {0, -30}}, color = {95, 95, 95}));
          connect(body1.frame_a, fixedTranslationUpper.frame_b) annotation(
            Line(points = {{20, 90}, {40, 90}}, color = {95, 95, 95}));
          connect(body2.frame_a, fixedTranslation111.frame_b) annotation(
            Line(points = {{-60, 30}, {-30, 30}, {-30, 20}}, color = {95, 95, 95}));
          connect(body3.frame_a, fixedTranslationUpper1.frame_b) annotation(
            Line(points = {{20, 50}, {50, 50}}, color = {95, 95, 95}));
          connect(fixedTranslationLower.frame_b, sphericalSpherical.frame_a) annotation(
            Line(points = {{40, -30}, {40, 0}}, color = {95, 95, 95}));
          connect(sphericalSpherical.frame_b, fixedTranslationUpper.frame_b) annotation(
            Line(points = {{40, 20}, {40, 90}}, color = {95, 95, 95}));
          annotation(
            Diagram(coordinateSystem(extent = {{-120, 100}, {140, -100}})));
        end KinCornerV2;

        model KinCornerV3
          import Modelica.Math.Vectors.normalize;
          import Modelica.Math.Vectors.norm;
          import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Units.SI;
          parameter SI.Length wheel_diameter = 16*0.0254 "Nominal diameter of tire";
          parameter SI.Position contact_patch[3] = {0, 0.609600, 0} "Position of contact patch";
          parameter SI.Position upper_fore_i[3] = {0.086868, 0.215900, 0.200000} "Position of upper-fore-inboard pickup";
          parameter SI.Position upper_aft_i[3] = {-0.095250, 0.215900, 0.200000} "Position of upper-aft-inboard pickup";
          parameter SI.Position upper_o[3] = {-0.006347, 0.523240, 0.287020} "Position of upper-outboard pickup";
          parameter SI.Position tie_i[3] = {0.041128, 0.215900, 0.117856} "Position of tie-inboard pickup";
          parameter SI.Position tie_o[3] = {0.056000, 0.532333, 0.164821} "Position of tie-outboard pickup";
          parameter SI.Position lower_fore_i[3] = {0.087376, 0.215900, 0.090000} "Position of lower-fore-inboard pickup";
          parameter SI.Position lower_aft_i[3] = {-0.095250, 0.215900, 0.090000} "Position of lower-aft-inboard pickup";
          parameter SI.Position lower_o[3] = {0, 0.556499, 0.124998} "Position of lower-fore-inboard pickup";
          parameter SI.Mass small_mass = 1e-3 "Mass of bodies inserted for state selection";
          parameter SI.Length constraint_diameter = 0.25*0.0254 "Diameter of constraints (internal links)";
          parameter SI.Length link_diameter = 0.625*0.0254 "Diameter of links";
          parameter SI.Length joint_diameter = 0.825*0.0254 "Diameter of joints";
          final parameter SI.Position r_upper_mount[3] = (upper_fore_i + upper_aft_i)/2;
          final parameter SI.Position r_lower_mount[3] = (lower_fore_i + lower_aft_i)/2;
          final parameter SI.Position r_upper_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
            Placement(visible = false, transformation(extent = {{0, 0}, {0, 0}})));
          final parameter SI.Position r_lower_mount_to_fore[3] = (upper_fore_i - upper_aft_i)/2 annotation(
            Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
          inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
            Placement(transformation(origin = {-1730, 30}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
          Modelica.Mechanics.MultiBody.Joints.Revolute revoluteUpper(n = normalize(upper_fore_i - upper_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-1490, 230}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Joints.Revolute revoluteLower(n = normalize(lower_fore_i - lower_aft_i), cylinderLength = joint_diameter, cylinderDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-1490, 50}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationUpper(r = (upper_o - upper_fore_i) + r_upper_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {-1530, 230}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationLower(r = (lower_o - lower_fore_i) + r_lower_mount_to_fore, width = link_diameter, height = link_diameter) annotation(
            Placement(transformation(origin = {-1530, 50}, extent = {{10, -10}, {-10, 10}})));
          Sources.ContactPatchSource contactPatchSource annotation(
            Placement(transformation(origin = {-1670, 110}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Blocks.Sources.Sine sine(amplitude = 3*0.0254, f = 1) annotation(
            Placement(transformation(origin = {-1710, 110}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower1(animation = false, r = tie_i) annotation(
            Placement(transformation(origin = {-1670, 170}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Sources.RackSource rackSource annotation(
            Placement(transformation(origin = {-1670, 210}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
          Modelica.Blocks.Sources.Sine sine1(amplitude = 1.5*0.0254, f = 1) annotation(
            Placement(transformation(origin = {-1710, 210}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1121(r = contact_patch - tie_o, width = constraint_diameter, height = constraint_diameter, animation = false) annotation(
            Placement(transformation(origin = {-1610, 140}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Mechanics.MultiBody.Joints.Spherical spherical(sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-1630, 110}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(shapeType = "sphere", length = joint_diameter, width = joint_diameter, height = joint_diameter, color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor, r_shape = {-joint_diameter/2, 0, 0}) annotation(
            Placement(transformation(origin = {-1550, 30}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape1(color = Modelica.Mechanics.MultiBody.Types.Defaults.JointColor, height = joint_diameter, length = joint_diameter, r_shape = {-joint_diameter/2, 0, 0}, shapeType = "sphere", width = joint_diameter) annotation(
            Placement(transformation(origin = {-1550, 250}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
          Real track_length;
          Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0}, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-1630, 250}, extent = {{-10, -10}, {10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationUpper1(height = link_diameter, r = tie_o - tie_i, width = link_diameter) annotation(
            Placement(transformation(origin = {-1630, 210}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation11211(animation = true, height = constraint_diameter, r = contact_patch - tie_o, width = constraint_diameter) annotation(
            Placement(transformation(origin = {-1630, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation112111(animation = true, height = constraint_diameter, r = upper_o - lower_o, width = constraint_diameter) annotation(
            Placement(transformation(origin = {-1630, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1121111(animation = true, height = constraint_diameter, r = tie_o - lower_o, width = constraint_diameter) annotation(
            Placement(transformation(origin = {-1610, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedLower(animation = false, r = r_lower_mount) annotation(
            Placement(transformation(origin = {-1450, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixedUpper(animation = false, r = r_upper_mount + {0, 0, 1e-6}) annotation(
            Placement(transformation(origin = {-1450, 230}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Linkages.SphericalSpherical sphericalSpherical(rodLength = norm(upper_o - lower_o)) annotation(
            Placement(transformation(origin = {-1530, 130}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Mechanics.MultiBody.Parts.Body body1(m = 1, r_CM = {0, 0, 0}, sphereDiameter = joint_diameter) annotation(
            Placement(transformation(origin = {-1550, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation11211111(animation = true, height = constraint_diameter, r = {1, 0, 0}, width = constraint_diameter) annotation(
            Placement(transformation(origin = {-1590, 190}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        
        Frames.Orientation local_frame = Modelica.Mechanics.MultiBody.Frames.from_nxy(tie_o - lower_o, upper_o - lower_o) annotation(
            Placement(transformation(origin = {-1554, 162}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        
        equation
          track_length = norm(fixedTranslation1121.frame_b.r_0 - fixedTranslationUpper.frame_b.r_0)/0.0254;
          connect(fixedTranslationUpper.frame_a, revoluteUpper.frame_b) annotation(
            Line(points = {{-1520, 230}, {-1500, 230}}, color = {95, 95, 95}));
          connect(fixedTranslationLower.frame_a, revoluteLower.frame_b) annotation(
            Line(points = {{-1520, 50}, {-1500, 50}}, color = {95, 95, 95}));
          connect(sine.y, contactPatchSource.realInput) annotation(
            Line(points = {{-1699, 110}, {-1680, 110}}, color = {0, 0, 127}));
          connect(fixedLower1.frame_b, rackSource.frame_b) annotation(
            Line(points = {{-1670, 180}, {-1670, 200}}, color = {95, 95, 95}));
          connect(sine1.y, rackSource.realInput) annotation(
            Line(points = {{-1699, 210}, {-1680, 210}}, color = {0, 0, 127}));
          connect(contactPatchSource.frame_b, spherical.frame_a) annotation(
            Line(points = {{-1660, 110}, {-1640, 110}}, color = {95, 95, 95}));
          connect(spherical.frame_b, fixedTranslation1121.frame_b) annotation(
            Line(points = {{-1620, 110}, {-1610, 110}, {-1610, 130}}, color = {95, 95, 95}));
          connect(fixedShape.frame_a, fixedTranslationLower.frame_b) annotation(
            Line(points = {{-1550, 40}, {-1550, 50}, {-1540, 50}}, color = {95, 95, 95}));
          connect(fixedShape1.frame_a, fixedTranslationUpper.frame_b) annotation(
            Line(points = {{-1550, 240}, {-1550, 230}, {-1540, 230}}, color = {95, 95, 95}));
          connect(rackSource.frame_a, fixedTranslationUpper1.frame_b) annotation(
            Line(points = {{-1660, 210}, {-1640, 210}}, color = {95, 95, 95}));
          connect(fixedTranslationUpper1.frame_a, fixedTranslation1121.frame_a) annotation(
            Line(points = {{-1620, 210}, {-1610, 210}, {-1610, 150}}, color = {95, 95, 95}));
          connect(body.frame_a, rackSource.frame_a) annotation(
            Line(points = {{-1640, 250}, {-1660, 250}, {-1660, 210}}, color = {95, 95, 95}));
          connect(fixedTranslation1121111.frame_a, fixedTranslation112111.frame_a) annotation(
            Line(points = {{-1610, 60}, {-1610, 40}, {-1630, 40}, {-1630, 60}}, color = {95, 95, 95}));
          connect(fixedLower.frame_b, revoluteLower.frame_a) annotation(
            Line(points = {{-1460, 50}, {-1480, 50}}, color = {95, 95, 95}));
          connect(fixedUpper.frame_b, revoluteUpper.frame_a) annotation(
            Line(points = {{-1460, 230}, {-1480, 230}}, color = {95, 95, 95}));
          connect(fixedTranslation112111.frame_a, fixedTranslation11211.frame_a) annotation(
            Line(points = {{-1630, 60}, {-1630, 20}}, color = {95, 95, 95}));
          connect(sphericalSpherical.frame_a, fixedTranslationUpper.frame_b) annotation(
            Line(points = {{-1530, 140}, {-1530, 200}, {-1540, 200}, {-1540, 230}}, color = {95, 95, 95}));
          connect(fixedTranslationLower.frame_b, sphericalSpherical.frame_b) annotation(
            Line(points = {{-1540, 50}, {-1550, 50}, {-1550, 68}, {-1530, 68}, {-1530, 120}}, color = {95, 95, 95}));
          connect(body1.frame_a, fixedTranslationLower.frame_b) annotation(
            Line(points = {{-1540, 10}, {-1540, 50}}, color = {95, 95, 95}));
  connect(fixedTranslation11211111.frame_a, sphericalSpherical.frame_C) annotation(
            Line(points = {{-1590, 180}, {-1590, 130}, {-1540, 130}}, color = {95, 95, 95}));
  connect(sphericalSpherical.frame_C, fixedTranslation11211.frame_a) annotation(
            Line(points = {{-1540, 130}, {-1590, 130}, {-1590, 40}, {-1630, 40}, {-1630, 20}}, color = {95, 95, 95}));
          annotation(
            Diagram(coordinateSystem(extent = {{-1780, 260}, {-1380, 0}})));
        end KinCornerV3;

        function invRotation
    import Modelica.Mechanics.MultiBody.Frames;
          import Modelica.Math.Matrices;
          input Frames.Orientation R "Orientation to be unapplied";
          input Real v_rotated[3] "Vector in rotated frame";
          output Real v_original[3] "Vector in original frame";
        protected
          Real R_matrix[3, 3];
        algorithm
          R_matrix := Frames.to_T(R);
          v_original := transpose(R_matrix)*v_rotated;
        end invRotation;
      end KinFMU;

      model DoubleWishbone
        inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
          Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed1(r = {0.5, 0, 1}, animation = false) annotation(
          Placement(transformation(origin = {90, 90}, extent = {{10, -10}, {-10, 10}})));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed2(r = {0, 0, 1}, animation = false) annotation(
          Placement(transformation(origin = {90, 50}, extent = {{10, -10}, {-10, 10}})));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed3(r = {0.5, 0, 0.5}, animation = false) annotation(
          Placement(transformation(origin = {90, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed4(r = {0.5, 0, 0}, animation = false) annotation(
          Placement(transformation(origin = {90, -50}, extent = {{10, -10}, {-10, 10}})));
        Modelica.Mechanics.MultiBody.Parts.Fixed fixed5(r = {0, 0, 0}, animation = false) annotation(
          Placement(transformation(origin = {90, -90}, extent = {{10, -10}, {-10, 10}})));
        Vehicle.Chassis.Suspension.Linkages.CompliantWishbone upperWishbone(fore_inboard_outboard = {-0.5, 1, 0}, aft_inboard_outboard = {0, 1, 0}) annotation(
          Placement(transformation(origin = {30, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
        Vehicle.Chassis.Suspension.Linkages.CompliantWishbone lowerWishbone(fore_inboard_outboard = {-0.5, 1, 0}, aft_inboard_outboard = {0, 1, 0}) annotation(
          Placement(transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}})));
        Vehicle.Chassis.Suspension.Linkages.CompliantLink tieRod(r = {-0.25, 1, 0}) annotation(
          Placement(transformation(origin = {50, 0}, extent = {{10, -10}, {-10, 10}})));
        Vehicle.Chassis.Suspension.Linkages.CompliantWheel wheel annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        Modelica.Mechanics.MultiBody.Parts.Body body1(r_CM = {0, 0, 0}, m = 1) annotation(
          Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Parts.Body body2(r_CM = {0, 0, 0}, m = 1) annotation(
          Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}})));
        Modelica.Mechanics.MultiBody.Parts.Body body3(r_CM = {0, 0, 0}, m = 1) annotation(
          Placement(transformation(origin = {20, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Vehicle.Chassis.Suspension.Sources.ContactPatchSource contactPatchSource annotation(
          Placement(transformation(origin = {-30, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
        Modelica.Blocks.Sources.Sine constantSource(amplitude = 0.25, f = 1, offset = 0) annotation(
          Placement(transformation(origin = {-70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
        Vehicle.Chassis.Suspension.Sources.RackSource rackSource annotation(
          Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));
        Modelica.Blocks.Sources.Sine constantSource2(amplitude = 0.125, f = 1, offset = 0) annotation(
          Placement(transformation(origin = {130, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Parts.Body body21(m = 1, r_CM = {0, 0, 0}) annotation(
          Placement(transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        connect(fixed1.frame_b, upperWishbone.frame_a_fore) annotation(
          Line(points = {{80, 90}, {60, 90}, {60, 76}, {40, 76}}, color = {95, 95, 95}));
        connect(fixed2.frame_b, upperWishbone.frame_a_aft) annotation(
          Line(points = {{80, 50}, {60, 50}, {60, 64}, {40, 64}}, color = {95, 95, 95}));
        connect(fixed4.frame_b, lowerWishbone.frame_a_fore) annotation(
          Line(points = {{80, -50}, {60, -50}, {60, -63}}, color = {95, 95, 95}));
        connect(fixed5.frame_b, lowerWishbone.frame_a_aft) annotation(
          Line(points = {{80, -90}, {60, -90}, {60, -77}}, color = {95, 95, 95}));
        connect(upperWishbone.frame_b_outer, wheel.frame_a_upper) annotation(
          Line(points = {{20, 70}, {10, 70}, {10, 6}}, color = {95, 95, 95}));
        connect(lowerWishbone.frame_b_outer, wheel.frame_a_lower) annotation(
          Line(points = {{20, -70}, {10, -70}, {10, -6}}, color = {95, 95, 95}));
        connect(lowerWishbone.frame_b_outer, wheel.frame_a_lower) annotation(
          Line(points = {{40, -70}, {12, -70}, {12, -6}, {10, -6}}, color = {95, 95, 95}));
        connect(body1.frame_a, upperWishbone.frame_b_outer) annotation(
          Line(points = {{0, 70}, {20, 70}}, color = {95, 95, 95}));
        connect(body2.frame_a, tieRod.frame_b) annotation(
          Line(points = {{40, -30}, {30, -30}, {30, 0}, {40, 0}}, color = {95, 95, 95}));
        connect(tieRod.frame_b, wheel.frame_a_tie) annotation(
          Line(points = {{40, 0}, {10, 0}}, color = {95, 95, 95}));
        connect(constantSource.y, contactPatchSource.realInput) annotation(
          Line(points = {{-58, -30}, {-40, -30}}, color = {0, 0, 127}));
        connect(body3.frame_a, lowerWishbone.frame_b_outer) annotation(
          Line(points = {{20, -80}, {20, -70}, {40, -70}}, color = {95, 95, 95}));
        connect(fixed3.frame_b, rackSource.frame_b) annotation(
          Line(points = {{90, -10}, {90, 0}}, color = {95, 95, 95}));
        connect(constantSource2.y, rackSource.realInput) annotation(
          Line(points = {{120, 10}, {100, 10}}, color = {0, 0, 127}));
        connect(rackSource.frame_a, tieRod.frame_a) annotation(
          Line(points = {{80, 10}, {70, 10}, {70, 0}, {60, 0}}, color = {95, 95, 95}));
        connect(contactPatchSource.frame_b, lowerWishbone.frame_b_outer) annotation(
          Line(points = {{-20, -30}, {20, -30}, {20, -70}, {40, -70}}, color = {95, 95, 95}));
        connect(body21.frame_a, wheel.frame_b_contact_patch) annotation(
          Line(points = {{-10, -60}, {0, -60}, {0, -10}}, color = {95, 95, 95}));
      end DoubleWishbone;
    end Suspension;
  end Chassis;
  annotation(
    uses(Modelica(version = "4.1.0")));
end Vehicle;

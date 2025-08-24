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
          Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
            Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
          Modelica.Mechanics.Translational.Sources.Position position annotation(
            Placement(transformation(origin = {30, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Blocks.Interfaces.RealInput realInput annotation(
            Placement(transformation(origin = {-120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
          Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
            Placement(transformation(origin = {-90, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticX(animation = false)  annotation(
            Placement(transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticY(n = {0, 1, 0}, animation = false) annotation(
            Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -180)));
          Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticZ(n = {0, 0, 1}, useAxisFlange = true) annotation(
            Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
        equation
          connect(position.s_ref, realInput) annotation(
            Line(points = {{18, 30}, {0, 30}, {0, 60}, {-120, 60}}, color = {0, 0, 127}));
          connect(fixed.frame_b, prismaticX.frame_a) annotation(
            Line(points = {{-80, 0}, {-60, 0}}, color = {95, 95, 95}));
          connect(prismaticX.frame_b, prismaticY.frame_a) annotation(
            Line(points = {{-40, 0}, {-10, 0}}, color = {95, 95, 95}));
          connect(prismaticY.frame_b, prismaticZ.frame_a) annotation(
            Line(points = {{10, 0}, {40, 0}}, color = {95, 95, 95}));
          connect(prismaticZ.frame_b, frame_b) annotation(
            Line(points = {{60, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(position.flange, prismaticZ.axis) annotation(
            Line(points = {{40, 30}, {58, 30}, {58, 6}}, color = {0, 127, 0}));
        end ContactPatchSource;

        model RackSource
          parameter Real rack_axis[3]={0,1,0} "Axis specifying direction of rack travel";
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
          Placement(transformation(origin = {100, 0}, extent = {{16, -16}, {-16, 16}}, rotation = -180), iconTransformation(origin = {0, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
        Modelica.Mechanics.Translational.Sources.Position position annotation(
          Placement(transformation(origin = {-50, 30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
        Modelica.Blocks.Interfaces.RealInput realInput annotation(
          Placement(transformation(origin = {-120, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -180), iconTransformation(origin = {-100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
          Placement(transformation(origin = {-60, -100}, extent = {{16, -16}, {-16, 16}}, rotation = -90), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}})));
        Modelica.Mechanics.MultiBody.Joints.Prismatic prismaticZ(n = rack_axis, useAxisFlange = true) annotation(
          Placement(transformation(extent = {{-10, -10}, {10, 10}})));
        equation
          connect(position.s_ref, realInput) annotation(
            Line(points = {{-62, 30}, {-62, 60}, {-120, 60}}, color = {0, 0, 127}));
          connect(prismaticZ.frame_b, frame_b) annotation(
            Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}));
          connect(position.flange, prismaticZ.axis) annotation(
            Line(points = {{-40, 30}, {8, 30}, {8, 6}}, color = {0, 127, 0}));
  connect(frame_a, prismaticZ.frame_a) annotation(
            Line(points = {{-60, -100}, {-60, 0}, {-10, 0}}));
        end RackSource;
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
          parameter Boolean animation=true "Show link in animation";
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
          Vehicle.Chassis.Suspension.Linkages.CompliantLink lower_to_contact_patch(r = r_lower_to_contact_patch, animation = false) annotation(
            Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
        Vehicle.Chassis.Suspension.Linkages.CompliantLink contact_patch_to_tie(r = r_lower_to_tie - r_lower_to_contact_patch, animation = false) annotation(
                Placement(transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation contact_patch_to_upper(r = r_lower_to_upper - r_lower_to_contact_patch, animation = false) annotation(
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
      end Linkages;

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
      
      Modelica.Mechanics.MultiBody.Parts.Body body1(r_CM = {0, 0, 0}, m = 1)  annotation(
          Placement(transformation(origin = {-10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Parts.Body body2(r_CM = {0, 0, 0}, m = 1) annotation(
          Placement(transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.Body body3(r_CM = {0, 0, 0}, m = 1)  annotation(
          Placement(transformation(origin = {20, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      
      Vehicle.Chassis.Suspension.Sources.ContactPatchSource contactPatchSource annotation(
          Placement(transformation(origin = {-30, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
      Modelica.Blocks.Sources.Sine constantSource(amplitude = 0, f = 1, offset = 0)  annotation(
          Placement(transformation(origin = {-70, -30}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));
      
      Vehicle.Chassis.Suspension.Sources.RackSource rackSource annotation(
          Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Blocks.Sources.Sine constantSource2(amplitude = 0.125, f = 1, offset = 0)   annotation(
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
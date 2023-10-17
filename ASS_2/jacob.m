function J0v = jacob(q)
%INPUT: joint position of the robot.
%           - q : 5x1 [5x1 rad]
%OUTPUT:    - J0v [3x5]: Jacobian containing the map between end effectors
%linear velocities and joint velocities
% For this question, DO NOT use the robotics toolbox, you should derive answers yourself!
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);
c4 = cos(q4);
c5 = cos(q5);
s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
s4 = sin(q4);
s5 = sin(q5);
c23 = cos(q2+q3);
s23 = sin(q2+q3);
f11 = 9*c1*c4*s5 - 9*s1*c23*s4*s5 + 9*s1*s23*c5 + s1*(4*c23 + 11*s23 + 17*s2);
f12 = 9*s1*c4*s5 + 9*c1*c23*s4*s5 - 9*c1*s23*c5 - c1*(4*c23 + 11*s23 + 17*s2);
f13 = 0;
f21 = -9*c1*s23*s4*s5 - 9*c1*c23*c5 - c1*(-4*s23 + 11*c23 +17*c2);
f22 = -9*s1*s23*s4*s5 - 9*s1*c23*c5 - s1*(-4*s23 + 11*c23 +17*c2);
f23 = -9*c23*s4*s5 - 9*s23*c5 -17*s2 - 11*s23 - 4*c23;
f31 = -9*c1*s23*s4*s5 - 9*c1*c23*c5 - c1*(-4*s23 + 11*c23 + 17*s2);
f32 = -9*s1*s23*s4*s5 - 9*s1*c23*c5 - s1*(-4*s23 + 11*c23 + 17*s2);
f33 = -9*c23*s4*s5 - 9*s23*c5 - 11*s23 - 4*c23;
f41 = -9*s1*s4*s5 + 9*c1*c23*c4*s5;
f42 = 9*c1*s4*s5 + 9*s1*c23*c4*s5;
f43 = -9*s23*c4*s5;
f51 = 9*s1*c4*c5 + 9*c1*c23*s4*c5 + 9*c1*s23*s5;
f52 = -9*c1*c4*c5 + 9*s1*c23*s4*c5 + 9*s1*s23*s5;
f53 = -9*s23*s4*c5 - 9*c23*s5;

J0v = [f11 f21 f31 f41 f51; 
    f12 f22 f32 f42 f52;
    f13 f23 f33 f43 f53];

end
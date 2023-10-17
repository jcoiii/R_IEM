function [p,R] = fk_sol(q)
l1 = 17; l2=17; l3=07; l4=04; l5=04; l6=09;
T1 = DHmatrix(0,0,q(1)+pi,l1);
T2 = DHmatrix(-pi/2,0,q(2)-pi/2,0);
T3 = DHmatrix(0,l2,q(3)+pi/2,0);
T4 = DHmatrix(pi/2,l4,q(4)+pi/2,l3+l5);
T5 = DHmatrix(-pi/2,0,q(5),0);
T6 = DHmatrix(pi/2,0,pi/2,l6);
T_06 = T1*T2*T3*T4*T5*T6;
p = T_06(1:3,4);
R = T_06(1:3,1:3);

function T = DHmatrix(alpha,a,theta,d)
Tz = [1,0,0,0
0,1,0,0
0,0,1,d
0,0,0,1];
Tx = [1,0,0,a
0,1,0,0
0,0,1,0
0,0,0,1];
Rx = [1,0,0,0
0,cos(alpha),-sin(alpha),0
0,sin(alpha),cos(alpha),0
0,0,0,1];
Rz = [cos(theta),-sin(theta),0,0
sin(theta),cos(theta),0,0
0,0,1,0
0,0,0,1];
T = Rx*Tx*Rz*Tz;
end
end
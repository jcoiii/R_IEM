function [q] = ik(p_Ex,p_Ey,p_Ez)   
%% ik
%INPUT: coordinates of the end effector position in expressed in the 0
%frame NOTICE: it is up to the user to provide a reachable position
%           - p_Ex : 1x1
%           - p_Ey : 1x1
%           - p_Ez : 1x1
%OUTPUT:    - theta1 [rad]: angle joint 1
%           - theta2 [rad]: angle joint 2
%           - theta3 [rad]: angle joint 3
%           - theta4 [rad]: angle joint 4
%           - theta5 [rad]: angle joint 5

% For this question, DO NOT use the robotics toolbox! You should use the
% function for the inverse kinematics computed as a solution of the
% theory in the case in which thta_4=0, theta_5 = pi/2

% link lengths
L1 = 17; L2 = 17; L3 = 7; 
L4 = 4; L5 = 4; L6 = 9;

%%%%%% Write your code below this line. DO NOT change other parts. %%%%%%

% Inside of this function, write down how you calculate the output from 
% the input. This is the place that has to be done by youself by using 
% the results you get in the previous subquestions.
%shorter notation

px = p_Ex;
py = p_Ey;
pz = p_Ez;

% Theta 3
p3 = sqrt((L3+L5)^2 + L4^2 + L6^2);
q3 = sqrt(px^2 + (pz-L1)^2 + py^2);
beta3 = atan2(L4,(L3+L5));
c3b = (p3^2 + L2^2 - q3^2)/(2*p3*L2);
s3b = -sqrt(1-c3b^2);
theta3 = atan2(s3b, c3b) - beta3;

% Theta 2
k1 = 4*cos(theta3) + 11*sin(theta3);
k2 = -4*sin(theta3) + 11*cos(theta3) + 17;
b = [sqrt(px^2 + py^2 - 81); (pz-L1)];
K = [k1 k2; k2 -k1];
c = (K^-1)*b;
theta2 = atan2(c(2), c(1)); 

% Theta 1
if theta2 < 0 % theta_1 for theta_2 < 0
    a1_1 = sqrt(px^2+py^2-L6^2);
    beta1 = atan2(L6,a1_1);
    beta2 = atan2(py,px);
    theta1 = beta1+beta2;
elseif theta2 > 0 % theta_1 for theta_2 > 0
    a1_1 = sqrt(px^2+py^2);
    beta1 = atan2(L6,sqrt(a1_1^2-L6^2));
    beta2 = atan2(py,px);
    theta1 = beta2-beta1;
end

% Constrained joints
theta4 = 0;
theta5 = pi/2;

q = [theta1, theta2, theta3, theta4, theta5];
%%%%%% Write your code above this line. DO NOT change other parts. %%%%%%

end 

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


theta1 = ...;

theta2 = ...;

theta3 = ...;



theta4=0;
theta5=pi/2;
q = [theta1, theta2, theta3,theta4,theta5];
%%%%%% Write your code above this line. DO NOT change other parts. %%%%%%

end 

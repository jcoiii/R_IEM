function end_pos = fk(theta1, theta2)   
% The inputs of this function are two angles: theta1 and theta2, the output
% of this function is the position of the end-effector: end_pos, which is
% a 3D column vector
%
% For this question, DO NOT use the robotics toolbox! You should calculate 
% the transformation matrices here and output the desired result. 

L1 = 1; % link lengths
L2 = 1;

%%%%%% Write your codes below this line. DO NOT change other parts. %%%%%%

%% Angles input and initialization
st1 = sin(theta1);
ct1 = cos(theta1);
st2 = sin(theta2);
ct2 = cos(theta2);

%% Homogeneous transformation matrix
% Inside of this function, write down how you calculate the output from 
% the input. This is the place which needs to be done by youself by using 
% the results you get in the previous subquestions.
T_03 = [ct1*ct2 -ct1*st2 st1 ct1*ct2*L2+ct1*L1;
        st1*ct2 -st1*st2 -ct1 st1*ct2*L2+st1*L1;
        st2 ct2 0 st2*L2;
        0 0 0 1
        ];
end_pos =  [T_03(1,4);
            T_03(2,4);
            T_03(3,4);            
            ];

% Made by J. Chandra
%%%%%% Write your codes above this line. DO NOT change other parts. %%%%%%

end 


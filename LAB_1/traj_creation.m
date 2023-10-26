%% %%%%% initial setup  %%%%%

% Add the toolbox into the workpath (downlocad from brightspace)
addpath('X:\Robotics Lab\Group 24\robotics-lib\PeterCorkeToolboxRug\rvctools\common');  
addpath('X:\Robotics Lab\Group 24\robotics-lib\PeterCorkeToolboxRug\rvctools\robot\interfaces');  

% Check the port COMx
arb = Arbotix('port','COM10','baud', 57610)


%% %%%%% Simulation setup  %%%%%
L1 = 17; % note: all lengths are given in cm.
L2 = 17;
L3 = 7;
L4 = 4;
L5 = 4;
L6 = 9;

L(1) = Link([ 0, L1, 0, 0, 0, 0], 'modified');
L(2) = Link([ 0, 0, 0, pi/2, 0, pi/2], 'modified');
L(3) = Link([ 0, 0, L2, 0, 0, pi/2], 'modified');
L(4) = Link([ 0, L3+L5, L4, pi/2, 0,pi/2], 'modified');
L(5) = Link([ 0, 0, 0, pi/2, 0, 0], 'modified');

pArb = SerialLink(L, 'name', 'pArb')
pArb.plotopt={'workspace', [-50 50 -50 50 -60 60]};
pArb.tool = [0 -1 0 0; 0 0 1 9; -1 0 0 0; 0 0 0 1];

alpha_a = [0 0 0 0 0];
alpha_b = [-pi/2 pi/4 pi/4 0 0];
% alpha_a = [-pi/2 pi/4 pi/4 0 0];
% alpha_b = [0 0 0 0 0];

% setup initial and final positions, for instance:
Pa =  pArb.fkine(alpha_a); 
Pb =  pArb.fkine(alpha_b); 
Start_loc = Pa.t
End_loc = Pb.t
% alphas stand for the joint angles of the (initial and ending) poses
% Be sure not to exceed the range of each joint when defining the alphas
% you may also change mask to [1 1 1 0 0 0] for different results

j_start = pArb.ikine(Pa,'mask', [1,1,1,0,0,0]);
j_stop = pArb.ikine(Pb,'mask', [1,1,1,0,0,0]);


% creat trajectory
t=0:0.05:3;
[q,qd,qdd] = jtraj(j_start,j_stop,t);

% plot in simulation
pArb.plot(q)



%% %%%%% Real Robot Arm setup  %%%%%
% this command puts all joints in position 0
arb.setposall(0, 0, 0, 0, 0, 0)
% arb.setposall(pi/2, pi/2, -pi/2, 0, 0, 0)
arb.relax

% if you have a sequence of angle vector, namely 'q', 
% you can use 'for' loop to load the angle data.

% be sure to pause each step with function 'pause()'to avoid sudden moves
for i =1:length(q)
    arb.setposall(q(i,1), q(i,2), q(i,3), q(i,4), q(i,5), 0)
end

% 


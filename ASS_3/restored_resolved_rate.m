clear all; close all; clc
%% Create a robot arm
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

pArb = SerialLink(L, 'name', 'pArb');
pArb.plotopt={'workspace', [-50 50 -50 50 -60 60]};
pArb.tool = [0 -1 0 0; 0 0 1 9; -1 0 0 0; 0 0 0 1];

%% define q0 and pose_stop
q0 = [0 -pi/7 -pi/7 0 0];
pose_stop = pArb.fkine([0 pi/2 -pi/3 pi/6 0]);

%% Your answer
%%%%%% Write your code below this line. DO NOT change other parts. %%%%%%
pArb.plot(q_traj, 'trail', {'k--o', 'LineWidth', 0.5});
dt = 0.05;                      % time step
n = 10/dt;                      % total steps
Qn = zeros(5,n);                % a matrix to store all the generated joint angles 
q = q0';                        % Joint angle var
pend = pose_stop.t;             % the goal point
K = 0.01;                          % Control gain 
tic
for i = 1:n
    J = pArb.jacob0(q');        % Jacobian
    pcurrent = pArb.fkine(q');  % HT matrix
    pcurrent = pcurrent.t;      % Trans of HT matrix
    
    v = [((pend - pcurrent) / dt); 
        0;
        0;
        0];                     % Lin velocity, 0;0;0 for ang velocity
    
    qdot = pinv(J) * v;         % Joint velocity
    
    q = q + K * dt * qdot;      % Joint Angle update
    
    Qn(:, i) = q;               % Store the new joint angles
end


cartesian_traj = zeros(n, 3); % Initialize matrix to store Cartesian coords
q_traj = Qn';
for k = 1:n
    T = pArb.fkine(q_traj(k, :)); % 
    cartesian_traj(k, :) = T.t(1:3); % 
end

% Plot the robot arm trajectories in 3D
figure(1);
plot3(cartesian_traj(:, 1), cartesian_traj(:, 2), cartesian_traj(:, 3), 'b', 'LineWidth', 2);
hold on;

% Plot the robot arm movement using the robotics toolbox
pArb.plot(q_traj, 'trail', {'k--o', 'LineWidth', 0.5});
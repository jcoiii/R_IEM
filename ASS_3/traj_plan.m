clear all
close all
clc

L1 = 17; % note: all lengths are given in cm.
L2 = 17;
L3 = 7;
L4 = 4;
L5 = 4;
L6 = 9;

L(1) = Link('revolute', 'd',L1,'a',0,'alpha',0, 'modified','offset', pi);
L(2) = Link('revolute', 'd',0,'a',0,'alpha',-pi/2, 'modified','offset', -pi/2);
L(3) = Link('revolute', 'd',0,'a',L2,'alpha',0, 'modified','offset', pi/2);
L(4) = Link('revolute', 'd',L3+L5,'a',L4,'alpha',pi/2, 'modified','offset', pi/2);
L(5) = Link('revolute', 'd',0,'a',0,'alpha',pi/2, 'modified','offset', 0);

pArb = SerialLink(L, 'name', 'pArb')
pArb.plotopt={'workspace', [-50 50 -50 50 -60 60]};
pArb.tool = [0 -1 0 0; 0 0 1 L6; -1 0 0 0; 0 0 0 1];

pose_start = [1, 0, 0, -4;
              0, 1, 0, 0;
              0, 0, 1, 54;     % This corresponds to pArb.fkine([0 0 0 0 0])
              0, 0, 0, 1];
pose_stop =  [-1, 0, 0, 0;
               0, 1, 0, 21;
               0, 0, -1, -3;
               0, 0, 0, 1];   % This corresponds to pArb.fkine([pi/2 -pi/2 -pi/2 0 0])
           
           
%% Your answer
%%%%%% Write your code below this line. DO NOT change other parts. Some hints are given, but you are allowed to change them %%%%%%
%%%% This is for the first sub-question

% Plan trajectories in the joint space using pose_start and pose_stop
q_start = pArb.ikine(pose_start, [0 0 0 0 0], 'mask', [1 1 1 0 0 0]);
q_stop = pArb.ikine(pose_stop, [0 0 0 0 0], 'mask', [1 1 1 0 0 0]);
num_steps = 50; 
[q_traj, qd_traj, qdd_traj] = jtraj(q_start, q_stop, num_steps); % p114

num_points = size(q_traj, 1);
cartesian_traj = zeros(num_points, 3); % Initialize matrix to store Cartesian coords

for k = 1:num_points
    T = pArb.fkine(q_traj(k, :)); % Compute forward kinematics
    cartesian_traj(k, :) = T.t(1:3); % Extract (x, y, z) position
end

% Plot the robot arm trajectories in 3D
figure(1);
plot3(cartesian_traj(:, 1), cartesian_traj(:, 2), cartesian_traj(:, 3), 'r--', 'LineWidth', 1.5);
hold on;

% Plot the robot arm movement using the robotics toolbox
pArb.plot(q_traj, 'trail', {'k-', 'LineWidth', 1}); % trail -> visualize movement

% Plot the trajectories of the joint variables
figure(2);
num_joints = size(q_traj, 2);
for i = 1:num_joints
    subplot(num_joints, 1, i);
    plot(q_traj(:, i), 'LineWidth', 1.5);
    hold on;
    title(['Joint ', num2str(i), ' Trajectory']);
    xlabel('Steps');
    ylabel('Angle (rad)');
    grid on;
end

%%%%% Write your code above this line. DO NOT change other parts. %%%%%%


%%%%%% Write your code below this line. DO NOT change other parts. %%%%%%
%%%% This is for the second sub-question

% % Plan trajectories in the Cartesian space using pose_start and pose_stop
% T = ctraj(pose_start, pose_stop, num_steps);
% % q_traj = zeros(num_steps, pArb.n);  % Initialize a matrix to store joint trajectories
% % Initialize matrix to store joint angles
% q_traj = zeros(num_steps, pArb.n);
% 
% % Use inverse kinematics to find joint angles for each Cartesian pose
% for i = 1:num_steps
%     q_traj(i, :) = pArb.ikine(T(:,:,i), 'mask', [1 1 1 0 0 0]);
% end
% 
% % Extract XYZ coordinates of the Cartesian path for plotting
% X = squeeze(T(1,4,:));
% Y = squeeze(T(2,4,:));
% Z = squeeze(T(3,4,:));
% 
% % Plot the robot arm trajectories in 3D; 
% figure(1);
% plot3(X, Y, Z, 'b-', 'LineWidth', 2);  % Plotting with a blue solid line
% hold on;
% 
% % Plot the robot arm movement using the robotics toolbox
% pArb.plot(q_traj);
% 
% % Plot the trajectories of the joint variables
% num_joints = size(q_traj, 2); % Assuming 'q_traj' is your joint trajectories matrix with size (num_steps, num_joints)
% figure(3);
% for i = 1:num_joints
%     subplot(num_joints, 1, i);  % Create a subplot for each joint
%     plot(q_traj(:, i), 'LineWidth', 1.5);  % Plot the trajectory for joint i
%     title(['Joint ', num2str(i), ' Trajectory']);  % Title for each subplot
%     xlabel('Steps');  % x-axis label
%     ylabel('Angle [rad]');  % y-axis label
%     grid on;  % Turn the grid on for better visibility of plot
% end


%%%%%% Write your code above this line. DO NOT change other parts. %%%%%%

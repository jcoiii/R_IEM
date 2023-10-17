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

dt = 0.05;          % time step
n = 10/0.05;        % total steps
Qn = zeros(5,n);    % a matrix to store all the generated joint angles 
q = q0';
pend = ...          % the goal point

for i = 1:n
    % calculate the new array of joints "qnew" and put them into the
    % matrix Qn using the resolve-rate controller
end

% Plot the robot arm trajectories in 3D
figure;
plot3(.....)
hold on;

% Plot the robot arm movement using the robotics toolbox
pArb.plot(Qn');

%%%%%% Write your code above this line. DO NOT change other parts. %%%%%%

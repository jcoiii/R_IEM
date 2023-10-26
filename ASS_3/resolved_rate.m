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
% Instruction
% The motion will take n steps with the step size dt. At each time step, from 1 to n, compute the
% joint coordinates that the robot must reach using the resolved rate motion control algorithm. Notice
% that v has to change at each time instant such that it helps the robot move towards the goal point

dt = 0.05;                      % time step
n = 10/dt;                      % total steps
Qn = zeros(5,n);                % a matrix to store all the generated joint angles 
q_null = q0';                   % Initial Joint angle var
pend = pose_stop.t;             % the goal point

% Qn_1, Qn_2 is the joint angles.
% ct_1, ct_2 is the cartesian trajectory.
[ct_1, Qn_1] = calculate_trajectory(10, Qn, q_null, pend, n, dt, pArb, 1); % For K=10, Figure 1
[ct_2, Qn_2] = calculate_trajectory(1, Qn, q_null, pend, n, dt, pArb, 2); % For K=1, Figure 2
[ct_3, Qn_3] = calculate_trajectory(0.03, Qn, q_null, pend, n, dt, pArb, 3); % For K=0.03, Figure 3

figure(4);
subplot(3,1,1);
plot(ct_1(:,1), 'r-', 'LineWidth', 2); hold on;
plot(ct_1(:,2), 'g-', 'LineWidth', 2); hold on;
plot(ct_1(:,3), 'b-', 'LineWidth', 2);
legend('$x$','$y$','$z$', 'Interpreter','latex', 'FontSize', 15);
title('End-Effector Position Over Time for $K=10$', 'Interpreter','latex', 'FontSize', 18);
xlabel('Steps', 'Interpreter','latex', 'FontSize', 15);
ylabel('Position (cm)', 'Interpreter','latex', 'FontSize', 15);

subplot(3,1,2);
plot(ct_2(:,1), 'r-', 'LineWidth', 2); hold on;
plot(ct_2(:,2), 'g-', 'LineWidth', 2); hold on;
plot(ct_2(:,3), 'b-', 'LineWidth', 2);
legend('$x$','$y$','$z$', 'Interpreter','latex', 'FontSize', 15);
title('End-Effector Position Over Time for $K=1$', 'Interpreter','latex', 'FontSize', 18);
xlabel('Steps', 'Interpreter','latex', 'FontSize', 15);
ylabel('Position (cm)', 'Interpreter','latex', 'FontSize', 15);

subplot(3,1,3);
plot(ct_3(:,1), 'r-', 'LineWidth', 2); hold on;
plot(ct_3(:,2), 'g-', 'LineWidth', 2); hold on;
plot(ct_3(:,3), 'b-', 'LineWidth', 2);
legend('$x$','$y$','$z$', 'Interpreter','latex', 'FontSize', 15);
title('End-Effector Position Over Time for $K=0.03$', 'Interpreter','latex', 'FontSize', 18);
xlabel('Steps', 'Interpreter','latex', 'FontSize', 15);
ylabel('Position (cm)', 'Interpreter','latex', 'FontSize', 15);



% This part is just to make sure the start and end position is the same as 
% the desired start and end position
pose_start = pArb.fkine(q0);
desired_pos = [(pose_start.t)' pend']
pos_final_k1 = [ct_1(1,:) ct_1(end,:)]
pos_final_k2 = [ct_2(1,:) ct_2(end,:)]
pos_final_k3 = [ct_3(1,:) ct_3(end,:)]
%  trajectory calculation function
function [cartesian_traj,q_traj] = calculate_trajectory(K, Qn, q, pend, n, dt, pArb, fig_num)
    % Initial variables
    cartesian_traj = zeros(n, 3);
    Qn(:, 1) = q;

    % Trajectory
    for i = 2:n
        J = pArb.jacob0(q');
        pcurrent = pArb.fkine(q');
        pcurrent = pcurrent.t;
        
        v = [((pend - pcurrent) / dt);
            0;
            0;
            0];
        
        qdot = pinv(J) * v;
        q = q + K * dt * qdot;
        Qn(:, i) = q;
    end
    
    q_traj = Qn';
    for k = 1:n
        T = pArb.fkine(q_traj(k, :));
        cartesian_traj(k, :) = T.t(1:3);
    end
    
    % Plot the trajectory
    figure(fig_num);
    plot3(cartesian_traj(:, 1), cartesian_traj(:, 2), cartesian_traj(:, 3), 'b-','LineWidth', 2);
    hold on

    % Plot the robot arm movement using the robotics toolbox
    pArb.plot(q_traj, 'trail', {'k--o', 'LineWidth', 0.5});
    set(gcf,'HandleVisibility','off');
end

%%%%%% Write your code above this line. DO NOT change other parts. %%%%%%

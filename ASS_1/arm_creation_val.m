% For this question, USE the robotics toolbox. 

L1 = 17; % note: all lengths are given in cm.
L2 = 17;
L3 = 7;
L4 = 4;
L5 = 4;
L6 = 9;

%%%%%% Write your codes below this line. DO NOT change other parts. %%%%%%

% Assigning values to the links using the robotic toolbox

L(1) = Revolute('d', L1, 'a', 0, 'alpha', 0, 'offset', pi, 'modified');
L(2) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset',-pi/2, 'modified');
L(3) = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', pi/2, 'modified');
L(4) = Revolute('d', (L3+L5), 'a', L4, 'alpha', pi/2, 'offset', pi/2,'modified');
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset',0,'modified');
pArb = SerialLink(L, 'name', 'pArb');

% adding end effector
pArb.plotopt={'workspace', [-20 20 -20 20 -20 70]};
pArb.tool =  trotx(90) * trotz(90) * transl(0, 0, L6);

% plot the arm in the zero configuration using the robotic toolbox
figure(1234);
pArb.plot([0, 0, 0, 0, 0])

%%%%%% Write your codes above this line. DO NOT change other parts. %%%%%%

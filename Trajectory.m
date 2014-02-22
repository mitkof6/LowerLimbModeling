clear all;
close all;
clc;

%% Load robotics toolbox
startup_rvc
LowerLimbRT

%% Model
model = load('Model.mat');


traj = load('Traj.mat');
bone = load('L.mat');

%% Syms

syms L1 L2 L3 L4 L5 L6
L = [L1 L2 L3 L4 L5 L6];

% Link mass
syms m1 m2 m3 m4 m5 m6
m = [m1 m2 m3 m4 m5 m6];


% Gravity
syms gz;
g = [0 0 gz]';

param = [m L gz];

%% 
traj = traj.traj;
bone = bone.L;

modelParam = [1 0 0 1 1 1 bone(1) 0 0 bone(2) bone(3) bone(4) 9.81];

% T0j = model.T0j;
% T0j = subs(T0j, param, modelParam);
% 
% q = model.q;
% dq = model.dq;
% ddq = model.ddq;
% Tau = model.Tau;



%% Trajectory generation

% t = 0:0.1:1;
% 
% q0 = [0 pi/18 pi/2 pi/6 -pi/6 pi/2];
% q1 = [0 pi/18 pi/2 pi/2 0 pi/18];
% q2 = [0 pi/18 pi/2 pi -pi/6 pi/2];
% 
% qTraj = t'*q1+(1-t)'*q0;
% dqTraj = (q1-q0)';
% ddqTraj = [0 0 0 0 0 0]';
% 
% qq = t'*q2+(1-t)'*q1;
% dqq = (q2-q1)';
% 
% qTraj = [qTraj; qq];
%dqTraj = [dqTraj; dqq];
%ddqTraj = [ddqTraj; ddqTraj];

% %% Inverse kinematics
% qeval = zeros(size(t),6);
% qeval(1,:) = q0;
% 
% for i = 2:length(t)
%    pos =  generatePositions(T0j, q, qtraj(i, :));
%    pos = subs(pos);
%    [qeval(i, :), f] = ikine(pos, qeval(i-1,:), T0j, q);
% end

%% Inverse kinematics
% figure;
% 
% qeval = zeros(length(traj(1, 1, :)), 6);
% len = length(traj(1, 1, :));
% for i = 2:len
%     
%     [qeval(i, :), f] = ikine(traj(:, :, i), qeval(i-1,:), T0j, q);
%     bot.plot(qeval(i, :))
% end
% %% Plot
% 
% figure;
% bot.plot(qeval)

%% kolpa
% qTraj = load('qeval.mat');
% qTraj=qTraj.qeval;
% 
% len = length(qTraj(:, 1));
% 
% for i=1:len
%     qTraj(i,1)=mod(qTraj(i,1),2*pi)-2*pi;
% end
% 
% qeval = qTraj(2:end,:);
% len = len -1;
% 
% dqTraj = zeros(len, 6);
% ddqTraj = zeros(len, 6);
% for i = 3:len-2
%     dqTraj(i, :) = 360*(-qeval(i+2, :)+8*qeval(i+1, :)-8*qeval(i-1, :)+qeval(i-2, :));
%     
%     ddqTraj(i, :) = 12*(30)^2*(-qeval(i+2, :)+16*qeval(i+1, :)-30*qeval(i, :)+16*qeval(i-1, :)-qeval(i-2, :));
% 
% end
% dqTraj = dqTraj(3:end-2, :);
% ddqTraj = ddqTraj(3:end-2, :);
% qTraj = qTraj(4:end-2, :);
% save('Tr,mat', 'qTraj', 'dqTraj', 'ddqTraj');

tr = load('Tr.mat');

qTraj = tr.qTraj;
dqTraj = tr.dqTraj./100;
ddqTraj = tr.ddqTraj./100;
    
len = length(qTraj(:, 1));
%% Generate torque
Tau = [];
for i = 1:len
   Tau = cat(1, Tau, fdyn(ddqTraj(i, :), dqTraj(i, :), qTraj(i, :)));
end


%% Simulate

TS = 1/30;
tStop = TS*len;
T = 0;
iter = 10;
Xn = [qTraj(1, :) dqTraj(1, :)];
q = [];
Xs = [];
Tau_c=[0 0 0 0 0 0]';

while T(end) <= tStop

    [tSolution, xSolution] = ode45(@idyn, [(iter-10)*TS (iter)*TS], Xn, [], [Tau(iter,:),qTraj(iter,:),dqTraj(iter, :)]');
    
    tSolution = tSolution(end); 
    xSolution = xSolution(end,1:end);
    
    
    Xn = xSolution(end, 1:end);
    q = [q; xSolution(:, 1:6)];
    T = [T; tSolution];
    bot.plot(q(end, :));
    
        
    %% Next Iteration
    
    iter = iter+10
end

figure;
bot.plot(q)



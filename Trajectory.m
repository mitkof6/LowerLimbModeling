clear all;
close all;
clc;

%% Load robotics toolbox
startup_rvc
LowerLimbRT

%% Model
model = load('Model.mat');
modelParam = [1 0 0 1 1 1 0.2 0 0 0.5 0.5 0.2 9.81];

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

T0j = model.T0j;
T0j = subs(T0j, param, modelParam);

q = model.q;
dq = model.dq;
ddq = model.ddq;
Tau = model.Tau;

%% Trajectory generation

t = 0:0.1:1;

q0 = [0 pi/18 pi/2 pi/6 -pi/6 pi/2];
q1 = [0 pi/18 pi/2 pi/2 0 pi/18];
q2 = [0 pi/18 pi/2 pi -pi/6 pi/2];

qTraj = t'*q1+(1-t)'*q0;
dqTraj = (q1-q0)';
ddqTraj = [0 0 0 0 0 0]';

qq = t'*q2+(1-t)'*q1;
dqq = (q2-q1)';

qTraj = [qTraj; qq];
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

%% Plot

figure;
bot.plot(qTraj)

%% Generate torque
Tau = [];
for i = 1:2*length(t)
   Tau = cat(1, Tau, fdyn(ddqTraj, dqTraj, qTraj(i, :)));
end


%% Simulate

TS = 0.1;
tStop = 2.0;
T = 0;
iter = 1;
Xn = [0 pi/18 pi/2 pi/6 -pi/6 pi/2 dqTraj'];
q = [];
Xs = [];
Tau_c=[0 0 0 0 0 0]';

while T(end) <= tStop

    [tSolution, xSolution] = ode45(@idyn, [(iter-1)*TS (iter)*TS], Xn, [], [Tau(iter,:),qTraj(iter,:),dqTraj']');
    
    tSolution = tSolution(end); 
    xSolution = xSolution(end,1:end);
    
    
    Xn = xSolution(end, 1:end);
    q = [q; xSolution(:, 1:6)];
    T = [T; tSolution];
    %bot.plot(q(end, :));
    
        
    %% Next Iteration
    
    iter = iter+1;
end

figure;
bot.plot(q)



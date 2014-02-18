clear all;
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

%% Trajectory designe

t = 0:0.05:1;

q0 = [0 pi/18 pi/2 pi/6 -pi/6 pi/2];
q1 = [0 pi/18 pi/2 pi/2 -pi/2 pi/18];

qtraj = t'*q1+(1-t)'*q0;
dqtraj = (q1-q0)';
ddqtraj = [0 0 0 0 0 0]';


%% Computation
qeval = zeros(size(t),6);
qeval(1,:) = q0;

for i = 2:length(t)
   pos =  generatePositions(T0j, q, qtraj(i, :));
   pos = subs(pos);
   [qeval(i, :), f] = ikine(pos, qeval(i-1,:), T0j, q);
end

%% Plot
bot.plot(qeval)







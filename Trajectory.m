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

t = 0:0.1:1;

q0 = [0 pi/18 pi/2 pi/6 -pi/6 pi/2];
q1 = [0 pi/18 pi/2 pi/2 0 pi/18];

qtraj = t'*q1+(1-t)'*q0;
dqtraj = (q1-q0)';
ddqtraj = [0 0 0 0 0 0]';


% %% Computation
% qeval = zeros(size(t),6);
% qeval(1,:) = q0;
% 
% for i = 2:length(t)
%    pos =  generatePositions(T0j, q, qtraj(i, :));
%    pos = subs(pos);
%    [qeval(i, :), f] = ikine(pos, qeval(i-1,:), T0j, q);
% end

%% Plot
%bot.plot(qeval)
figure;
bot.plot(qtraj)
%%
Tau = [];
for i = 1:length(t)
   Tau = cat(1, Tau, fdyn(ddqtraj, dqtraj, qtraj(i, :)));
end

ts = 0.1;
tstop = 1.0;
T = 0;
iter = 1;
Xn = [0 pi/18 pi/2 pi/6 -pi/6 pi/2 dqtraj'];
q = [];
Xs = [];
Tau_c=[0 0 0 0 0 0]';

e_i=0;

while T(end)<=tstop

    [tsol, Xsol] = ode45(@idyn, [(iter-1)*ts (iter)*ts], Xn, [], [Tau(iter,:),qtraj(iter,:),dqtraj']');
    
    %q = [q;Xsol(:, 1:6)];
    tsol = tsol(end); Xsol=Xsol(end,1:end);
    
    
    
    T = [T, tsol];
    Xs = [Xs;Xsol];
    Xn = Xs(end, 1:end);
    q=[q;Xn(1:6)];
    %bot.plot(q(end, :));
    
    
%     %% Control
%     
%     e=qtraj(iter,:)-q(iter,:);
%     e_d=dqtraj'-Xsol(7:12);
%     Kp=diag([10 10 10 100 100 100]);
%     Kd=diag([3 4 4 1 1 1]);
%     Ki=diag([3 4 4 15 11 11]);
%     e_i=e_i+e.*ts;
%     
%     Tau_c=Kp*e'%+Kd*e_d'+Ki*e_i'
    
    %% Next Iteration
    iter = iter+1
end

figure;
bot.plot(q)



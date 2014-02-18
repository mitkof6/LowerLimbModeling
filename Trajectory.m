clear all;
clc;

startup_rvc
LowerLimbRT

syms L1 L2 L3 L4 L5 L6
L = [L1 L2 L3 L4 L5 L6];

% Link mass
syms m1 m2 m3 m4 m5 m6
m = [m1 m2 m3 m4 m5 m6];


% Gravity
syms gz;
g = [0 0 gz]';


param = [m L gz];

the = [1 0 0 1 1 1 0.2 0 0 0.5 0.5 0.2 9.81];

A=load('Model.mat');
T0j = A.T0j;
T0j = subs(T0j, param, the);

q = A.q;
dq = A.dq;
ddq=A.ddq;
Tau=A.Tau;

t=0:0.05:1;

q0=[0 pi/18 pi/2 pi/6 -pi/6 pi/2];
q1=[0 pi/18 pi/2 pi/2 -pi/2 pi/18];

qtraj=t'*q1+(1-t)'*q0;
dqtraj=(q1-q0)';
ddqtraj=[0 0 0 0 0 0]';

%bot.plot(qtraj)

qeval=zeros(size(t),6);
qeval(1,:) = q0;
for i = 2:length(t)
   pos =  Pos(T0j, q, qtraj(i, :));
   pos=subs(pos);
   [qeval(i, :), f] = InvKin(pos, qeval(i-1,:), T0j, q);
end


%bot.plot(qeval)







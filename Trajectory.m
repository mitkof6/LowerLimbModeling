clear all;
clc;

startup_rvc
LowerLimbRT

A=load('Model.mat');
T0j=A.T0j;
q=A.q;
dq=A.dq;
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

Tau_traj=subs(Tau,[dq ddq],[dqtraj ddqtraj]);
Tau_traj=subs(Tau_traj,q,qtraj);


%bot.plot(qeval)


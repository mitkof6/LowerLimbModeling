% Given a model and model parameters generates 
% inverse dynamics and forword dynamics m-Files

clear all;
clc;

%% Model
model = load('Model.mat');
bone = load('L.mat');
bone = bone.L;
modelParam = [1 0 0 1 1 1 bone(1) 0 0 bone(2) bone(3) bone(4) 9.81];
M = load('M.mat');
G = load('G.mat');
M = M.M;
G = G.G;



%% Syms
syms th1 th2 th3 th4 th5 th6

% General coordinates
q = [th1 th2 th3 th4 th5 th6];

syms dth1 dth2 dth3 dth4 dth5 dth6

dq = [dth1 dth2 dth3 dth4 dth5 dth6];

syms ddth1 ddth2 ddth3 ddth4 ddth5 ddth6

ddq = [ddth1 ddth2 ddth3 ddth4 ddth5 ddth6];

% Base translation
syms x0 y0 z0

% Link length
syms L1 L2 L3 L4 L5 L6
L = [L1 L2 L3 L4 L5 L6];

% Link mass
syms m1 m2 m3 m4 m5 m6
m = [m1 m2 m3 m4 m5 m6];

% Gravity
syms gz;
g = [0 0 gz]';

param = [m L gz];

%% fdyn code generator

%Tau = subs(model.Tau, param, modelParam);
Tau = M*ddq'+model.C*dq'+G;
Tau = subs(Tau, param, modelParam);
digits(3);
Tau = vpa(Tau);

fDyn = fopen('fdyn.m', 'w');

fprintf(fDyn, '%% Tau = torque \n%% ddq = joint''s acceleration \n');
fprintf(fDyn, '%% dq = joints''s velocoty \n%% q = joints''s angles \n');

fprintf(fDyn, 'function [Tau] = fdyn(ddq, dq, q)\n');

fprintf(fDyn, '\n');

for i=1:length(q)
    fprintf(fDyn, 'th%d = q( %d ); \ndth%d = dq( %d ); \nddth%d = ddq( %d ); \n',i,i,i,i,i,i);
end

fprintf(fDyn, '\n');

for i=1:length(q)
    fprintf(fDyn, 'Tau( %d ) = %s ; \n',i , char(Tau(i)));
end

fprintf(fDyn, '\n');

fprintf(fDyn, 'end\n');

fclose(fDyn);

%% idyn code generator

M = subs(M, param, modelParam);
C = subs(model.C, param, modelParam);
G = subs(G, param, modelParam);

digits(3);
M = vpa(M);
C = vpa(C);
G = vpa(G);

iDyn = fopen('idyn.m', 'w');

fprintf(iDyn, '%% Xdot = state \n%% t = time for evaluation \n');
fprintf(iDyn, '%% X = initial state \n%% U = input \n');

fprintf(iDyn, 'function [Xdot] = idyn(t, X, U)\n');

fprintf(iDyn, '\n');

for i=1:length(q)
    fprintf(iDyn, 'th%d = X(%d); \ndth%d = X(%d); \n',i,i,i,i+length(q));
end

fprintf(iDyn, 'M = zeros(%d,%d); \n', length(q), length(q));
fprintf(iDyn, 'C = zeros(%d,%d); \n',length(q),length(q));
fprintf(iDyn, 'G = zeros(%d,1); \n\n',length(q));

for i=1:length(q)
    for j=1:length(q)
        fprintf(iDyn, 'M(%d,%d) = %s ; \n',i ,j, char(M(i,j)));
        
    end
end

fprintf(iDyn, '\n');

for i=1:length(q)
    for j=1:length(q)
        fprintf(iDyn, 'C(%d,%d) = %s ; \n',i ,j, char(C(i,j)));        
    end
end

fprintf(iDyn, '\n');

for i=1:length(q)
    fprintf(iDyn, 'G(%d) = %s ; \n', i, char(G(i)));
end

fprintf(iDyn, '\n');

fprintf(iDyn, 'Minv = inv(M) ; \n');

fprintf(iDyn, '\n');

fprintf(iDyn, 'ddq = Minv*(U-C*X(%d:end)-G); \n', length(q)+1);

fprintf(iDyn, 'Xdot = [ ');
for i = 1:length(q)
    fprintf(iDyn, 'X(%d) ',i+length(q));
    
end

for i = 1:length(q)
    fprintf(iDyn, 'ddq(%d) ',i);
    
end

fprintf(iDyn, ']'';\n');

fprintf(iDyn, 'end\n');

fclose(iDyn);







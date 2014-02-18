clear all;
clc;

%% Model
mod = load('Model.mat');

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

modelParam = [1 0 0 1 1 1 0.2 0 0 0.5 0.5 0.2 9.81];

% %% Tau
% Tau = subs(mod.Tau, param, modelParam);
% digits(3);
% Tau = vpa(Tau);
% 
% ftau = fopen('fdyn.m', 'w');
% fprintf(ftau, 'function [Tau] = fdyn(ddq, dq, q)\n');
% 
% for i=1:length(q)
%     fprintf(ftau, 'th%d = q( %d ); \n dth%d = dq( %d ); \n ddth%d = ddq( %d ); \n\n\n',i,i,i,i,i,i);
% end
% 
% for i=1:length(q)
%     fprintf(ftau, 'Tau( %d ) = %s ; \n\n',i , char(Tau(i)));
% end
% fprintf(ftau, 'end\n');
% 
% fclose(ftau);

%% invdyn

M = subs(mod.M, param, modelParam);
C = subs(mod.C, param, modelParam);
G = subs(mod.G, param, modelParam);

digits(3);
M = vpa(M);
C = vpa(C);
G = vpa(G);

ftau = fopen('idyn.m', 'w');
fprintf(ftau, 'function [Xdot] = idyn(t, X, U)\n');

for i=1:length(q)
    fprintf(ftau, 'th%d = X( %d ); \n dth%d = X( %d ); \n',i,i,i,i+length(q));
end

fprintf(ftau, 'M = zeros(%d,%d); \n',length(q),length(q));
fprintf(ftau, 'C = zeros(%d,%d); \n',length(q),length(q));
fprintf(ftau, 'G = zeros(%d,1); \n',length(q));

for i=1:length(q)
    fprintf(ftau, 'G(%d) = %s ; \n', i, char(G(i)));
    for j=1:length(q)
        fprintf(ftau, 'M(%d,%d) = %s ; \n',i ,j, char(M(i,j)));
        fprintf(ftau, 'C(%d,%d) = %s ; \n',i ,j, char(C(i,j)));
    end
end

fprintf(ftau, 'Minv = inv(M) ; \n');

fprintf(ftau, 'ddq = Minv*(U-C*X(%d:end)-G); \n', length(q)+1);

fprintf(ftau, 'Xdot = [ ');
for i = 1:length(q)
    fprintf(ftau, 'X(%d) ',i+length(q));
    
end

for i = 1:length(q)
    fprintf(ftau, 'ddq(%d) ',i);
    
end
fprintf(ftau, ']';\n');

fprintf(ftau, 'end\n');

fclose(ftau);







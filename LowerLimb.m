%%
clear all;
clc;
%close all;

% initial pose
% subs(M, param, qz)
% subs(C, [param dq], [qz 1 1 1 1 1 1])

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

param = [m L q];

% Gravity
syms gz;
g = [0 0 gz]';

%% DH Parameters a->twist aa->link length d->d th->thi

% Base translation
x00 = 0;
y00 = 0;
z00 = 0;

DH = [ 0 L1 -L1 th1; 
       -pi/2 0 0 th2;
       pi/2 0 0 th3;
       0 L4 0 th4;
       0 L5 0 th5;
       0 L6 0 th6 ];

% two link debugging
% DH = [ 0 L1 0 th1;
%        0 L2 0 th2];

DOF = length(DH(:, 1));

%% Links inertia

I = [];
for i = 1:DOF
    I = cat(3, I, [ 0 0 0 ;0 m(i)/12*L(i)^2 0;0 0 m(i)/12*L(i)^2 ]);
end

%% General Link Transformation

aa = sym('aa');
a = sym('a');
d = sym('d');
th = sym('th');

state = [a aa d th];

T = [ cos(th) -sin(th)*cos(a) sin(th)*sin(a) aa*cos(th);
      sin(th) cos(th)*cos(a) -cos(th)*sin(a) aa*sin(th);
      0 sin(a) cos(a) d;
      0 0 0 1];

%% Joint Transformation from link i->j

T00 = [ 1 0 0 x0;
        0 1 0 y0;
        0 0 1 z0;
        0 0 0 1];
Tij = [];
for i = 1:DOF+1
    if i==1
       Tij = cat(3, Tij, subs(T00, [x0 y0 z0], [x00 y00 z00]));
    else
       Tij = cat(3, Tij, subs(T, state, DH(i-1,:)));
    end
    
end

%% Joint Transformation from link 0->j

T0j = [];
for i = 1:DOF
    if i == 1
        T0j = cat(3, T0j, Tij(:, :, i)*Tij(:, :, i+1));
    else
        T0j = cat(3, T0j, T0j(:, :, i-1)*Tij(:, :, i+1));
    end
    
end

%T0j = simplify(T0j)
%save('T0j.mat', 'T0j');

%% Rotation submatrix from link i->j

Rij = [];
for i = 1:DOF
   Rij = cat(3, Rij, Tij(1:3, 1:3, i+1)); 
end

%% Kinematics

%% Absolute joint position

p0j = [];
for i = 1:DOF+1
   if i == 1
       p0j = cat(2, p0j, Tij(1:3, 4, i));
   else
       p0j = cat(2, p0j, T0j(1:3, 4, i-1));
   end
end

%% Absolute joint axis

z0j = [];
for i = 1:DOF+1
    if i == 1
        z0j = cat(2, z0j, Tij(1:3, 3, i));
    else
        z0j = cat(2, z0j, T0j(1:3, 3, i-1));
    end
end

%% Jacobian

Jv = [];
Jw = [];
for i = 1:DOF
   Jv = [Jv, cross(z0j(:, i), (p0j(:, DOF+1) - p0j(:, i)))];
   Jw = [Jw, z0j(:, i)];
end

J = [ Jv;
      Jw];
    
%J = simplify(J)
%save('J.mat', 'J');

%% Absolute CoM positions

pc0j = [];
for i = 1:DOF
   pc0j = cat(2, pc0j, subs(T0j(1:3, 4, i) , L(i), L(i)/2));
end

%pc0j = simplify(pc0j)
%save('pc0j.mat', 'pc0j');

%% Jacobian diff

Jc = [];
for i = 1:DOF
    Jvc=[];
    Jwc=[];
    for j = 1:i
        Jvc = [Jvc, diff(pc0j(:,i), q(j))];
        Jwc = [Jwc, z0j(:, j)];
    end
    
    for j = i+1:DOF
       Jvc = [Jvc, [0 0 0]' ];
       Jwc = [Jwc, [0 0 0]' ];
    end
    
    Jc = cat(3, Jc, [Jvc; Jwc]);
end

%Jc = simplify(Jc)
%save('Jc.mat', 'Jc');

%% Inertia matrix

M = zeros(DOF, DOF);
for i = 1:DOF
    M = M + m(i)*(Jc(1:3, :, i)')*Jc(1:3, :, i) + (Jc(4:6, :, i)')*Rij(:, :, i)*I(:, :, i)*(Rij(:, :, i)')*Jc(4:6, :, i);
end

%M = simplify(M)
%save('M.mat', 'M');

%% C(q, dq) Coriolis matrix 6x6 Christoffel Symbols

C = M;
for k = 1:DOF
    for j = 1:DOF
        C(k, j) = 0;
        for i = 1:DOF
            C(k,j) = C(k,j)+1/2*(diff(M(k,j), q(i)) + diff(M(k, i), q(j)) - diff(M(i, j), q(k)))*dq(i);
        end
    end
end

%C = simplify(C)
%save('C.mat', 'C');

%% Gravity

Jg = [];
GM = [];
for i = 1:DOF
   Jg = [Jg -Jc(1:3, :, i)'];
   GM = [GM ; -m(i)*g];
end
G = Jg*GM;

%G = simplify(G)
%save('G.mat', 'G');

%% Dynamics equation
Tau = M*ddq' + C*dq' + G;
%Tau = simplify(Tau)

%save('Tau.mat', 'Tau');



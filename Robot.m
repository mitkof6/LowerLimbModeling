%% Jim Stanev
clear all;
clc;
close all;

% initial pose
% subs(pc0, [m1 m4 m5 m6 L1 L4 L5 L6 th1 th2 th3 th4 th5 th6], [1 1 1 1 1 4 4 1 0 0 pi/2 0 0 pi/2])
% subs(J, [m1 m4 m5 m6 L1 L4 L5 L6 th1 th2 th3 th4 th5 th6], [1 1 1 1 0.2828 0.5 0.5 0.2 0 0 pi/2 0 0 pi/2])

%% Robot Parameters
% Right leg

L11 = 1;
L44 = 4;
L55 = 4;
L66 = 1;
f = pi/4;%angle hip-base

g = [0 0 -9.81]';

%% Syms

syms th1 th2 th3 th4 th5 th6

% General coordinates
q = [th1 th2 th3 th4 th5 th6];

syms dth1 dth2 dth3 dth4 dth5 dth6

dq = [dth1 dth2 dth3 dth4 dth5 dth6];

% Base translation
syms x0 y0 z0

% Link length
syms L1 L4 L5 L6

% Link mass
syms m1 m4 m5 m6
m = [m1 0 0 m4 m5 m6];

%% DH Parameters a->twist aa->link length d->d th->thi

% Base translation
x00 = -0.2;
y00 = -0.2;
z00 = 1.2;

DH = [ 0 L1*sin(f) -L1*cos(f) th1;
       -pi/2 0 0 th2;
       pi/2 0 0 th3;
       0 L4 0 th4;
       0 L5 0 th5;
       0 L6 0 th6 ];

% Links inertia

I1 = [ 0 0 0;
       0 m1/12*L1^2 0;
       0 0 m1/12*L1^2 ];

I2 = zeros(3, 3);

I3 = zeros(3, 3);

I4 = [ 0 0 0;
       0 m4/12*L4^2 0;
       0 0 m4/12*L4^2 ];
   
I5 = [ 0 0 0;
       0 m5/12*L5^2 0;
       0 0 m5/12*L5^2 ];

I6 = [ 0 0 0;
       0 m6/12*L6^2 0;
       0 0 m6/12*L6^2 ];
   
I = struct('I', []);
I(1).I = I1;
I(2).I = I2;
I(3).I = I3;
I(4).I = I4;
I(5).I = I5;
I(6).I = I6;

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
T00 = subs(T00, [x0 y0 z0], [x00 y00 z00]);

T01 = subs(T, state, DH(1,:));
T12 = subs(T, state, DH(2,:));
T23 = subs(T, state, DH(3,:));
T34 = subs(T, state, DH(4,:));
T45 = subs(T, [a aa d th], DH(5,:));
T56 = subs(T, [a aa d th], DH(6,:));

T01 = T00*T01;
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;

% Rotation submatrix
R = struct('R', []);
R(1).R = T01(1:3, 1:3);
R(2).R = T02(1:3, 1:3);
R(3).R = T03(1:3, 1:3);
R(4).R = T04(1:3, 1:3);
R(5).R = T05(1:3, 1:3);
R(6).R = T06(1:3, 1:3);

%% Kinematics

% Absolute joint position
p00 = T00(1:3, 4);
p01 = T01(1:3, 4);
p02 = T02(1:3, 4);
p03 = T03(1:3, 4);
p04 = T04(1:3, 4);
p05 = T05(1:3, 4);
p06 = T06(1:3, 4);
p0=[p00, p01, p02, p03, p04, p05, p06];

% Absolute joint axis
z00 = T00(1:3, 3);
z01 = T01(1:3, 3);
z02 = T02(1:3, 3);
z03 = T03(1:3, 3);
z04 = T04(1:3, 3);
z05 = T05(1:3, 3);
z06 = T06(1:3, 3);
z0 = [z00, z01, z02, z03, z04, z05, z06];

% Jacobian
Jv = [];
Jw = [];
for i = 1:length(DH(:, 1))
   Jv = [Jv, cross(z0(:, i), (p0(:, length(DH(:, 1))+1)-p0(:, i)))];
   Jw = [Jw, z0(:, i)];
end

J = [ Jv;
      Jw];
  
% Jt = [ cross(z00, (p06-p00)) cross(z01, (p06-p01)) cross(z02, (p06-p02)) cross(z03, (p06-p03)) cross(z04, (p06-p04)) cross(z05, (p06-p05));
%        z00 z01 z02 z03 z04 z05 ];
  
%% Dynamics

syms L
Translate=[1,0,0,-L/2;
            0,1,0,0;    
            0,0,1,0;    
            0,0,0,1];

% Absolute CoM positions
pc01 = subs(p01, L1, L1/2);
pc02 = p01;
pc03 = p01;
pc04 = subs(p04, L4, L4/2);
pc05 = subs(p05, L5, L5/2);
pc06 = subs(p06, L6, L6/2);
pc0 = [pc01, pc02, pc03, pc04, pc05, pc06];

% Jacobian CoM
for i = 1:length(DH(:,1))
    Jvc=[];
    Jwc=[];
    
    for j = 1:i
       Jvc = [Jvc, cross(z0(:, j), (pc0(:, i)-p0(:, j)))];
       Jwc = [Jwc, z0(:, j)];
    end
    
    for j = i+1:length(DH(:,1))
       Jvc = [Jvc, [0 0 0]' ];
       Jwc = [Jwc, [0 0 0]' ];
    end
    
    Jc(i).Jv=Jvc;
    Jc(i).Jw=Jwc;
end

% Inertia matrix M(q) 6x6

M =zeros(6,6);
for i = 1:length(DH(:,1))
    
    M = M + m(i)*Jc(i).Jv'*Jc(i).Jv + Jc(i).Jw'*R(i).R*I(i).I*R(i).R'*Jc(i).Jw;
end

% C(q,dq) matrix 6x6 Christoffel Symbols
C=M;
for k=1:length(DH(:,1)),
    for j=1:length(DH(:,1)),
        C(k,j)=0;
        for i=1:length(DH(:,1)),
    
        C(k,j)=C(k,j)+1/2*(diff(M(k,j),q(i))+diff(M(k,i),q(j))-diff(M(i,j),q(k)))*dq(i);
        end
    end
end

% Potential energy

P = 0;
for i = i:length(DH(:,1))
    P = P + m(i)*g'*pc0(:, i); 
end

% Gravity matrix
G = [ diff(P,th1);
      diff(P,th2);
      diff(P,th3);
      diff(P,th4);
      diff(P,th5);
      diff(P,th6)];

%% Mat

%Jr = subs(J, [m1 m4 m5 m6 L1 L4 L5 L6 th1 th2 th3 th4 th5 th6], [1 1 1 1 0.2828 0.5 0.5 0.2 0 0 pi/2 0 0 pi/2]);
%Mr = subs(M, [m1 m4 m5 m6 L1 L4 L5 L6 th1 th2 th3 th4 th5 th6], [1 1 1 1 0.2828 0.5 0.5 0.2 0 0 pi/2 0 0 pi/2]);
%Cr = subs(C, [m1 m4 m5 m6 L1 L4 L5 L6 th1 th2 th3 th4 th5 th6], [1 1 1 1 0.2828 0.5 0.5 0.2 0 0 pi/2 0 0 pi/2]);

clear all

% bot.jacob0([0 0 pi/2 0 0 pi/2])
q = [0 0 pi/2 0 0 pi/2];
qz =[0 0 0 0 0 0];
dq = [0 0 0 0 0 0];

L1 = 0.2;
L2 = 0;
L3 = 0;
L4 = 0.5;
L5 = 0.5;
L6 = 0.2;

m1 = 1;
m2 = 0;
m3 = 0;
m4 = 1;
m5 = 1;
m6 = 1;

m = [1 0 0 1 1 1];

I1 = [ 0 0 0 ;0 m1/12*L1^2 0;0 0 m1/12*L1^2 ];
I2 = zeros(3, 3);
I3 = zeros(3, 3);
I4 = [ 0 0 0 ;0 m4/12*L4^2 0;0 0 m4/12*L4^2 ];
I5 = [ 0 0 0 ;0 m5/12*L5^2 0;0 0 m5/12*L5^2 ];
I6 = [ 0 0 0 ;0 m6/12*L6^2 0;0 0 m6/12*L6^2 ];
   
pc1 = [-0.1 0 0];
pc2 = [0 0 0];
pc3 = [0 0 0];
pc4 = [-0.25 0 0];
pc5 = [-0.25 0 0];
pc6 = [-0.1 0 0];

% th d aa a
L(1) = Revolute('d', -0.2, 'a', 0.2, 'alpha', 0, ...
    'I', I1, ...
    'r', pc1, ...
    'm', m1, ...
    'Jm', 0, ...
    'G', 1);

L(2) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, ...
    'I', I2, ...
    'r', pc2, ...
    'm', m2, ...
    'Jm', 0, ...
    'G', 1, ...
    'B', 0, ...
    'Tc', [0 0]);

L(3) = Revolute('d', 0, 'a', 0, 'alpha', pi/2,  ...
    'I', I3, ...
    'r', pc3, ...
    'm', m3, ...
    'Jm', 0, ...
    'G', 1, ...
    'B', 0, ...
    'Tc', [0 0]);

L(4) = Revolute('d', 0, 'a', 0.5, 'alpha', 0,  ...
    'I', I4, ...
    'r', pc4, ...
    'm', m4, ...
    'Jm', 0, ...
    'G', 1, ...
    'B', 0, ...
    'Tc', [0 0]);

L(5) = Revolute('d', 0, 'a', 0.5, 'alpha', 0,  ...
    'I', I5, ...
    'r', pc5, ...
    'm', m5, ...
    'Jm', 0, ...
    'G', 1, ...
    'B', 0, ...
    'Tc', [0 0]);


L(6) = Revolute('d', 0, 'a', 0.2, 'alpha', 0,  ...
    'I', I6, ...
    'r', pc6, ...
    'm', m6, ...
    'Jm', 0, ...
    'G', 1, ...
    'B', 0, ...
    'Tc', [0 0]);



bot = SerialLink(L, 'name', 'LowerLimnb')

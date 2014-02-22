clear all;
clc;
close all;

%% Model
X = load('trajectoryX.dat');
Y = load('trajectoryY.dat');
Z = load('trajectoryZ.dat');

%% Estimate right leg

data = length(X(:, 1));
joints = length(X(1, :));
L = zeros(data, 4);

L(:, 1) = L(:, 1) + sqrt((X(:, 1)-X(:, 17)).*(X(:, 1)-X(:, 17))+(Y(:, 1)-Y(:, 17)).*(Y(:, 1)-Y(:, 17))+(Z(:, 1)-Z(:, 17)).*(Z(:, 1)-Z(:, 17)));
L(:, 2) = L(:, 2) + sqrt((X(:, 17)-X(:, 18)).*(X(:, 17)-X(:, 18))+(Y(:, 17)-Y(:, 18)).*(Y(:, 17)-Y(:, 18))+(Z(:, 17)-Z(:, 18)).*(Z(:, 17)-Z(:, 18)));
L(:, 3) = L(:, 3) + sqrt((X(:, 18)-X(:, 19)).*(X(:, 18)-X(:, 19))+(Y(:, 18)-Y(:, 19)).*(Y(:, 18)-Y(:, 19))+(Z(:, 18)-Z(:, 19)).*(Z(:, 18)-Z(:, 19)));
L(:, 4) = L(:, 4) + sqrt((X(:, 19)-X(:, 20)).*(X(:, 19)-X(:, 20))+(Y(:, 19)-Y(:, 20)).*(Y(:, 19)-Y(:, 20))+(Z(:, 19)-Z(:, 20)).*(Z(:, 19)-Z(:, 20)));

L = mean(L(:, :))

save('L.mat', 'L');

%% Generate traj
traj = [];
temp = zeros(6, 3);
for i = 3:data

   
    temp(1, :) = [-(X(i, 17)-X(i, 1)), Z(i, 17)-Z(i, 1) , Y(i, 17)-Y(i, 1)];
    temp(2, :) = [-(X(i, 17)-X(i, 1)), Z(i, 17)-Z(i, 1) , Y(i, 17)-Y(i, 1)];
    temp(3, :) = [-(X(i, 17)-X(i, 1)), Z(i, 17)-Z(i, 1) , Y(i, 17)-Y(i, 1)];
    temp(4, :) = [-(X(i, 18)-X(i, 1)), Z(i, 18)-Z(i, 1) , Y(i, 18)-Y(i, 1)];
    temp(5, :) = [-(X(i, 19)-X(i, 1)), Z(i, 19)-Z(i, 1) , Y(i, 19)-Y(i, 1)];
    temp(6, :) = [-(X(i, 20)-X(i, 1)), Z(i, 20)-Z(i, 1) , Y(i, 20)-Y(i, 1)];
    traj = cat(3, traj, temp);
    
end

save('Traj.mat', 'traj'); 

plot3(traj(:, 1, 100), traj(:, 2, 100), traj(:, 3, 100))
xlabel('X(t)')
ylabel('Y(t)')
zlabel('Z(t)')
%% Plot traj
joint = 19;
scale = 1;

X = smooth(X(3:end, joint));
Y = smooth(Y(3:end, joint));
Z = smooth(Z(3:end, joint));

plot3(scale*X, scale*Y, scale*Z)
xlabel('X(t)')
ylabel('Y(t)')
zlabel('Z(t)')
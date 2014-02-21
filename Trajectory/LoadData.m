clear all;
clc;
close all;

X = load('trajectoryX.dat');
Y = load('trajectoryY.dat');
Z = load('trajectoryZ.dat');

joint = 3;

plot3(X(:, joint), Y(:, joint), Z(:, joint))
xlabel('X(t)')
ylabel('Y(t)')
zlabel('Z(t)')
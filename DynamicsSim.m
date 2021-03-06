clear all;
clc;

%% Load robotics toolbox

startup_rvc
LowerLimbRT

%% Simulation parameters

ts = 0.05;
tstop = 0.5;
T = 0;
iter = 0;
Xn = [0 pi/18 pi/2 0 -pi/18 pi/2 0 0 0 0 0 0];
Tau = [16 0 0 0 0 0]';
q = [];
Xs = [];

%% Computation

while T(end)<=tstop

    [tsol, Xsol] = ode45(@idyn, [iter*ts (iter+1)*ts], Xn, [], Tau);
    
    q = [q;Xsol(:, 1:6)];
    tsol = tsol(end); %Xsol=Xsol(end,1:end);
    
    iter = iter+1;
    
    T = [T, tsol];
    Xs = [Xs;Xsol];
    Xn = Xs(end, 1:end);
    %q=[q;Xn(1:6)];
    %bot.plot(q(end, :));
end

%% Plot
figure;
bot.plot(q);


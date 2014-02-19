%% PD Control
q = X(1:6)';
dq = X(7:12)';
qRef = U(7:12)';
dqRef = U(13:18)';
error_P = qRef-q;
error_D = dqRef-dq;

Kp = diag([10 10 10 100 100 100]);
Kd = diag([3 4 4 1 1 1]);
%Ki = diag([3 1 1 1 1 1]);
%e_i = 0;
%e_i=e_i+e.*0.01;

U_control = Kp*error_P'+Kd*error_D';%+Ki*e_i';

Unew = U(1:6) + U_control ;
U = [];
U = Unew;
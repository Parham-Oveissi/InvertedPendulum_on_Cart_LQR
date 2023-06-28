clear; close all; clc
%% Initialization
m = 1;
M = 5;
L = 1;
g = 10;
d = 1;

A = [0 1 0 0;
    0 -d/M -m*g/M 0;
    0 0 0 1;
    0 -1*d/(M*L) -1*(m+M)*g/(M*L) 0];

B = [0; 1/M; 0; 1/(M*L)];

Q = eye(4)*10;
Qbar = [1 0 0 0;0 1 0 0;0 0 100 0; 0 0 0 100];
R = 0.1;
Rbar = 0.001;
%% Stability and Controllability
disp('Eigen Values:')
disp(eig(A))
disp('Rank of Controllability Matrix:')
disp(rank(ctrb(A,B)))

%% Feedback Gain
% K = lqr(A,B,Q,R);
[~,K1,~] = icare(A,B,Qbar,Rbar);
% [~,K2,~] = icare(A,B,Qbar,Rbar,'anti');
% K2 = icare(A,B,Qbar,Rbar,'anti');
% K2 = K2(4,:);
K2 = [-31.6228  75.7457 -665.0964  -253.8395];
% K2 = [105   -258 -400  -25];
% desired_poles = [-1; 2; 3; 4];
% K2 = place(A,B,desired_poles);

% K = [-10.0000  -24.4555  284.5793  123.0397];
% K2 = [-5.0000  5.0000  84.5793  -23.0397];
%% Initial Conditions and Desired States (x, x_dot, theta, theta_dot)
Initial_States = [0; 0; pi-pi/2; 0];
Desired_States = [0; 0; pi; 0];

%% Simulation
sampling_time = 0.1;
total_time = 0:sampling_time:10;

[t,state] = ode45(@(t,y)cart_pend_diff(y,m,M,L,g,d,K1,K2,(y-Desired_States)),total_time,Initial_States);

plot(t,state(:,1))
hold on
plot(t,state(:,2))
plot(t,state(:,3))
plot(t,state(:,4))
title('LQR Controller for an Inverted Pendulum')
legend('Position (m)','Velocity (m/s)','Angle (deg)','Angular Velocity (deg/s)')
hold off
pause(1.5)

figure

time = numel(total_time);
draw_cart_pendulum(state,time,sampling_time)

%% Differential Equations
function dy = cart_pend_diff(y,m,M,L,g,d,K1,K2,u)

if y(3)<0
    y(3) = 2*pi + y(3);
end

% sat = @(x, x_max, x_min) min(x_max, max(x_min,x));

if y(3)>= 3*pi/4 && y(3)<= 5*pi/4
    K = K1;
else
    K = K2;
end

u = -K*u;

Sy = sin(y(3));
Cy = cos(y(3));
D = m*L*L*(M+m*(1-Cy^2));

dy(1,1) = y(2);
dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
dy(3,1) = y(4);
dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;


end
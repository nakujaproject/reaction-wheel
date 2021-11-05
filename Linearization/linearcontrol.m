close all

%%initial conditions
x0 = [3;
      0];

%% system dynamics
A = [0 1;
    0.01 0];
B = [0;
     1];
C = [1 0];
D = 0;

%% Control law
Q = [1 0; % penalize angular error
     0 1]; % penalizes angular rate
R = 1; % penalize motor effort
K = lqr(A, B, Q, R);

%% Closed loop system
sys = ss((A-B*K), B, C, D);

%% Run response to initial condition
t = 0:0.005:30;
[y, t, x] = initial(sys, x0, t);

%% plot graphs
%% response
%set(0, 'currentfigure', h1);
hold all
p1 = plot(t, y(:, 1), 'LineWidth', 4);
%% actuator effort
%set(0, 'currentfigure', h2);
hold all
p2 = plot(t, -K * x', 'LineWidth', 4);
%% poles
%set(0, 'currentfigure', h3);
pzmap(sys);
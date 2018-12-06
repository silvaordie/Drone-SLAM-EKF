%% Extended Kalman FIlter algorithm
clear all; clc;

%% Simulation time
T=500;
ts = 1:1:T;

%% State vector
% x = [x y xl1 yl1 xl2 yl2 xl3 yl3 vx vy ax ay tetax tetay wx wy] (16 lines)
% landmark 1 (xl1, yl1) = (1,1)
% landmark 2 (xl2, yl2) = (2,3) 
% landmark 3 (xl3, yl3) = (4,1)
% vx = vy = 1m/s
% tetax = tetay = 0 rad
x = [0 0 1 1 2 3 4 1 0.2 0.2 0.05 0.05 0 0 0 0]';
LM = [1 1 2 3 4 1]';
deltaT = 0.1; %deltaT = 0.1 seconds between measurements
n_medicoes = 5; % number of landmarks (3) + 2 velocidades angulares
n_LM = 3; % number of landmarks (3)

%% Simulation of the movement
% v = [vx; vy]
% pos(1,t) = x(t); pos(2,t) = y(t)
% d(k,t) - distance to landmark k at time t
% v = [0.2; 0.1]; % velocity m/s
% pos = zeros(2,T);
% d = zeros(3,T);
% for t = 2:T
%     pos(1,t) = pos(1,t-1) + v(1).*deltaT*cos(ts(t));
%     pos(1,t) = pos(1,t-1) + v(2).*deltaT*sin(ts(t));
%     d(1,t) = (pos(1,t)-x(3))^2+(pos(2,t)-x(4))^2; 
%     d(2,t) = (pos(1,t)-x(5))^2+(pos(2,t)-x(6))^2;
%     d(3,t) = (pos(1,t)-x(7))^2+(pos(2,t)-x(8))^2;
% end

%% Control input vector 
u = 0; % there are no control actuators such as motors, so the control vector (u) and matrix (B) are ignored

%% P - process error matrix
P = 0.01*eye(length(x)); % identity matrix

%% Q - process covariance matrix (action uncertainty)
Q = diag([0 0 0.01*ones(1, 2*3) 0.01 0.01 0 0 0 0 0 0]);

%% z - measurements vector
xreal= [1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)]; % trajetória curvilínea
%xreal = [1.5*ones(size(ts)).*ts/60; 1.5*ones(size(ts)).*ts/60]; %trajetória retilínea

%% R - sensor noise matrix
R = eye(n_medicoes);
for t=1:T
    %% Prediction Step
    [x, P] = motion_model(x, u, t, P, Q);
    display(x);
    %% Update Step
    %% H - measurement matrix
    H = jacobian_H(x, n_LM);
    z = observation(xreal,LM, t, n_medicoes); % simulates sensor measurements
    [x, P] = update_step(x, z, P, H, R);   
    display(x);
    %% Graphics 
     clf;
     hold on;
     plot(xreal(1,t),xreal(2,t),'*'); % trajetória real
     plot(x(1),x(2),'+'); % trajetória obtida pelo ekf
     plot(x(3),x(4),'o'); % posição da landmark 1 obtida pelo ekf
     plot(x(5),x(6),'o'); % posição da landmark 2 obtida pelo ekf
     plot(x(7),x(8),'o');  % posição da landmark 3 obtida pelo ekf 
     plot(xreal(1,1:t),xreal(2,1:t),'b'); % trajetória real
     legend('Posiçao Real', 'Posição atual (EKF)', 'Posição LM1', 'Posição LM2', 'Posição LM3', 'Trajeto Real');
     pause(0.01);     
end
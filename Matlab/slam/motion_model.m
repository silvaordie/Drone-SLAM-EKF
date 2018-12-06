%% Motion model of the system
% x = [x y vx vy tetax tetay xl1 yl1 xl2 yl2 xl3 yl3] (12 lines)
% (x,y) - robot position
% (xli, yli) - landmark i position
% u - control input vector
% t - time
% P - process error matrix
% Q - process covariance matrix (action uncertainty)
function [x, P] = motion_model(x, u, t, P, Q)
%% Build state transition matrix [F]
F = eye(length(x)); % identity matrix
F(1,9) = t; F(2,10) = t;
F(9,11) = 1; F(10,12) = 1;
%% Build input control matrix 
B = 0; % there are no control actuators such as motors, so the control vector (u) and matrix (B) are ignored
% you should call the function with u = 0
%% prediction step
x = F*x + B*u;
P = F*P*F'+Q;
end

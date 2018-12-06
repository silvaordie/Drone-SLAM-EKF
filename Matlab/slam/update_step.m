%% Update step of the Kalman Filter
% x - state vector
% z - measurements vector
% P - process error matrix
% R - sensor noise matrix
% H - measurement matrix
function [x, P] = update_step(x, z, P, H, R)
S = H*P*H'+ R;
K = (P*H')/S;
r = calculated_distances(x);
display(z); display(r); display(K);
x = x+K*(z - r);
P = (eye(length(x)) - K*H)*P;
end
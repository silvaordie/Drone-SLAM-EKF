function z = observation(xreal, LM, t, n_med)
z = zeros(n_med,1);
z(1) = (LM(1)-xreal(1,t))^2+(LM(2)-xreal(2,t))^2 + 0.2*rand(); %distance^2 do landmark 1 at time t
z(2) = (LM(3)-xreal(1,t))^2+(LM(4)-xreal(2,t))^2 + 0.2*rand(); %distance^2 do landmark 2 at time t
z(3) = (LM(5)-xreal(1,t))^2+(LM(6)-xreal(2,t))^2 + 0.2*rand(); %distance^2 do landmark 3 at time t
z(4) = 0; % wx - nao influencia (agora)
z(5) = 0; % wy - nao influencia (agora)
end
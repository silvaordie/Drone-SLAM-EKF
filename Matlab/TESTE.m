v=[1 1 0 ; 3 1 2];
z=[1.1 0.9 1.03; 0.89 0.99 1.05; 1.11 1.15 0.9];

[lm, e]=trilat(v,z)

load matlab.mat;

v=x(1:2, 1:1:length(x));
z=z(1, 1:1:length(z));

[lm, e]=trilat(v,z)
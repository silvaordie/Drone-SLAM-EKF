clear;

T=500;
LANDMARKS = 3;
ts = 1:1:T;
LS=[ 2 2; 0 2; -2 0];

x=zeros(6+2*LANDMARKS, T);

x(:, 1) = [0; 0; 0; 0; 0; 0; 1; 2; 0; 2; -2; 0];

xreal= [1-cos(ts) ; sin(ts)];
cov=diag([ 0 0 0 0 0 0 1 2*zeros(1, 2*LANDMARKS-1)]);

F=eye(6+2*LANDMARKS);
F(1,4)=1;
F(2,5)=1;
F(3,6)=1;
for t=2:1:T
    
   
   %Prediction
   xp(1:3)=x(1:3, t-1)+x(4:6,t-1);
   xp(4:6)=x(4:6,t-1);
   xp(7:6+2*LANDMARKS)=x(7:6+2*LANDMARKS,t-1);
   covp=F*cov*F'+diag([10 10 pi/10 1 1 1 0 0 0 0 0 0 ]);
   
   %Update
   z=obs(xreal(:,t)',LS);
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([1 1 1]);
   K=covp*H(xp, LANDMARKS)'*inv(S);
   cov=covp-K*S*K';
   x(:,t)=xp'+K*(z-hp(xp));
   
end
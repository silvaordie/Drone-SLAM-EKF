clear;

T=500;
LANDMARKS = 3;
ts = 1:1:T;
LS=[ 0 1.5; 3 2; 2 -2]; 

x=zeros(6+2*LANDMARKS, T);
z=zeros(3, T);
x(:, 1) = [0; 0; 0; 0; 0; 0; 0.1; 1.3; 3; 2.1; 2.1; -1.9];

xreal= [ 1.5*ones(size(ts)) - 1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)];
cov=diag([ 0 0 0 0 0 0 0.3*ones(1, 2*LANDMARKS)]);

F=eye(6+2*LANDMARKS);
F(1,4)=1;
F(2,5)=1;
F(3,6)=1;

figure;
hold on;
for t=2:1:T
    
   
   %% Prediction
   %Assume um valor de velocidades parecido ao do instante anterior
   xp(4:6)=x(4:6,t-1);
   %Itera a posição com base na velocidade assumida
   xp(1:3)=x(1:3, t-1)+xp(4:6)';
   %As landmarks não são afetadas pelo movimento do drone
   xp(7:6+2*LANDMARKS)=x(7:6+2*LANDMARKS,t-1);
   
   %Atualiza a matriz das covariancias
   covp=F*cov*F'+diag([1 1 pi/10 10 10 10 0 0 0 0 0 0 ]);
   
   
   
   %% Update
   %Determina as dstâncias às landmarks (com erros maximos de 0.2)
   z(:,t)=obs(xreal(:,t)',LS);
   %Determina o S 
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([0.04 0.04 0.04]);
   %Calcula o ganho de Kalman
   K=covp*H(xp, LANDMARKS)'*inv(S);
   %Corrige a matriz das covariâncias
   cov=covp-K*S*K';
   
   %Corrige a estimativa do vetor de estado
   %Inovation
   
   i=z(:,t)-hp(xp);
   %Filtro de outliers
   e=(abs(i)<5);
   x(:,t)=xp'+(K*(e.*i));   
   %Filtro passa baixo para suavizar o sinal
   %x(1:3,t)=(xpre(1:3)+x(1:3,t-1))/2;
   %x(4:12,t)=xpre(4:12);
   
   
   %% Grafismos
   clf;
   hold on;
   plot(xreal(1,t),xreal(2,t),'+');
   plot(x(1,t),x(2,t),'+');
   plot(x(7,t),x(8,t),'o');
   plot(x(9,t),x(10,t),'o');
   plot(x(11,t),x(12,t),'o');   
   
   plot(x(1,1:t),x(2,1:t),'r');
   plot(xreal(1,1:t),xreal(2,1:t),'b');
   
   plot(LS(1,1),LS(1,2),'.');
   plot(LS(2,1),LS(2,2),'.');
   plot(LS(3,1),LS(3,2),'.');
   
   legend('Posição atual (EKF)', 'Posiçao Real', 'Posição LM1', 'Posição LM2', 'Posição LM3', 'Trajeto (EKF)', 'Trajeto Real');
   axis([-3 7 -4 4]);
   grid on;
   title('Espaço de Estados');
   xlabel('X1');
   ylabel('X2');
   pause(0.1);
   
end
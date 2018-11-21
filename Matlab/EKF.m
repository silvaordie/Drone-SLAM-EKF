clear;

T=1000;
LANDMARKS = 3;
ts = 1:1:T;
LS=[ 1 2; 0 -2; 2 0];

x=zeros(6+2*LANDMARKS, T);
z=zeros(3, T);
x(:, 1) = [0; 0; 0; 0; 0; 0; 1; 2; 0; -2; 2; 0];

xreal= [ 1.5*ones(size(ts)) - 1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)];
cov=diag([ 0 0 0 10 10 10 0 2*zeros(1, 2*LANDMARKS-1)]);

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
   %Itera a posi��o com base na velocidade assumida
   xp(1:3)=x(1:3, t-1)+xp(4:6)';
   %As landmarks n�o s�o afetadas pelo movimento do drone
   xp(7:6+2*LANDMARKS)=x(7:6+2*LANDMARKS,t-1);
   
   %Atualiza a matriz das covariancias
   covp=F*cov*F'+diag([1 1 pi/10 10 10 10 0 0 0 0 0 0 ]);
   
   
   
   %% Update
   %Determina as dst�ncias �s landmarks (com erros maximos de 0.2)
   z(:,t)=obs(xreal(:,t)',LS);
   %Determina o S 
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([0.04 0.04 0.04]);
   %Calcula o ganho de Kalman
   K=covp*H(xp, LANDMARKS)'*inv(S);
   %Corrige a matriz das covari�ncias
   cov=covp-K*S*K';
   %Corrige a estimativa do vetor de estado
   x(:,t)=xp'+K*(z(:,t)-hp(xp));   
   %Filtro passa baixo para suavizar o sinal
   %x(1:3,t)=(xpre(1:3)+x(1:3,t-1))/2;
   %x(4:12,t)=xpre(4:12);
   
   
   %% Grafismos
   hold on;
   plot(x(1,t),x(2,t),'+');
   plot(xreal(1,t),xreal(2,t),'o');
   plot(x(7,t),x(8,t),'o');
   plot(x(9,t),x(10,t),'o');
   plot(x(11,t),x(12,t),'o');
   
   
   plot(x(1,1:t),x(2,1:t),'r');
   plot(xreal(1,1:t),xreal(2,1:t),'y');
   legend('Posi��o atual (EKF)', 'Posi�ao Real', 'Posi��o LM1', 'Posi��o LM2', 'Posi��o LM3', 'Trajeto (EKF)', 'Trajeto Real');
   axis([-1 4 -2.5 2.5]);
   grid on;
   title('Espa�o de Estados');
   xlabel('X1');
   ylabel('X2');
   pause(0.1);
   clf;
end
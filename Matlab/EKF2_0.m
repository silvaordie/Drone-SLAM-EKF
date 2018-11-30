clear;

LS=[ -2 0 -3 ; 0 0 4 ];
x(1:2,1)=[LS(:,1)];
V=LS(:,2)-LS(:,1)/50;
d(1,1)=0;
d(2,1)=(x(1,1)-LS(1,2))^2+(x(2,1)-LS(2,2))^2 + 0.2*rand(1,1);
d(3,1)=(x(1,1)-LS(1,3))^2+(x(2,1)-LS(2,3))^2 + 0.2*rand(1,1);
for k=2:1:50
    x(:,k)=x(:,k-1)+V;
    d(1,k)=(x(1,k)-LS(1,1))^2+(x(2,k)-LS(2,1))^2 + 0.2*rand(1,1);
    d(2,k)=(x(1,k)-LS(1,2))^2+(x(2,k)-LS(2,2))^2 + 0.2*rand(1,1);
    d(3,k)=(x(1,k)-LS(1,3))^2+(x(2,k)-LS(2,3))^2 + 0.2*rand(1,1);
end
v=[-d(2,:).^0.5;zeros(1,length(d(3,:)))];
z=d(3,:);
[lm, e]=trilat(v, z, d(3,k-1));
D=-sqrt(d(2,1));

clear -lm -e -D

LS=LS';
T=500;
LANDMARKS = 3;
ts = 1:1:T;

x=zeros(6+2*LANDMARKS, T);
z=zeros(3, T);
x(:, 1) = [0; 0; 0; 0; 0; 0; D; 0; 0; 0; lm(1,1); lm(2,1)];

xreal= [ -1.5*ones(size(ts)) + 1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)];
cov=diag([ 0 0 0 0 0 0 e^2*ones(1, 2*LANDMARKS)]);

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
   
   legend('Posi��o atual (EKF)', 'Posi�ao Real', 'Posi��o LM1', 'Posi��o LM2', 'Posi��o LM3', 'Trajeto (EKF)', 'Trajeto Real');
   axis([-6 4 -4 4]);
   grid on;
   title('Espa�o de Estados');
   xlabel('X1');
   ylabel('X2');
   pause(0.02);
   
end
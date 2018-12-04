clear;

%% Inicializa��o
%Posi��o das landmarks
%1- D 0
%2- 0 0
%3- ? ?
LS=[ -2 0 -3 ; 0 0 4 ];

%Posi��o inicial corresponde � 1� landmark
x(1:2,1)=[LS(:,1)];

%Assume-se velocidade constante (A norma pode variar, a dire��o � que n�o)
V=LS(:,2)-LS(:,1)/50;
%Primeiras observa��es
d(1,1)=0;
d(2,1)=(x(1,1)-LS(1,2))^2+(x(2,1)-LS(2,2))^2 + 0.2*rand(1,1);
d(3,1)=(x(1,1)-LS(1,3))^2+(x(2,1)-LS(2,3))^2 + 0.2*rand(1,1);

%Simula o movimento
for k=2:1:50
    x(:,k)=x(:,k-1)+V;
    d(1,k)=(x(1,k)-LS(1,1))^2+(x(2,k)-LS(2,1))^2 + 0.2*rand(1,1);
    d(2,k)=(x(1,k)-LS(1,2))^2+(x(2,k)-LS(2,2))^2 + 0.2*rand(1,1);
    d(3,k)=(x(1,k)-LS(1,3))^2+(x(2,k)-LS(2,3))^2 + 0.2*rand(1,1);
end

%Trajeto do ve�culo corresponde a um movimento rectilineo com y=0
%A posi��o x do veiculo corresponde ao sim�trico dist�ncia � 2� landmark 
v=[-d(2,:).^0.5;zeros(1,length(d(3,:)))];

z=d(3,:);

%Determina a posi��o da 3� Landmark por trilatera��o
[lm, e]=trilat(v, z, d(3,k-1));

D=-sqrt(d(2,1));
clear -lm -e -D
%%Execu��o
%Copa as landmarks
LS=LS';
%Define o 'tempo' de execu��o 
T=500;
%N�mero de Landmarks
LANDMARKS = 3;
%Simula a trajet�ria real
ts = 1:1:T;
xreal= [ -1.5*ones(size(ts)) + 1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)];
areal= [ zeros(1,length(ts)); 0:pi/(6*(T-1)):pi/6 ; 0:pi/(T-1):pi ];
%Vetor de estado ao longo do tempo
x=zeros(10+2*LANDMARKS, T);
%Vetor de observa��es ao longo do tempo
z=zeros(3+LANDMARKS, T);

%INICIALIZA��O
%Posi��o e velocidade do ve�culo a (0,0) e segunda landmark a (0,0)
%Segunda Landmark Inicializada a (D,0)
%Terceira Landmark inicializada com o resultado da trilatera��o
x(:, 1) = [0; 0; 0; 0; D; 0; 0; 0; lm(1,1); lm(2,1) ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%Var�nicia da posi�ao , velocidade do ve�culo e da segunda landmark � nula (assume-se que est� 100% correta)
%Variancia do x primeira landmark assume-se a variancia das medi��es
%Vari�ncia da 3� landmark corresponde ao maior desvio quadr�tico proveniente da trilatera��o
cov=diag([ 0 0 0 0 0.04 0 0 0 e^2*ones(1, 2) 0 0 0 0 0 0]);

%Matriz F
F=eye(10+2*LANDMARKS);
F(1,3)=1;
F(2,4)=1;
F(length(x(:, 1))-5,length(x(:, 1))-2)=1;
F(length(x(:, 1))-4,length(x(:, 1))-1)=1;
F(length(x(:, 1))-3,length(x(:, 1)))=1;

for t=2:1:T
    
   
   %% Prediction
   %Assume um valor de velocidades parecido ao do instante anterior
   xp(3:4)=x(3:4,t-1);
   %Itera a posi��o com base na velocidade assumida
   xp(1:2)=x(1:2, t-1)+xp(3:4)';
   %As landmarks n�o s�o afetadas pelo movimento do drone
   xp(5:10+2*LANDMARKS)=x(5:10+2*LANDMARKS,t-1);
   
   %Atualiza a matriz das covariancias
   covp=F*cov*F'+diag([0.1 0.1 0.1 0.1 0 0 0 0 0 0 pi/10 pi/10 pi/10 0.0001 0.0001 0.0001 ]);
   
   
   
   %% Update
   %Determina as dst�ncias �s landmarks (com erros maximos de 0.2)
   z(:,t)=obs(xreal(:,t)',LS, areal(:,k));
   %Determina o S 
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([0.04 0.04 0.04 0 0 0]);
   %Calcula o ganho de Kalman
   K=covp*H(xp, LANDMARKS)'*inv(S);
   %Corrige a matriz das covari�ncias
   cov=covp-K*S*K';
   
   %Corrige a estimativa do vetor de estado
   %Inovation
   
   i=z(:,t)-hp(xp);
   %Filtro de outliers
   x(:,t)=xp'+(K*i);   
   %Filtro passa baixo para suavizar o sinal
   %x(1:3,t)=(xpre(1:3)+x(1:3,t-1))/2;
   %x(4:12,t)=xpre(4:12);
   
   
   %% Grafismos
   figure(1);
    hold on;
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
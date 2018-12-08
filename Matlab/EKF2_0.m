clear;

%% Inicialização
%Posição das landmarks
%1- D 0
%2- 0 0
%3- ? ?
%LS=[ -2 0 -3 ; 0 0 1.5 ];
LS=[ -2 0 -1 ; 0 0 1.5 ];
%Posição inicial corresponde à 1ª landmark
x(1:2,1)=[LS(:,1)];

%Assume-se velocidade constante (A norma pode variar, a direção é que não)
V=LS(:,2)-LS(:,1)/50;
%Primeiras observações
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

%Trajeto do veículo corresponde a um movimento rectilineo com y=0
%A posição x do veiculo corresponde ao simétrico distância à 2ª landmark 
v=[-d(2,:).^0.5;zeros(1,length(d(3,:)))];

z=d(3,:);

%Determina a posição da 3ª Landmark por trilateração
[lm, e]=trilat(v, z, d(3,k-1));

D=-sqrt(d(2,1));
clear -lm -e -D
%%Execução
%Copa as landmarks
LS=LS';
%Define o 'tempo' de execução 

%Número de Landmarks
LANDMARKS = 3;
SIZE=10+2*LANDMARKS;
%Simula a trajetória real

%T=200;
%ts = 1:1:T;
%xreal= [ -1.5*ones(size(ts)) + 1.5*cos(0.1*ts) ; 1.5*sin(0.1*ts)];

%T=180;
%xreal = [ 0*ones(1,20) , 0:0.05:1-0.05 , 0.95*ones(1,20) , 1-0.05:-0.05:0 , 0*ones(1,20), 0:0.05:1-0.05 , 0.95*ones(1,20) , 1-0.05:-0.05:0 , 0*ones(1,20) ; 0:0.05:1-0.05 , 0.95*ones(1,20) , 1-0.05:-0.05:0 , 0*ones(1,20) , 0:0.05:1-0.05 , 0.95*ones(1,20) , 1-0.05:-0.05:0 , 0*ones(1,20) , 0:0.05:1-0.05];
%xreal=-xreal;

T=100;
ts = 1:1:T;
xreal = [ -1.5*ones(size(ts)) + 1.5*cos(0.1*ts) + 0.1*sin(0.8*ts) ; 1.5*sin(0.1*ts)+0.3*sin(0.4*ts)];

areal= [ 0.1*sin(1/(T/2)*ts) ; 0.3*sin(1/(T/4)*ts) ; zeros(1,length(ts)) ];
angreal=cumsum(areal,2);
%Vetor de estado ao longo do tempo1
x=zeros(10+2*LANDMARKS, T);
%Vetor de observações ao longo do tempo
z=zeros(3+LANDMARKS, T);

%INICIALIZAÇÃO
%Posição e velocidade do veículo a (0,0) e segunda landmark a (0,0)
%Segunda Landmark Inicializada a (D,0)
%Terceira Landmark inicializada com o resultado da trilateração
x(:, 1) = [0; 0; 0; 0; D; 0; 0; 0; lm(1,1); lm(2,1) ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];
%Varânicia da posiçao , velocidade do veículo e da segunda landmark é nula (assume-se que está 100% correta)
%Variancia do x primeira landmark assume-se a variancia das medições
%Variância da 3ª landmark corresponde ao maior desvio quadrático proveniente da trilateração
cov=diag([ 0 0 0 0 0.04 0 0 0 e^2*ones(1, 2) 0 0 0 0.3 0.3 0.3]);

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
   xp(SIZE-2:SIZE)=x(SIZE-2:SIZE,t-1);
   %Itera a posição com base na velocidade assumida
   xp(1:2)=x(1:2, t-1)+xp(3:4)';
   xp(SIZE-5:SIZE-3)=x(SIZE-5:SIZE-3, t-1)+xp(SIZE-2:SIZE)';
   %As landmarks não são afetadas pelo movimento do drone
   xp(5:SIZE-6)=x(5:SIZE-6,t-1);
   
   %Atualiza a matriz das covariancias
   covp=F*cov*F'+diag([0.01 0.01 0.1 0.1 0 0 0 0 0 0 pi/15 pi/15 pi/15 1 1 1]);
   
   
   
   %% Update
   %Determina as dstâncias às landmarks (com erros maximos de 0.2)
   z(:,t)=obs(xreal(:,t)',LS, areal(:,t));
   %Determina o S 
   S=H(xp, LANDMARKS)*covp*H(xp, LANDMARKS)' + diag([0.2 0.2 0.2 0.01 0.01 0.01]);
   %Calcula o ganho de Kalman
   K=covp*H(xp, LANDMARKS)'*inv(S);
   %Corrige a matriz das covariâncias
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
   
   ref_real=rotation(angreal(:,t));
   ref_ekf=rotation(x(SIZE-5:SIZE-3,t));
   
   %% Posições
   figure(1);
   clf;
   hold on;
   plot(x(1,t),x(2,t),'*', 'MarkerEdgeColor', 'r', 'LineWidth', 2);
   plot(xreal(1,t),xreal(2,t),'+', 'MarkerEdgeColor', 'b', 'LineWidth', 2);
   plot(x(5,t),x(6,t),'o', 'MarkerEdgeColor', 'g', 'LineWidth', 2);
   plot(LS(1,1),LS(1,2),'s', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
   plot(x(7,t),x(8,t),'o', 'MarkerEdgeColor', 'g', 'LineWidth', 2);
   plot(x(9,t),x(10,t),'o', 'MarkerEdgeColor', 'g', 'LineWidth', 2);   
   plot(x(1,1:t),x(2,1:t),'r', 'LineWidth', 2);
   plot(xreal(1,1:t),xreal(2,1:t),'b', 'LineWidth', 2);
   plot(LS(2,1),LS(2,2),'s', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
   plot(LS(3,1),LS(3,2),'s', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
   
   legend('Current Estimate (EKF)', 'Actual Position', 'Estimated Landmarks', 'Actual Landmarks');
   axis([-3.2 1 -2 2]);
   grid on;
   title(['State Space (t=' num2str(t) ')']);
   xlabel('X1');
   ylabel('X2');
   
   %% Orientação
   figure(2);
   clf;
   hold on
   
   title(['Drone Orientation (t=' num2str(t) ')'])
   quiver3(0,0,0,ref_real(1,1),ref_real(2,1),ref_real(3,1), 'b');
   quiver3(0,0,0,ref_ekf(1,1),ref_ekf(2,1),ref_ekf(3,1), 'r');
   quiver3(0,0,0,ref_real(1,2),ref_real(2,2),ref_real(3,2), 'b');
   quiver3(0,0,0,ref_real(1,3),ref_real(2,3),ref_real(3,3), 'b');
   quiver3(0,0,0,ref_ekf(1,2),ref_ekf(2,2),ref_ekf(3,2), 'r');
   quiver3(0,0,0,ref_ekf(1,3),ref_ekf(2,3),ref_ekf(3,3), 'r');  
   text(ref_real(1,1),ref_real(2,1),ref_real(3,1), 'x');
   text(ref_real(1,2),ref_real(2,2),ref_real(3,2), 'y');
   text(ref_real(1,3),ref_real(2,3),ref_real(3,3), 'z');
   text(ref_ekf(1,1),ref_ekf(2,1),ref_ekf(3,1), 'x"');
   text(ref_ekf(1,2),ref_ekf(2,2),ref_ekf(3,2), 'y"');
   text(ref_ekf(1,3),ref_ekf(2,3),ref_ekf(3,3), 'z"');
   view(135,45);
   
   legend('Actual Orientation','Estimated Orientation');
   grid on;
   set(gca,'XTick',[], 'YTick', [], 'ZTick', []);
   pause(0.0001);
   
end